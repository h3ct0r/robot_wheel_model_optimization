"""MP4 encoding through ffmpeg. Command construction is pure; encoding needs the binary.

The point of the split is that the part most likely to be wrong — the argument list — is
testable on a machine with no encoder at all, which is the same argument `fea/deck.py` makes
about CalculiX decks.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from wheelopt.video import (
    FFMPEG_ENV_VAR,
    VideoUnavailable,
    ffmpeg_command,
    find_ffmpeg,
    write_mp4,
)

HAVE_FFMPEG = find_ffmpeg() is not None


def frames(n: int = 8, w: int = 64, h: int = 48) -> list[np.ndarray]:
    """A moving bar, so a wrong frame order or a dropped frame is visible in the output."""
    out = []
    for i in range(n):
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:, (i * 5) % w:(i * 5) % w + 4] = 255
        out.append(frame)
    return out


class TestFindFfmpeg(unittest.TestCase):
    def test_an_explicit_path_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp) / "ffmpeg"
            fake.write_text("")
            self.assertEqual(find_ffmpeg(fake), fake)

    def test_the_env_var_is_consulted(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp) / "ffmpeg"
            fake.write_text("")
            previous = os.environ.get(FFMPEG_ENV_VAR)
            os.environ[FFMPEG_ENV_VAR] = str(fake)
            try:
                self.assertEqual(find_ffmpeg(), fake)
            finally:
                if previous is None:
                    del os.environ[FFMPEG_ENV_VAR]
                else:
                    os.environ[FFMPEG_ENV_VAR] = previous

    def test_a_path_that_does_not_exist_falls_through(self):
        self.assertNotEqual(find_ffmpeg("/definitely/not/here"), Path("/definitely/not/here"))


class TestCommand(unittest.TestCase):
    CMD = ffmpeg_command(Path("/usr/bin/ffmpeg"), 900, 506, 25, Path("/tmp/out.mp4"))

    def test_the_input_is_raw_rgb_of_the_right_size(self):
        self.assertIn("rawvideo", self.CMD)
        self.assertIn("rgb24", self.CMD)
        self.assertIn("900x506", self.CMD)
        self.assertEqual(self.CMD[self.CMD.index("-i") + 1], "-")

    def test_it_is_h264_in_a_pixel_format_players_accept(self):
        self.assertIn("libx264", self.CMD)
        self.assertIn("yuv420p", self.CMD)

    def test_odd_dimensions_are_padded_rather_than_failing_late(self):
        """H.264 in yuv420p needs even width and height, and the renderer's height is
        `pixels * 9 / 16` — 900 gives 506 but 902 gives 507. Without the pad, ffmpeg fails
        with a message about the pixel format that says nothing about the real cause."""
        filters = self.CMD[self.CMD.index("-vf") + 1]
        self.assertIn("ceil(iw/2)*2", filters)
        self.assertIn("ceil(ih/2)*2", filters)

    def test_the_frame_rate_is_carried_through(self):
        command = ffmpeg_command(Path("ffmpeg"), 10, 10, 60, Path("x.mp4"))
        self.assertEqual(command[command.index("-r") + 1], "60")

    def test_quality_is_settable(self):
        command = ffmpeg_command(Path("ffmpeg"), 10, 10, 25, Path("x.mp4"), crf=30)
        self.assertEqual(command[command.index("-crf") + 1], "30")


class TestWriteMp4(unittest.TestCase):
    def test_no_frames_is_refused(self):
        with self.assertRaises(VideoUnavailable):
            write_mp4([], Path("/tmp/x.mp4"))

    def test_a_named_binary_that_is_not_there_is_an_error_and_not_a_fallback(self):
        """Falling through to PATH would encode with a *different* ffmpeg than the caller
        asked for and say nothing — and the whole reason to name one is that the one on PATH
        is not the one you want. Typed absence, either way: the caller still writes its GIF."""
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(VideoUnavailable) as ctx:
            write_mp4(frames(), Path(tmp) / "x.mp4", ffmpeg="/definitely/not/here")
        self.assertIn("/definitely/not/here", str(ctx.exception))

    @unittest.skipIf(HAVE_FFMPEG, "ffmpeg is installed")
    def test_no_encoder_at_all_says_how_to_get_one(self):
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(VideoUnavailable) as ctx:
            write_mp4(frames(), Path(tmp) / "x.mp4")
        self.assertIn("ffmpeg", str(ctx.exception))

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg not installed")
    def test_ragged_frames_are_refused_before_encoding(self):
        bad = frames(3)
        bad[1] = np.zeros((10, 10, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(VideoUnavailable):
            write_mp4(bad, Path(tmp) / "x.mp4")

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg not installed")
    def test_a_non_rgb_frame_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(VideoUnavailable):
            write_mp4([np.zeros((8, 8), dtype=np.uint8)], Path(tmp) / "x.mp4")

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg not installed")
    def test_it_writes_a_playable_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = write_mp4(frames(12), Path(tmp) / "clip.mp4", fps=12)
            self.assertTrue(out.is_file())
            self.assertGreater(out.stat().st_size, 200)
            self.assertEqual(out.read_bytes()[4:8], b"ftyp")

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg not installed")
    def test_odd_dimensions_encode(self):
        odd = [np.zeros((45, 91, 3), dtype=np.uint8) for _ in range(4)]
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(write_mp4(odd, Path(tmp) / "odd.mp4").is_file())

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg not installed")
    def test_it_creates_the_output_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = write_mp4(frames(4), Path(tmp) / "deep" / "er" / "clip.mp4")
            self.assertTrue(out.is_file())


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
