"""Write an MP4 from rendered frames, by piping raw video to ``ffmpeg``.

An external binary invoked as a subprocess, which is the pattern ADR-0005 already establishes
for CalculiX: no Python encoder is added to the dependency set, the tool that does this best is
used directly, and its absence is a **typed absence** rather than an exception — a run with no
``ffmpeg`` still writes its GIF and says why there is no MP4.

Why bother, when there is already a GIF. A GIF is limited to 256 colours and one alpha bit,
which is precisely wrong for these renders: the translucent CAD overlay dithers into bands and
the frame count has to be kept low to keep the file sane. The same 125 frames come out roughly
an order of magnitude smaller as H.264, at full colour, and scrub frame by frame in any player
— which is what a visual regression check actually needs (`13-engineering.md`).
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

__all__ = ["FFMPEG_ENV_VAR", "VideoUnavailable", "find_ffmpeg", "write_mp4"]

#: Overrides discovery, for a machine where ``ffmpeg`` is not on ``PATH``. Mirrors
#: ``WHEELOPT_CCX`` in :mod:`wheelopt.fea.runner`.
FFMPEG_ENV_VAR = "WHEELOPT_FFMPEG"


class VideoUnavailable(RuntimeError):
    """No encoder, or the encode failed. Callers should treat the MP4 as optional."""


def find_ffmpeg(explicit: str | Path | None = None) -> Path | None:
    """``ffmpeg``, or ``None``. Explicit argument, then the env var, then ``PATH``.

    Returns ``None`` rather than raising, so a caller can decide that no video is fine —
    which it always is here, because nothing downstream reads the file.
    """
    for candidate in (explicit, os.environ.get(FFMPEG_ENV_VAR)):
        if candidate:
            path = Path(candidate)
            if path.is_file():
                return path
    found = shutil.which("ffmpeg")
    return Path(found) if found else None


def ffmpeg_command(ffmpeg: Path, width: int, height: int, fps: int, out: Path,
                   *, crf: int = 18) -> list[str]:
    """The encode command, as a list. Separated out so it can be tested without encoding.

    ``-vf pad=...`` rounds odd dimensions up to even. H.264 in ``yuv420p`` requires even width
    and height, and the renderer's height is ``pixels * 9 / 16`` — 900 gives 506, but 902 gives
    507 and the encode fails with a message about the pixel format that says nothing about the
    real cause. Padding is one line and removes the whole class.
    """
    return [
        str(ffmpeg), "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
        "-an",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-c:v", "libx264", "-preset", "medium", "-crf", str(crf),
        "-pix_fmt", "yuv420p",
        # So a player can start before the whole file is buffered, which matters when these
        # are opened straight off a network share.
        "-movflags", "+faststart",
        str(out),
    ]


def write_mp4(frames, out: Path, fps: int = 25, *, ffmpeg: str | Path | None = None,
              crf: int = 18) -> Path:
    """Encode ``frames`` — a sequence of ``(h, w, 3)`` uint8 RGB arrays — to H.264.

    Args:
        frames: rendered frames, all the same shape. MuJoCo's ``Renderer.render()`` output.
        out: destination ``.mp4``. Parent directories are created.
        fps: playback rate. Should match the rate the frames were sampled at, or the video
            runs at the wrong speed and every visual judgement made from it is off.
        ffmpeg: explicit binary, overriding discovery.
        crf: H.264 quality, lower is better. 18 is visually lossless for synthetic renders.

    Raises:
        VideoUnavailable: no ``ffmpeg``, no frames, ragged frames, or a non-zero exit.
    """
    frames = list(frames)
    if not frames:
        raise VideoUnavailable("no frames to encode")
    # An explicitly named binary that is not there is an error, not a hint. Falling through to
    # PATH would encode with a *different* ffmpeg than the caller asked for and say nothing —
    # and the whole reason to name one is that the one on PATH is not the one you want.
    if ffmpeg is not None and not Path(ffmpeg).is_file():
        raise VideoUnavailable(f"no ffmpeg at {ffmpeg}")
    binary = find_ffmpeg(ffmpeg)
    if binary is None:
        raise VideoUnavailable(
            "no ffmpeg on PATH; install it (brew install ffmpeg) or set "
            f"{FFMPEG_ENV_VAR}. The GIF is unaffected."
        )
    first = np.asarray(frames[0])
    if first.ndim != 3 or first.shape[2] != 3:
        raise VideoUnavailable(f"frames must be (h, w, 3) RGB; got {first.shape}")
    height, width = first.shape[:2]
    if any(np.asarray(f).shape != first.shape for f in frames):
        raise VideoUnavailable("every frame must have the same shape")

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    command = ffmpeg_command(binary, width, height, fps, out, crf=crf)
    with subprocess.Popen(command, stdin=subprocess.PIPE,
                          stderr=subprocess.PIPE) as process:
        try:
            for frame in frames:
                process.stdin.write(np.ascontiguousarray(frame, dtype=np.uint8).tobytes())
            process.stdin.close()
        except BrokenPipeError as exc:  # ffmpeg died early; its stderr says why
            process.wait()
            raise VideoUnavailable(
                f"ffmpeg closed the pipe: {process.stderr.read().decode(errors='replace')}"
            ) from exc
        stderr = process.stderr.read().decode(errors="replace")
        code = process.wait()
    if code != 0:
        raise VideoUnavailable(f"ffmpeg exited {code}: {stderr}")
    return out
