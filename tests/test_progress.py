"""Stage lines and the progress bar. Pure text, so all of it runs anywhere.

Progress output is the kind of code that rots silently — nobody notices a bar that stopped
being drawn, and nobody notices one that writes escape sequences into a log file. Both are
pinned here, along with the property that actually matters for the simulation loop: calling
`update` on every one of tens of thousands of steps must not cost tens of thousands of writes.
"""

from __future__ import annotations

import io
import unittest

from wheelopt.progress import Bar, Stage, format_duration


class Fake(io.StringIO):
    """A stream that can pretend to be, or not be, a terminal."""

    def __init__(self, tty: bool = True) -> None:
        super().__init__()
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty


class TestFormatDuration(unittest.TestCase):
    def test_it_stays_short_at_every_scale(self):
        for seconds, expected in ((0.42, "0.4s"), (12.34, "12.3s"), (59.9, "59.9s"),
                                  (60.0, "1m00s"), (252.0, "4m12s"), (3600.0, "1h00m"),
                                  (7830.0, "2h10m")):
            with self.subTest(seconds=seconds):
                self.assertEqual(format_duration(seconds), expected)
                self.assertLessEqual(len(format_duration(seconds)), 6)


class TestStage(unittest.TestCase):
    def test_it_announces_before_and_times_after(self):
        stream = Fake()
        with Stage("meshing", stream=stream):
            self.assertTrue(stream.getvalue().startswith("-> meshing ... "))
            self.assertNotIn("\n", stream.getvalue())
        self.assertTrue(stream.getvalue().endswith("s\n"))

    def test_notes_land_on_the_completion_line(self):
        stream = Fake()
        with Stage("FEA", stream=stream) as stage:
            stage.note("cached")
            stage.note("492 elements")
        self.assertIn("(cached, 492 elements)", stream.getvalue())

    def test_an_empty_note_is_dropped(self):
        stream = Fake()
        with Stage("x", stream=stream) as stage:
            stage.note("")
        self.assertNotIn("()", stream.getvalue())

    def test_a_failing_block_closes_its_line_and_re_raises(self):
        """A half-written progress line above a traceback reads as though the traceback came
        from the *next* stage, which sends you looking in the wrong place."""
        stream = Fake()
        with self.assertRaises(ValueError), Stage("solve", stream=stream):
            raise ValueError("boom")
        self.assertTrue(stream.getvalue().endswith("FAILED\n"))

    def test_it_records_its_own_duration(self):
        with Stage("x", stream=Fake()) as stage:
            pass
        self.assertGreaterEqual(stage.seconds, 0.0)

    def test_a_stage_holding_a_bar_gives_the_label_its_own_line(self):
        """The bar redraws with `\\r`, which returns to column zero of the *terminal* — so an
        inline stage's `-> label ...` prefix gets eaten by the first redraw and the timing
        below it reads as though it belonged to nothing."""
        stream = Fake()
        with Stage("simulating", stream=stream, inline=False) as stage:
            self.assertEqual(stream.getvalue(), "-> simulating\n")
            stage.note("125 frames")
        lines = stream.getvalue().splitlines()
        self.assertEqual(lines[0], "-> simulating")
        self.assertTrue(lines[1].startswith("   "))
        self.assertIn("(125 frames)", lines[1])

    def test_a_multiline_stage_still_closes_on_failure(self):
        stream = Fake()
        with self.assertRaises(ValueError), Stage("x", stream=stream, inline=False):
            raise ValueError("boom")
        self.assertTrue(stream.getvalue().endswith("   FAILED\n"))


class TestBar(unittest.TestCase):
    def test_it_draws_only_to_a_terminal(self):
        """A `\\r` bar in a redirected log is thousands of lines of noise, and piped output
        is what CI consumes."""
        piped = Fake(tty=False)
        bar = Bar(10, stream=piped, min_interval_s=0.0)
        for i in range(10):
            bar.update(i + 1)
        bar.close()
        self.assertEqual(piped.getvalue(), "")

    def test_it_draws_when_it_is_a_terminal(self):
        stream = Fake(tty=True)
        bar = Bar(10, "climbing", stream=stream, min_interval_s=0.0)
        bar.update(5)
        text = stream.getvalue()
        self.assertIn("climbing", text)
        self.assertIn("50%", text)
        self.assertIn("#", text)
        self.assertIn(".", text)
        self.assertTrue(text.startswith("\r"))

    def test_redraw_is_rate_limited(self):
        """The property the simulation loop depends on: 24 000 `update` calls must not be
        24 000 writes, or the bar costs more than the physics."""
        stream = Fake(tty=True)
        bar = Bar(24000, stream=stream, min_interval_s=60.0)
        for i in range(24000):
            bar.update(i + 1)
        self.assertLessEqual(stream.getvalue().count("\r"), 2)

    def test_the_fill_tracks_the_fraction(self):
        stream = Fake(tty=True)
        bar = Bar(4, stream=stream, width=8, min_interval_s=0.0)
        bar.update(2)
        self.assertIn("[####....]", stream.getvalue())

    def test_the_eta_is_withheld_until_there_is_something_to_extrapolate(self):
        """Measured on a real run, the first two redraws claimed 5m32s and then 32m57s on a
        job that took 3.6 s -- MuJoCo's first steps carry model compilation. A wildly wrong
        ETA in the first half second is the number someone reads before walking away."""
        stream = Fake(tty=True)
        bar = Bar(1000, stream=stream, min_interval_s=0.0, eta_after_s=3600.0)
        bar.update(1)
        self.assertIn("-- left", stream.getvalue())

    def test_the_eta_appears_once_the_run_has_settled(self):
        stream = Fake(tty=True)
        bar = Bar(1000, stream=stream, min_interval_s=0.0, eta_after_s=0.0)
        bar.update(500)
        self.assertNotIn("-- left", stream.getvalue())
        self.assertIn("left", stream.getvalue())

    def test_it_never_overflows_its_own_width(self):
        """A total that turns out to be wrong must not print a longer bar than it reserved —
        which is exactly what a segmented run does, since its timestep is tightened after the
        total was computed."""
        stream = Fake(tty=True)
        bar = Bar(10, stream=stream, width=8, min_interval_s=0.0)
        bar.update(999)
        self.assertIn("[########]", stream.getvalue())
        self.assertIn("100%", stream.getvalue())

    def test_close_leaves_the_line_clean(self):
        stream = Fake(tty=True)
        bar = Bar(4, stream=stream, min_interval_s=0.0)
        bar.update(2)
        bar.close()
        self.assertTrue(stream.getvalue().endswith("\r"))

    def test_close_is_idempotent(self):
        stream = Fake(tty=True)
        bar = Bar(4, stream=stream, min_interval_s=0.0)
        bar.update(1)
        bar.close()
        before = stream.getvalue()
        bar.close()
        self.assertEqual(stream.getvalue(), before)

    def test_a_zero_length_job_is_complete_and_not_a_division_by_zero(self):
        stream = Fake(tty=True)
        bar = Bar(0, stream=stream, min_interval_s=0.0)
        bar.update(0)
        self.assertIn("100%", stream.getvalue())

    def test_a_negative_total_is_refused(self):
        with self.assertRaises(ValueError):
            Bar(-1)

    def test_advance_accumulates(self):
        bar = Bar(10, stream=Fake(tty=False))
        bar.advance()
        bar.advance(3)
        self.assertEqual(bar.n, 4)

    def test_it_closes_itself_as_a_context_manager(self):
        stream = Fake(tty=True)
        with Bar(4, stream=stream, min_interval_s=0.0) as bar:
            bar.update(2)
        self.assertTrue(stream.getvalue().endswith("\r"))


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
