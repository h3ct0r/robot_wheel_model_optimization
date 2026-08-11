"""Stage announcements and a progress bar for the long-running CLIs. No dependencies.

A rover run with compliant wheels is three CalculiX solves, a MuJoCo integration of tens of
thousands of steps and a render, and uncached that is minutes of complete silence. Silence is
not just unpleasant — it is indistinguishable from a hang, which is how the ``fit_range.py``
episode of 2026-08-09 came to run for over an hour before anyone asked whether it was working.

Two things, deliberately small:

:class:`Stage` — a context manager that prints what is starting, and on exit how long it took
plus whatever the block wants to add ("cached", "492 elements"). The timing is the point: it is
what tells you the difference between a cache hit and a cold solve without reading the cache.

:class:`Bar` — a single-line progress bar on **stderr**, so that piping stdout to a file gets
clean output and the bar still reaches a terminal. Off automatically when stderr is not a TTY:
a `\\r` bar in a log file is thousands of lines of noise, and CI logs are the main consumer of
piped output here.
"""

from __future__ import annotations

import sys
import time
from types import TracebackType
from typing import Self

__all__ = ["Bar", "Stage", "format_duration"]


def format_duration(seconds: float) -> str:
    """``0.4s``, ``12.3s``, ``4m12s``. Short enough to sit at the end of a line."""
    if seconds < 60.0:
        return f"{seconds:.1f}s"
    minutes, rest = divmod(round(seconds), 60)
    if minutes < 60:
        return f"{minutes}m{rest:02d}s"
    hours, minutes = divmod(minutes, 60)
    return f"{hours}h{minutes:02d}m"


class Stage:
    """Announce a step, then report how long it took.

        with Stage("whole-wheel FEA") as stage:
            ...
            stage.note("cached")

    Prints ``-> whole-wheel FEA ... 0.2s (cached)``. On an exception the line is closed with
    ``FAILED`` rather than left dangling, because a half-written progress line above a
    traceback reads as though the traceback came from the *next* stage.

    **Set ``inline=False`` for any stage that contains a** :class:`Bar`. The default holds the
    line open across the block, and a bar drawn on stderr starts every redraw with ``\\r`` —
    which returns to column zero of the *terminal*, not of the stream, and eats the ``->``
    prefix that stdout left there. With ``inline=False`` the label gets its own line, the bar
    owns the next one, and the timing lands underneath.
    """

    def __init__(self, label: str, *, stream=None, enabled: bool = True,
                 inline: bool = True) -> None:
        self.label = label
        self.stream = stream if stream is not None else sys.stdout
        self.enabled = enabled
        self.inline = inline
        self.notes: list[str] = []
        self.seconds = 0.0
        self._start = 0.0

    def note(self, text: str) -> None:
        """Add a parenthetical to the completion line. Called from inside the block."""
        if text:
            self.notes.append(text)

    def __enter__(self) -> Self:
        self._start = time.perf_counter()
        if self.enabled:
            self.stream.write(f"-> {self.label} ... " if self.inline
                              else f"-> {self.label}\n")
            self.stream.flush()
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None,
                 tb: TracebackType | None) -> bool:
        self.seconds = time.perf_counter() - self._start
        if self.enabled:
            head = "" if self.inline else "   "
            if exc_type is not None:
                self.stream.write(f"{head}FAILED\n")
            else:
                tail = f"  ({', '.join(self.notes)})" if self.notes else ""
                self.stream.write(f"{head}{format_duration(self.seconds)}{tail}\n")
            self.stream.flush()
        return False


class Bar:
    """A one-line progress bar on stderr, with an ETA. Silent when stderr is not a TTY.

    ``update(i)`` is safe to call on every simulation step: it redraws at most every
    ``min_interval_s``, so driving it from a 24 000-step MuJoCo loop costs a clock read per
    step and nothing else. That matters — a bar that halves the speed of the thing it is
    measuring is worse than no bar.
    """

    def __init__(self, total: int, label: str = "", *, width: int = 28,
                 stream=None, min_interval_s: float = 0.1, eta_after_s: float = 1.0,
                 enabled: bool | None = None) -> None:
        if total < 0:
            raise ValueError("total must be non-negative")
        self.total = total
        self.label = label
        self.width = width
        self.stream = stream if stream is not None else sys.stderr
        self.min_interval_s = min_interval_s
        self.eta_after_s = eta_after_s
        self.enabled = (getattr(self.stream, "isatty", lambda: False)()
                        if enabled is None else enabled)
        self.n = 0
        self._start = time.perf_counter()
        self._last_draw = 0.0
        self._drawn = False

    def update(self, n: int) -> None:
        """Set the count to ``n`` and redraw if enough time has passed."""
        self.n = n
        now = time.perf_counter()
        if self.enabled and now - self._last_draw >= self.min_interval_s:
            self._last_draw = now
            self._draw(now)

    def advance(self, by: int = 1) -> None:
        self.update(self.n + by)

    def _draw(self, now: float) -> None:
        fraction = 1.0 if self.total == 0 else min(1.0, max(0.0, self.n / self.total))
        filled = round(fraction * self.width)
        elapsed = now - self._start
        # ETA from the average rate so far. Deliberately not a windowed estimate: the phases
        # of these runs have genuinely different costs and a jumpy ETA invites more attention
        # than it deserves.
        #
        # Withheld until there is something to extrapolate from. Measured on a real run, the
        # first two redraws claimed "5m32s left" and then "32m57s left" on a job that took
        # 3.6 s, because MuJoCo's first steps include model compilation and contact setup.
        # A wildly wrong ETA in the first half second is worse than no ETA: it is the number
        # someone reads before deciding whether to walk away.
        settled = elapsed >= self.eta_after_s and fraction > 0.0
        eta = (format_duration(elapsed * (1.0 - fraction) / fraction) if settled else "--")
        head = f"   {self.label} " if self.label else "   "
        self.stream.write(
            f"\r{head}[{'#' * filled}{'.' * (self.width - filled)}] "
            f"{fraction:4.0%}  {format_duration(elapsed)} elapsed, {eta} left "
        )
        self.stream.flush()
        self._drawn = True

    def close(self, note: str = "") -> None:
        """Erase the bar and leave the line clean. Safe to call twice."""
        if self.enabled and self._drawn:
            self.stream.write("\r" + " " * (self.width + 56) + "\r")
            if note:
                self.stream.write(f"   {note}\n")
            self.stream.flush()
        self._drawn = False

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.close()
        return False
