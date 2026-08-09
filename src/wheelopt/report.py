"""Self-contained HTML reports for one design, or a handful compared.

Why HTML rather than another PDF
--------------------------------
:mod:`wheelopt.viz` already writes vector PDFs, and they are the right thing for a figure
that goes in a paper. This is for the other job — turning a knob and looking at what moved —
which wants an animation next to the curves, several designs on shared axes, and one file to
open. The page embeds everything (SVG inline, GIF as a data URI) so it survives being copied
to another machine with no `data/` directory alongside it.

Provenance is not decoration here
---------------------------------
Every panel carries the tier that produced it and whether that tier **screens or decides**.
That is the whole reason this module has a :class:`Panel` type instead of string
concatenation: this project's recurring failure is a number that reads as innocuous and means
something else, and a plot is far better at hiding that than a table is. A reader who looks at
a contact patch from the plane-strain tier and does not know it cannot see lateral buckling
has been misled by the page. :attr:`Panel.caution` is for the stronger case — a panel whose
numbers are known to be untrustworthy right now — and it renders loudly rather than as a
footnote.
"""

from __future__ import annotations

import base64
import html
import io
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

__all__ = ["Panel", "figure_svg", "gif_data_uri", "new_figure", "table", "write_report"]

#: Point size for figure text. Larger than matplotlib's 10 on purpose: an inline SVG is
#: scaled down to the column width, and a 9-inch figure in a 34-rem column loses about half
#: its linear size — so a default-sized label lands at roughly 5 pt and is unreadable. Set the
#: type big and let the browser shrink it.
_FONT_PT = 13


@dataclass(frozen=True, slots=True)
class Panel:
    """One section of the report.

    Attributes:
        title: heading.
        body: raw HTML — an inline ``<svg>``, a table, or both.
        provenance: which tier produced this and what it is good for. Shown under the
            heading in every panel; leave empty only for panels that are pure geometry.
        caution: a known reason not to trust these numbers. Rendered as a banner, not a
            footnote. ``None`` when there is nothing specific to say.
    """

    title: str
    body: str
    provenance: str = ""
    caution: str | None = None


def new_figure(plt: Any, *, ncols: int = 1, width: float = 9.0, height: float = 3.2):
    """A figure and its axes, styled to survive being scaled into the page column.

    Returns ``(fig, axes)`` with ``axes`` always a list, so a caller does not have to branch
    on whether matplotlib handed back a bare Axes or an array of them.
    """
    fig, axes = plt.subplots(1, ncols, figsize=(width, height), dpi=100)
    axes = [axes] if ncols == 1 else list(axes)
    for ax in axes:
        ax.grid(True, linewidth=0.7, alpha=0.45)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(labelsize=_FONT_PT - 2)
        ax.xaxis.label.set_size(_FONT_PT)
        ax.yaxis.label.set_size(_FONT_PT)
    return fig, axes


def figure_svg(fig: Any) -> str:
    """Render a matplotlib figure to an inline ``<svg>`` element.

    Inline rather than a base64 ``<img>``: it stays vector, it scales with the page, and the
    text in it is selectable. The XML declaration and DOCTYPE matplotlib emits are stripped,
    because an XML prolog part-way down an HTML document is invalid and browsers recover from
    it in different ways.
    """
    # Transparent, so the page's own background shows through. This matters for the dark
    # theme: the CSS inverts SVG to keep matplotlib's black text legible, and an opaque white
    # figure patch would invert to a black slab sitting on a nearly-black page.
    fig.patch.set_alpha(0.0)
    for ax in fig.get_axes():
        ax.patch.set_alpha(0.0)
    buffer = io.StringIO()
    fig.savefig(buffer, format="svg", bbox_inches="tight", transparent=True)
    markup = buffer.getvalue()
    return markup[markup.index("<svg") :]


def gif_data_uri(path: Path) -> str:
    """Embed a GIF as a ``data:`` URI so the page is one file."""
    encoded = base64.b64encode(Path(path).read_bytes()).decode("ascii")
    return f"data:image/gif;base64,{encoded}"


def table(rows: list[tuple[str, ...]], *, header: tuple[str, ...] | None = None,
          classes: str = "") -> str:
    """A small HTML table. Every cell is escaped — values come from solver output."""
    parts = [f'<table class="{classes}">']
    if header is not None:
        cells = "".join(f"<th>{html.escape(str(c))}</th>" for c in header)
        parts.append(f"<thead><tr>{cells}</tr></thead>")
    parts.append("<tbody>")
    for row in rows:
        cells = "".join(f"<td>{html.escape(str(c))}</td>" for c in row)
        parts.append(f"<tr>{cells}</tr>")
    parts.append("</tbody></table>")
    return "".join(parts)


#: Deliberately plain. This is an instrument, not a landing page: the reader is comparing
#: curves, and anything with an opinion competes with them. One accent, used only for links
#: and the caution rule. Dark mode is a media query rather than a toggle because the page has
#: no scripts at all — it opens from a file:// URL and must work with everything disabled.
_CSS = """
:root {
  --ink: #16191c; --muted: #5b6570; --rule: #d9dee3; --bg: #ffffff;
  --panel: #fbfcfd; --accent: #1f6feb; --warn-bg: #fff4e5; --warn-ink: #7a4a00;
  --warn-rule: #e0a95f;
}
@media (prefers-color-scheme: dark) {
  :root {
    --ink: #e6e9ec; --muted: #9aa4ae; --rule: #2c3238; --bg: #14171a;
    --panel: #191d21; --accent: #6ea8ff; --warn-bg: #33270f; --warn-ink: #f0c986;
    --warn-rule: #7a5a1f;
  }
  svg { filter: invert(0.92) hue-rotate(180deg); }
}
* { box-sizing: border-box; }
body {
  margin: 0; padding: 2.5rem 1.5rem 5rem; background: var(--bg); color: var(--ink);
  font: 15px/1.6 -apple-system, BlinkMacSystemFont, "Segoe UI", system-ui, sans-serif;
}
main { max-width: 60rem; margin: 0 auto; }
h1 { font-size: 1.6rem; margin: 0 0 .25rem; letter-spacing: -0.01em; }
h2 { font-size: 1.1rem; margin: 0 0 .2rem; letter-spacing: -0.005em; }
.sub { color: var(--muted); margin: 0 0 2.5rem; font-size: .9rem; }
section {
  background: var(--panel); border: 1px solid var(--rule); border-radius: 8px;
  padding: 1.25rem 1.5rem 1.5rem; margin-bottom: 1.5rem;
}
.prov {
  color: var(--muted); font-size: .8rem; margin: 0 0 1rem;
  border-left: 2px solid var(--rule); padding-left: .6rem;
}
.caution {
  background: var(--warn-bg); color: var(--warn-ink);
  border: 1px solid var(--warn-rule); border-left-width: 4px;
  border-radius: 4px; padding: .6rem .8rem; margin: 0 0 1rem; font-size: .87rem;
}
.caution strong { letter-spacing: .02em; text-transform: uppercase; font-size: .78rem; }
table { border-collapse: collapse; width: 100%; font-size: .88rem; margin: .25rem 0 .5rem; }
th, td {
  text-align: left; padding: .35rem .9rem .35rem 0; border-bottom: 1px solid var(--rule);
  /* Let the row overflow its container and scroll, rather than wrapping every cell into a
     column one word wide. The .scroll wrapper is what handles the overflow. */
  white-space: nowrap;
}
th { color: var(--muted); font-weight: 600; font-size: .78rem; text-transform: uppercase;
     letter-spacing: .04em; }
td { font-variant-numeric: tabular-nums; }
table.kv td:first-child { color: var(--muted); width: 18rem; }
.scroll { overflow-x: auto; }
svg, img { max-width: 100%; height: auto; display: block; }
img.frames { margin-top: .75rem; border: 1px solid var(--rule); border-radius: 4px; }
code { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: .86em; }
footer { color: var(--muted); font-size: .8rem; margin-top: 2rem; text-align: center; }
"""


def write_report(
    path: Path,
    *,
    title: str,
    subtitle: str,
    panels: list[Panel],
    command: str = "",
) -> Path:
    """Write the panels to a single self-contained HTML file. Returns the path."""
    out = [
        "<!doctype html>",
        '<html lang="en"><head><meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        f"<title>{html.escape(title)}</title>",
        f"<style>{_CSS}</style></head><body><main>",
        f"<h1>{html.escape(title)}</h1>",
        f'<p class="sub">{html.escape(subtitle)}</p>',
    ]
    for panel in panels:
        out.append("<section>")
        out.append(f"<h2>{html.escape(panel.title)}</h2>")
        if panel.provenance:
            out.append(f'<p class="prov">{html.escape(panel.provenance)}</p>')
        if panel.caution:
            out.append(
                f'<p class="caution"><strong>Caution</strong><br>'
                f"{html.escape(panel.caution)}</p>"
            )
        out.append(f'<div class="scroll">{panel.body}</div>')
        out.append("</section>")

    stamp = datetime.now(UTC).strftime("%Y-%m-%d %H:%M UTC")
    out.append(f'<footer>generated {stamp}')
    if command:
        out.append(f"<br><code>{html.escape(command)}</code>")
    out.append("</footer></main></body></html>")

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(out), encoding="utf-8")
    return path
