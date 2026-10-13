#!/usr/bin/env python3
"""Build the portable step-by-step setup guide from its Markdown source.

Supports the deliberately small Markdown subset used by this guide. No package,
network, or browser dependencies. --check catches stale committed HTML.
"""

from __future__ import annotations

import argparse
import base64
import html
import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "docs/engineer-setup-guide.md"
TEMPLATE = ROOT / "tools/docs/engineer-guide-template.html"
OUTPUT = ROOT / "docs/engineer-setup.html"
# Steps shown only for one route; every other step applies to both.
LIVE_STEPS = ("05", "06", "07")
RECORDED_STEPS = ("08", "09", "10")


def inline(text: str) -> str:
    """Escape first; only permit known Markdown inline constructs."""
    tokens: list[str] = []

    def stash(value: str) -> str:
        tokens.append(value)
        return f"\x00{len(tokens) - 1}\x00"

    text = re.sub(r"`([^`]+)`", lambda m: stash(f"<code>{html.escape(m[1])}</code>"), text)

    def link(match: re.Match) -> str:
        label, url = match.groups()
        if re.match(r"^[a-z]+:", url) and not url.startswith(("https://", "http://")):
            raise ValueError(f"Unsupported link scheme: {url}")
        if not url.startswith(("https://", "http://", "#")):
            target = (SOURCE.parent / url.split("#")[0]).resolve()
            # The output itself is allowed on the first generation.
            if target != OUTPUT and not target.exists():
                raise ValueError(f"Missing local link target: {url}")
        return stash(f'<a href="{html.escape(url, quote=True)}">{html.escape(label)}</a>')

    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", link, text)
    text = html.escape(text)
    text = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", text)
    return re.sub(r"\x00(\d+)\x00", lambda m: tokens[int(m[1])], text)


def render_block(lines: list[str]) -> str:
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        if line.startswith("```"):
            language = line[3:].strip()
            body = []
            i += 1
            while i < len(lines) and not lines[i].startswith("```"):
                body.append(lines[i])
                i += 1
            if i == len(lines):
                raise ValueError("Unclosed code fence")
            command = html.escape("\n".join(body))
            location = {
                "bash host": "Ubuntu terminal · any folder",
                "bash clone": "Ubuntu terminal · folder to download into",
            }.get(language, "Ubuntu terminal · project folder")
            out.append(
                '<div class="command" data-kind="' + html.escape(language) + '">'
                '<div class="command-bar"><span>' + location + '</span>'
                '<button type="button" class="copy">Copy</button></div>'
                f'<pre><code>{command}</code></pre></div>'
            )
            i += 1
            continue
        image = re.fullmatch(r"!\[([^\]]*)\]\(([^)]+)\)", line)
        if image:
            label, url = image.groups()
            path = (SOURCE.parent / url).resolve()
            if SOURCE.parent not in path.parents:
                raise ValueError("Images must be local to docs")
            mime = {".jpg": "image/jpeg", ".png": "image/png"}[path.suffix]
            src = f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode()
            out.append(f'<figure><img src="{src}" alt="{html.escape(label)}">'
                       f'<figcaption>{html.escape(label)}</figcaption></figure>')
        elif line.startswith("> SCREENSHOT:"):
            text = line.removeprefix("> SCREENSHOT:").strip()
            out.append('<figure class="placeholder"><div aria-hidden="true">▧</div>'
                       f'<figcaption><strong>Screenshot placeholder</strong><p>{inline(text)}</p>'
                       '</figcaption></figure>')
        elif line.startswith("### "):
            out.append(f"<h3>{inline(line[4:])}</h3>")
        elif line.startswith("| "):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                row = [cell.strip() for cell in lines[i].strip().strip("|").split("|")]
                if not all(re.fullmatch(r":?-+:?", cell) for cell in row):
                    rows.append(row)
                i += 1
            out.append('<div class="table-scroll" tabindex="0" aria-label="Scrollable reference table"><table><thead><tr>'
                       + "".join(f"<th scope=\"col\">{inline(c)}</th>" for c in rows[0])
                       + "</tr></thead><tbody>"
                       + "".join("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in row) + "</tr>"
                                 for row in rows[1:]) + "</tbody></table></div>")
            continue
        elif re.match(r"^(\d+\. |\- )", line):
            ordered = bool(re.match(r"^\d+\.", line))
            tag = "ol" if ordered else "ul"
            pattern = r"^\d+\. " if ordered else r"^- "
            items = []
            while i < len(lines) and re.match(pattern, lines[i]):
                items.append(re.sub(pattern, "", lines[i]))
                i += 1
            out.append(f"<{tag}>" + "".join(f"<li>{inline(item)}</li>" for item in items) + f"</{tag}>")
            continue
        else:
            out.append(f"<p>{inline(line)}</p>")
        i += 1
    return "\n".join(out)


def build() -> str:
    source = SOURCE.read_text(encoding="utf-8")
    chunks = re.split(r"^## ", source, flags=re.M)[1:]
    sections, links = [], []
    for chunk in chunks:
        heading, body = chunk.split("\n", 1)
        number = heading[:2]
        if not number.isdecimal():
            raise ValueError(f"Section must have a stable two-digit number: {heading}")
        title = heading[5:]
        route = "recorded" if number in RECORDED_STEPS else "live" if number in LIVE_STEPS else "both"
        sections.append(f'<section id="step-{number}" data-route="{route}">'
                        f'<div class="section-head"><span class="step-number">{number}</span>'
                        f'<h2>{html.escape(title)}</h2><span class="step-of"></span></div>'
                        f'{render_block(body.splitlines())}'
                        '<div class="step-foot"><button type="button" class="back">← Back</button>'
                        f'<label><input type="checkbox" data-step="{number}"> I finished this step</label>'
                        '<button type="button" class="next primary">Next step →</button></div></section>')
        links.append(f'<a href="#step-{number}" data-route="{route}"><span>{number}</span>{html.escape(title)}</a>')
    template = TEMPLATE.read_text(encoding="utf-8")
    return (template.replace("<!-- GUIDE_SECTIONS -->", "\n".join(sections))
            .replace("<!-- GUIDE_NAV -->", "\n".join(links))
            .replace("/* GUIDE_MARKDOWN */", json.dumps(source).replace("<", "\\u003c")))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if committed HTML differs")
    args = parser.parse_args()
    result = build()
    if args.check:
        if not OUTPUT.is_file() or OUTPUT.read_text(encoding="utf-8") != result:
            print("STALE: run python3 tools/docs/build_engineer_guide.py")
            return 1
        print("PASS: portable guide matches Markdown, template and images")
        return 0
    OUTPUT.write_text(result, encoding="utf-8")
    print(f"Built {OUTPUT.relative_to(ROOT)} ({len(result.encode()):,} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
