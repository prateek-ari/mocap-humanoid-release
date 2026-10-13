#!/usr/bin/env python3
"""Recapture the Setup-tab screenshots used by docs/engineer-setup-guide.md.

Run from the checkout after changing the Setup tab (needs google-chrome):

    .venv/bin/python tools/docs/setup_screens/capture.py

It serves the console with fake tools (server.py) in a temporary home, drives
headless Chrome through both routes and writes docs/engineer-guide-assets/setup-*.png.
Then rebuild the portable guide: python3 tools/docs/build_engineer_guide.py
"""
import argparse
import asyncio
import json
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import websockets

sys.path.insert(0, str(Path(__file__).parent))
from cdp import Page, launch  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
W, H = 1440, 900

FILL = """((sel, value) => { const el = document.querySelector(sel);
  const proto = el.tagName === 'SELECT' ? HTMLSelectElement.prototype : HTMLInputElement.prototype;
  Object.getOwnPropertyDescriptor(proto, 'value').set.call(el, value);
  el.dispatchEvent(new Event(el.tagName === 'SELECT' ? 'change' : 'input', {bubbles: true}));
  el.dispatchEvent(new Event('change', {bubbles: true})); return true })"""


async def capture(out: Path, port: int, scratch: Path):
    BASE = f"http://127.0.0.1:{port}/"
    proc, ws_url = launch(scratch / "chrome-profile")
    try:
        async with websockets.connect(ws_url, max_size=2**27) as ws:
            p = Page(ws)
            await p.call("Page.enable"); await p.call("Runtime.enable")
            await p.size(W, H)
            await p.goto(BASE)
            await p.js("localStorage.setItem('mocap-g1-theme','light')")

            async def fill(sel, value):
                try:
                    await p.wait(f"document.querySelector({json.dumps(sel)})")
                except TimeoutError:
                    await p.shot(scratch.parent / "setup-screens-failure.png")
                    raise
                await p.js(f"{FILL}({json.dumps(sel)}, {json.dumps(value)})")

            async def step(name):
                await p.goto(BASE + f"#setup/{name}")
                await p.js("location.reload()")
                await asyncio.sleep(1.2)
                await p.wait(f"document.querySelector('section.setup-panel[data-step=\"{name}\"]')")

            async def verdict(state, timeout=30):
                await p.wait(f"document.querySelector('.verdict-plate.is-{state}')", timeout)
                await asyncio.sleep(0.6)

            async def shot(name):
                # Grow the viewport to the whole step panel so results below the fold are included.
                await p.js("document.activeElement && document.activeElement.blur()")
                bottom = await p.js("Math.ceil(document.querySelector('section.setup-panel').getBoundingClientRect().bottom + window.scrollY)")
                await p.size(W, max(H, bottom + 24))
                await asyncio.sleep(0.5)
                await p.js("document.querySelectorAll('pre.job-log').forEach(e => e.scrollTop = e.scrollHeight)")
                await asyncio.sleep(0.2)
                await p.shot(out / f"{name}.png")
                await p.size(W, H)

            # 03 Session (live)
            await step("session")
            await p.js("document.querySelector('input[name=setup-robot][value=unitree_g1]').click()")
            await fill("[data-field=human_height_m] input", "1.72")
            await fill("[data-field=windows_xsens_ip] input", "192.168.1.246")
            await fill("[data-field=linux_xsens_ip] select", "192.168.1.20")
            await p.click_text("Save session")
            await p.wait_text("session.env")
            await asyncio.sleep(0.5)
            await p.click_text("Run system check")
            await verdict("passed")
            await shot("setup-03-session")

            # 05 Windows values
            await step("windows")
            await shot("setup-05-windows")

            # 06 Stream check
            await step("stream")
            await p.click_text("Run 30-second check")
            await verdict("passed", 60)
            await shot("setup-06-stream")

            # 07 Live simulation
            await step("live")
            await p.js("""[...document.querySelectorAll('label')].find(l => l.innerText.includes('Auto-calibrate'))
                          .querySelector('input').click()""")
            await p.click_text("Start live simulation")
            await p.wait_text("tracking ON", 20)
            await asyncio.sleep(1.5)
            await shot("setup-07-live")
            await p.click_text("Stop")
            await verdict("stopped", 20)

            # Recorded route: switch the saved session to recorded.
            await step("session")
            await p.js("document.querySelector('input[name=setup-route][value=recorded]').click()")
            await asyncio.sleep(0.4)
            await p.click_text("Save session")
            await asyncio.sleep(1.0)
            if await p.js("document.body.innerText.includes('Replace it')"):
                await p.click_text("Replace it")
            await asyncio.sleep(1.0)
            await shot("setup-03-session-recorded")

            await step("bvh")
            await p.click_text("Check files")
            await verdict("passed")
            await shot("setup-08-bvh")

            await step("retarget")
            await p.click_text("Make preview")
            await verdict("passed")
            await shot("setup-09-retarget")

            await step("render")
            await p.click_text("Record MP4")
            await verdict("passed")
            await p.wait("document.querySelector('video')", 10)
            await p.js("""(async () => { const v = document.querySelector('video'); v.muted = true;
                await new Promise(r => v.readyState >= 2 ? r() : v.addEventListener('loadeddata', r, {once: true}));
                v.currentTime = 4; await new Promise(r => v.addEventListener('seeked', r, {once: true})) })()""")
            await asyncio.sleep(0.5)
            await shot("setup-10-render")

            await step("finish")
            await p.click_text("Save report")
            await verdict("passed")
            await shot("setup-11-finish")
    finally:
        proc.terminate()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=ROOT / "docs/engineer-guide-assets")
    parser.add_argument("--port", type=int, default=8811)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    scratch = Path(tempfile.mkdtemp(prefix="setup-screens-"))
    server = subprocess.Popen([sys.executable, str(Path(__file__).with_name("server.py")),
                               str(scratch / "work"), str(ROOT), str(args.port)])
    try:
        for _ in range(100):
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{args.port}/api/v1/setup", timeout=1)
                break
            except OSError:
                if server.poll() is not None:
                    raise SystemExit("the screenshot server stopped")
                time.sleep(0.1)
        asyncio.run(capture(args.out, args.port, scratch))
    finally:
        server.terminate()
        server.wait(timeout=10)
        shutil.rmtree(scratch, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
