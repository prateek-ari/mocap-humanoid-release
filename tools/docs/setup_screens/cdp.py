"""Tiny Chrome DevTools driver for documentation screenshots (headless Chrome)."""
from __future__ import annotations

import asyncio
import base64
import itertools
import json
import subprocess
import time
import urllib.request
from pathlib import Path



class Page:
    def __init__(self, ws):
        self.ws, self.ids = ws, itertools.count(1)

    async def call(self, method, **params):
        i = next(self.ids)
        await self.ws.send(json.dumps({"id": i, "method": method, "params": params}))
        while True:
            msg = json.loads(await self.ws.recv())
            if msg.get("id") == i:
                if "error" in msg:
                    raise RuntimeError(f"{method}: {msg['error']}")
                return msg.get("result", {})

    async def js(self, expression):
        r = await self.call("Runtime.evaluate", expression=expression, awaitPromise=True,
                            returnByValue=True)
        if "exceptionDetails" in r:
            raise RuntimeError(r["exceptionDetails"])
        return r.get("result", {}).get("value")

    async def wait(self, expression, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if await self.js(f"Boolean({expression})"):
                return
            await asyncio.sleep(0.1)
        raise TimeoutError(expression)

    async def wait_text(self, text, timeout=15.0):
        await self.wait(f"document.body.innerText.includes({json.dumps(text)})", timeout)

    async def click_text(self, text, selector="button"):
        ok = await self.js(f"""(() => {{
            const el = [...document.querySelectorAll({json.dumps(selector)})]
              .find(e => e.innerText.trim().includes({json.dumps(text)}) && !e.disabled);
            if (!el) return false; el.scrollIntoView({{block: 'center'}}); el.click(); return true }})()""")
        if not ok:
            raise RuntimeError(f"no enabled {selector} with text {text!r}")

    async def goto(self, url):
        await self.call("Page.navigate", url=url)
        await asyncio.sleep(1.0)

    async def size(self, width, height):
        await self.call("Emulation.setDeviceMetricsOverride", width=width, height=height,
                        deviceScaleFactor=1, mobile=False)

    async def shot(self, path, selector=None, pad=8):
        params = {"format": "jpeg", "quality": 88} if str(path).endswith(".jpg") else {"format": "png"}
        if selector:
            box = await self.js(f"""(() => {{ const r = document.querySelector({json.dumps(selector)})
                .getBoundingClientRect(); return [r.x, r.y, r.width, r.height] }})()""")
            params["clip"] = {"x": max(box[0] - pad, 0), "y": max(box[1] - pad, 0),
                              "width": box[2] + 2 * pad, "height": box[3] + 2 * pad, "scale": 1}
        data = await self.call("Page.captureScreenshot", **params)
        Path(path).write_bytes(base64.b64decode(data["data"]))
        print("saved", path)


def launch(profile: Path, port=9333):
    proc = subprocess.Popen(["google-chrome", "--headless=new", f"--remote-debugging-port={port}",
                             f"--user-data-dir={profile}", "--no-first-run", "--hide-scrollbars",
                             "--window-size=1440,900", "about:blank"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(100):
        try:
            targets = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/json"))
            page = next(t for t in targets if t["type"] == "page")
            return proc, page["webSocketDebuggerUrl"]
        except Exception:
            time.sleep(0.1)
    raise RuntimeError("chrome did not start")
