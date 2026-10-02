"""Offscreen Swift: a headless Chromium renders the scene, CDP grabs each frame.

Swift's own screenshot/recording buttons download files through the browser, so
nothing tells Python when a file is done. Here Python owns the clock instead:
step the scene, wait for the browser to paint, grab the frame over CDP.
"""
import base64
import json
import shutil
import socket
import tempfile
import subprocess
import time
import urllib.request
import webbrowser
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import swift
import swift.SwiftRoute
import websockets.legacy.server
from websockets.sync.client import connect

# websockets >= 14 dropped the handler signature swift uses (same patch as
# hri_manager/monitor_dashboards/swift_view.py).
swift.SwiftRoute.websockets = SimpleNamespace(serve=websockets.legacy.server.serve)

# Snap Chromium can only write inside its own snap directory.
PROFILES = Path.home() / "snap/chromium/common"
HIDE_TOOLBAR = """document.head.insertAdjacentHTML('beforeend',
  '<style>[class^="SwiftInfo-module_info"]{display:none !important}</style>')"""
# Swift draws a fixed world-axes helper that the page gives no handle to. It is
# the only line geometry in the scene, so skip GL line draws. ponytail: also
# hides any line shape added later; filter by vertex count if one is ever needed.
HIDE_AXES = """for (const C of [WebGLRenderingContext, WebGL2RenderingContext]) {
  const draw = C.prototype.drawArrays;
  C.prototype.drawArrays = function (mode, ...rest) {
    if (mode !== this.LINES) return draw.call(this, mode, ...rest);
  };
}"""
PAINTED = "new Promise(r => requestAnimationFrame(r))"
# Headless Chromium defaults to software GL; --enable-gpu gets the NVIDIA card.
GPU_FLAGS = ["--enable-gpu"]
CPU_FLAGS = ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"]


class SwiftCamera:
    """`with SwiftCamera() as cam: cam.env.add(...); cam.env.step(0); cam.frame()`"""

    def __init__(self, width=1280, height=720, visible=False, browser="chromium", gpu=True):
        self.size, self.visible, self.browser, self.gpu = (width, height), visible, browser, gpu
        self.chrome = self.ws = None
        self._id = 0

    def __enter__(self):
        # Own port and profile per run: a closing Chromium lingers a few seconds.
        with socket.socket() as s:
            s.bind(("localhost", 0))
            self.port = s.getsockname()[1]
        PROFILES.mkdir(parents=True, exist_ok=True)
        self.profile = tempfile.mkdtemp(prefix="swift_render_", dir=PROFILES)
        opener = webbrowser.open_new_tab
        webbrowser.open_new_tab = self._open  # swift opens its page through this
        try:
            self.env = swift.Swift()
            self.env.launch(realtime=False)
        finally:
            webbrowser.open_new_tab = opener
        self.ws = connect(self._page_ws(), max_size=None)
        self._cdp("Emulation.setDeviceMetricsOverride", width=self.size[0],
                  height=self.size[1], deviceScaleFactor=1, mobile=False)
        self._cdp("Runtime.evaluate", expression=HIDE_TOOLBAR)
        self._cdp("Runtime.evaluate", expression=HIDE_AXES)
        return self

    def __exit__(self, *exc):
        # Browser.close, not terminate(): AppArmor denies signals to snap Chromium.
        if self.ws:
            try:
                self._cdp("Browser.close")
            except Exception:  # noqa: BLE001 -- the socket drops as it closes
                pass
            self.ws.close()
        shutil.rmtree(self.profile, ignore_errors=True)
        # Swift leaks its server threads; the calling script exits right after.

    def _open(self, url, *args, **kwargs):
        flags = [f"--remote-debugging-port={self.port}", f"--user-data-dir={self.profile}",
                 f"--window-size={self.size[0]},{self.size[1]}", "--no-first-run",
                 *(GPU_FLAGS if self.gpu else CPU_FLAGS)]
        if not self.visible:
            flags.append("--headless=new")
        self.chrome = subprocess.Popen([self.browser, *flags, url],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True

    def _page_ws(self, timeout=30):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(f"http://localhost:{self.port}/json") as r:
                    pages = [p for p in json.load(r) if p["type"] == "page"]
                if pages:
                    return pages[0]["webSocketDebuggerUrl"]
            except OSError:
                pass
            time.sleep(0.2)
        raise RuntimeError("Chromium never exposed a page over CDP")

    def _cdp(self, method, **params):
        self._id += 1
        self.ws.send(json.dumps({"id": self._id, "method": method, "params": params}))
        while True:
            reply = json.loads(self.ws.recv(timeout=30))
            if reply.get("id") == self._id:
                if "error" in reply:
                    raise RuntimeError(f"{method}: {reply['error']}")
                return reply["result"]

    def look(self, eye):
        """Camera position in the scene frame; it always looks at the origin.

        Swift 1.1 drops set_camera_pose's look_at (the orbit target stays at
        the origin), so to aim elsewhere, shift the scene instead.
        """
        self.env.set_camera_pose(list(eye), [0.0, 0.0, 0.0])

    def frame(self) -> np.ndarray:
        """BGR image of what the browser shows after the last env.step()."""
        # env.step() already waited for the browser to apply the poses; one
        # animation frame later they are drawn. JPEG: PNG encoding costs more.
        self._cdp("Runtime.evaluate", expression=PAINTED, awaitPromise=True)
        jpg = self._cdp("Page.captureScreenshot", format="jpeg", quality=92)["data"]
        return cv2.imdecode(np.frombuffer(base64.b64decode(jpg), np.uint8), cv2.IMREAD_COLOR)
