"""The browser-facing half: a stdlib HTTP server over the control loop.

Deliberately no web framework. The robot PC's environment is the one that has
to keep working -- RLinf, franky, pyrealsense2 and a PREEMPT_RT kernel -- and
adding FastAPI plus its dependency tree to it to draw six buttons is a poor
trade. ``http.server`` plus ``cv2.imencode`` covers everything needed here:
MJPEG is a multipart response, and state is a JSON poll.

A browser UI rather than a native window, for three reasons this repo already
demonstrates: ``collect_episode.sh`` has to export ``DISPLAY`` and
``XAUTHORITY=/run/user/1000/gdm/Xauthority`` to get a cv2 window up at all;
``VideoPlayer`` in RLinf silently disables itself when ``DISPLAY`` is unset; and
the operator often wants the preview on a laptop next to the rig rather than on
the robot PC's monitor.
"""
from __future__ import annotations

import http.server
import json
import mimetypes
import secrets
import socketserver
import threading
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2

from gui.session import ControlLoop

STATIC_DIR = Path(__file__).parent / "static"

#: MJPEG boundary. Any token works as long as the header and parts agree.
BOUNDARY = "pnp7frame"


class GuiServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, loop: ControlLoop, token: str | None,
                 mock_backend=None):
        self.loop = loop
        self.token = token
        self.mock_backend = mock_backend
        super().__init__(addr, GuiHandler)


class GuiHandler(http.server.BaseHTTPRequestHandler):
    server: GuiServer
    protocol_version = "HTTP/1.1"

    # --- plumbing ---------------------------------------------------------
    def log_message(self, fmt: str, *args) -> None:
        # The default logs every MJPEG chunk request; drowns the real output.
        return

    def _authorized(self) -> bool:
        token = self.server.token
        if not token:
            return True
        header = self.headers.get("X-Auth-Token")
        if header and secrets.compare_digest(header, token):
            return True
        # Also accept it as a query parameter, so a plain <img src> stream and a
        # pasted URL both work without scripting the header in.
        query = urlparse(self.path).query
        for part in query.split("&"):
            if part.startswith("token="):
                return secrets.compare_digest(part[len("token="):], token)
        return False

    def _send_json(self, payload: Any, status: int = 200) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, body: bytes, content_type: str,
                    status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # --- routes -----------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler's spelling
        path = urlparse(self.path).path
        if not self._authorized():
            self._send_json({"error": "unauthorized"}, 401)
            return

        if path in ("/", "/index.html"):
            self._serve_static("index.html")
        elif path.startswith("/static/"):
            self._serve_static(path[len("/static/"):])
        elif path == "/api/state":
            self._send_json(self.server.loop.snapshot().to_json())
        elif path == "/api/configs":
            self._send_json({
                "configs": self.server.loop.backend.list_configs(),
                "mock": self.server.mock_backend is not None,
            })
        elif path.startswith("/stream/"):
            self._serve_mjpeg(path[len("/stream/"):])
        else:
            self._send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if not self._authorized():
            self._send_json({"error": "unauthorized"}, 401)
            return

        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw or b"{}")
        except json.JSONDecodeError:
            self._send_json({"error": "bad json"}, 400)
            return

        if path == "/api/command":
            self._handle_command(payload)
        elif path == "/api/mock/deadman":
            backend = self.server.mock_backend
            if backend is None:
                self._send_json({"error": "not running the mock backend"}, 400)
                return
            backend.deadman_held = bool(payload.get("held"))
            self._send_json({"deadman_held": backend.deadman_held})
        else:
            self._send_json({"error": "not found"}, 404)

    # --- handlers ---------------------------------------------------------
    def _handle_command(self, payload: dict[str, Any]) -> None:
        name = payload.get("name")
        if not name:
            self._send_json({"error": "missing command name"}, 400)
            return
        args = payload.get("args") or {}
        cmd = self.server.loop.submit(name, **args)
        # Opening a session builds the env and runs an alignment check; restoring
        # the arm is a real move. Both take seconds, so wait generously and let
        # the UI fall back to polling if it still has not landed.
        if not cmd.done.wait(timeout=float(payload.get("timeout", 60.0))):
            self._send_json({"accepted": True, "pending": True})
            return
        if cmd.error:
            self._send_json({"accepted": False, "error": cmd.error}, 409)
            return
        self._send_json({"accepted": True,
                         "state": self.server.loop.snapshot().to_json()})

    def _serve_static(self, name: str) -> None:
        # Refuse traversal outright rather than normalising it.
        target = (STATIC_DIR / name).resolve()
        if not str(target).startswith(str(STATIC_DIR.resolve())) or not target.is_file():
            self._send_json({"error": "not found"}, 404)
            return
        ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self._send_bytes(target.read_bytes(), ctype)

    def _serve_mjpeg(self, camera: str) -> None:
        backend = self.server.loop.backend
        if camera not in getattr(backend, "camera_names", []):
            self._send_json({"error": f"no camera {camera!r}"}, 404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        period = 1.0 / 15.0  # preview only; the recorded rate is the env's
        try:
            while True:
                t0 = time.perf_counter()
                # A backend that already holds JPEG bytes hands them straight
                # over; only raw-array backends pay for an encode here.
                chunk = backend.latest_jpeg(camera)
                if chunk is None:
                    frame = backend.latest_frame(camera)
                    if frame is not None:
                        ok, buf = cv2.imencode(
                            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                        chunk = buf.tobytes() if ok else None
                if chunk:
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(chunk)}\r\n\r\n".encode())
                    self.wfile.write(chunk)
                    self.wfile.write(b"\r\n")
                elapsed = time.perf_counter() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
        except (BrokenPipeError, ConnectionResetError):
            # The tab was closed or reloaded. Normal, not an error.
            return


def serve(loop: ControlLoop, host: str = "127.0.0.1", port: int = 8770,
          token: str | None = None, mock_backend=None) -> GuiServer:
    """Start the server on a background thread and return it."""
    server = GuiServer((host, port), loop, token, mock_backend)
    threading.Thread(target=server.serve_forever, name="gui-http",
                     daemon=True).start()
    return server
