"""Entrypoint for the collection GUI.

On the robot PC, from the repo root:

    .venv/bin/python -m gui.app

That supervises `bin/pnp7_teleop` and `collect/record_cameras.py` — the same
processes `scripts/collect_episode.sh` runs, in the same order, but with the
operator deciding where each take starts and ends. Nothing touches the robot
until a session is opened.

Anywhere else:

    python -m gui.app --mock

which drives a fake rig so the interface can be worked on without hardware.

It binds loopback by default. This page starts and stops a 7-DoF arm, so
reaching it from another machine should take a secret and not just the right
IP: pass `--host 0.0.0.0` and it generates a token.
"""
from __future__ import annotations

import argparse
import secrets
import signal
import sys
import threading
from pathlib import Path

from gui.server import serve
from gui.session import ControlLoop

REPO = Path(__file__).resolve().parent.parent


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mock", action="store_true",
                    help="drive a fake rig instead of the real one; the only "
                         "mode that runs off the robot PC")
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address. Loopback by default, on purpose.")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--episodes-dir", default=str(REPO / "episodes"),
                    help="where episode directories are created")
    ap.add_argument("--preview-dir", default="/tmp/pnp7_preview",
                    help="scratch directory for the camera recorder's JPEG tap")
    ap.add_argument("--status-path", default="/tmp/pnp7_gui_status.json",
                    help="where the bridge publishes its 10 Hz status JSON")
    ap.add_argument("--fps", type=float, default=10.0,
                    help="control-loop rate. 10 Hz matches the bridge's status "
                         "publisher, so polling faster only re-reads a file "
                         "that has not changed.")
    ap.add_argument("--token", default=None,
                    help="require this token on every request. Generated "
                         "automatically when --host is not loopback.")
    args = ap.parse_args(argv)

    token = args.token
    if token is None and args.host not in ("127.0.0.1", "localhost", "::1"):
        token = secrets.token_urlsafe(16)

    mock_backend = None
    if args.mock:
        from gui.mock import MockBackend

        backend = MockBackend()
        mock_backend = backend
    else:
        from gui.legacy_backend import BRIDGE, VENV_PYTHON, LegacyBackend

        # Fail here with something readable, rather than at the first click
        # with a subprocess error from four layers down.
        if not VENV_PYTHON.is_file():
            print(f"missing {VENV_PYTHON} -- see 'Setting up a checkout' in "
                  "the README", file=sys.stderr)
            return 1
        if not BRIDGE.is_file():
            print(f"missing {BRIDGE} -- run ./build.sh", file=sys.stderr)
            return 1
        backend = LegacyBackend(
            episodes_dir=args.episodes_dir,
            preview_dir=args.preview_dir,
            status_path=args.status_path,
        )

    loop = ControlLoop(backend, fps=args.fps)
    loop.start()
    server = serve(loop, host=args.host, port=args.port, token=token,
                   mock_backend=mock_backend)

    suffix = f"?token={token}" if token else ""
    print(f"GUI on http://{args.host}:{args.port}/{suffix}")
    if token:
        print(f"auth token: {token}")
    if args.mock:
        print("mock rig -- hold SPACE (or the Hold F3 button) for the pedal")
    else:
        print(f"episodes -> {args.episodes_dir}")
        print("nothing touches the robot until you open a session")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        print("\nshutting down")
        server.shutdown()
        # Stops the loop, which closes the session -- SIGINT to the bridge so
        # its in-RAM log still reaches disk.
        loop.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
