"""Standalone entrypoint. Runs the GUI against the mock backend.

    python -m gui.app --mock

This is how the interface is developed and demonstrated off the rig: no RLinf,
no Ray, no hardware, no Linux. For the real thing see `gui.collect_gui`, which
has to run inside an RLinf Ray worker on the robot PC.
"""
from __future__ import annotations

import argparse
import secrets
import signal
import sys
import threading

from gui.mock import MockBackend
from gui.server import serve
from gui.session import ControlLoop


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mock", action="store_true",
                    help="run against the mock backend (the only backend that "
                         "works outside the robot PC)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address. The default is loopback on purpose: "
                         "this page moves a robot arm, so exposing it needs to "
                         "be a decision, not a default.")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--fps", type=float, default=10.0,
                    help="control-loop rate; match the config's data_collection.fps")
    ap.add_argument("--config-dir", default=None,
                    help="directory of *.yaml configs to offer in the picker")
    ap.add_argument("--token", default=None,
                    help="require this token on every request. Generated "
                         "automatically when --host is not loopback.")
    args = ap.parse_args(argv)

    if not args.mock:
        print("gui.app only runs the mock backend. On the robot PC use:\n"
              "  python -m gui.collect_gui --config-name "
              "realworld_collect_data_gello_franky", file=sys.stderr)
        return 2

    token = args.token
    if token is None and args.host not in ("127.0.0.1", "localhost", "::1"):
        token = secrets.token_urlsafe(16)
        print(f"non-loopback bind: requiring token {token}")

    backend = MockBackend(config_dir=args.config_dir)
    loop = ControlLoop(backend, fps=args.fps)
    loop.start()
    server = serve(loop, host=args.host, port=args.port, token=token,
                   mock_backend=backend)

    suffix = f"?token={token}" if token else ""
    print(f"GUI on http://{args.host}:{args.port}/{suffix}")
    print("mock backend -- hold SPACE (or the Hold F3 button) to simulate the pedal")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        print("\nshutting down")
        server.shutdown()
        loop.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
