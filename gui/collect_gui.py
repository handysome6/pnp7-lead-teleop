"""GUI-driven collection on the rig. The RLinf-side entrypoint.

This is `examples/embodiment/collect_real_data.py` with the fixed
``while success_cnt < target`` loop replaced by an operator: same Ray launch,
same ``RealWorldEnv``, same ``CollectEpisode``, but the episode boundaries come
from the browser and the recording intervals come from the F3 pedal.

Run it from the RLinf checkout on the robot PC, with this repo importable:

    cd ~/workspace/andyls/RLinf
    export EMBODIED_PATH=$PWD/examples/embodiment
    export PYTHONPATH=$PWD:$HOME/workspace/andyls/pnp7-lead-teleop:$PYTHONPATH
    python -m gui.collect_gui \
        --config-path $EMBODIED_PATH/config \
        --config-name realworld_collect_data_gello_franky \
        runner.logger.log_path=$PWD/logs/gui

Then open the printed URL. It binds loopback by default; to drive it from a
laptop, pass ``gui.host=0.0.0.0`` and use the token it prints.

A note on why this is a separate entrypoint rather than a patch to
``collect_real_data.py``: the single-arm Franky/GELLO work is headed for an
upstream RLinf PR, and a GUI-driven collector is rig-specific. Keeping it here
means the upstream diff stays about the robot, and this file is free to change
without touching anything the dual-arm path shares.
"""
from __future__ import annotations

import os
import secrets
import threading

import hydra

from gui.rlinf_backend import RlinfBackend
from gui.server import serve
from gui.session import ControlLoop


def _make_worker_class():
    """Build the Worker subclass lazily, so importing this module is cheap.

    ``rlinf.scheduler`` pulls in Ray, which is a slow and Linux-flavoured
    import; the module-level docstring above should be readable on a laptop.
    """
    from rlinf.scheduler import Worker

    class GuiCollector(Worker):
        """Holds the env and serves the GUI from inside the Ray worker.

        Everything the GUI touches -- cameras, arm, GELLO, pedal -- is claimed
        by this process. That is not a convenience; ``pipeline.start()`` and the
        FCI connection admit exactly one owner, so a GUI in any other process
        could only ever watch.
        """

        def __init__(self, cfg):
            super().__init__()
            self.cfg = cfg

        def run(self):
            gui_cfg = self.cfg.get("gui", {})
            host = str(gui_cfg.get("host", "127.0.0.1"))
            port = int(gui_cfg.get("port", 8770))
            token = gui_cfg.get("token", None)

            dc = self.cfg.env.eval.get("data_collection")
            fps = float(gui_cfg.get("fps", (dc and dc.get("fps")) or 10))

            loopback = host in ("127.0.0.1", "localhost", "::1")
            if token is None and not loopback:
                # This page commands a 7-DoF arm. Reaching it from another
                # machine should take a secret, not just the right IP.
                token = secrets.token_urlsafe(16)

            config_dir = gui_cfg.get(
                "config_dir", os.environ.get("EMBODIED_PATH", "") + "/config")

            backend = RlinfBackend(self.cfg, self.worker_info,
                                   config_dir=config_dir or None)
            loop = ControlLoop(backend, fps=fps)
            loop.start()
            server = serve(loop, host=host, port=port, token=token)

            suffix = f"?token={token}" if token else ""
            self.log_info(f"collection GUI on http://{host}:{port}/{suffix}")
            if not loopback:
                self.log_info(f"auth token: {token}")
            self.log_info(
                "Open a session from the browser. Nothing touches the robot "
                "until you do -- the env, and with it the FCI connection and "
                "the RealSense pipelines, is built on 'Open'.")

            stop = threading.Event()
            try:
                # Held open by the operator; Ctrl-C in the launching shell ends it.
                stop.wait()
            except KeyboardInterrupt:
                pass
            finally:
                self.log_info("shutting down the GUI")
                server.shutdown()
                loop.stop()

    return GuiCollector


@hydra.main(version_base="1.1", config_path=None,
            config_name="realworld_collect_data_gello_franky")
def main(cfg):
    from rlinf.scheduler import Cluster, ComponentPlacement

    GuiCollector = _make_worker_class()
    cluster = Cluster(cluster_cfg=cfg.cluster)
    placement = ComponentPlacement(cfg, cluster).get_strategy("env")
    collector = GuiCollector.create_group(cfg).launch(
        cluster, name=cfg.env.group_name, placement_strategy=placement)
    collector.run().wait()


if __name__ == "__main__":
    main()
