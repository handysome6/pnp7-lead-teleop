#!/usr/bin/env python3
"""Serve the trained PNP-7 PI0.5 policy over the robot/training LAN.

This runs inside the same RLinf Docker image used for training.  The base model
is constructed with the exact saved training config, LoRA modules are inserted,
and the final full state dict is then loaded strictly before the port opens.
"""

from __future__ import annotations

import argparse
import json
import socketserver
import sys
import threading
import time
from pathlib import Path

import numpy as np

from protocol import (
    decode_inference_request,
    decode_json,
    encode_json,
    recv_frame,
    send_frame,
)


class PolicyRuntime:
    def __init__(self, args):
        import torch
        from omegaconf import OmegaConf

        self.torch = torch
        self.args = args
        saved = OmegaConf.load(args.config)
        cfg = saved.actor.model
        cfg.model_path = args.base_model
        cfg.load_to_device = False

        # Importing the registry is what installs the PNP7-specific OpenPI
        # builder and LoRA layout used during SFT.
        from rlinf.models import get_model

        print("MODEL_LOAD constructing base+LoRA model", flush=True)
        model = get_model(cfg)
        if model is None:
            raise RuntimeError("RLinf did not register model_type={}".format(cfg.model_type))

        print("MODEL_LOAD reading {}".format(args.weights), flush=True)
        state = torch.load(args.weights, map_location="cpu", weights_only=True)
        incompatible = model.load_state_dict(state, strict=True)
        if incompatible.missing_keys or incompatible.unexpected_keys:
            raise RuntimeError("strict checkpoint load returned incompatible keys")
        del state

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available in the inference container")
        model.to("cuda")
        model.eval()
        self.model = model
        self.cfg = cfg
        self.lock = threading.Lock()
        self.requests = 0
        self.last_inference_ms = None
        self.started = time.time()
        torch.cuda.empty_cache()

        # A real forward pass is the final readiness gate.  It catches missing
        # norm stats, camera-slot mismatches and CUDA/dtype errors before a live
        # client is ever allowed to connect.
        zeros = np.zeros((args.image_size, args.image_size, 3), dtype=np.uint8)
        state10 = np.array(
            [0.503003, 0.037494, 0.106219, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0],
            dtype=np.float32,
        )
        actions, elapsed_ms = self.infer(zeros, zeros, state10, args.prompt)
        if actions.shape != (10, 7) or not np.isfinite(actions).all():
            raise RuntimeError("warmup produced invalid actions")
        print(
            "MODEL_READY strict=true warmup_ms={:.1f} shape={} min={:.6f} max={:.6f}".format(
                elapsed_ms, list(actions.shape), float(actions.min()), float(actions.max())
            ),
            flush=True,
        )

    def infer(self, base_rgb, wrist_rgb, state, prompt):
        torch = self.torch
        env_obs = {
            "main_images": torch.from_numpy(base_rgb[None].copy()),
            "wrist_images": torch.from_numpy(wrist_rgb[None].copy()),
            "extra_view_images": None,
            "states": torch.from_numpy(state[None].astype(np.float32, copy=False)),
            "task_descriptions": [prompt],
        }
        with self.lock:
            # PI uses sampled flow noise.  Resetting both generators makes the
            # deployed policy deterministic for a given observation.
            torch.manual_seed(self.args.seed)
            torch.cuda.manual_seed_all(self.args.seed)
            torch.cuda.synchronize()
            started = time.perf_counter()
            with torch.inference_mode():
                actions, _ = self.model.predict_action_batch(
                    env_obs, mode="eval", compute_values=False
                )
            torch.cuda.synchronize()
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            actions = actions.detach().to(torch.float32).cpu().numpy()[0]
            self.requests += 1
            self.last_inference_ms = elapsed_ms
        if actions.shape != (10, 7):
            raise RuntimeError("unexpected model output shape {}".format(actions.shape))
        if not np.isfinite(actions).all():
            raise RuntimeError("model produced NaN or Inf")
        return actions, elapsed_ms

    def health(self):
        return {
            "ok": True,
            "kind": "health",
            "model": self.args.model_id,
            "base_model": str(self.args.base_model),
            "checkpoint": str(self.args.weights),
            "strict_checkpoint": True,
            "requests": self.requests,
            "last_inference_ms": self.last_inference_ms,
            "uptime_s": time.time() - self.started,
            "cuda": self.torch.cuda.get_device_name(0),
        }


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(self.server.request_timeout)
        while True:
            try:
                payload = recv_frame(self.request)
            except ConnectionError:
                return
            except TimeoutError:
                return
            try:
                if payload.startswith(b"{"):
                    request = decode_json(payload)
                    if request.get("kind") != "health":
                        raise ValueError("unknown JSON request kind")
                    response = self.server.runtime.health()
                else:
                    metadata, base_bytes, wrist_bytes = decode_inference_request(payload)
                    if metadata.get("token") != self.server.token:
                        raise PermissionError("invalid token")
                    shape = tuple(int(v) for v in metadata["image_shape"])
                    if shape != (
                        self.server.runtime.args.image_size,
                        self.server.runtime.args.image_size,
                        3,
                    ):
                        raise ValueError("unexpected image shape {}".format(shape))
                    base = np.frombuffer(base_bytes, dtype=np.uint8).reshape(shape)
                    wrist = np.frombuffer(wrist_bytes, dtype=np.uint8).reshape(shape)
                    state = np.asarray(metadata["state"], dtype=np.float32)
                    if state.shape != (10,) or not np.isfinite(state).all():
                        raise ValueError("state must be 10 finite numbers")
                    prompt = str(metadata["prompt"])
                    actions, elapsed_ms = self.server.runtime.infer(
                        base, wrist, state, prompt
                    )
                    response = {
                        "ok": True,
                        "kind": "actions",
                        "actions": actions.tolist(),
                        "inference_ms": elapsed_ms,
                        "request_id": metadata.get("request_id"),
                    }
            except Exception as exc:
                response = {"ok": False, "error": "{}: {}".format(type(exc).__name__, exc)}
            send_frame(self.request, encode_json(response))


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5559)
    parser.add_argument("--token", default="pnp7-local-demo")
    parser.add_argument("--request-timeout", type=float, default=30.0)
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--model-id", default="pi05_pnp7_30merged_step7500")
    parser.add_argument(
        "--prompt", default="pick up the blue cube and place it in the plate"
    )
    parser.add_argument("--base-model", default="/models/pi05-pnp7-30merged")
    parser.add_argument(
        "--config",
        default=(
            "/workspace/RLinf/logs/20260827-05:50:29-pnp7_sft_pi05_30merged/"
            "tensorboard/config.yaml"
        ),
    )
    parser.add_argument(
        "--weights",
        default=(
            "/workspace/RLinf/logs/20260827-05:50:29-pnp7_sft_pi05_30merged/"
            "pnp7_sft_pi05_30merged/checkpoints/global_step_7500/actor/"
            "model_state_dict/full_weights.pt"
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    for path in (args.config, args.weights, args.base_model):
        if not Path(path).exists():
            raise FileNotFoundError(path)
    runtime = PolicyRuntime(args)
    with Server((args.bind, args.port), Handler) as server:
        server.runtime = runtime
        server.token = args.token
        server.request_timeout = args.request_timeout
        print("SERVER_READY bind={}:{} token_required=true".format(args.bind, args.port), flush=True)
        server.serve_forever(poll_interval=0.2)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("SERVER_FATAL {}: {}".format(type(exc).__name__, exc), file=sys.stderr, flush=True)
        raise
