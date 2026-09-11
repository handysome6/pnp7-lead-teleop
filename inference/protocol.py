"""Small length-prefixed protocol shared by the PNP-7 policy client/server.

Inference requests contain a JSON metadata block followed by two raw uint8 RGB
images.  Keeping the images raw avoids depending on a particular JPEG decoder
on the training PC and is still only about 300 kB per request at 224x224.
"""

from __future__ import annotations

import json
import socket
import struct


_U32 = struct.Struct("!I")
MAX_MESSAGE_BYTES = 16 * 1024 * 1024


def _read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("peer closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_frame(sock: socket.socket, payload: bytes) -> None:
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ValueError("message is too large: {} bytes".format(len(payload)))
    sock.sendall(_U32.pack(len(payload)) + payload)


def recv_frame(sock: socket.socket) -> bytes:
    size = _U32.unpack(_read_exact(sock, _U32.size))[0]
    if size > MAX_MESSAGE_BYTES:
        raise ValueError("peer announced an oversized message: {} bytes".format(size))
    return _read_exact(sock, size)


def encode_json(value: dict) -> bytes:
    return json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")


def decode_json(payload: bytes) -> dict:
    value = json.loads(payload.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON payload must be an object")
    return value


def encode_inference_request(metadata: dict, base_rgb, wrist_rgb) -> bytes:
    if base_rgb.shape != wrist_rgb.shape:
        raise ValueError("camera images must have the same shape")
    if base_rgb.dtype.name != "uint8" or wrist_rgb.dtype.name != "uint8":
        raise ValueError("camera images must be uint8")
    if len(base_rgb.shape) != 3 or base_rgb.shape[2] != 3:
        raise ValueError("camera images must be HxWx3")
    base = base_rgb.tobytes(order="C")
    wrist = wrist_rgb.tobytes(order="C")
    header = dict(metadata)
    header.update(
        {
            "kind": "infer",
            "image_shape": list(base_rgb.shape),
            "base_nbytes": len(base),
            "wrist_nbytes": len(wrist),
        }
    )
    header_bytes = encode_json(header)
    return _U32.pack(len(header_bytes)) + header_bytes + base + wrist


def decode_inference_request(payload: bytes):
    if len(payload) < _U32.size:
        raise ValueError("truncated inference request")
    header_size = _U32.unpack(payload[: _U32.size])[0]
    header_end = _U32.size + header_size
    if header_end > len(payload):
        raise ValueError("truncated inference metadata")
    metadata = decode_json(payload[_U32.size : header_end])
    base_size = int(metadata["base_nbytes"])
    wrist_size = int(metadata["wrist_nbytes"])
    expected = header_end + base_size + wrist_size
    if expected != len(payload):
        raise ValueError(
            "image payload size mismatch: expected {}, got {}".format(expected, len(payload))
        )
    base = payload[header_end : header_end + base_size]
    wrist = payload[header_end + base_size :]
    return metadata, base, wrist
