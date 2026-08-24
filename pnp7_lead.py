"""Driver for the PNP-7 lead (master) arm.

Hardware discovered on this machine:
  - 8x Dynamixel XL330 on one half-duplex bus behind an FT232H (/dev/pnp7_lead)
  - Protocol 2.0, 1 Mbps
  - IDs 1..7 = arm joints J1..J7 (XL330-M288-T, model 1200)
  - ID  8    = gripper trigger    (XL330-M077-T, model 1190)

The lead arm is a passive input device: torque is disabled on every servo so
the operator can backdrive it. This driver is READ-ONLY on the servo bus --
it never writes to the torque-enable register or any goal register.

Encoder wrap
------------
Present Position is a SIGNED int32 that the SDK hands back as unsigned. Two
joints on this arm rest right on the wrap boundary (J6 near 0, J5 near 4095),
so a raw read of J6 while it is backdriven below zero comes back as 2^32-3
rather than -3. Feeding that into a relative mapping would command a ~360 deg
step to the Franka.

This driver therefore does two things:
  1. decodes Present Position as signed int32, and
  2. accumulates a CONTINUOUS unwrapped tick count per servo, by taking each
     successive delta modulo one revolution and folding it into [-2048, 2048).

Consumers should use `ticks_cont` / `q_rad`, which are continuous and safe to
subtract. `ticks_raw` is kept for logging fidelity.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import serial
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead

DEFAULT_PORT = "/dev/pnp7_lead"
DEFAULT_BAUD = 1000000

ARM_IDS = (1, 2, 3, 4, 5, 6, 7)
GRIPPER_ID = 8
ALL_IDS = ARM_IDS + (GRIPPER_ID,)

ADDR_PRESENT_VELOCITY = 128
LEN_VEL_POS = 8          # 128..135 -> velocity(4) + position(4)

ADDR_TORQUE_ENABLE = 64

TICKS_PER_REV = 4096
HALF_REV = TICKS_PER_REV // 2
CENTER_TICKS = 2048
TICKS_TO_RAD = 2.0 * math.pi / TICKS_PER_REV

# XL330 present_velocity unit is 0.229 rev/min.
VEL_UNIT_RAD_S = 0.229 * 2.0 * math.pi / 60.0

# A human backdriving the arm cannot exceed this. Anything faster is a glitched
# frame, not motion, so the sample is rejected rather than propagated.
MAX_TICKS_PER_S = 40000.0

# The FT232H occasionally reports readiness and then returns nothing, which
# pyserial raises as SerialException. It is transient, but a teleop session must
# never die on it -- nor silently keep using a stale target.
REOPEN_BACKOFF_S = 0.25


def _u32_to_i32(value: int) -> int:
    return value - (1 << 32) if value >= (1 << 31) else value


def wrap_delta(curr: int, prev: int) -> int:
    """Shortest signed tick delta between two single-turn encoder readings."""
    return (curr - prev + HALF_REV) % TICKS_PER_REV - HALF_REV


def ticks_to_rad(ticks: float) -> float:
    """Convert a tick count to radians about the servo centre."""
    return (ticks - CENTER_TICKS) * TICKS_TO_RAD


@dataclass
class LeadSample:
    """One synchronised read of the whole lead arm."""

    t_monotonic_ns: int
    ticks_raw: list[int] = field(default_factory=list)    # signed, as reported
    ticks_cont: list[int] = field(default_factory=list)   # unwrapped, continuous
    q_rad: list[float] = field(default_factory=list)      # J1..J7, from ticks_cont
    dq_rad_s: list[float] = field(default_factory=list)   # J1..J7, rad/s
    gripper_ticks: int = 0
    gripper_rad: float = 0.0
    seq: int = 0

    # Backwards-compatible alias; prefer ticks_cont for any arithmetic.
    @property
    def ticks(self) -> list[int]:
        return self.ticks_cont


class PNP7Lead:
    """Synchronous reader for the lead arm.

    Use `read()` for a blocking one-shot read, or `start()`/`latest()` to run a
    background sampling thread that keeps only the newest sample. The threaded
    mode is what the teleop bridge consumes, so the Franka realtime loop never
    blocks on USB traffic.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self.port_name = port
        self.baud = baud
        self._port = PortHandler(port)
        self._packet = PacketHandler(2.0)
        self._sync = None
        self._seq = 0

        self._last_raw: list[int] | None = None
        self._cont: list[int] | None = None
        self._last_t_ns: int | None = None

        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: LeadSample | None = None
        self.read_failures = 0
        self.rejected_jumps = 0
        self.serial_errors = 0
        self.reopens = 0
        self._last_good_ns: int | None = None
        self._next_reopen_ns = 0

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        if not self._port.openPort():
            raise RuntimeError(f"could not open {self.port_name}")
        if not self._port.setBaudRate(self.baud):
            raise RuntimeError(f"could not set baud {self.baud}")
        self._sync = GroupSyncRead(
            self._port, self._packet, ADDR_PRESENT_VELOCITY, LEN_VEL_POS
        )
        for sid in ALL_IDS:
            if not self._sync.addParam(sid):
                raise RuntimeError(f"syncread addParam failed for id {sid}")

    def close(self) -> None:
        self.stop()
        self._port.closePort()

    # -- safety check ------------------------------------------------------
    def assert_torque_disabled(self) -> dict[int, int]:
        """Verify every servo still has torque off, so the arm stays passive."""
        states: dict[int, int] = {}
        for sid in ALL_IDS:
            val, comm, err = self._packet.read1ByteTxRx(
                self._port, sid, ADDR_TORQUE_ENABLE
            )
            if comm != 0 or err != 0:
                raise RuntimeError(f"could not read torque state of id {sid}")
            states[sid] = val
        enabled = [sid for sid, v in states.items() if v != 0]
        if enabled:
            raise RuntimeError(
                f"torque is ENABLED on servo ids {enabled}; the lead arm must "
                f"stay passive. Refusing to run."
            )
        return states

    # -- reading -----------------------------------------------------------
    def read(self) -> LeadSample | None:
        """One blocking SyncRead of all 8 servos. Returns None on a bad frame."""
        try:
            comm = self._sync.txRxPacket()
            t_ns = time.monotonic_ns()
            if comm != 0:
                self.read_failures += 1
                return None

            raw, vel = [], []
            for sid in ALL_IDS:
                if not self._sync.isAvailable(sid, ADDR_PRESENT_VELOCITY,
                                              LEN_VEL_POS):
                    self.read_failures += 1
                    return None
                raw.append(_u32_to_i32(
                    self._sync.getData(sid, ADDR_PRESENT_VELOCITY + 4, 4)))
                vel.append(_u32_to_i32(
                    self._sync.getData(sid, ADDR_PRESENT_VELOCITY, 4)))
        except (serial.SerialException, OSError):
            self.serial_errors += 1
            self._try_reopen()
            return None

        if self._last_raw is None:
            self._cont = list(raw)
        else:
            dt = max((t_ns - self._last_t_ns) / 1e9, 1e-4)
            deltas = [wrap_delta(raw[i], self._last_raw[i]) for i in range(len(raw))]
            if any(abs(d) / dt > MAX_TICKS_PER_S for d in deltas):
                # Implausible for a human-driven arm: drop the frame rather than
                # let a glitch reach the Franka.
                self.rejected_jumps += 1
                self._last_raw = raw
                self._last_t_ns = t_ns
                return None
            self._cont = [self._cont[i] + deltas[i] for i in range(len(raw))]

        self._last_raw = raw
        self._last_t_ns = t_ns
        self._last_good_ns = t_ns
        self._seq += 1

        cont = list(self._cont)
        return LeadSample(
            t_monotonic_ns=t_ns,
            ticks_raw=raw,
            ticks_cont=cont,
            q_rad=[ticks_to_rad(cont[i]) for i in range(len(ARM_IDS))],
            dq_rad_s=[vel[i] * VEL_UNIT_RAD_S for i in range(len(ARM_IDS))],
            gripper_ticks=cont[-1],
            gripper_rad=ticks_to_rad(cont[-1]),
            seq=self._seq,
        )

    # -- fault handling ----------------------------------------------------
    def _try_reopen(self) -> None:
        """Re-establish the port after a transient FTDI fault.

        Rate-limited, so a genuinely unplugged adapter does not spin. Position
        continuity is deliberately reset: after a gap we cannot know how far the
        arm moved, so the next sample re-seeds the reference rather than
        inventing a delta.
        """
        now = time.monotonic_ns()
        if now < self._next_reopen_ns:
            return
        self._next_reopen_ns = now + int(REOPEN_BACKOFF_S * 1e9)
        try:
            self._port.closePort()
        except Exception:
            pass
        try:
            if self._port.openPort() and self._port.setBaudRate(self.baud):
                self._sync = GroupSyncRead(
                    self._port, self._packet, ADDR_PRESENT_VELOCITY, LEN_VEL_POS
                )
                for sid in ALL_IDS:
                    self._sync.addParam(sid)
                self._last_raw = None
                self._last_t_ns = None
                self.reopens += 1
        except Exception:
            pass

    def age_s(self) -> float:
        """Seconds since the last good sample; inf if there has never been one.

        The teleop layer must freeze the Franka target when this exceeds its
        watchdog, rather than acting on a stale lead-arm pose.
        """
        if self._last_good_ns is None:
            return float("inf")
        return (time.monotonic_ns() - self._last_good_ns) / 1e9

    def healthy(self, max_age_s: float = 0.05) -> bool:
        return self.age_s() <= max_age_s

    # -- background sampling ----------------------------------------------
    def _loop(self) -> None:
        while self._running:
            sample = self.read()
            if sample is not None:
                with self._lock:
                    self._latest = sample

    def start(self) -> None:
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name="pnp7-lead-reader", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def latest(self) -> LeadSample | None:
        with self._lock:
            return self._latest
