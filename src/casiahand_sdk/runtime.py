"""Threaded direct control runtime for a dual CASIA Hand-M device."""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)

CASIA_JOINT_SUFFIXES = (
    "thumb_proximal",
    "thumb_intermediate",
    "index_proximal",
    "middle_proximal",
    "ring_proximal",
    "pinky_proximal",
    "index_intermediate",
    "middle_intermediate",
    "ring_intermediate",
    "pinky_intermediate",
)
CASIA_LEFT_JOINT_NAMES = tuple(f"left_{suffix}" for suffix in CASIA_JOINT_SUFFIXES)
CASIA_RIGHT_JOINT_NAMES = tuple(f"right_{suffix}" for suffix in CASIA_JOINT_SUFFIXES)
CASIA_JOINT_NAMES = (*CASIA_LEFT_JOINT_NAMES, *CASIA_RIGHT_JOINT_NAMES)

_ONE_HAND_UPPER_LIMITS = np.asarray([1.57, 1.57 * 7.0 / 9.0, *([1.57] * 8)], dtype=np.float64)
CASIA_LEFT_LIMITS = np.column_stack((np.zeros(10), _ONE_HAND_UPPER_LIMITS))
CASIA_RIGHT_LIMITS = CASIA_LEFT_LIMITS.copy()

@dataclass(frozen=True)
class CasiaHandConfig:
    """Transport and freshness configuration for a dual CASIA Hand-M."""

    left_hand_id: int = 2
    right_hand_id: int = 0x20
    baudrate: int = 115200
    port_name: str = "/dev/ttyUSB0"
    command_timeout_s: float = 0.25
    joint_state_fps: float = 100.0
    joint_state_timeout_s: float = 0.25
    startup_timeout_s: float = 5.0

    def __post_init__(self):
        if not 0 <= self.left_hand_id <= 255 or not 0 <= self.right_hand_id <= 255:
            raise ValueError("CASIA hand IDs must be in [0, 255]")
        if self.left_hand_id == self.right_hand_id:
            raise ValueError("CASIA left and right hand IDs must differ")
        if self.baudrate <= 0:
            raise ValueError("CASIA baudrate must be positive")
        if not self.port_name.strip():
            raise ValueError("CASIA serial port_name must not be empty")
        for name in ("command_timeout_s", "joint_state_fps", "joint_state_timeout_s", "startup_timeout_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"CASIA {name} must be positive")


HandFactory = Callable[[CasiaHandConfig], object]


@dataclass(frozen=True)
class CasiaHandCommandFrame:
    """One inseparable left/right command originating from one control frame."""

    joint_positions: np.ndarray
    source_timestamp_ns: int
    frame_id: int | None
    is_return_to_default_command: bool = False


class CasiaHandRuntime:
    """Apply commands and expose separately measured CASIA joint positions.

    The robot loop never performs serial I/O. One worker owns the pybind SDK
    object, applies the newest atomic dual-hand command, and consumes hardware
    samples for Recorder state data.
    """

    def __init__(self, cfg: CasiaHandConfig, *, hand_factory: HandFactory | None = None):
        self.cfg = cfg
        self._hand_factory = hand_factory or self._create_hand
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: Exception | None = None
        self._enabled = False
        self._last_command_at: float | None = None
        self._last_joint_state_at: float | None = None
        self._applied_commands = np.zeros(20, dtype=np.float32)
        self._measured_joint_positions = np.zeros(20, dtype=np.float32)
        self._applied_source_timestamp_ns: int | None = None
        self._applied_frame_id: int | None = None
        self._command_queue: queue.Queue[CasiaHandCommandFrame] = queue.Queue(maxsize=1)

        self._thread = threading.Thread(target=self._run, name="CasiaHandRuntime", daemon=True)
        self._thread.start()
        if not self._ready.wait(cfg.startup_timeout_s):
            self.close()
            raise TimeoutError(f"CASIA Hand runtime did not start within {cfg.startup_timeout_s:.1f} seconds")
        if self._error is not None:
            self.close()
            raise RuntimeError("failed to start CASIA Hand runtime") from self._error

    @staticmethod
    def _create_hand(cfg: CasiaHandConfig):
        from ._native import CasiaHand

        hand = CasiaHand(
            left_hand_id=cfg.left_hand_id,
            right_hand_id=cfg.right_hand_id,
            baudrate=cfg.baudrate,
            port_name=cfg.port_name,
        )
        if not hand.init():
            hand.close()
            raise RuntimeError(f"CASIA SDK failed to initialize hands on {cfg.port_name}")
        return hand

    def set_takeover_enabled(self, enabled: bool, *, return_to_default: bool = False):
        """Open or close the command gate, optionally returning both hands to zero."""

        enabled = bool(enabled)
        with self._lock:
            if enabled == self._enabled:
                return
            self._enabled = enabled
            self._last_command_at = None
            self._applied_source_timestamp_ns = None
            self._applied_frame_id = None
        self._discard_pending_commands()
        if not enabled and return_to_default:
            self._enqueue_command(
                CasiaHandCommandFrame(
                    joint_positions=np.zeros(20, dtype=np.float32),
                    source_timestamp_ns=time.time_ns(),
                    frame_id=None,
                    is_return_to_default_command=True,
                )
            )
            logger.warning("CASIA Hand returning to default pose")
        logger.warning("CASIA Hand control %s", "enabled" if enabled else "disabled")

    def set_joint_commands(
        self,
        left_joint_positions,
        right_joint_positions,
        source_timestamp_ns: int | None = None,
        frame_id: int | None = None,
    ) -> np.ndarray:
        """Queue the newest atomic command and return the values Recorder should store as action."""

        left_command = self._validate_command(left_joint_positions, "left")
        right_command = self._validate_command(right_joint_positions, "right")
        left_command = np.clip(left_command, CASIA_LEFT_LIMITS[:, 0], CASIA_LEFT_LIMITS[:, 1])
        right_command = np.clip(right_command, CASIA_RIGHT_LIMITS[:, 0], CASIA_RIGHT_LIMITS[:, 1])
        combined = np.concatenate((left_command, right_command))
        command_frame = CasiaHandCommandFrame(
            joint_positions=combined.copy(),
            source_timestamp_ns=time.time_ns() if source_timestamp_ns is None else int(source_timestamp_ns),
            frame_id=None if frame_id is None else int(frame_id),
        )
        self._enqueue_command(command_frame)
        return combined.astype(np.float32)

    def _enqueue_command(self, command_frame: CasiaHandCommandFrame):
        try:
            self._command_queue.put_nowait(command_frame)
        except queue.Full:
            try:
                self._command_queue.get_nowait()
            except queue.Empty:  # pragma: no cover - no other command consumer exists
                pass
            self._command_queue.put_nowait(command_frame)

    @staticmethod
    def _validate_command(value, side: str) -> np.ndarray:
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (10,):
            raise ValueError(f"CASIA {side} joint command must have shape (10,)")
        if not np.isfinite(result).all():
            raise ValueError(f"CASIA {side} joint command must contain only finite values")
        return result

    @staticmethod
    def _validate_measured_positions(value) -> np.ndarray:
        result = np.asarray(value, dtype=np.float32)
        if result.shape != (20,) or not np.isfinite(result).all():
            raise RuntimeError("CASIA measured joint positions must contain 20 finite angles")
        return result

    def _discard_pending_commands(self):
        while True:
            try:
                self._command_queue.get_nowait()
            except queue.Empty:
                return

    def _run(self):
        hand = None
        try:
            hand = self._hand_factory(self.cfg)
            self._ready.set()
            state_period_s = 1.0 / self.cfg.joint_state_fps
            next_state_at = time.monotonic()

            while not self._stop.is_set():
                try:
                    command_frame = self._command_queue.get(timeout=min(0.01, state_period_s))
                except queue.Empty:
                    pass
                else:
                    with self._lock:
                        enabled = self._enabled
                    if enabled or command_frame.is_return_to_default_command:
                        applied = self._validate_measured_positions(
                            hand.set_joint_positions(command_frame.joint_positions.tolist())
                        )
                        now = time.monotonic()
                        with self._lock:
                            self._applied_commands = applied
                            self._last_command_at = now
                            self._applied_source_timestamp_ns = command_frame.source_timestamp_ns
                            self._applied_frame_id = command_frame.frame_id

                now = time.monotonic()
                if now >= next_state_at:
                    sample = hand.try_get_joint_positions()
                    if sample is not None:
                        measured = self._validate_measured_positions(sample)
                        with self._lock:
                            self._measured_joint_positions = measured
                            self._last_joint_state_at = now
                    next_state_at = now + state_period_s
        except Exception as exc:
            self._error = exc
            logger.exception("CASIA Hand runtime stopped: %s", exc)
            self._ready.set()
        finally:
            if hand is not None:
                try:
                    hand.close()
                except Exception:
                    logger.exception("failed to close CASIA Hand SDK")

    def get_data(self) -> dict:
        """Return consistent copies of measured state and actually applied commands."""

        if self._error is not None:
            raise RuntimeError("CASIA Hand runtime is unavailable") from self._error
        now = time.monotonic()
        with self._lock:
            enabled = self._enabled
            command_age = None if self._last_command_at is None else now - self._last_command_at
            state_age = None if self._last_joint_state_at is None else now - self._last_joint_state_at
            measured_positions = self._measured_joint_positions.copy()
            applied_commands = self._applied_commands.copy()
            applied_source_timestamp_ns = self._applied_source_timestamp_ns
            applied_frame_id = self._applied_frame_id

        command_fresh = enabled and command_age is not None and command_age <= self.cfg.command_timeout_s
        joint_state_fresh = state_age is not None and state_age <= self.cfg.joint_state_timeout_s
        return {
            "joint_names": list(CASIA_JOINT_NAMES),
            "joint_positions": measured_positions,
            "joint_position_commands": applied_commands,
            "left_fresh": command_fresh,
            "right_fresh": command_fresh,
            "joint_state_fresh": joint_state_fresh,
            "fresh": command_fresh and joint_state_fresh,
            "age_s": command_age,
            "enabled": enabled,
            "applied_source_timestamp_ns": applied_source_timestamp_ns,
            "applied_frame_id": applied_frame_id,
        }

    def reset(self):
        self.set_takeover_enabled(False)

    def close(self):
        self._stop.set()
        thread = self._thread
        if thread is not None:
            # Do not return while the worker still owns the native SDK/tty.
            # CASIA serial transactions have their own bounded timeout, so an
            # unconditional join provides a real close guarantee to callers.
            thread.join()
            self._thread = None
