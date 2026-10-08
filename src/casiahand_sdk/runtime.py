"""Threaded direct control runtime for a dual CASIA Hand-M device."""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from .serial_port import SerialPortResolver

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
    auto_reconnect: bool = False
    reconnect_timeout_s: float = 1.0
    reconnect_interval_s: float = 1.0

    def __post_init__(self):
        if not 0 <= self.left_hand_id <= 255 or not 0 <= self.right_hand_id <= 255:
            raise ValueError("CASIA hand IDs must be in [0, 255]")
        if self.left_hand_id == self.right_hand_id:
            raise ValueError("CASIA left and right hand IDs must differ")
        if self.baudrate <= 0:
            raise ValueError("CASIA baudrate must be positive")
        if not self.port_name.strip():
            raise ValueError("CASIA serial port_name must not be empty")
        for name in (
            "command_timeout_s",
            "joint_state_fps",
            "joint_state_timeout_s",
            "startup_timeout_s",
            "reconnect_timeout_s",
            "reconnect_interval_s",
        ):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"CASIA {name} must be positive and finite")
        if self.reconnect_timeout_s < self.joint_state_timeout_s:
            raise ValueError("CASIA reconnect timeout must not be shorter than joint state timeout")


HandFactory = Callable[[CasiaHandConfig], object]


class _HandReleaseError(Exception):
    """A failed release must never be followed by opening another serial owner."""


@dataclass(frozen=True)
class CasiaHandCommandFrame:
    """One inseparable left/right command originating from one control frame."""

    joint_positions: np.ndarray
    source_timestamp_ns: int
    frame_id: int | None
    is_return_to_default_command: bool = False
    queued_at: float = field(default_factory=time.monotonic)
    connection_generation: int = 0
    gate_epoch: int = 0


class CasiaHandRuntime:
    """Apply commands and expose separately measured CASIA joint positions.

    The robot loop never performs serial I/O. One worker owns the pybind SDK
    object, applies the newest atomic dual-hand command, and consumes hardware
    samples for Recorder state data.
    """

    def __init__(self, cfg: CasiaHandConfig, *, hand_factory: HandFactory | None = None):
        self.cfg = cfg
        self._auto_reconnect = bool(getattr(cfg, "auto_reconnect", False))
        self._port_resolver = SerialPortResolver(cfg.port_name)
        if self._auto_reconnect and hand_factory is None:
            from ._native import CasiaHand

            if not hasattr(CasiaHand, "try_get_joint_sample"):
                raise RuntimeError("CASIA auto reconnect requires rebuilding the native SDK")
        self._hand_factory = hand_factory or self._create_hand
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: Exception | None = None
        self._enabled = False
        self._connected = False
        self._connection_state = "CONNECTING"
        self._connection_generation = 0
        self._gate_epoch = 0
        self._reconnect_attempts = 0
        self._last_error: str | None = None
        self._last_command_at: float | None = None
        self._last_joint_state_at: float | None = None
        self._applied_commands = np.zeros(20, dtype=np.float32)
        self._measured_joint_positions = np.zeros(20, dtype=np.float32)
        self._applied_source_timestamp_ns: int | None = None
        self._applied_frame_id: int | None = None
        self._command_queue: queue.Queue[CasiaHandCommandFrame] = queue.Queue(maxsize=1)

        self._thread = threading.Thread(target=self._run, name="CasiaHandRuntime", daemon=True)
        self._thread.start()
        if self._auto_reconnect:
            # Give a present device its first qualification opportunity before
            # the owner starts USB cameras. Missing hardware still returns on
            # the first failed attempt, and never blocks startup indefinitely.
            self._ready.wait(cfg.startup_timeout_s)
            return
        if not self._ready.wait(cfg.startup_timeout_s):
            self.close()
            raise TimeoutError(f"CASIA Hand runtime did not start within {cfg.startup_timeout_s:.1f} seconds")
        if self._error is not None:
            self.close()
            raise RuntimeError("failed to start CASIA Hand runtime") from self._error

    def _create_hand(self, cfg: CasiaHandConfig):
        from ._native import CasiaHand

        hand = CasiaHand(
            left_hand_id=cfg.left_hand_id,
            right_hand_id=cfg.right_hand_id,
            baudrate=cfg.baudrate,
            port_name=self._port_resolver.resolve() if self._auto_reconnect else cfg.port_name,
        )
        try:
            initialized = hand.init(cfg.startup_timeout_s) if self._auto_reconnect else hand.init()
            if not initialized:
                raise ConnectionError(f"CASIA SDK failed to initialize hands on {cfg.port_name}")
        except Exception:
            try:
                hand.close()
            except Exception as exc:
                raise _HandReleaseError("failed to release CASIA SDK after initialization failure") from exc
            raise
        return hand

    def set_takeover_enabled(self, enabled: bool, *, return_to_default: bool = False):
        """Open or close the command gate, optionally returning both hands to zero."""

        enabled = bool(enabled)
        with self._lock:
            if enabled == self._enabled:
                return
            self._enabled = enabled
            self._gate_epoch += 1
            self._last_command_at = None
            self._applied_source_timestamp_ns = None
            self._applied_frame_id = None
        self._discard_pending_commands()
        if not enabled and return_to_default and (not self._auto_reconnect or self.get_data()["joint_state_fresh"]):
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
        with self._lock:
            if self._auto_reconnect and (
                not self._connected
                or self._last_joint_state_at is None
                or time.monotonic() - self._last_joint_state_at > self.cfg.joint_state_timeout_s
            ):
                return
            command_frame = replace(
                command_frame, connection_generation=self._connection_generation, gate_epoch=self._gate_epoch
            )
            try:
                self._command_queue.put_nowait(command_frame)
            except queue.Full:
                self._discard_pending_commands()
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
            raise ValueError("CASIA measured joint positions must contain 20 finite angles")
        return result

    def _discard_pending_commands(self):
        while True:
            try:
                self._command_queue.get_nowait()
            except queue.Empty:
                return

    def _invalidate_connection(self, error: Exception | None = None):
        with self._lock:
            self._connected = False
            self._last_joint_state_at = None
            self._last_command_at = None
            self._applied_source_timestamp_ns = None
            self._applied_frame_id = None
            self._gate_epoch += 1
            self._discard_pending_commands()
            if error is not None:
                self._connection_state = "RECONNECTING"
                self._last_error = str(error)

    def _run(self):
        while not self._stop.is_set():
            hand = None
            retry = False
            try:
                hand = self._hand_factory(self.cfg)
                self._serve_hand(hand)
            except Exception as exc:
                self._invalidate_connection(exc)
                if self._auto_reconnect and isinstance(exc, (OSError, RuntimeError)):
                    with self._lock:
                        self._reconnect_attempts += 1
                    logger.warning("CASIA Hand reconnect attempt %d: %s", self._reconnect_attempts, exc)
                    retry = True
                else:
                    self._error = exc
                    logger.exception("CASIA Hand runtime stopped: %s", exc)
                self._ready.set()
            finally:
                if hand is not None:
                    try:
                        hand.close()
                    except Exception as exc:
                        # Never open another instance when release of the old tty is uncertain.
                        self._error = exc
                        retry = False
                        logger.exception("failed to close CASIA Hand SDK")
            if not retry or self._stop.wait(self.cfg.reconnect_interval_s):
                break
        self._invalidate_connection()
        with self._lock:
            self._connection_state = "STOPPED"

    def _serve_hand(self, hand):
        state_period = 1.0 / self.cfg.joint_state_fps
        # init() has its own native deadline. Do not consume the feedback
        # qualification budget while probing the serial devices.
        started_at = time.monotonic()
        next_state_at = started_at
        last_sample_at = None
        consecutive_samples = 0
        last_received_sample_at = None
        rejected_samples = 0
        active_frame = None
        recovering = self._auto_reconnect
        recovery_position = None
        last_send_at = started_at
        previous_gate_epoch = self._gate_epoch
        previously_fresh = False
        with self._lock:
            self._connected = not self._auto_reconnect
            if self._connected:
                self._connection_state = "CONNECTED"
                self._connection_generation += 1
        if not self._auto_reconnect:
            self._ready.set()
        while not self._stop.is_set():
            now = time.monotonic()
            if getattr(hand, "transport_failed", False):
                raise ConnectionError("CASIA native serial worker failed")
            if now >= next_state_at:
                sample_reader = getattr(hand, "try_get_joint_sample", None)
                sample = sample_reader() if sample_reader is not None else hand.try_get_joint_positions()
                if sample_reader is not None and sample is not None:
                    # A native read releases the GIL. If another startup worker
                    # held it before our return, this consumed sample may be old
                    # while the native worker has already published a fresh one.
                    age = time.monotonic() - sample[1]
                    if age < 0 or age > self.cfg.joint_state_timeout_s:
                        latest = sample_reader()
                        if latest is not None:
                            sample = latest
                if sample is not None:
                    positions, sampled_at = sample if sample_reader is not None else (sample, time.monotonic())
                    measured = self._validate_measured_positions(positions)
                    last_received_sample_at = sampled_at
                    now = time.monotonic()
                    # Native timestamps use Linux CLOCK_MONOTONIC, like time.monotonic().
                    if 0 <= now - sampled_at <= self.cfg.joint_state_timeout_s and (
                        last_sample_at is None or sampled_at > last_sample_at
                    ):
                        consecutive_samples = (
                            consecutive_samples + 1
                            if last_sample_at is None or sampled_at - last_sample_at <= self.cfg.joint_state_timeout_s
                            else 1
                        )
                        last_sample_at = sampled_at
                        with self._lock:
                            self._measured_joint_positions = measured
                            self._last_joint_state_at = sampled_at
                            if self._auto_reconnect and not self._connected and consecutive_samples >= 3:
                                self._discard_pending_commands()
                                self._connection_generation += 1
                                self._connected = True
                                self._connection_state = "CONNECTED"
                                self._last_error = None
                                recovery_position = measured.copy()
                                last_send_at = now
                                logger.warning("CASIA Hand connected, generation %d", self._connection_generation)
                                self._ready.set()
                    else:
                        rejected_samples += 1
                next_state_at = now + state_period

            with self._lock:
                connected = self._connected
                enabled = self._enabled
                generation = self._connection_generation
                gate_epoch = self._gate_epoch
            state_fresh = last_sample_at is not None and now - last_sample_at <= self.cfg.joint_state_timeout_s
            if self._auto_reconnect:
                if (previously_fresh and not state_fresh) or previous_gate_epoch != gate_epoch:
                    clear_commands = getattr(hand, "clear_joint_commands", None)
                    if clear_commands is not None:
                        clear_commands()
                previously_fresh = state_fresh
                previous_gate_epoch = gate_epoch
                if not connected and not state_fresh and now - started_at >= self.cfg.startup_timeout_s:
                    sample_age = None if last_received_sample_at is None else now - last_received_sample_at
                    raise ConnectionError(
                        "CASIA dual-hand feedback did not recover before startup timeout "
                        f"(valid_samples={consecutive_samples}/3, rejected_samples={rejected_samples}, "
                        f"latest_sample_age_s={sample_age})"
                    )
                if connected and now - last_sample_at >= self.cfg.reconnect_timeout_s:
                    raise ConnectionError("CASIA dual-hand feedback timed out")
                if not state_fresh:
                    active_frame = None
                    self._discard_pending_commands()
                    # Re-anchor a resumed slew to feedback rather than advancing during an outage.
                    recovering = True
                    recovery_position = None

            try:
                frame = self._command_queue.get(timeout=min(0.01, state_period))
            except queue.Empty:
                frame = None
            if frame is not None:
                active_frame = frame
            now = time.monotonic()
            state_fresh = last_sample_at is not None and now - last_sample_at <= self.cfg.joint_state_timeout_s
            if active_frame is None:
                last_send_at = now
                continue
            frame = active_frame
            with self._lock:
                enabled = self._enabled
                generation = self._connection_generation
                gate_epoch = self._gate_epoch
            allowed = enabled or frame.is_return_to_default_command
            if (
                not allowed
                or frame.gate_epoch != gate_epoch
                or frame.connection_generation != generation
                or now - frame.queued_at > self.cfg.command_timeout_s
                or (self._auto_reconnect and (not connected or not state_fresh))
            ):
                active_frame = None
                last_send_at = now
                continue
            desired = frame.joint_positions
            if recovering and not frame.is_return_to_default_command:
                if recovery_position is None:
                    with self._lock:
                        recovery_position = self._measured_joint_positions.copy()
                # One radian/second during recovery; no catch-up jump after an idle interval.
                max_delta = min(max(0.0, now - last_send_at), max(0.01, state_period))
                desired = np.clip(desired, recovery_position - max_delta, recovery_position + max_delta)
            applied = self._validate_measured_positions(hand.set_joint_positions(desired.tolist()))
            last_send_at = time.monotonic()
            with self._lock:
                if self._gate_epoch == frame.gate_epoch:
                    self._applied_commands = applied
                    self._last_command_at = frame.queued_at
                    self._applied_source_timestamp_ns = frame.source_timestamp_ns
                    self._applied_frame_id = frame.frame_id
            recovery_position = applied
            if np.allclose(applied, frame.joint_positions, atol=1e-6):
                if not frame.is_return_to_default_command:
                    recovering = False
                active_frame = None

    def get_data(self) -> dict:
        """Return consistent copies of measured state and actually applied commands."""

        if self._error is not None:
            raise RuntimeError("CASIA Hand runtime is unavailable") from self._error
        now = time.monotonic()
        with self._lock:
            enabled = self._enabled
            connected = self._connected
            connection_state = self._connection_state
            connection_generation = self._connection_generation
            reconnect_attempts = self._reconnect_attempts
            last_error = self._last_error
            command_age = None if self._last_command_at is None else now - self._last_command_at
            state_age = None if self._last_joint_state_at is None else now - self._last_joint_state_at
            measured_positions = self._measured_joint_positions.copy()
            applied_commands = self._applied_commands.copy()
            applied_source_timestamp_ns = self._applied_source_timestamp_ns
            applied_frame_id = self._applied_frame_id

        command_fresh = enabled and command_age is not None and command_age <= self.cfg.command_timeout_s
        joint_state_fresh = connected and state_age is not None and state_age <= self.cfg.joint_state_timeout_s
        return {
            "connected": connected,
            "connection_state": connection_state,
            "connection_generation": connection_generation,
            "reconnect_attempts": reconnect_attempts,
            "last_error": last_error,
            "joint_state_age_s": state_age,
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
