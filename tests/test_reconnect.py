"""Exercise hot-unplug recovery without opening a real serial device."""

import queue
import threading
import time
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import numpy as np

from casiahand_sdk import CasiaHandConfig, CasiaHandRuntime
from casiahand_sdk.serial_port import SerialPortResolver, UsbSerialDevice, discover_usb_serial


class StreamingHand:
    def __init__(self, position=0.2):
        self.position = np.full(20, position, dtype=np.float32)
        self.online = True
        self.transport_failed = False
        self.samples = queue.Queue()
        self.commands = []
        self.closed = False

    def try_get_joint_sample(self):
        try:
            return self.samples.get_nowait()
        except queue.Empty:
            if self.online:
                return self.position.copy(), time.monotonic()
            return None

    def set_joint_positions(self, positions):
        if self.closed:
            raise AssertionError("used a closed native instance")
        self.commands.append((time.monotonic(), np.asarray(positions)))
        return positions

    def close(self):
        self.closed = True


class TestReconnect(unittest.TestCase):
    def cfg(self, **overrides):
        options = dict(
            auto_reconnect=True,
            joint_state_fps=200,
            joint_state_timeout_s=0.03,
            reconnect_timeout_s=0.07,
            reconnect_interval_s=0.02,
            startup_timeout_s=0.1,
        )
        options.update(overrides)
        return CasiaHandConfig(**options)

    def wait_for(self, predicate, timeout=1.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.002)
        self.fail("condition did not become true before deadline")

    def test_present_device_qualifies_before_constructor_returns(self):
        hand = StreamingHand()
        runtime = CasiaHandRuntime(self.cfg(), hand_factory=lambda _: hand)
        try:
            self.assertTrue(runtime.get_data()["connected"])
            self.assertTrue(runtime.get_data()["joint_state_fresh"])
        finally:
            runtime.close()

    def test_initialization_does_not_consume_feedback_qualification_budget(self):
        class SlowSamplingHand(StreamingHand):
            def try_get_joint_sample(self):
                time.sleep(0.01)
                return super().try_get_joint_sample()

        hand = SlowSamplingHand()
        calls = []

        def factory(cfg):
            calls.append(True)
            time.sleep(0.035)
            return hand

        runtime = CasiaHandRuntime(self.cfg(startup_timeout_s=0.05), hand_factory=factory)
        try:
            self.wait_for(lambda: runtime.get_data()["connected"])
            self.assertEqual(len(calls), 1)
            self.assertFalse(hand.closed)
        finally:
            runtime.close()

    def test_delayed_python_read_rechecks_latest_feedback_before_reconnecting(self):
        class DelayedReadHand(StreamingHand):
            pause_once = False

            def try_get_joint_sample(self):
                sample = super().try_get_joint_sample()
                if self.pause_once:
                    self.pause_once = False
                    time.sleep(0.12)
                    delayed.set()
                return sample

        delayed = threading.Event()
        hand = DelayedReadHand()
        calls = []

        def factory(cfg):
            calls.append(True)
            return hand

        runtime = CasiaHandRuntime(self.cfg(), hand_factory=factory)
        try:
            self.assertTrue(runtime.get_data()["connected"])
            hand.pause_once = True
            self.assertTrue(delayed.wait(1.0))
            self.wait_for(lambda: runtime.get_data()["joint_state_fresh"])
            self.assertEqual(len(calls), 1)
            self.assertEqual(runtime.get_data()["connection_generation"], 1)
            self.assertFalse(hand.closed)
        finally:
            runtime.close()

    def test_missing_at_startup_retries_without_blocking_or_raising(self):
        allow_connect = threading.Event()
        hand = StreamingHand()
        attempts = []

        def factory(cfg):
            attempts.append(time.monotonic())
            if not allow_connect.is_set():
                raise ConnectionError("USB absent")
            return hand

        runtime = CasiaHandRuntime(self.cfg(), hand_factory=factory)
        try:
            self.wait_for(lambda: len(attempts) >= 2)
            self.assertFalse(runtime.get_data()["connected"])
            self.assertIn("USB absent", runtime.get_data()["last_error"])
            runtime.set_takeover_enabled(True)
            runtime.set_joint_commands(np.ones(10), np.ones(10))
            allow_connect.set()
            self.wait_for(lambda: runtime.get_data()["connected"])
            self.assertTrue(runtime.get_data()["enabled"])
            self.assertEqual(hand.commands, [])
        finally:
            runtime.close()
        self.assertTrue(hand.closed)

    def test_feedback_timeout_closes_before_reopening_and_discards_old_commands(self):
        first, second = StreamingHand(), StreamingHand(0.4)
        allow_reopen = threading.Event()
        calls = []

        def factory(cfg):
            calls.append(True)
            if len(calls) == 1:
                return first
            self.assertTrue(first.closed)
            if not allow_reopen.is_set():
                raise ConnectionError("power still absent")
            return second

        runtime = CasiaHandRuntime(self.cfg(), hand_factory=factory)
        try:
            self.wait_for(lambda: runtime.get_data()["connected"])
            runtime.set_takeover_enabled(True)
            runtime.set_joint_commands(np.full(10, 0.2), np.full(10, 0.2), frame_id=1)
            self.wait_for(lambda: bool(first.commands))
            first.online = False
            self.wait_for(lambda: first.closed)
            self.assertFalse(runtime.get_data()["joint_state_fresh"])
            self.assertIsNone(runtime.get_data()["applied_frame_id"])
            runtime.set_joint_commands(np.ones(10), np.ones(10), frame_id=2)
            allow_reopen.set()
            self.wait_for(lambda: runtime.get_data()["connection_generation"] == 2)
            self.assertEqual(second.commands, [])
            self.assertIsNone(runtime.get_data()["applied_frame_id"])
            runtime.set_joint_commands(np.full(10, 0.6), np.full(10, 0.6), frame_id=3)
            self.wait_for(lambda: bool(second.commands))
            _, first_target = second.commands[0]
            self.assertTrue(np.all(first_target >= 0.4 - 1e-6))
            self.assertTrue(np.all(first_target <= 0.411))
            self.wait_for(lambda: np.allclose(second.commands[-1][1], 0.6, atol=1e-5))
            for (prev_at, prev), (at, target) in zip(second.commands, second.commands[1:], strict=False):
                self.assertLessEqual(np.max(np.abs(target - prev)), min(at - prev_at + 0.004, 0.011))
        finally:
            runtime.close()
        self.assertTrue(second.closed)

    def test_disabling_offline_does_not_replay_default_after_reconnect(self):
        first, second = StreamingHand(), StreamingHand(0.4)
        runtime = CasiaHandRuntime(self.cfg(), hand_factory=lambda _: second if first.closed else first)
        try:
            self.wait_for(lambda: runtime.get_data()["connected"])
            runtime.set_takeover_enabled(True)
            first.online = False
            self.wait_for(lambda: not runtime.get_data()["joint_state_fresh"])
            runtime.set_takeover_enabled(False, return_to_default=True)
            self.wait_for(lambda: runtime.get_data()["connection_generation"] == 2)
            runtime.set_joint_commands(np.ones(10), np.ones(10))
            time.sleep(0.04)
            self.assertFalse(runtime.get_data()["enabled"])
            self.assertEqual(second.commands, [])
        finally:
            runtime.close()

    def test_transport_exception_recovers_and_close_interrupts_retry_wait(self):
        first = StreamingHand()
        runtime = CasiaHandRuntime(self.cfg(reconnect_interval_s=10), hand_factory=lambda _: first)
        self.wait_for(lambda: runtime.get_data()["connected"])
        first.transport_failed = True
        self.wait_for(lambda: first.closed)
        started = time.monotonic()
        runtime.close()
        self.assertLess(time.monotonic() - started, 0.2)
        self.assertEqual(runtime.get_data()["connection_state"], "STOPPED")

    def test_stale_cached_samples_never_qualify_a_connection(self):
        hand = StreamingHand()
        hand.online = False
        for _ in range(10):
            hand.samples.put((hand.position, time.monotonic() - 5))
        runtime = CasiaHandRuntime(self.cfg(reconnect_interval_s=10), hand_factory=lambda _: hand)
        try:
            self.wait_for(lambda: hand.closed)
            self.assertFalse(runtime.get_data()["connected"])
            self.assertEqual(runtime.get_data()["connection_generation"], 0)
        finally:
            runtime.close()

    def test_recovery_stops_advancing_when_command_expires(self):
        hand = StreamingHand(0.0)
        runtime = CasiaHandRuntime(self.cfg(command_timeout_s=0.03), hand_factory=lambda _: hand)
        try:
            self.wait_for(lambda: runtime.get_data()["connected"])
            runtime.set_takeover_enabled(True)
            runtime.set_joint_commands(np.ones(10), np.ones(10))
            self.wait_for(lambda: bool(hand.commands))
            time.sleep(0.06)
            count = len(hand.commands)
            time.sleep(0.04)
            self.assertEqual(count, len(hand.commands))
            self.assertFalse(runtime.get_data()["fresh"])
            self.assertLess(np.max(hand.commands[-1][1]), 0.05)
        finally:
            runtime.close()

    def test_failed_release_is_fatal_and_never_opens_a_second_owner(self):
        hand = StreamingHand()
        hand.transport_failed = True
        calls = []

        def factory(cfg):
            calls.append(True)
            return hand

        def fail_close():
            raise RuntimeError("serial release failed")

        hand.close = fail_close
        runtime = CasiaHandRuntime(self.cfg(), hand_factory=factory)
        try:
            self.wait_for(lambda: runtime._error is not None)
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                runtime.get_data()
            self.assertEqual(len(calls), 1)
        finally:
            runtime.close()

    def test_only_three_distinct_recent_samples_qualify_connection(self):
        hand = StreamingHand()
        hand.online = False
        stamp = time.monotonic()
        for _ in range(3):
            hand.samples.put((hand.position, stamp))
        runtime = CasiaHandRuntime(self.cfg(startup_timeout_s=0.5), hand_factory=lambda _: hand)
        try:
            time.sleep(0.02)
            self.assertFalse(runtime.get_data()["connected"])
            hand.online = True
            self.wait_for(lambda: runtime.get_data()["connected"])
            self.assertEqual(hand.commands, [])
        finally:
            runtime.close()


class TestSerialIdentity(unittest.TestCase):
    def device(self, name="ttyCH341USB0", serial=None, topology="/sys/usb/1-2"):
        return UsbSerialDevice(f"/dev/{name}", "1a86", "5523", serial, topology)

    def test_renumbering_matches_original_physical_port(self):
        resolver = SerialPortResolver("/dev/ttyCH341USB0")
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=[self.device()]):
            self.assertEqual(resolver.resolve(), "/dev/ttyCH341USB0")
        moved = self.device("ttyCH341USB1")
        wrong = self.device("ttyCH341USB0", topology="/sys/usb/1-3")
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=[wrong, moved]):
            self.assertEqual(resolver.resolve(), moved.port)
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=[wrong]):
            with self.assertRaises(ConnectionError):
                resolver.resolve()

    def test_unique_serial_tracks_device_and_duplicate_identity_is_rejected(self):
        resolver = SerialPortResolver("/dev/ttyCH341USB0")
        first = self.device(serial="hand-adapter")
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=[first]):
            resolver.resolve()
        moved = self.device("ttyCH341USB1", serial="hand-adapter", topology="/sys/usb/1-4")
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=[moved]):
            self.assertEqual(resolver.resolve(), moved.port)
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=[first, moved]):
            with self.assertRaises(ConnectionError):
                resolver.resolve()

    def test_initial_ambiguity_waits_instead_of_picking_an_adapter(self):
        resolver = SerialPortResolver("/dev/absent-casia")
        devices = [self.device(), self.device("ttyCH341USB1", topology="/sys/usb/1-3")]
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=devices):
            with self.assertRaises(ConnectionError):
                resolver.resolve()
        self.assertIsNone(resolver.identity)
        with patch("casiahand_sdk.serial_port.discover_usb_serial", return_value=devices[1:]):
            self.assertEqual(resolver.resolve(), devices[1].port)

    def test_discovers_usb_ancestor_and_optional_serial_from_sysfs(self):
        with TemporaryDirectory() as root:
            base = Path(root)
            usb = base / "devices" / "usb1" / "1-2"
            interface = usb / "1-2:1.0"
            interface.mkdir(parents=True)
            (usb / "idVendor").write_text("1a86\n")
            (usb / "idProduct").write_text("5523\n")
            (usb / "serial").write_text("adapter\n")
            tty = base / "class" / "tty" / "ttyCH341USB1"
            tty.mkdir(parents=True)
            (tty / "device").symlink_to(interface)
            devices = discover_usb_serial(base / "class" / "tty")
            self.assertEqual(devices, [self.device("ttyCH341USB1", "adapter", str(usb))])


if __name__ == "__main__":
    unittest.main()
