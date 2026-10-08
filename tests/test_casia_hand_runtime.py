import queue
import threading
import time
import unittest

import numpy as np
from casiahand_sdk import (
    CASIA_JOINT_NAMES,
    CASIA_LEFT_LIMITS,
    CASIA_RIGHT_LIMITS,
    CasiaHandConfig,
    CasiaHandRuntime,
)


class FakeCasiaHand:
    def __init__(self):
        self.commands = []
        self.samples = queue.Queue()
        self.closed = False

    def set_joint_positions(self, positions):
        command = np.asarray(positions, dtype=np.float32)
        self.commands.append(command)
        return command

    def try_get_joint_positions(self):
        try:
            return self.samples.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


class TestCasiaHandConfig(unittest.TestCase):
    def test_rejects_invalid_serial_configuration(self):
        with self.assertRaisesRegex(ValueError, "port_name"):
            CasiaHandConfig(port_name=" ")
        with self.assertRaisesRegex(ValueError, "must differ"):
            CasiaHandConfig(left_hand_id=2, right_hand_id=2)


class TestCasiaHandRuntime(unittest.TestCase):
    def test_reports_measured_state_separately_from_applied_action(self):
        hand = FakeCasiaHand()
        runtime = CasiaHandRuntime(CasiaHandConfig(joint_state_fps=200.0), hand_factory=lambda _: hand)
        try:
            runtime.set_takeover_enabled(True)
            expected_action = runtime.set_joint_commands(
                np.full(10, 0.2),
                np.full(10, 0.3),
                source_timestamp_ns=123,
                frame_id=9,
            )
            measured = np.linspace(0.01, 0.2, 20, dtype=np.float32)
            hand.samples.put(measured)

            deadline = time.monotonic() + 1.0
            data = runtime.get_data()
            while (data["applied_frame_id"] != 9 or not data["joint_state_fresh"]) and time.monotonic() < deadline:
                time.sleep(0.005)
                data = runtime.get_data()

            self.assertEqual(data["joint_names"], list(CASIA_JOINT_NAMES))
            self.assertEqual(data["applied_source_timestamp_ns"], 123)
            self.assertEqual(data["applied_frame_id"], 9)
            self.assertTrue(data["fresh"])
            np.testing.assert_allclose(data["joint_positions"], measured)
            np.testing.assert_allclose(data["joint_position_commands"], expected_action)
            self.assertFalse(np.array_equal(data["joint_positions"], data["joint_position_commands"]))
        finally:
            runtime.close()
        self.assertTrue(hand.closed)

    def test_command_queue_keeps_only_newest_clipped_atomic_frame(self):
        runtime = CasiaHandRuntime.__new__(CasiaHandRuntime)
        runtime._command_queue = queue.Queue(maxsize=1)
        runtime._lock = threading.Lock()
        runtime._auto_reconnect = False
        runtime._connection_generation = 0
        runtime._gate_epoch = 0

        first = runtime.set_joint_commands(np.full(10, 100.0), np.full(10, -100.0), 10, 1)
        second = runtime.set_joint_commands(np.zeros(10), np.zeros(10), 20, 2)

        np.testing.assert_allclose(first[:10], CASIA_LEFT_LIMITS[:, 1])
        np.testing.assert_allclose(first[10:], CASIA_RIGHT_LIMITS[:, 0])
        frame = runtime._command_queue.get_nowait()
        self.assertEqual(frame.frame_id, 2)
        self.assertEqual(frame.source_timestamp_ns, 20)
        np.testing.assert_array_equal(second, np.zeros(20))

    def test_takeover_gate_does_not_apply_disabled_commands(self):
        hand = FakeCasiaHand()
        runtime = CasiaHandRuntime(CasiaHandConfig(joint_state_fps=100.0), hand_factory=lambda _: hand)
        try:
            runtime.set_joint_commands(np.full(10, 0.2), np.full(10, 0.3))
            time.sleep(0.03)
            self.assertEqual(hand.commands, [])
            self.assertFalse(runtime.get_data()["fresh"])
        finally:
            runtime.close()

    def test_disabling_takeover_can_return_both_hands_to_default(self):
        hand = FakeCasiaHand()
        runtime = CasiaHandRuntime(CasiaHandConfig(joint_state_fps=100.0), hand_factory=lambda _: hand)
        try:
            runtime.set_takeover_enabled(True)
            runtime.set_joint_commands(np.full(10, 0.2), np.full(10, 0.3))
            deadline = time.monotonic() + 1.0
            while len(hand.commands) < 1 and time.monotonic() < deadline:
                time.sleep(0.005)

            runtime.set_takeover_enabled(False, return_to_default=True)
            deadline = time.monotonic() + 1.0
            while len(hand.commands) < 2 and time.monotonic() < deadline:
                time.sleep(0.005)

            self.assertEqual(len(hand.commands), 2)
            np.testing.assert_array_equal(hand.commands[-1], np.zeros(20, dtype=np.float32))
            self.assertFalse(runtime.get_data()["fresh"])
        finally:
            runtime.close()

    def test_state_becomes_stale_without_a_new_hardware_sample(self):
        hand = FakeCasiaHand()
        hand.samples.put(np.full(20, 0.1, dtype=np.float32))
        cfg = CasiaHandConfig(joint_state_fps=200.0, joint_state_timeout_s=0.02)
        runtime = CasiaHandRuntime(cfg, hand_factory=lambda _: hand)
        try:
            deadline = time.monotonic() + 0.5
            while not runtime.get_data()["joint_state_fresh"] and time.monotonic() < deadline:
                time.sleep(0.002)
            self.assertTrue(runtime.get_data()["joint_state_fresh"])
            time.sleep(0.04)
            self.assertFalse(runtime.get_data()["joint_state_fresh"])
        finally:
            runtime.close()

    def test_rejects_invalid_commands(self):
        runtime = CasiaHandRuntime.__new__(CasiaHandRuntime)
        runtime._command_queue = queue.Queue(maxsize=1)
        with self.assertRaisesRegex(ValueError, "shape"):
            runtime.set_joint_commands(np.zeros(9), np.zeros(10))
        with self.assertRaisesRegex(ValueError, "finite"):
            runtime.set_joint_commands(np.full(10, np.nan), np.zeros(10))


if __name__ == "__main__":
    unittest.main()
