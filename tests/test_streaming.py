from __future__ import annotations

import time
import unittest

from soarm_sdk import ConfigurationError, SOARM


def _wait_for(predicate, *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for condition")


class JointStreamingControllerTest(unittest.TestCase):
    def test_moves_toward_latest_target_with_mock_bus(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        stream = arm.start_joint_stream(output_hz=100, target_timeout_s=0.5)
        try:
            stream.update_target({"shoulder_pan": 0.3})
            _wait_for(lambda: stream.snapshot().writes >= 8)
            snapshot = stream.snapshot()
            self.assertGreater(snapshot.output["shoulder_pan"], 0.0)
            self.assertLessEqual(snapshot.output["shoulder_pan"], 0.3)
            self.assertGreater(snapshot.velocities["shoulder_pan"], 0.0)
            self.assertIsNone(snapshot.error)
        finally:
            stream.stop()
            arm.disconnect()

    def test_holds_when_target_goes_stale(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        stream = arm.start_joint_stream(output_hz=80, target_timeout_s=0.05)
        try:
            stream.update_target({"shoulder_pan": 0.2})
            _wait_for(lambda: stream.snapshot().writes >= 3)
            _wait_for(lambda: stream.snapshot().stale, timeout=0.5)
            before = stream.snapshot()
            time.sleep(0.08)
            after = stream.snapshot()
            self.assertTrue(after.stale)
            self.assertAlmostEqual(
                after.output["shoulder_pan"],
                before.output["shoulder_pan"],
                delta=0.02,
            )
        finally:
            stream.stop()
            arm.disconnect()

    def test_rejects_unknown_joints(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        stream = arm.start_joint_stream(output_hz=50)
        try:
            with self.assertRaises(ConfigurationError):
                stream.update_target({"not_a_joint": 0.1})
        finally:
            stream.stop()
            arm.disconnect()


if __name__ == "__main__":
    unittest.main()
