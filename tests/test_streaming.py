from __future__ import annotations

from dataclasses import replace
import time
import unittest

from soarm_sdk import (
    ConfigurationError,
    LimitViolation,
    MotionError,
    SOARM,
    SOARMConfig,
    default_config_path,
)


def _wait_for(predicate, *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for condition")


def _mock_arm_with_joint_limits(*, max_vel: float, max_acc: float) -> SOARM:
    config = SOARMConfig.from_file(default_config_path())
    joints = {
        name: replace(joint, max_vel_rad_s=max_vel, max_acc_rad_s2=max_acc)
        for name, joint in config.joints.items()
    }
    return SOARM.mock(replace(config, joints=joints))


class JointStreamingControllerTest(unittest.TestCase):
    def test_blocking_move_allows_interpolated_target_larger_than_max_step(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        try:
            arm.move_joints({"shoulder_pan": 0.8}, duration=1.1, wait=True)
            position = arm.motion.read_positions_rad()["shoulder_pan"]
            self.assertAlmostEqual(position, 0.8, delta=0.02)
        finally:
            arm.disconnect()

    def test_single_setpoint_still_rejects_target_larger_than_max_step(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        try:
            with self.assertRaises(LimitViolation):
                arm.move_joints({"shoulder_pan": 0.8}, duration=1.1, wait=False)
        finally:
            arm.disconnect()

    def test_moves_toward_latest_target_with_mock_bus(self) -> None:
        arm = _mock_arm_with_joint_limits(max_vel=1.5, max_acc=3.0)
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
        arm = _mock_arm_with_joint_limits(max_vel=1.5, max_acc=3.0)
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

    def test_rejects_removed_tracking_mode(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        try:
            with self.assertRaises(MotionError):
                arm.start_joint_stream(mode="tracking")
        finally:
            arm.disconnect()

    def test_direct_mode_outputs_latest_target_without_soft_motion_lag(self) -> None:
        arm = SOARM.mock()
        arm.connect()
        stream = arm.start_joint_stream(
            output_hz=100,
            target_timeout_s=0.5,
            mode="direct",
        )
        try:
            stream.update_target({"shoulder_pan": 0.4})
            _wait_for(lambda: stream.snapshot().writes >= 1)
            snapshot = stream.snapshot()
            self.assertEqual(snapshot.mode, "direct")
            self.assertAlmostEqual(snapshot.output["shoulder_pan"], 0.4, delta=1e-9)
            self.assertAlmostEqual(snapshot.velocities["shoulder_pan"], 0.0, delta=1e-9)
            self.assertIsNone(snapshot.error)
        finally:
            stream.stop()
            arm.disconnect()


if __name__ == "__main__":
    unittest.main()
