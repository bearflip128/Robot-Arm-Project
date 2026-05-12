from __future__ import annotations

import unittest

from app.control.controller_mapping import PS4ControllerMapper, PS4Snapshot
from app.control.input_adapters import PS4InputAdapter, PlaybackInputAdapter, UIInputAdapter
from app.control.joint_config import JointRuntimeConfig
from app.control.motion_filter import MotionFilter
from app.control.runtime import RobotControlRuntime
from app.control.state_store import JointStateStore
from app.control.verification import build_verification_report
from app.robot_arm import JointDefinition, RobotArmAdapter


def make_joint(
    name: str,
    *,
    minimum: float = -100,
    maximum: float = 100,
    default: float = 0,
    controller_binding=None,
    max_delta: float | None = None,
    max_rate: float | None = None,
) -> JointRuntimeConfig:
    return JointRuntimeConfig(
        name=name,
        label=name,
        servo_id=1,
        minimum=minimum,
        maximum=maximum,
        default=default,
        neutral=default,
        default_start=default,
        inverted=False,
        enabled=True,
        semantic_negative_label="Min",
        semantic_positive_label="Max",
        notes="",
        controller_mapping={},
        controller_binding=controller_binding,
        max_delta_per_command=max_delta,
        max_degrees_per_second=max_rate,
        controller_max_degrees_per_second=max_rate,
    )


class ControllerMappingTests(unittest.TestCase):
    def test_trigger_pair_maps_to_single_joint(self) -> None:
        from app.control.joint_config import ControllerBinding

        mapper = PS4ControllerMapper(
            {
                "shoulder_pan": make_joint(
                    "shoulder_pan",
                    controller_binding=ControllerBinding(
                        kind="trigger_pair",
                        negative_trigger="l2",
                        positive_trigger="r2",
                        deadzone=0.0,
                        curve=1.0,
                    ),
                )
            }
        )

        intents = mapper.map_snapshot(
            PS4Snapshot(axes={}, buttons={"l2": 0.2, "r2": 0.8})
        )
        self.assertAlmostEqual(intents["shoulder_pan"], 0.6)

    def test_axis_deadzone_is_applied(self) -> None:
        from app.control.joint_config import ControllerBinding

        mapper = PS4ControllerMapper(
            {
                "wrist_roll": make_joint(
                    "wrist_roll",
                    controller_binding=ControllerBinding(
                        kind="axis",
                        axis="left_x",
                        deadzone=0.2,
                        curve=1.0,
                    ),
                )
            }
        )

        intents = mapper.map_snapshot(
            PS4Snapshot(axes={"left_x": 0.1}, buttons={})
        )
        self.assertEqual(intents, {})


class MotionFilterTests(unittest.TestCase):
    def test_clamps_and_rate_limits(self) -> None:
        filt = MotionFilter(
            {
                "joint": make_joint(
                    "joint",
                    minimum=-10,
                    maximum=10,
                    max_delta=4,
                    max_rate=20,
                )
            }
        )
        result = filt.step({"joint": 10}, {"joint": 0}, dt=0.1)
        self.assertAlmostEqual(result["joint"], 2.0)


class JointStateStoreTests(unittest.TestCase):
    def test_store_separates_desired_commanded_observed(self) -> None:
        store = JointStateStore({"joint": make_joint("joint", minimum=-5, maximum=5)})
        store.set_desired({"joint": 4})
        store.set_commanded({"joint": 2})
        store.set_observed({"joint": 1})
        store.set_filtered({"joint": 3})
        snap = store.snapshot()
        self.assertEqual(snap.desired["joint"], 4)
        self.assertEqual(snap.commanded["joint"], 2)
        self.assertEqual(snap.observed["joint"], 1)
        self.assertEqual(snap.filtered["joint"], 3)


class InputAdapterTests(unittest.TestCase):
    def test_ui_adapter_emits_absolute_targets(self) -> None:
        command = UIInputAdapter().absolute_targets({"joint": 5}, source="slider")
        self.assertEqual(command.targets["joint"], 5)
        self.assertEqual(command.source, "slider")

    def test_ps4_adapter_maps_snapshot(self) -> None:
        from app.control.joint_config import ControllerBinding

        adapter = PS4InputAdapter(
            PS4ControllerMapper(
                {
                    "joint": make_joint(
                        "joint",
                        controller_binding=ControllerBinding(
                            kind="axis",
                            axis="left_x",
                            deadzone=0.0,
                            curve=1.0,
                        ),
                    )
                }
            )
        )
        command, intents = adapter.from_snapshot({"left_x": 0.5}, {}, source="PS4")
        self.assertEqual(command.axes["left_x"], 0.5)
        self.assertAlmostEqual(intents["joint"], 0.5)

    def test_playback_adapter_emits_frame(self) -> None:
        frame = PlaybackInputAdapter().frame({"joint": 2}, 1.25)
        self.assertEqual(frame.joints["joint"], 2)
        self.assertEqual(frame.timestamp_s, 1.25)


class VerificationTests(unittest.TestCase):
    def test_verification_report_counts_flags(self) -> None:
        report = build_verification_report(
            state={"connected": True},
            diagnostics={},
            joint_meta={"joint": {"servo_id": 1, "label_for_ui": "Joint", "controller_mapping": {}}},
            joint_debug={
                "joint": {
                    "actual_joint": 0,
                    "commanded_joint": 1,
                    "desired_joint": 2,
                    "filtered_joint": 1.5,
                    "raw_step": 123,
                    "torque_enabled": True,
                    "truth_confidence": "high",
                    "active_source": "PS4",
                    "target_mismatch": True,
                    "unreadable": False,
                    "rate_limited": True,
                    "clamped_by_limit": False,
                    "last_error": None,
                }
            },
            limits={"joint": {"min": -1, "max": 1, "default": 0}},
        )
        self.assertEqual(report["summary"]["tracking_gaps"], 1)
        self.assertEqual(report["summary"]["rate_limited"], 1)


class FakeAdapter(RobotArmAdapter):
    def __init__(self, joint_definitions):
        super().__init__(joint_definitions)
        self.joints = {name: definition.default for name, definition in joint_definitions.items()}

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        return None

    def disable_torque(self) -> None:
        return None

    def enable_torque(self) -> None:
        return None

    def get_state(self):
        return {
            "connected": True,
            "joints": dict(self.joints),
            "commanded_joints": dict(self.joints),
            "hardware_error": None,
        }

    def move_joints(self, targets):
        self.joints.update(targets)
        return dict(self.joints)

    def set_joint_torque(self, joint_name: str, enabled: bool) -> None:
        return None

    def get_joint_raw_step(self, joint_name: str) -> int:
        return 0


class RuntimeTests(unittest.TestCase):
    def test_runtime_clamps_targets_and_tracks_safety_state(self) -> None:
        joint_definitions = {"joint": JointDefinition("joint", -5, 5, 0)}
        joint_configs = {"joint": make_joint("joint", minimum=-5, maximum=5, default=0, max_delta=2, max_rate=20)}
        runtime = RobotControlRuntime(
            adapter=FakeAdapter(joint_definitions),
            joint_definitions=joint_definitions,
            joint_configs=joint_configs,
            state_store=JointStateStore(joint_configs),
            motion_filter=MotionFilter(joint_configs),
            config={"robot": {"teleop": {"intent_timeout_s": 0.3, "default_controller_max_degrees_per_second": 50}}},
            loop_hz=20.0,
        )
        desired = runtime.set_desired_targets({"joint": 12}, source="slider")
        self.assertEqual(desired["joint"], 5)
        self.assertFalse(runtime.motion_allowed())
        runtime.arm()
        self.assertTrue(runtime.motion_allowed())
        runtime.disarm(reason="Testing disarm")
        self.assertFalse(runtime.motion_allowed())
        self.assertEqual(runtime.safety_snapshot().lock_reason, "Testing disarm")


if __name__ == "__main__":
    unittest.main()
