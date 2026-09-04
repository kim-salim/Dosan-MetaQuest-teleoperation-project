#!/usr/bin/env python3
"""Guarded preflight for the A0509 LeRobot shadow-record path."""

from __future__ import annotations

import argparse
import json

from lerobot_robot_doosan_a0509.config_doosan_a0509_ros import (
    DoosanA0509RosConfig,
)
from lerobot_robot_doosan_a0509.config_metaquest_a0509 import (
    MetaQuestA0509Config,
)
from lerobot_robot_doosan_a0509.doosan_a0509_ros import DoosanA0509Ros
from lerobot_robot_doosan_a0509.metaquest_a0509 import MetaQuestA0509
from lerobot_robot_doosan_a0509.recording_gripper_initializer import (
    ensure_recording_gripper_state,
)


def main(
    *,
    skip_cameras: bool = False,
    connect_timeout_sec: float = 10.0,
    allow_unprepared_teleop: bool = False,
    initial_gripper_state: str = "none",
    gripper_initialize_timeout_sec: float = 10.0,
) -> None:
    initial_gripper_state = str(initial_gripper_state).strip().lower()
    if initial_gripper_state not in {"open", "close", "none"}:
        raise ValueError("initial_gripper_state must be open, close, or none")
    gripper_initialization = "unchanged"
    if initial_gripper_state != "none":
        gripper_initialization = ensure_recording_gripper_state(
            initial_gripper_state,
            timeout_sec=gripper_initialize_timeout_sec,
        )

    robot_config = DoosanA0509RosConfig(
        id="shadow_record_preflight",
        mode="shadow_record",
        connect_timeout_sec=connect_timeout_sec,
    )
    if skip_cameras:
        robot_config.cameras = {}
        robot_config.require_camera = False
    robot = DoosanA0509Ros(robot_config)
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="shadow_record_preflight",
            allow_latched_state_in_shadow_record=True,
            require_fresh_action_on_connect=not allow_unprepared_teleop,
            defer_calibration_on_connect=allow_unprepared_teleop,
            connect_timeout_sec=connect_timeout_sec,
        )
    )
    robot_connected = False
    teleop_connected = False
    try:
        robot.connect()
        robot_connected = True
        teleop.connect()
        teleop_connected = True

        observation = robot.get_observation()
        if allow_unprepared_teleop:
            action_keys = sorted(teleop.action_features)
            teleop_status = "awaiting_episode_reset"
        else:
            action = teleop.get_action()
            returned_action = robot.send_action(action)
            if returned_action != action:
                raise RuntimeError("shadow_record changed the teacher action")
            action_keys = sorted(action)
            teleop_status = "ready"
        if robot.live_publish_count != 0 or robot.debug_publish_count != 0:
            raise RuntimeError("shadow_record unexpectedly published an action")

        scalar_observation_keys = [
            key for key in observation if key not in robot.cameras
        ]
        summary = {
            "action_keys": action_keys,
            "camera_shapes": {
                key: list(observation[key].shape) for key in robot.cameras
            },
            "debug_publish_count": robot.debug_publish_count,
            "gripper_initialization": gripper_initialization,
            "live_publish_count": robot.live_publish_count,
            "mode": robot.config.mode,
            "observation_scalar_count": len(scalar_observation_keys),
            "status": teleop_status,
        }
        print(json.dumps(summary, indent=2, sort_keys=True))
    finally:
        if teleop_connected:
            teleop.disconnect()
        if robot_connected:
            robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--skip-cameras",
        action="store_true",
        help="Check ROS state/action freshness without opening camera devices.",
    )
    parser.add_argument(
        "--connect-timeout-sec",
        type=float,
        default=10.0,
        help="Maximum ROS state discovery wait for the robot and teleoperator.",
    )
    parser.add_argument(
        "--allow-unprepared-teleop",
        action="store_true",
        help=(
            "Defer calibration and teleop/action freshness to the episode reset "
            "orchestrator."
        ),
    )
    parser.add_argument(
        "--initial-gripper-state",
        choices=("open", "close", "none"),
        default="none",
        help=(
            "Physically initialize and verify the JRT gripper state before "
            "connecting the recording adapter."
        ),
    )
    parser.add_argument(
        "--gripper-initialize-timeout-sec",
        type=float,
        default=10.0,
        help="Maximum wait for fresh accepted/completed gripper confirmation.",
    )
    args = parser.parse_args()
    main(
        skip_cameras=args.skip_cameras,
        connect_timeout_sec=args.connect_timeout_sec,
        allow_unprepared_teleop=args.allow_unprepared_teleop,
        initial_gripper_state=args.initial_gripper_state,
        gripper_initialize_timeout_sec=args.gripper_initialize_timeout_sec,
    )
