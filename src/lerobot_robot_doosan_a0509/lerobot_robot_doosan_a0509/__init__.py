"""LeRobot 0.6 plugin registrations for the ROS 2 Doosan A0509 stack."""

try:
    from lerobot_robot_doosan_a0509.config_doosan_a0509_ros import DoosanA0509RosConfig
    from lerobot_robot_doosan_a0509.config_metaquest_a0509 import MetaQuestA0509Config
    from lerobot_robot_doosan_a0509.config_zed_left_camera import ZedLeftCameraConfig
    from lerobot_robot_doosan_a0509.doosan_a0509_ros import DoosanA0509Ros
    from lerobot_robot_doosan_a0509.metaquest_a0509 import (
        MetaQuestA0509,
        MetaQuestA0509Teleoperator,
    )
    from lerobot_robot_doosan_a0509.zed_left_camera import ZedLeftCamera
except ModuleNotFoundError as exc:
    if exc.name != "lerobot":
        raise
    __all__: list[str] = []
else:
    __all__ = [
        "DoosanA0509Ros",
        "DoosanA0509RosConfig",
        "MetaQuestA0509",
        "MetaQuestA0509Config",
        "MetaQuestA0509Teleoperator",
        "ZedLeftCamera",
        "ZedLeftCameraConfig",
    ]
