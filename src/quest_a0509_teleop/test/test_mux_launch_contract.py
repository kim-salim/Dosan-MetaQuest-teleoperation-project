from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_full_bringup_starts_exactly_one_mux_and_defaults_disabled():
    full = (PACKAGE_ROOT / "launch" / "a0509_full_bringup.launch.py").read_text()
    wrapped = (
        PACKAGE_ROOT / "launch" / "a0509_full_bringup_with_gripper.launch.py"
    ).read_text()
    assert full.count('executable="a0509_command_mux_node"') == 1
    assert 'executable="a0509_command_mux_node"' not in wrapped
    assert 'DeclareLaunchArgument("use_command_mux", default_value="true")' in full
    assert 'DeclareLaunchArgument("initial_control_source", default_value="DISABLED")' in full
    assert 'DeclareLaunchArgument("dry_run", default_value="true")' in full


def test_mux_mapper_and_gripper_topics_are_source_specific():
    full = (PACKAGE_ROOT / "launch" / "a0509_full_bringup.launch.py").read_text()
    wrapped = (
        PACKAGE_ROOT / "launch" / "a0509_full_bringup_with_gripper.launch.py"
    ).read_text()
    gripper = (
        PACKAGE_ROOT.parent / "jrt_gripper_io" / "launch" / "jrt_gripper_robot_bringup.launch.py"
    ).read_text()
    assert '"target_posx_topic": mapper_target_topic' in full
    assert 'default_value="/control/metaquest/target_posx"' in full
    assert '"mapper_command_topic": gripper_mapper_topic' in wrapped
    assert '"driver_command_topic": gripper_command_topic' in wrapped
    assert '"command_topic": mapper_command_topic' in gripper
    assert '"command_topic": driver_command_topic' in gripper


def test_mux_streamer_watchdog_is_enabled_from_same_launch_switch():
    full = (PACKAGE_ROOT / "launch" / "a0509_full_bringup.launch.py").read_text()
    assert '"require_mux_heartbeat": ParameterValue(' in full
    assert '"selected_heartbeat_topic": selected_heartbeat_topic' in full
    assert '"mux_heartbeat_timeout_sec": ParameterValue(' in full


def test_jazzy_controller_prefix_and_rt_defaults_are_consistent():
    full = (PACKAGE_ROOT / "launch" / "a0509_full_bringup.launch.py").read_text()
    wrapped = (
        PACKAGE_ROOT / "launch" / "a0509_full_bringup_with_gripper.launch.py"
    ).read_text()
    streamer = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "servol_rt_streamer_node.py"
    ).read_text()
    prep = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "robot_prep_node.py"
    ).read_text()
    gripper = (
        PACKAGE_ROOT.parent
        / "jrt_gripper_io"
        / "jrt_gripper_io"
        / "jrt_tool_io_driver_node.py"
    ).read_text()

    expected_servol = "/dsr01/dsr_controller2/servol_rt_stream"
    expected_tool_do = "/dsr01/dsr_controller2/io/set_tool_digital_output"
    assert expected_servol in full
    assert expected_servol in wrapped
    assert expected_servol in streamer
    assert expected_tool_do in wrapped
    assert expected_tool_do in gripper
    assert 'default_value="192.168.137.100"' in full
    assert "self.controller_prefix" in streamer
    assert 'prefix = f"/{self.robot_namespace}/{self.controller_name}"' in prep


def test_metaquest_watchdog_uses_only_mapper_accepted_pose_heartbeat():
    mapper = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "xyz_mapper_node.py"
    ).read_text()
    mux = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "a0509_command_mux_node.py"
    ).read_text()
    config = (PACKAGE_ROOT / "config" / "xyz_position_only.yaml").read_text()
    full = (PACKAGE_ROOT / "launch" / "a0509_full_bringup.launch.py").read_text()
    topic = "/control/metaquest/valid_pose_heartbeat"

    assert topic in mapper
    assert topic in mux
    assert topic in config
    assert topic in full
    assert "self.valid_pose_heartbeat_pub.publish(Empty())" in mapper
    jump_index = mapper.index("jump = self._pose_jump")
    assert mapper.index("self.latest_raw_vr_pose_m = raw_pose_m[:]") < jump_index
    assert jump_index < mapper.index(
        "self.valid_pose_heartbeat_pub.publish(Empty())"
    )
    assert mapper.index("return", jump_index) < mapper.index(
        "self.last_input_time = now"
    )
    assert "PoseStamped" not in mux
    assert "self.core.note_metaquest_valid_pose" in mux


def test_mapper_pose_subscription_keeps_only_the_latest_sample():
    mapper = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "xyz_mapper_node.py"
    ).read_text()
    qos_start = mapper.index("pose_qos = QoSProfile(")
    subscription_start = mapper.index(
        "self.pose_sub = self.create_subscription",
        qos_start,
    )
    subscription_end = mapper.index(
        "self.robot_anchor_sub = self.create_subscription",
        subscription_start,
    )
    pose_path = mapper[qos_start:subscription_end]

    assert "history=HistoryPolicy.KEEP_LAST" in pose_path
    assert "depth=1" in pose_path
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in pose_path
    assert "durability=DurabilityPolicy.VOLATILE" in pose_path
    assert "pose_qos" in mapper[subscription_start:subscription_end]


def test_mapper_resamples_bursty_pose_input_on_the_existing_control_tick():
    mapper = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "xyz_mapper_node.py"
    ).read_text()
    resampler = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "pose_resampler.py"
    ).read_text()
    config = (PACKAGE_ROOT / "config" / "xyz_position_only.yaml").read_text()

    callback_start = mapper.index("def _on_pose")
    callback_end = mapper.index("def _on_robot_anchor_posx", callback_start)
    callback = mapper[callback_start:callback_end]
    tick_start = mapper.index("def _tick")
    tick_end = mapper.index("def _update_control_pose", tick_start)
    tick = mapper[tick_start:tick_end]

    assert "self.pose_resampler.push(" in callback
    assert "self._filtered_pose(" not in callback
    assert "self._update_control_pose(now)" in tick
    assert "query_time = now - self.pose_interpolation_delay_sec" in mapper
    assert "quaternion_slerp(" in resampler
    assert 'mode="interpolate"' in resampler
    assert 'mode = "decay"' in resampler
    assert "enable_pose_resampling: true" in config
    assert "pose_interpolation_delay_sec: 0.05" in config
    assert "pose_full_prediction_sec: 0.02" in config
    assert "pose_prediction_decay_sec: 0.08" in config
    assert "pose_tracking_invalid_after_sec: 0.3" in config
    assert "self.live_state_sub = self.create_subscription" in mapper
    assert "self._on_live_state" in mapper
    assert '\"Live output enabled\"' in mapper


def test_mapper_receives_retained_prepare_state_after_restart():
    prep = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "robot_prep_node.py"
    ).read_text()
    mapper = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "xyz_mapper_node.py"
    ).read_text()
    prep_qos_start = prep.index("state_qos = QoSProfile(")
    prep_qos_end = prep.index("self.recenter_client =", prep_qos_start)
    prep_state_publishers = prep[prep_qos_start:prep_qos_end]
    qos_start = mapper.index("state_qos = QoSProfile(")
    pose_start = mapper.index("pose_qos = QoSProfile(", qos_start)
    state_qos = mapper[qos_start:pose_start]
    subscription_start = mapper.index("self.teleop_ready_sub = self.create_subscription")
    subscription_end = mapper.index("self.recenter_srv =", subscription_start)
    subscription = mapper[subscription_start:subscription_end]
    robot_anchor_start = mapper.index("self.robot_anchor_sub = self.create_subscription")
    robot_anchor_end = mapper.index("self.teleop_ready_sub =", robot_anchor_start)
    robot_anchor_subscription = mapper[robot_anchor_start:robot_anchor_end]

    assert "depth=1" in state_qos
    assert "reliability=ReliabilityPolicy.RELIABLE" in state_qos
    assert "durability=DurabilityPolicy.TRANSIENT_LOCAL" in state_qos
    assert "self.teleop_ready_pub" in prep_state_publishers
    assert "self.robot_anchor_pub" in prep_state_publishers
    assert prep_state_publishers.count("state_qos") >= 3
    assert "state_qos" in subscription
    assert "state_qos" in robot_anchor_subscription


def test_safety_guard_clamps_at_fixed_rate_without_duplicate_ramp():
    safety = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "safety_guard_node.py"
    ).read_text()
    config = (PACKAGE_ROOT / "config" / "xyz_position_only.yaml").read_text()
    on_target_start = safety.index("def _on_target")
    anchor_start = safety.index("def _on_robot_anchor_posx", on_target_start)
    on_target = safety[on_target_start:anchor_start]
    tick_start = safety.index("def _tick", anchor_start)
    publish_event_start = safety.index("def _publish_event", tick_start)
    tick = safety[tick_start:publish_event_start]
    safety_config_start = config.index("safety_guard_node:")
    safety_config_end = config.index("servol_rt_streamer_node:", safety_config_start)
    safety_config = config[safety_config_start:safety_config_end]

    assert "self.latest_target = target" in on_target
    assert "self.create_timer(1.0 / self.publish_rate_hz, self._tick)" in safety
    assert 'self.declare_parameter("publish_rate_hz", 30.0)' in safety
    assert "publish_rate_hz: 30.0" in safety_config
    assert "processing_mode=fixed_rate_clamp_only" in safety
    assert "ramp_owner=streamer" in safety
    assert "max_step_xyz_mm" not in safety
    assert "max_step_rpy_deg" not in safety
    assert "ramp_limited_axes" not in tick
    assert "max_step_xyz_mm" not in safety_config
    assert "max_step_rpy_deg" not in safety_config
    assert "state_qos" in safety


def test_calibration_gui_is_separate_and_metaquest_gate_is_wired():
    full = (PACKAGE_ROOT / "launch" / "a0509_full_bringup.launch.py").read_text()
    wrapped = (
        PACKAGE_ROOT / "launch" / "a0509_full_bringup_with_gripper.launch.py"
    ).read_text()
    gui = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "metaquest_calibration_gui.py"
    ).read_text()
    mux = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "a0509_command_mux_node.py"
    ).read_text()
    mapper = (PACKAGE_ROOT / "quest_a0509_teleop" / "xyz_mapper_node.py").read_text()
    topic = "/vr/metaquest_calibration/valid"

    assert 'DeclareLaunchArgument("start_calibration_gui", default_value="false")' in full
    assert '"start_calibration_gui": LaunchConfiguration("start_calibration_gui")' in wrapped
    assert 'executable="metaquest_calibration_gui"' in full
    assert "Prepare Robot" not in gui
    assert "Enable Live ServoL" not in gui
    assert topic in gui and topic in mux and topic in mapper
    assert "require_metaquest_calibration" in full


def test_streamer_keeps_live_control_responsive_with_topic_state_cache():
    streamer = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "servol_rt_streamer_node.py"
    ).read_text()

    assert "MutuallyExclusiveCallbackGroup" in streamer
    assert "self.control_service_callback_group" in streamer
    assert "self.robot_client_callback_group" in streamer
    assert "self.robot_state_callback_group" in streamer
    assert "self.stream_timer_callback_group" in streamer
    assert "callback_group=self.control_service_callback_group" in streamer
    assert "callback_group=self.robot_client_callback_group" in streamer
    assert "callback_group=self.robot_state_callback_group" in streamer
    assert "callback_group=self.stream_timer_callback_group" in streamer
    teleop_ready_sub = streamer.index("self.teleop_ready_sub = self.create_subscription")
    mux_heartbeat_sub = streamer.index("self.mux_heartbeat_sub", teleop_ready_sub)
    assert "state_qos" in streamer[teleop_ready_sub:mux_heartbeat_sub]
    robot_state_sub = streamer.index("self.robot_state_sub = self.create_subscription")
    set_live_service = streamer.index("self.set_live_srv =", robot_state_sub)
    robot_state_subscription = streamer[robot_state_sub:set_live_service]
    assert "Float64MultiArray" in robot_state_subscription
    assert "self.robot_state_topic" in robot_state_subscription
    assert "self._on_robot_state" in robot_state_subscription
    assert "state_qos" in robot_state_subscription
    assert "callback_group=self.robot_state_callback_group" in robot_state_subscription
    assert "state not in self.safe_robot_states" in streamer
    assert "previous_state not in self.safe_robot_states" in streamer
    assert 'self.declare_parameter("executor_num_threads", 4)' in streamer
    assert 'self.declare_parameter("executor_yield_sec", 0.001)' in streamer
    assert "MultiThreadedExecutor(num_threads=executor_num_threads)" in streamer
    assert "executor.spin_once(timeout_sec=0.1)" in streamer
    assert "time.sleep(executor_yield_sec)" in streamer
    assert "tick_generation = self._live_generation" in streamer
    assert "tick_generation != self._live_generation" in streamer

    hold_start = streamer.index("def _on_hold_servol")
    hold_end = streamer.index("def _enable_live_robot_output", hold_start)
    assert "self._mark_live_disabled()" in streamer[hold_start:hold_end]

    tick_start = streamer.index("def _tick")
    tick_end = streamer.index("def _robot_state_reject_reason", tick_start)
    tick = streamer[tick_start:tick_end]
    assert "_call_service" not in tick
    assert "_read_robot_state" not in tick
    assert "time.sleep" not in tick
    assert "_robot_state_reject_reason(now)" in tick
    assert "def _maybe_check_robot_ready" not in streamer
    assert "GetRobotState" not in streamer
    assert "robot_state_poll" not in streamer


def test_streamer_defaults_match_first_low_latency_operating_profile():
    streamer = (
        PACKAGE_ROOT / "quest_a0509_teleop" / "servol_rt_streamer_node.py"
    ).read_text()
    config = (PACKAGE_ROOT / "config" / "xyz_position_only.yaml").read_text()

    assert 'self.declare_parameter("publish_rate_hz", 30.0)' in streamer
    assert 'self.declare_parameter("servol_time_sec", 0.1)' in streamer
    assert (
        'self.declare_parameter("servol_use_auto_velocity_acceleration", False)'
        in streamer
    )
    assert (
        'self.declare_parameter("robot_state_topic", "/rt_topic/robot_state")'
        in streamer
    )
    assert 'self.declare_parameter("robot_state_stale_timeout_sec", 1.0)' in streamer
    assert (
        'self.declare_parameter("stream_ramp_linear_mm_per_tick", 6.67)'
        in streamer
    )
    assert 'self.declare_parameter("stream_ramp_rot_deg_per_tick", 1.0)' in streamer
    assert "publish_rate_hz: 30.0" in config
    assert "servol_time_sec: 0.1" in config
    assert "servol_use_auto_velocity_acceleration: true" in config
    assert "robot_state_topic: /rt_topic/robot_state" in config
    assert "robot_state_stale_timeout_sec: 1.0" in config
    assert "stream_ramp_linear_mm_per_tick: 6.67" in config
    assert "stream_ramp_rot_deg_per_tick: 1.0" in config
