# ROS 2 워크스페이스 상세 분석 보고서

## 0. 문서 목적과 사용 방법

이 문서는 `/home/salim2001/ros2_ws`의 2026-07-24 현재 상태를 다른 AI 모델,
개발자 또는 검토자에게 그대로 전달하기 위한 기술 컨텍스트 문서다.

이 문서가 답하려는 질문은 다음과 같다.

1. 이 워크스페이스는 무엇을 제어하는가?
2. 각 ROS 2 패키지의 책임과 관계는 무엇인가?
3. Meta Quest 입력이 Doosan A0509 명령으로 어떻게 변환되는가?
4. 실제 로봇 출력은 어떤 조건에서만 허용되는가?
5. JRT 그리퍼는 어떤 토픽과 서비스로 제어되는가?
6. 현재 Git, 빌드, 테스트, ROS 그래프 상태는 어떠한가?
7. 코드와 문서 사이의 불일치, 안전상 검토 항목, 재현성 문제는 무엇인가?

다른 모델은 다음 원칙으로 이 문서를 해석해야 한다.

- `build/`, `install/`, `log/`보다 `src/`의 원본을 우선한다.
- 아래의 "코드상 설계"와 "현재 런타임 관측"을 구분한다.
- 실제 로봇이 현재 연결되어 있다고 가정하지 않는다.
- `quest_a0509_teleop`을 현재 주 운영 파이프라인으로 본다.
- `a0509_vr_teleop`은 별도의 초기/대안 구현으로 본다.
- 로봇 안전과 관련된 변경은 실기 검증 전에 dry-run, RViz, 저속 검증을 거친다.

## 1. 분석 기준

### 1.1 분석 시점

- 날짜: 2026-07-24
- 타임존: Asia/Seoul
- 워크스페이스: `/home/salim2001/ros2_ws`
- 운영체제: Ubuntu 22.04.5 LTS
- ROS 배포판: ROS 2 Humble
- Python: 3.10.12
- GCC: 11.4.0
- 호스트 `ROS_DOMAIN_ID`: `16`
- Dockerfile 기본 `ROS_DOMAIN_ID`: `29`

### 1.2 분석 범위

포함:

- 루트 Git 저장소 상태
- 외부 Doosan 드라이버 Git 저장소 상태
- `src/` 아래 모든 colcon 패키지
- 텔레옵, 안전 제한, 실기 출력, 준비 동작, GUI
- JRT 그리퍼 Tool Digital Output 제어
- Doosan 드라이버의 핵심 연결 및 명령 경로
- MoveIt, Gazebo, MuJoCo, RViz 보조 패키지
- Docker 빌드 파일
- 현재 빌드, 테스트, launch 구문, ROS 그래프 관측

제외 또는 제한:

- 실제 로봇을 움직이는 실기 시험은 수행하지 않음
- Quest 헤드셋 및 Quest2ROS 앱의 실제 패킷 시험은 수행하지 않음
- Gazebo, MuJoCo, MoveIt GUI 실행 시험은 수행하지 않음
- 490 MB 규모의 모든 mesh/DAE/STL/USD 파일의 형상 자체를 시각 검증하지 않음
- 외부 하드웨어 배선과 JRT 제조사 pinout은 저장소 내용만으로 확정하지 않음

## 2. 한눈에 보는 결론

이 워크스페이스는 Doosan A0509 로봇을 Meta Quest 컨트롤러로 task-space
텔레오퍼레이션하고, Quest A/B 입력으로 JRT JEGB 계열 그리퍼를 제어하기 위한
ROS 2 Humble 시스템이다.

주 경로는 다음과 같다.

```text
Quest2ROS PoseStamped
  /q2r_right_hand_pose
        |
        v
quest_a0509_teleop/xyz_mapper_node
  - VR anchor 기반 상대 이동
  - 축 교환/부호/스케일
  - 위치 및 quaternion 저역통과 필터
  - jump rejection
  - XY yaw 보정
  - 현재 설정에서는 Quest roll -> robot ry만 사용
        |
        v
  /vr/target_posx
        |
        v
quest_a0509_teleop/safety_guard_node
  - XYZ box clamp
  - anchor 기준 RPY clamp
  - tick당 증분 제한
        |
        v
  /vr/safe_posx
        |
        v
quest_a0509_teleop/servol_rt_streamer_node
  - dry_run 차단
  - teleop_ready 차단
  - runtime live-enable 차단
  - robot_state 검사
  - 실제 TCP에서 시작하는 추가 ramp
        |
        v
  /dsr01/servol_rt_stream
        |
        v
dsr_controller2::servol_rt_cb
        |
        v
Doosan DRFL servol_rt()
```

그리퍼 경로는 다음과 같다.

```text
Quest2ROS OVR2ROSInputs
  /q2r_right_hand_inputs
        |
        v
quest_inputs_ab_gripper_mapper_node
  A only -> "close"
  B only -> "open"
  neither/both/timeout -> "stop"
        |
        v
  /jrt_gripper/cmd
        |
        v
jrt_tool_io_driver_node
  interlock + pulse/level plan
        |
        v
  /dsr01/io/set_tool_digital_output
        |
        v
Doosan Tool DO1/DO2
```

현재 코드의 중요한 상태는 다음과 같다.

- 실제 로봇 출력은 기본적으로 비활성화되어 있다.
- 통합 launch의 `dry_run` 기본값은 `true`다.
- 실제 ServoL 출력에는 준비 완료와 별도 live-enable 서비스 호출이 필요하다.
- 현재 ROS 그래프에는 실행 중인 사용자 노드가 없다.
- 현재 호스트에서 설정된 로봇 IP `192.168.137.100`은 ping에 응답하지 않았다.
- 사용자 패키지 4개와 로컬 변경된 `dsr_controller2` 빌드는 성공했다.
- 단위 테스트 19개가 통과했지만, 주 운영 패키지 `quest_a0509_teleop`에는 테스트가 없다.
- Doosan 드라이버는 사용자 fork를 가리키는 Git submodule로 구성했다.
- Doosan 로컬 변경 2개는 submodule commit `8f4fa87`로 고정했다.
- 입력 timeout 후 마지막 위치를 계속 스트리밍하는 것이 현재 주 운영 파이프라인의 핵심 안전 검토 항목이다.

## 3. 디스크 및 디렉터리 구조

### 3.1 용량

```text
src/a0509_vr_teleop     276 KB
src/dh_robot_rviz        36 KB
src/doosan-robot2       490 MB
src/jrt_gripper_io      236 KB
src/quest_a0509_teleop  380 KB
src/quest_jrt_gripper    24 KB (디렉터리만 있고 파일 없음)
build/                  383 MB
install/                139 MB
log/                     43 MB
```

Doosan 소스가 큰 이유는 다수 로봇 모델의 DAE/STL/URDF/USD/MuJoCo asset을
포함하기 때문이다.

### 3.2 소스 파일 개요

`.git`, Python cache, pytest cache를 제외한 `src/` 파일은 약 1,349개다.

주요 확장자 수:

```text
DAE mesh      393
STL mesh      180 (대소문자 확장자 합계)
Python        127
ROS service   140
YAML          113
Xacro          87
XML            52
URDF           31
ROS message    18
C++            11
C/C++ header   19
ROS action      3
```

### 3.3 최상위 구조

```text
ros2_ws/
├── .git/                         상위 사용자 프로젝트 Git
├── .gitignore
├── .gitmodules                  Doosan fork submodule 정의
├── .dockerignore
├── Dockerfile
├── README.md                    프로젝트 시작 문서
├── README.docker.md
├── ROS2_WS_ANALYSIS_REPORT_KO.md
├── docker/
│   └── ros_entrypoint.sh
├── src/
│   ├── a0509_vr_teleop/         초기/대안 VR 텔레옵
│   ├── dh_robot_rviz/           독립적인 표준 DH 4축 시각화
│   ├── doosan-robot2/           별도 Git의 공식/포크 Doosan 드라이버
│   ├── jrt_gripper_io/          Quest/Joy -> Tool DO 그리퍼 제어
│   ├── quest_a0509_teleop/      현재 주 A0509 Quest 텔레옵
│   └── quest_jrt_gripper/       빈 디렉터리, colcon 패키지 아님
├── build/                        colcon 생성물
├── install/                      colcon 설치 공간
└── log/                          colcon 빌드/테스트 이력
```

## 4. Git과 재현성 상태

### 4.1 상위 프로젝트 저장소

```text
경로: /home/salim2001/ros2_ws
업로드 브랜치: agent/publish-latest-workspace
기준 main commit: 28edfb3
원격:
  https://github.com/kim-salim/Dosan-MetaQuest-teleoperation-project.git
```

중요:

- `src/doosan-robot2/`는 일반 파일로 중복 저장하지 않고 Git submodule로 관리한다.
- `.gitmodules`는 `https://github.com/kim-salim/doosan-robot2.git`의
  `agent/a0509-runtime-changes` 브랜치를 가리킨다.
- 상위 저장소는 Doosan commit `8f4fa87`를 gitlink로 고정한다.
- clone 시 `--recurse-submodules` 또는 `git submodule update --init --recursive`가 필요하다.
- `quest_a0509_teleop`의 launch는 `dsr_bringup2`, `dsr_msgs2`,
  `jrt_gripper_io`를 전제로 한다.

### 4.2 Doosan 외부 저장소

```text
경로: /home/salim2001/ros2_ws/src/doosan-robot2
업로드 브랜치: agent/a0509-runtime-changes
HEAD: 8f4fa87
HEAD 메시지: Request controller access for A0509 activation
upstream base: ec92425 (doosan-robotics/doosan-robot2 humble)
```

commit `8f4fa87`에 포함된 변경:

1. `dsr_controller2/config/dsr_controller2.yaml`
2. `dsr_controller2/src/dsr_controller2.cpp`

변경 내용:

- RT 선택 발행 필드가 `actual_joint_position`에서 `actual_tcp_position`으로 변경됨.
- `use_rt_topic_pub` 자체는 여전히 `false`임.
- `RobotController::on_activate()`에서
  `manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST)`를 호출하고 결과를 로그로 남김.
- TP 초기화 완료 callback에서도 동일한 access-control request를 다시 수행함.

해석:

- 이 변경은 실제 컨트롤 권한을 더 적극적으로 요청하기 위한 현장 대응으로 보인다.
- 사용자 fork commit과 상위 submodule gitlink로 다른 환경에서도 같은 소스를 복원할 수 있다.
- RT topic 키 변경은 `use_rt_topic_pub=true`로 바꾸지 않는 한 실제 topic 생성에 영향을 주지 않는다.
- 로컬 C++ 변경은 현재 `dsr_controller2` 빌드에 성공했다.

## 5. colcon 패키지 목록과 역할

현재 `colcon list`는 총 26개 패키지를 인식한다.

### 5.1 사용자 작성 패키지

| 패키지 | 빌드 타입 | 버전 | 역할 |
|---|---|---:|---|
| `a0509_vr_teleop` | `ament_python` | 0.1.0 | UDP/모의 VR 입력, workspace projection, RViz marker, ServoL dry-run |
| `quest_a0509_teleop` | `ament_python` | 0.1.0 | Quest2ROS Pose 기반 운영용 A0509 텔레옵 |
| `jrt_gripper_io` | `ament_python` | 0.1.0 | Quest/Joy 입력을 Doosan Tool DO로 변환 |
| `dh_robot_rviz` | `ament_cmake` | 0.0.1 | 표준 DH 모델을 joint GUI와 RViz로 시각화 |

### 5.2 Doosan 기반 패키지

| 패키지 | 역할 |
|---|---|
| `dsr_bringup2` | 실제/가상 로봇, RViz, Gazebo, MoveIt, MuJoCo launch 조합 |
| `dsr_common2` | DRFL 공용 라이브러리, Python API, servicepack, emulator 관련 binary |
| `dsr_controller2` | ros2_control controller plugin, 스트림 subscriber, 약 100개 수준 서비스 노출 |
| `dsr_description2` | 로봇별 URDF/Xacro/mesh/ros2_control/MuJoCo/USD 모델 |
| `dsr_gazebo2` | Gazebo world, robot SDF, spawn launch |
| `dsr_hardware2` | 실제/가상 DRCF 연결 및 ros2_control SystemInterface |
| `dsr_msgs2` | 18 msg, 140 srv, 3 action의 인터페이스 정의 |
| `dsr_mujoco` | MuJoCo scene/controller/launch |
| `dsr_example` | Python motion/action 예제 |
| `dsr_realtime_control` | C++ RT control 예제 |
| `dsr_visualservoing` | 카메라/marker/Gazebo visual servoing 예제 |
| `dsr_tests` | Doosan bringup 및 CLI launch test |
| `dsr_moveit_config_a0509` | A0509 MoveIt 설정 |
| `dsr_moveit_config_a0912` | A0912 MoveIt 설정 |
| `dsr_moveit_config_e0509` | E0509 MoveIt 설정 |
| `dsr_moveit_config_h2017` | H2017 MoveIt 설정 |
| `dsr_moveit_config_h2515` | H2515 MoveIt 설정 |
| `dsr_moveit_config_m0609` | M0609 MoveIt 설정 |
| `dsr_moveit_config_m0617` | M0617 MoveIt 설정 |
| `dsr_moveit_config_m1013` | M1013 MoveIt 설정 |
| `dsr_moveit_config_m1509` | M1509 MoveIt 설정 |
| `dsr_moveit_config_p3020` | P3020 MoveIt 설정 |

### 5.3 colcon 의존성 그래프의 핵심

명시된 패키지 메타데이터 기준:

```text
dsr_controller2
  -> dsr_common2
  -> dsr_msgs2
  -> dsr_hardware2

dsr_hardware2
  -> dsr_common2
  -> dsr_msgs2

dsr_gazebo2
  -> dsr_description2

a0509_vr_teleop
  -> dsr_msgs2

jrt_gripper_io
  -> dsr_msgs2
```

`quest_a0509_teleop`이 그래프에서 `dsr_msgs2`에 연결되지 않는 이유는
`package.xml`에 해당 의존성이 없고 코드가 `try/except` 동적 import를 사용하기 때문이다.
실제 기능상으로는 분명히 `dsr_msgs2`, `dsr_bringup2`, `jrt_gripper_io`에 의존한다.

## 6. 현재 주 운영 파이프라인: quest_a0509_teleop

### 6.1 패키지 구성

```text
src/quest_a0509_teleop/
├── config/xyz_position_only.yaml
├── launch/
│   ├── xyz_position_only.launch.py
│   ├── xyz_position_only_real.launch.py
│   ├── xyz_position_only_gui.launch.py
│   ├── a0509_full_bringup.launch.py
│   └── a0509_full_bringup_with_gripper.launch.py
└── quest_a0509_teleop/
    ├── xyz_mapper_node.py
    ├── safety_guard_node.py
    ├── servol_rt_streamer_node.py
    ├── robot_prep_node.py
    ├── quest_input_button_node.py
    └── teleop_check_gui.py
```

코드 규모는 약 3,200줄이며, README와 launch/config를 합치면 약 4,269줄이다.

### 6.2 xyz_mapper_node

파일:

`src/quest_a0509_teleop/quest_a0509_teleop/xyz_mapper_node.py`

입력:

```text
/q2r_right_hand_pose
type: geometry_msgs/msg/PoseStamped
position unit: meter
orientation: quaternion xyzw
```

출력:

```text
/vr/target_posx
type: std_msgs/msg/Float64MultiArray
layout: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
```

추가 구독:

```text
/vr/robot_anchor_posx   Float64MultiArray
/vr/teleop_ready        Bool
```

서비스:

```text
/vr/recenter                    std_srvs/srv/Trigger
/vr/toggle_roll_lock            std_srvs/srv/Trigger
/vr/calibrate_xy_yaw_to_x_plus  std_srvs/srv/Trigger
```

#### 위치 변환

기본 수식:

```text
vr_delta_m[i] = filtered_vr_pose_m[i] - vr_anchor_m[i]
mapped_delta_m[i] = vr_delta_m[axis_map[i]]
delta_mm[i] = axis_sign[i] * scale_xyz[i] * mapped_delta_m[i] * 1000
target_xyz_mm = robot_anchor_xyz_mm + yaw_corrected(delta_mm)
```

현재 YAML:

```yaml
robot_anchor_xyz_mm: [400.0, 0.0, 350.0]
scale_xyz: [0.5, 0.5, 0.5]
axis_sign: [1.0, -1.0, 1.0]
axis_map: [1, 0, 2]
```

따라서:

```text
robot X <- +0.5 * Quest Y
robot Y <- -0.5 * Quest X
robot Z <- +0.5 * Quest Z
```

Quest 10 cm 이동은 해당 로봇 축에서 기본 50 mm 이동이 된다.

#### 위치 필터와 jump rejection

현재 YAML:

```yaml
enable_pose_low_pass_filter: true
pose_filter_alpha: 0.4
max_vr_jump_m: 0.15
input_timeout_sec: 0.5
```

저역통과:

```text
filtered = previous_filtered + alpha * (raw - previous_filtered)
```

연속 accepted raw pose 사이 거리가 0.15 m를 넘으면 해당 sample을 거부한다.
`/vr/recenter`는 최신 raw pose를 새 anchor로 받아 jump-filter 기준도 초기화한다.

주의:

- `_on_pose()`는 jump rejection 전에 `last_input_time`을 갱신한다.
- 따라서 계속 큰 jump만 들어오는 경우에도 input timeout은 발생하지 않는다.
- 이 경우 마지막 accepted pose가 유지되고 같은 target이 계속 계산될 수 있다.

#### XY yaw 보정

기능:

- `xy_yaw_correction_deg`로 매핑된 XY delta를 회전시킨다.
- `/vr/calibrate_xy_yaw_to_x_plus` 호출 후 일정 시간 컨트롤러를 원하는 robot +X
  방향으로 움직이면 관측 벡터 각도의 음수를 보정값으로 사용한다.
- 현재 보정값은 런타임 메모리에만 있고 YAML로 자동 저장되지 않는다.

현재 YAML:

```yaml
enable_xy_yaw_correction: true
xy_yaw_correction_deg: 0.0
xy_yaw_calibration_duration_sec: 2.0
xy_yaw_calibration_min_distance_m: 0.04
```

#### 자세 변환

일반 경로:

```text
q_relative = q_now * conjugate(q_anchor)
relative Euler XYZ를 axis_map/sign/scale로 변환
mapped delta quaternion 생성
q_target = q_mapped_delta * q_robot_anchor
이전 target에 가까운 Euler family 선택
```

현재 YAML은 일반 3축 자세 제어가 아니라 roll-only 모드다.

```yaml
enable_orientation_mapping: true
scale_rpy: [0.0, 0.7, 0.0]
rot_axis_sign: [-1.0, -1.0, -1.0]
rot_axis_map: [1, 0, 2]
```

실제 활성 성분:

```text
robot ry delta <- -0.7 * Quest relative roll
robot rx = anchor rx 유지
robot rz = anchor rz 유지
```

추가 roll-only 안정화:

```yaml
roll_only_deadband_deg: 1.0
roll_only_filter_alpha: 0.35
```

Quest grip rising edge는 `quest_input_button_node`를 통해 roll lock을 toggle한다.
roll lock 중에는 robot `ry`를 고정한다. 해제 시 현재 고정 각도를 새 robot anchor
`ry`로 만들고 VR orientation anchor를 재설정하여 불연속을 줄인다.

#### target 발행 조건

다음 조건을 모두 만족해야 `/vr/target_posx`를 발행한다.

1. accepted Quest pose가 있음
2. VR anchor가 있음
3. orientation mapping 사용 시 quaternion anchor가 있음
4. 마지막 입력 수신 후 0.5초 이내
5. XY yaw calibration 중이 아님
6. `require_prepare_before_target=true`일 때 `/vr/teleop_ready=true`

### 6.3 safety_guard_node

파일:

`src/quest_a0509_teleop/quest_a0509_teleop/safety_guard_node.py`

입력:

```text
/vr/target_posx
/vr/robot_anchor_posx
```

출력:

```text
/vr/safe_posx
/vr/status
```

현재 공간 제한:

```yaml
workspace_min_xyz_mm: [250.0, -350.0, 150.0]
workspace_max_xyz_mm: [650.0, 350.0, 600.0]
```

현재 tick당 위치 제한:

```yaml
publish_rate_hz: 30.0
max_step_xyz_mm: [20.0, 20.0, 20.0]
```

이는 축별 제한이다. 30 Hz에서 각 축 이론상 최대 변화율은 600 mm/s이고,
3축 동시 이동의 유클리드 속도는 더 클 수 있다. 이 수치는 Doosan 내부 속도/가속도
제한과 동일한 의미가 아니다.

자세 제한:

```yaml
enable_orientation_limits: true
max_orientation_delta_deg: [90.0, 90.0, 90.0]
max_step_rpy_deg: [2.0, 2.0, 2.0]
```

자세 anchor는 `/vr/robot_anchor_posx`가 오면 그 RPY를 사용한다. 아직 없으면
첫 target RPY를 anchor로 사용한다. angle delta는 `[-180, 180)` 최단각으로 계산한다.

중요한 상태 동작:

- 최초 target은 ramp 없이 clamp된 값으로 즉시 `last_safe`가 된다.
- 이후에는 축별 ramp를 적용한다.
- target에 timestamp가 없다.
- 새 target이 끊겨도 `latest_target`을 계속 30 Hz로 처리하고 발행한다.
- 따라서 mapper input timeout은 "새 목표 정지"이며 "safe topic 정지"가 아니다.

### 6.4 servol_rt_streamer_node

파일:

`src/quest_a0509_teleop/quest_a0509_teleop/servol_rt_streamer_node.py`

입력:

```text
/vr/safe_posx
/vr/teleop_ready
```

출력:

```text
/dsr01/servol_rt_stream
type: dsr_msgs2/msg/ServolRtStream
```

제공 서비스:

```text
/vr/set_live_robot_output  std_srvs/srv/SetBool
/vr/hold_servol            std_srvs/srv/Trigger
```

Doosan 서비스 client:

```text
/dsr01/system/get_robot_state
/dsr01/realtime/read_data_rt
/dsr01/aux_control/get_current_posx
```

실제 출력의 필수 조건:

```text
dry_run == false
robot publisher 생성 성공
latest_safe 존재
teleop_ready == true
/vr/set_live_robot_output {data: true} 성공
robot_state in [1, 2]
```

Doosan state 의미:

```text
1 = STATE_STANDBY
2 = STATE_MOVING
```

live-enable 시:

1. 로봇 state를 서비스로 확인한다.
2. `ReadDataRt` 또는 `GetCurrentPosx`로 실제 TCP를 읽는다.
3. 실제 TCP를 `current_command`로 설정하고 한 번 발행한다.
4. 이후 `/vr/safe_posx`까지 별도 stream ramp를 적용한다.

현재 stream ramp:

```yaml
publish_rate_hz: 10.0
stream_ramp_linear_mm_per_tick: 7.5
stream_ramp_rot_deg_per_tick: 3.0
servol_time_sec: 0.5
```

즉 축별 위치 명령 변화는 10 Hz 기준 75 mm/s에 해당한다.

발행 message:

```text
pos  = ramped safe_posx
vel  = [0, 0, 0, 0, 0, 0]
acc  = [0, 0, 0, 0, 0, 0]
time = 0.5
```

로봇 state는 기본 1초마다 다시 확인한다. unsafe state 또는 service 예외가 나면
live gate를 false로 바꾸고 현재 TCP 또는 마지막 command를 3회 best-effort hold로
발행한다.

중요:

- `/vr/safe_posx` 자체에 freshness 검사가 없다.
- Quest 입력이 끊겨 mapper가 발행을 중단해도 safety guard가 마지막 target을 계속
  발행하므로 streamer는 live 상태에서 마지막 위치를 계속 ServoL로 보낸다.
- 이는 "last pose hold" 정책으로 해석할 수 있지만, 자동 live disable이나 MoveStop은 아니다.
- 현재 주 파이프라인에는 로봇 motion을 위한 별도 deadman button gate가 없다.

### 6.5 robot_prep_node

파일:

`src/quest_a0509_teleop/quest_a0509_teleop/robot_prep_node.py`

제공 서비스:

```text
/vr/prepare_robot
/vr/set_robot_anchor_to_current_tcp
/vr/stop_robot
/vr/reset_safe_off
/vr/start_rt_control
/vr/stop_rt_control
```

발행:

```text
/vr/robot_anchor_posx
/vr/teleop_ready
/vr/status
```

사용하는 Doosan 서비스:

```text
/dsr01/system/get_robot_state
/dsr01/system/set_robot_control
/dsr01/aux_control/get_current_posj
/dsr01/aux_control/get_current_posx
/dsr01/motion/move_joint
/dsr01/motion/move_spline_joint
/dsr01/motion/move_stop
/dsr01/motion/move_wait
/dsr01/motion/check_motion
/dsr01/realtime/start_rt_control
/dsr01/realtime/stop_rt_control
```

현재 prep target:

```yaml
prepare_joint_deg: [0.0, 0.0, 90.0, 0.0, 30.0, 0.0]
prepare_joint_tolerance_deg: 2.0
prepare_vel_deg_per_sec: 30.0
prepare_acc_deg_per_sec2: 30.0
prepare_preflight_wait_sec: 3.0
```

준비 시퀀스:

1. `/vr/teleop_ready=false`
2. 필요한 Doosan service type/client 존재 확인
3. robot state가 `[1, 2]`인지 확인
4. 3초 preflight 대기
5. 현재 joint pose 읽기
6. 필요 시 J5 signed escape waypoint 생성
7. 필요 시 J3 signed escape waypoint 생성
8. 나머지 joint를 waypoint당 최대 10도씩 target으로 이동
9. `MoveSplineJoint` 또는 단일 point이면 `MoveJoint` 호출
10. 각 phase 후 현재 joint와 state를 polling
11. 최종 오차가 2도 이하인지 확인
12. 현재 TCP를 읽어 `/vr/robot_anchor_posx` 발행
13. mapper `/vr/recenter` best-effort 호출
14. `/vr/teleop_ready=true`

주의:

- 이 준비 경로는 MoveIt collision planning을 사용하지 않는 joint waypoint 계획이다.
- waypoint당 각도 제한은 충돌 회피를 보장하지 않는다.
- `MoveSplineJoint.sync_type=1`로 비동기 요청 후 자체 polling한다.
- `teleop_ready`는 기본 volatile QoS로 3회 발행할 뿐 transient-local이 아니다.
- 중간 노드 재시작 시 ready 상태를 놓칠 수 있지만, 기본 상태 false이므로 fail-safe 방향이다.

`/vr/stop_robot`:

1. `teleop_ready=false`
2. live output false 요청
3. Doosan `MoveStop`을 `stop_mode=1`로 호출

`/vr/reset_safe_off`:

- `SetRobotControl.robot_control=3`을 호출한다.
- `3`은 코드상 `CONTROL_RESET_SAFE_OFF` 상수다.

문서 불일치:

- README는 "이미 목표 위치에 있으면 prepare 없이 현재 TCP anchor만 설정"할 수 있는 것처럼
  설명한다.
- 실제 `_on_set_anchor_to_current_tcp()`는 `teleop_ready=false`이면 거부한다.
- 따라서 현재 코드에서는 prepare 성공 전 이 서비스가 대체 경로가 될 수 없다.

### 6.6 quest_input_button_node

입력:

```text
/q2r_right_hand_inputs
type: quest2ros/msg/OVR2ROSInputs
configured field: press_middle
```

동작:

- grip rising edge를 감지한다.
- 0.7초 debounce를 적용한다.
- `teleop_ready=true`일 때 `/vr/toggle_roll_lock`을 호출한다.
- 이 입력은 robot output deadman이 아니다.

Quest2ROS type import:

- 정상 Python import를 먼저 시도한다.
- 실패하면 아래 절대 경로를 `sys.path`와 shared-library preload에 사용한다.

```text
/home/salim2001/quest2ros2_ws/install/quest2ros/local/lib/python3.10/dist-packages
/home/salim2001/quest2ros2_ws/install/quest2ros/lib
```

이는 현재 사용자 홈 구조에 결합된 설정이다.

### 6.7 teleop_check_gui

Tkinter GUI이며 ROS executor는 별도 thread에서 돈다.

표시:

```text
Quest Pose
Target PosX
Safe PosX
Robot Anchor
Teleop Ready
/vr/status log
```

명령:

```text
Recenter VR
Anchor = Current TCP
Calibrate XY +X
Prepare Robot
Start RT Control
Stop RT Control
Enable/Disable Live ServoL
Hold ServoL
Stop Robot
Reset SAFE_OFF
```

위험 버튼은 GUI의 `Enable real-action buttons` 체크가 있어야 활성화되고 confirmation
dialog를 띄운다. 그러나 이는 GUI 레벨 보호이며 ROS service를 직접 호출하는 사용자를
막지는 않는다.

### 6.8 launch 조합

#### `xyz_position_only.launch.py`

시작:

- xyz mapper
- Quest grip button node
- safety guard
- ServoL streamer

기본 `dry_run=true`. robot prep node는 시작하지 않으므로
`require_prepare_before_target=true` 설정에서는 `teleop_ready`를 true로 만들 수 없다.
따라서 이름과 달리 현재 기본 설정에서는 target이 발행되지 않는 대기 상태가 된다.

#### `xyz_position_only_real.launch.py`

위 구성에 `robot_prep_node`를 추가한다.

#### `xyz_position_only_gui.launch.py`

real 구성에 Tkinter GUI를 추가한다.

#### `a0509_full_bringup.launch.py`

추가:

- `dsr_bringup2_rviz.launch.py`
- 실제 Doosan A0509 연결 기본값

기본값:

```text
mode=real
host=192.168.137.100
rt_host=192.168.137.10
port=12345
model=a0509
name=dsr01
color=white
dry_run=true
start_robot_bringup=true
start_teleop=true
start_gui=true
```

주의:

- `robot_namespace`와 `name`은 별개 launch argument다.
- 기본값은 서로 일치하지만 하나만 바꾸면 topic/service namespace가 갈라질 수 있다.
- `doosan_servol_topic`도 독립 문자열이므로 namespace 변경 시 함께 맞춰야 한다.

#### `a0509_full_bringup_with_gripper.launch.py`

추가:

- 선택적 `ros_tcp_endpoint`
- full A0509 bringup
- JRT gripper bringup

기본:

```text
start_endpoint=false
start_gripper=true
dry_run=true
```

JRT 쪽에는 `start_robot_bringup=false`를 넘겨 Doosan controller stack 중복 실행을 피한다.

## 7. 초기/대안 파이프라인: a0509_vr_teleop

### 7.1 성격

이 패키지는 실제 Quest2ROS `PoseStamped`를 직접 쓰지 않고 다음 JSON 형식의 자체 controller
state를 사용한다.

```json
{
  "pose": [400.0, 0.0, 350.0, 0.0, 150.0, 0.0],
  "tracking_ok": true,
  "deadman": true,
  "clutch": false,
  "stamp": 123.4
}
```

단위는 이미 robot convention인 mm/deg라고 가정한다.

구성 노드:

```text
mock_quest_input_node 또는 quest_gateway_node
vr_frame_mapper_node
safety_guard_node
servol_rt_streamer_node
robot_state_monitor_node
rviz_visualizer_node
```

### 7.2 입력 gateway

`quest_gateway_node`:

- UDP `0.0.0.0:5005`에서 JSON 수신
- valid object에 `stamp`가 없으면 local `time.monotonic()` 삽입
- `/vr/controller_state`의 `std_msgs/String`으로 발행

시간축 주의:

- 외부 sender가 `stamp`를 보내면 그대로 보존한다.
- safety guard는 이 stamp를 local `time.monotonic()`과 비교한다.
- sender가 Unix epoch, ROS time 또는 다른 monotonic origin을 쓰면 timeout 계산이 잘못될 수 있다.

`mock_quest_input_node`:

- 10 Hz sinusoidal 6D pose
- 기본 deadman/tracking true
- dry-run과 RViz 검증에 사용

### 7.3 frame mapper

`vr_frame_mapper_node`:

```text
target = robot_anchor + (vr_pose - vr_anchor)
```

- 별도 scale이나 축 변환 없이 1:1
- 첫 pose에서 anchor 설정
- clutch/recenter rising edge에서 VR anchor와 robot anchor 갱신
- quaternion 회전이 아니라 RPY 성분을 직접 더하고 뺀다.

이 패키지는 실제 Quest2ROS m/quaternion 입력에 바로 연결할 수 없다. 외부 앱 또는 gateway가
이미 mm/deg의 올바른 base-frame pose를 만들어야 한다.

### 7.4 safety core

기본 workspace:

```text
x: 250..750 mm
y: -350..350 mm
z: 120..600 mm
rx, ry, rz: -180..180 deg
```

정책:

- tracking lost -> last safe hold
- deadman release -> last safe hold
- 0.3초 timeout -> last safe hold
- workspace 외부 -> last safe에서 raw target으로 가는 선분의 첫 box 경계로 projection
- projection 후 위치 축당 20 mm/tick, 자세 축당 3 deg/tick ramp

세부:

- projection은 XYZ뿐 아니라 RPY까지 포함한 6차원 axis-aligned box 교차다.
- 어느 한 orientation 축이 먼저 경계에 닿아도 전체 6D interpolation 비율이 제한될 수 있다.
- ramp는 유클리드 norm이 아닌 각 축 독립 제한이다.

### 7.5 ServoL streamer

- `enable_robot_output=false`이면 실제 robot publisher 자체를 만들지 않는다.
- 항상 `/teleop/debug_servol_rt_stream`을 발행한다.
- `enable_robot_output=true`일 때만 `/dsr01/servol_rt_stream` publisher를 만든다.
- runtime service로 나중에 enable하는 구조가 아니라 launch 시 고정된다.

이 초기 파이프라인에는 `quest_a0509_teleop`의 다음 보호가 없다.

- robot prep 완료 gate
- 별도 live-enable service
- robot state 주기 검사
- 실제 TCP에서 시작하는 stream ramp

대신 tracking/deadman/watchdog를 safety core에서 명시적으로 처리한다.

### 7.6 robot monitor와 RT topic

기본 구독:

```text
/rt_topic/actual_tcp_position
/rt_topic/actual_joint_position
/rt_topic/robot_state
/dsr01/error
```

Doosan `dsr_controller2.yaml`의 현재 설정은:

```yaml
use_rt_topic_pub: false
rt_topic_keys:
  - actual_tcp_position
```

따라서 설정을 바꾸지 않으면 `/rt_topic/actual_tcp_position`은 생성되지 않는다.
또한 Doosan controller code는 topic 이름을 `"/rt_topic/" + key`로 만들어 namespace 밖의
절대 topic으로 발행한다.

### 7.7 테스트

이 패키지는 현재 10개 단위 테스트가 있다.

```text
projection utils       5
safety core            3
ServoL message/gating  2
```

테스트하지 않는 범위:

- UDP thread 종료
- 실제 ROS topic 통합
- robot state monitor message type 호환
- Quest clock stamp 호환
- launch 전체 동작
- 실제 DRFL 명령

## 8. JRT 그리퍼: jrt_gripper_io

### 8.1 목표와 하드웨어 가정

대상:

```text
JRT JEGB-4285P-A-X340
```

저장소가 가정하는 명령 매핑:

```text
Tool DO1 -> CLOSE
Tool DO2 -> OPEN
0 -> OFF
1 -> ON
```

JRT 모터 전원을 Tool DO에서 공급하는 구조가 아니다. 문서상 별도 24 V 전원과 공통 ground,
PNP 호환, 입력 전류 검증이 필요하다.

### 8.2 입력 mapper 두 종류

1. `quest_ab_gripper_mapper_node`

```text
input: sensor_msgs/msg/Joy, default /joy
A/B는 정수 index parameter
```

2. `quest_inputs_ab_gripper_mapper_node`

```text
input: quest2ros/msg/OVR2ROSInputs
default /q2r_right_hand_inputs
A field: button_lower
B field: button_upper
```

공통 truth table:

```text
A=1, B=0 -> close
A=0, B=1 -> open
A=0, B=0 -> stop
A=1, B=1 -> stop
```

공통 watchdog:

- 기본 0.3초
- 최초 입력이 없거나 입력이 timeout되면 `stop`
- command가 바뀔 때만 `/jrt_gripper/cmd`에 발행

Quest2ROS mapper도 사용자 홈의 절대 Python/library path를 fallback으로 사용한다.

### 8.3 Tool IO driver

입력:

```text
/jrt_gripper/cmd
type: std_msgs/msg/String
valid: close, open, stop
unknown: stop으로 정규화
```

출력:

```text
/dsr01/io/set_tool_digital_output
type: dsr_msgs2/srv/SetToolDigitalOutput
```

기본 parameter:

```yaml
close_do_index: 1
open_do_index: 2
active_value: 1
inactive_value: 0
command_mode: pulse
pulse_sec: 0.20
interlock_sec: 0.05
debounce_sec: 0.30
startup_all_off: true
shutdown_all_off: true
dry_run: false
```

`pulse` close:

```text
open OFF
wait 0.05 s
close ON
wait 0.20 s
close OFF
```

`pulse` open:

```text
close OFF
wait 0.05 s
open ON
wait 0.20 s
open OFF
```

`level` close/open은 반대 출력을 끈 뒤 요청 출력을 계속 ON으로 유지한다.

비동기 service plan이 진행 중이면 새 command를 무시한다. 여기에는 watchdog의 `stop`도
포함될 수 있다. 기본 pulse plan은 약 0.25초로 watchdog 0.3초보다 짧지만, service 지연이
크면 stop 반응이 지연될 수 있다.

service 실패:

- 현재 plan을 취소
- 실패 command가 stop이 아니면 두 출력 OFF stop plan을 재시도

종료:

- 정상 shutdown이면 두 출력 OFF를 동기 best-effort 호출
- SIGKILL, 전원 차단, 네트워크 단절에서는 보장되지 않음

### 8.4 중요한 인터페이스 주석 오류

`dsr_msgs2/srv/io/SetToolDigitalOutput.srv`의 주석은:

```text
value: 0 = ON, 1 = OFF
```

이라고 되어 있다.

그러나 같은 Doosan 소스의:

```text
dsr_common2/imp/DSR_ROBOT2.py
dsr_controller2/include/dsr_controller2/dsr_controller2.hpp
```

에서는:

```text
ON = 1
OFF = 0
```

으로 정의한다. JRT 코드와 문서는 후자를 따른다. 즉 `.srv` 주석이 구현과 상충하는 것으로
보이며, 실기에서는 multimeter로 반드시 검증해야 한다.

### 8.5 launch 안전 기본값

- 독립 `jrt_gripper_io.launch.py`의 `dry_run` 기본값은 `false`다.
- `jrt_gripper_robot_bringup.launch.py`의 `dry_run` 기본값도 `false`다.
- 통합 `a0509_full_bringup_with_gripper.launch.py`에서는 상위 `dry_run=true`를 전달하므로
  기본 통합 실행은 dry-run이다.

독립 gripper launch를 직접 실행할 때는 기본값이 실제 Tool DO 호출을 허용한다는 점이 중요하다.

### 8.6 테스트

현재 순수 로직 테스트 9개가 있다.

검증:

- A/B truth table
- close/open/stop plan
- level close
- pulse interlock/delay sequence

미검증:

- 비동기 ROS service failure 경쟁 상태
- active plan 중 watchdog stop
- Quest2ROS type import
- 실제 Tool DO 극성
- 실제 그리퍼 feedback

## 9. dh_robot_rviz

이 패키지는 Doosan A0509 파이프라인과 직접 연결되지 않는 독립적인 표준 DH 시각화 도구다.

구성:

```text
world
 -> base_link
 -> q0
 -> frame1
 -> q1
 -> frame2
 -> q2_clockwise
 -> frame3
 -> q3_clockwise
 -> frame4
 -> tool0
```

DH 치수:

```text
d1 = 0.095 m
a2 = 0.244 m
a3 = 0.1628 m
a4 = 0.096 m
tool offset = 0.011 m
```

launch:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

현재 Xacro는 `check_urdf`로 정상 parsing되었다.

## 10. Doosan 드라이버 핵심 구조

### 10.1 bringup 흐름

`dsr_bringup2_rviz.launch.py`의 기본 흐름:

1. robot model Xacro 생성
2. virtual이면 emulator node 실행
3. namespace 아래 `controller_manager/ros2_control_node` 실행
4. `joint_state_broadcaster` spawner
5. `dsr_controller2` spawner
6. `robot_state_publisher`
7. RViz

A0509 full bringup은 다음 값을 넘긴다.

```text
namespace: dsr01
mode: real
controller host: 192.168.137.100:12345
RT host argument: 192.168.137.10
model: a0509
```

관찰된 launch 구현 특성:

- RViz node는 GroupAction 안에 포함되어 직접 시작된다.
- 일부 `OnProcessExit` 지연 handler를 정의하지만 최종 `nodes` 목록에 포함하지 않은 코드가 있다.
- `robot_controller_spawner`와 `joint_state_broadcaster_spawner`가 모두 직접 목록에 들어간다.
- 의도한 순차 시작과 실제 구현이 완전히 일치하는지 별도 검토할 가치가 있다.

### 10.2 robot description과 hardware plugin

A0509 Xacro:

```text
dsr_description2/xacro/a0509.urdf.xacro
```

실제/가상 기본 ros2_control:

```text
dsr_description2/ros2_control/a0509.ros2_control.xacro
plugin: dsr_hardware2/DRHWInterface
```

전달 parameter:

```text
host
rt_host
port
mode
model
update_rate
```

6개 joint 각각 position/velocity command 및 state interface를 가진다.

### 10.3 dsr_hardware2

`DRHWInterface::on_init()`의 주요 동작:

1. hardware parameter와 6개 joint 검증
2. DRCF control connection을 최대 약 10초 재시도
3. access control force request
4. servo on 및 standby 확인
5. controller/DRFL version 확인
6. monitoring callback/version 설정
7. autonomous robot mode 설정
8. real 또는 virtual robot system 설정
9. auto servo-off 비활성화
10. real이면 RT control connection, output 설정, start
11. RT joint velocity/acceleration limit 설정

중요:

- 실제 모드 hardware 초기화가 이미 `start_rt_control()`을 수행한다.
- 따라서 `robot_prep_node`의 start/stop RT helper는 수동 진단/복구 용도에 가깝다.
- virtual controller는 RT connection을 지원하지 않는다고 코드에 명시되어 있다.
- `rt_host`는 `drcf_rt_ip` 변수에 저장되지만 현재 RT 연결 코드는
  `Drfl.connect_rt_control(drcf_ip)`를 호출한다.
- 즉 현재 소스 기준으로 launch의 `rt_host=192.168.137.10`은 실제
  `connect_rt_control()` 대상 주소에 사용되지 않고 control `host`가 다시 사용된다.
- real `read()`는 `read_data_rt()`에서 joint position/velocity를 읽는다.
- ros2_control joint command `write()`는 real에서 `servoj_rt`, virtual에서 `amovej`를 사용한다.
- 텔레옵의 task-space `servol_rt_stream`은 이 joint write 경로가 아니라
  `dsr_controller2` subscriber callback에서 DRFL로 직접 전달된다.

### 10.4 dsr_controller2

controller plugin은 다음 streaming topic을 namespace 상대 이름으로 구독한다.

```text
alter_motion_stream
servoj_stream
servol_stream
speedj_stream
speedl_stream
servoj_rt_stream
servol_rt_stream
speedj_rt_stream
speedl_rt_stream
torque_rt_stream
```

namespace가 `dsr01`이면 주 텔레옵 출력은:

```text
/dsr01/servol_rt_stream
```

`servol_rt_cb`는 message의 `pos`, `vel`, `acc`, `time`을 float array로 복사하여:

```cpp
Drfl->servol_rt(target_pos.data(), target_vel.data(), target_acc.data(), time);
```

를 직접 호출한다. controller callback 자체에는 workspace clamp나 deadman이 없다.
따라서 안전 제한은 상위 텔레옵 노드와 Doosan controller/safety controller에 의존한다.

서비스 범주:

```text
system/*
motion/*
aux_control/*
force/*
io/*
modbus/*
tcp/*
tool/*
drl/*
realtime/*
plc/*
```

이 프로젝트가 직접 사용하는 핵심 서비스:

```text
system/get_robot_state
system/set_robot_control
motion/move_joint
motion/move_spline_joint
motion/move_stop
motion/check_motion
aux_control/get_current_posj
aux_control/get_current_posx
realtime/read_data_rt
realtime/start_rt_control
realtime/stop_rt_control
io/set_tool_digital_output
```

RT 선택 발행:

- `use_rt_topic_pub=true`일 때 `rt_topic_keys`별 `Float32MultiArray` publisher 생성
- topic은 절대 경로 `/rt_topic/<key>`
- 현재 설정은 `use_rt_topic_pub=false`

### 10.5 dsr_msgs2

규모:

```text
18 messages
140 services
3 actions
```

이 프로젝트의 핵심 message:

```text
ServolRtStream:
  float64[6] pos
  float64[6] vel
  float64[6] acc
  float64 time
```

핵심 service:

```text
GetRobotState:
  response robot_state int8
  response success bool

GetCurrentPosx:
  request ref int8
  response Float64MultiArray[] task_pos_info
  response success bool

SetToolDigitalOutput:
  request index int8
  request value int8
  response success bool
```

### 10.6 MoveIt

A0509 MoveIt package:

```text
dsr_moveit_config_a0509
```

설정:

- KDL kinematics
- OMPL 기본
- CHOMP 및 Pilz pipeline 포함
- `dsr_moveit_controller/follow_joint_trajectory`
- 6 joint trajectory

현재 Quest 텔레옵은 MoveIt을 통과하지 않고 ServoL RT로 직접 task pose를 보낸다.
MoveIt collision checking은 prep 및 실시간 텔레옵 경로에 적용되지 않는다.

### 10.7 Gazebo와 MuJoCo

- `dsr_gazebo2`: Gazebo SDF/world 및 spawn launch
- `dsr_mujoco`: MuJoCo scene build, gripper merge, controller config
- `dsr_description2`: Gazebo/MuJoCo용 ros2_control Xacro 분기

주 텔레옵 launch는 기본 `mode=real`이며 이 simulator 경로를 사용하지 않는다.

## 11. 토픽 및 서비스 요약

### 11.1 Quest 텔레옵 토픽

| 이름 | 타입 | 생산자 | 소비자 |
|---|---|---|---|
| `/q2r_right_hand_pose` | `PoseStamped` | Quest2ROS endpoint | xyz mapper, GUI |
| `/q2r_right_hand_inputs` | `OVR2ROSInputs` | Quest2ROS endpoint | roll-lock button, gripper mapper |
| `/vr/target_posx` | `Float64MultiArray` | xyz mapper | safety guard, GUI |
| `/vr/safe_posx` | `Float64MultiArray` | safety guard | ServoL streamer, GUI |
| `/vr/robot_anchor_posx` | `Float64MultiArray` | robot prep | mapper, safety guard, GUI |
| `/vr/teleop_ready` | `Bool` | robot prep | mapper, streamer, button node, GUI |
| `/vr/status` | `String` | 여러 텔레옵 노드 | GUI/operator |
| `/dsr01/servol_rt_stream` | `ServolRtStream` | streamer | dsr_controller2 |

### 11.2 그리퍼 토픽

| 이름 | 타입 | 생산자 | 소비자 |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | 외부 bridge | Joy mapper/probe |
| `/q2r_right_hand_inputs` | `OVR2ROSInputs` | Quest2ROS | Quest input mapper |
| `/jrt_gripper/cmd` | `std_msgs/String` | input mapper | Tool IO driver |

### 11.3 텔레옵 자체 서비스

```text
/vr/recenter
/vr/toggle_roll_lock
/vr/calibrate_xy_yaw_to_x_plus
/vr/prepare_robot
/vr/set_robot_anchor_to_current_tcp
/vr/set_live_robot_output
/vr/hold_servol
/vr/stop_robot
/vr/reset_safe_off
/vr/start_rt_control
/vr/stop_rt_control
```

## 12. 설정값과 문서의 불일치

### 12.1 quest_a0509_teleop README 대 현재 YAML

README 예시와 현재 실행 YAML이 다른 항목:

| 항목 | README 설명/예시 | 현재 YAML |
|---|---|---|
| `pose_filter_alpha` | 0.2 | 0.4 |
| `orientation_filter_alpha` | 0.2 예시 | 0.4 |
| `scale_rpy` | `[0.5, 0.5, 0.5]` | `[0.0, 0.7, 0.0]` |
| `rot_axis_sign` | `[1.0, -1.0, -1.0]` | `[-1.0, -1.0, -1.0]` |
| orientation limit | `[15, 15, 20]` | `[90, 90, 90]` |
| prep joint J5 | README에 60도 | 코드/YAML은 30도 |

결론:

- 실제 동작 판단에는 README가 아니라 `config/xyz_position_only.yaml`을 우선해야 한다.
- 현재 시스템은 3축 자세가 아니라 Quest roll로 robot `ry`만 조작하는 특수 설정이다.

### 12.2 robot anchor 서비스 설명

- README는 current TCP anchor가 prepare의 대안처럼 읽힌다.
- 코드는 prepare 성공으로 `teleop_ready=true`가 되기 전 호출을 거부한다.

### 12.3 Tool DO value 주석

- `.srv` 주석: 0 ON, 1 OFF
- Doosan constants와 JRT 코드: 1 ON, 0 OFF
- 실기 전 전기적 검증 필요

### 12.4 a0509_vr_teleop RT monitor

- 패키지 config는 `/rt_topic/actual_tcp_position` 구독
- Doosan config는 RT topic 발행 비활성
- 따라서 기본 실행에서 actual TCP marker가 갱신되지 않을 수 있다.

## 13. 안전 분석

### 13.1 현재 존재하는 보호

주 운영 파이프라인:

- launch 기본 dry-run
- prepare 전 target 발행 차단
- prepare 전 live-enable 차단
- runtime live-enable service 필요
- robot state 확인
- XYZ workspace clamp
- anchor 기준 orientation clamp
- safety guard ramp
- streamer 추가 ramp
- live-enable 시 실제 TCP에서 시작
- unsafe robot state/service 예외 시 live gate 해제
- GUI 위험 동작 체크 및 confirmation

그리퍼:

- A/B 동시 입력은 stop
- 입력 watchdog
- 반대 DO를 먼저 OFF
- interlock delay
- pulse 기본
- startup/shutdown all-off
- service 실패 시 stop 재시도

### 13.2 높은 우선순위 검토 항목

#### P0. 실기 배선 및 Tool DO 극성 미검증

`.srv` 주석과 구현 상수가 상충한다. 잘못된 극성은 gripper가 launch 또는 shutdown 시
예상과 반대로 동작하게 할 수 있다.

조치:

- 그리퍼 분리 상태에서 multimeter로 DO1/DO2 0/1 실제 전압 확인
- 공통 ground와 PNP 입력 호환 확인
- JRT pinout을 제조사 문서로 별도 고정

#### P0. Quest input timeout이 live output을 disable하지 않음

현재 흐름:

```text
Quest timeout
 -> mapper target 발행 중단
 -> safety guard는 마지막 target 계속 발행
 -> streamer는 마지막 safe target 계속 ServoL 발행
```

이는 last-pose hold로는 합리적일 수 있으나, operator deadman release와 동일하지 않다.

권장 설계:

- `/vr/input_fresh` 또는 stamped target 추가
- streamer에서 freshness timeout 시 즉시 `live_enabled=false`
- 필요 정책에 따라 hold 3회 후 `MoveStop` 또는 RT stop
- timeout 이유를 latched diagnostic으로 발행

#### P0. 주 텔레옵에 motion deadman이 없음

현재 grip은 roll lock 용도이고 A/B는 gripper 용도다. 로봇 arm ServoL을 지속 허용하는 별도
hold-to-run 입력이 없다.

권장:

- 명시적 deadman field와 topic 정의
- mapper뿐 아니라 최종 streamer gate에서도 독립 확인
- input timeout과 deadman false를 같은 최종 출력 차단 경로로 연결

#### P1. prep가 collision-aware가 아님

J3/J5 escape와 10도 waypoint는 수치적 완만함만 제공한다. 주변 장비, 케이블, gripper,
작업물과의 충돌을 검사하지 않는다.

권장:

- 고정된 검증 pose만 허용하거나 MoveIt planning scene 사용
- 실기 환경별 허용 joint corridor 설정
- prep 중 외부 stop/deadman 상태 확인

#### P1. 독립 gripper launch의 실제 출력 기본값

독립 launch의 `dry_run=false`는 오조작 가능성을 높인다.

권장:

- 모든 launch 기본 `dry_run=true`
- 실제 출력에는 별도 runtime arm service 추가

#### 해결됨. Doosan dependency 재현성

업로드 구조에서는 다음 방식으로 기존 재현성 문제를 해결했다.

- Doosan upstream `ec92425` 기반 사용자 fork commit `8f4fa87`
- 상위 저장소의 `.gitmodules`
- `src/doosan-robot2` gitlink를 `8f4fa87`로 고정

사용자는 clone 시 submodule을 함께 받아야 한다. 사용자 fork나 branch를 삭제하면
submodule 복원이 다시 깨지므로 fork와 branch는 유지해야 한다.

#### P1. `rt_host`와 실제 RT 연결 주소가 다름

launch와 Xacro는 `rt_host`를 별도 parameter로 전달하지만, 현재
`dsr_hardware2/src/dsr_hw_interface2.cpp`는 이를 저장한 뒤 실제 연결에서
`Drfl.connect_rt_control(drcf_ip)`를 호출한다.

조치:

- 사용 중인 DRFL 버전에서 `connect_rt_control()`이 요구하는 주소 의미를 확인
- `rt_host`가 robot RT endpoint라면 `drcf_rt_ip`를 사용하도록 수정
- `host`와 `rt_host`가 다른 현재 실기 설정에 대한 통합 테스트 추가

### 13.3 중간 우선순위 검토 항목

#### P2. 주 운영 패키지 테스트 없음

가장 복잡한 3,200줄 Python 패키지에 pytest가 0개다.

필요 테스트:

- axis map/sign/scale
- quaternion anchor-relative 변환
- Euler continuity
- roll-only lock/unlock
- input timeout
- jump rejection 반복 입력
- safety clamp/ramp
- live gate truth table
- robot state failure
- prep waypoint planner
- anchor 및 teleop_ready 전달

#### P2. package.xml 의존성 누락

`quest_a0509_teleop`에 기능상 필요한 다음 의존성이 명시되지 않았다.

```text
dsr_msgs2
dsr_bringup2
jrt_gripper_io
launch
launch_ros
python3-tk 또는 배포 방식에 맞는 GUI 의존성
quest2ros 또는 별도 외부 의존성 문서
```

`a0509_vr_teleop`도 `ament_python` buildtool 및 launch 관련 메타데이터를 보강할 필요가 있다.

#### P2. namespace parameter 분리

`name`, `robot_namespace`, `doosan_servol_topic`, gripper service 이름이 각각 독립이다.
하나만 변경하면 조용히 다른 namespace를 볼 수 있다.

권장:

- 단일 `robot_name`에서 topic/service를 조합
- launch 시 불일치 검증

#### P2. Quest2ROS 절대 경로

`/home/salim2001/quest2ros2_ws/...`에 고정된 fallback은 다른 사용자와 Docker에서 깨진다.

권장:

- overlay workspace를 정상 source
- package dependency 또는 configurable empty default
- shared-library 수동 preload 제거 가능성 검토

#### P2. Docker와 호스트 ROS domain 불일치

현재:

```text
host ROS_DOMAIN_ID=16
Docker ENV ROS_DOMAIN_ID=29
```

명시적으로 맞추지 않으면 container와 host ROS graph가 서로 보이지 않는다.

### 13.4 낮은 우선순위 및 유지보수 항목

- `robot_prep_node.py`에 중복된 unreachable `raise RuntimeError` 한 줄이 있음.
- `MoveSplineJoint.request.pos_cnt` 할당이 연속 두 번 중복됨.
- `quest_jrt_gripper`은 빈 scaffold로 혼동을 줄 수 있음.
- `/vr/status`가 여러 노드의 자유형 문자열을 하나의 topic에 섞어 구조적 처리가 어렵다.
- 상태 topic에 표준 diagnostics 또는 구조화 message가 없음.
- XY yaw calibration 결과가 영구 저장되지 않음.
- `build/`, `install/`, `log/`가 크므로 전달/백업 시 원본 `src/`와 혼동하지 않아야 함.

## 14. 빌드, 테스트, 구문 검증 결과

### 14.1 2026-07-24 수행한 빌드

성공:

```text
a0509_vr_teleop
quest_a0509_teleop
jrt_gripper_io
dh_robot_rviz
dsr_controller2
```

빌드 명령의 핵심:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select ...
```

`dsr_controller2`의 현재 로컬 수정 C++도 target build 및 install에 성공했다.

### 14.2 테스트

결과:

```text
Summary: 19 tests, 0 errors, 0 failures, 0 skipped
```

패키지별:

```text
a0509_vr_teleop     10 passed
jrt_gripper_io       9 passed
quest_a0509_teleop   0 collected
```

### 14.3 Python 구문

다음 소스에 `python3 -m compileall -q` 성공:

```text
src/a0509_vr_teleop
src/quest_a0509_teleop
src/jrt_gripper_io
src/dh_robot_rviz
```

### 14.4 launch 분석

다음 통합 launch의 `--show-args` 평가 성공:

```text
quest_a0509_teleop/a0509_full_bringup_with_gripper.launch.py
```

Doosan update rate 100 Hz와 외부 저장소 Git 정보가 launch 평가 중 정상 출력되었다.

### 14.5 URDF

`dh_robot_rviz` Xacro를 생성한 뒤 `check_urdf` parsing에 성공했다.

### 14.6 수행하지 않은 검증

- 전체 26개 패키지의 현 시점 clean rebuild
- 전체 Doosan test suite
- 실제 robot service call
- 실제 ServoL RT publish
- 실제 Tool DO call
- Quest2ROS endpoint 연결
- GUI/RViz 화면 검증

## 15. 현재 런타임 스냅샷

관측 시점: 2026-07-24

ROS 그래프:

```text
ros2 node list:
  사용자 노드 없음

ros2 topic list:
  /parameter_events
  /rosout

ros2 service list:
  서비스 없음
```

즉 보고서 작성 시점에는 Doosan bringup, Quest endpoint, 텔레옵, 그리퍼 노드가 실행 중이지 않다.

네트워크:

```text
wlp195s0: 192.168.0.142/24
tailscale0: 100.127.9.78/32
docker0: DOWN
```

설정된 robot control IP:

```text
192.168.137.100
```

현재 host에는 `192.168.137.0/24` 직접 인터페이스가 없으며 해당 주소는 기본 gateway
`192.168.0.1`로 route된다. 1회 ping 결과 응답이 없었다.

해석:

- 현재 물리 네트워크 상태에서는 설정된 robot IP에 직접 연결할 수 없는 가능성이 높다.
- 실제 bringup 전에 유선 NIC 또는 적절한 `192.168.137.x/24` 설정을 확인해야 한다.
- ping 무응답만으로 로봇 전원/방화벽/ICMP 정책까지 단정할 수는 없다.

## 16. Docker 상태

Docker 관련 파일은 상위 Git 업로드 대상에 포함했다.

`Dockerfile`:

- base: `osrf/ros:humble-desktop`
- ROS control, MoveIt, Gazebo, ros-gz 의존성 설치
- `COPY src ./src`
- rosdep install
- `colcon build --symlink-install`
- `DRCF_VER` build argument 지원
- 기본 `ROS_DOMAIN_ID=29`

entrypoint:

```text
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
```

재현성 및 실행 시 고려사항:

- `src/doosan-robot2`는 사용자 fork의 commit `8f4fa87`를 가리키는 submodule이므로
  clone 및 Docker build 전에 submodule 초기화가 필요하다.
- Quest2ROS 패키지는 이 workspace에 포함되지 않는다.
- Quest input 노드의 절대 `/home/salim2001/...` fallback은 container 내부에서 유효하지 않다.
- GUI는 X11/Wayland bridge가 추가로 필요하다.
- Doosan virtual emulator가 별도 Docker container를 실행하므로 host Docker socket과 image가 필요하다.

## 17. 권장 실행 순서

### 17.1 정적 및 dry-run

```bash
cd /home/salim2001/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py \
  start_robot_bringup:=false \
  start_endpoint:=false \
  dry_run:=true
```

전제:

- Quest2ROS endpoint는 별도 workspace에서 실행
- 또는 실제 Quest topic 대신 별도 test publisher 준비

확인:

```text
/q2r_right_hand_pose
/vr/teleop_ready
/vr/target_posx
/vr/safe_posx
/vr/status
/jrt_gripper/cmd
```

현재 설정에서는 prep node가 실제 robot service 없이는 `teleop_ready=true`를 만들 수 없으므로
완전한 pure dry-run용 readiness override 또는 test fixture가 필요할 수 있다.

### 17.2 실제 bringup 전 점검

1. host NIC를 robot subnet에 설정
2. `192.168.137.100:12345` 접근 확인
3. `rt_host` 의미와 host/robot RT NIC 설정 확인
4. `ROS_DOMAIN_ID` 통일
5. Doosan access control 권한 확인
6. Tool DO 극성 확인
7. gripper 전원과 공통 ground 확인
8. workspace와 orientation limit 재확인
9. prep joint path 주변 충돌물 제거
10. physical emergency stop 접근 가능 상태 확인

### 17.3 실제 출력 승인 순서

코드상 의도:

```text
launch dry_run=false
 -> robot bringup 정상
 -> Quest pose 정상
 -> /vr/prepare_robot 성공
 -> robot anchor와 recenter 확인
 -> /vr/target_posx 및 /vr/safe_posx 검증
 -> /vr/set_live_robot_output true
```

중단:

```text
/vr/set_live_robot_output false
/vr/hold_servol
/vr/stop_robot
physical E-stop
```

## 18. 개선 우선순위 제안

### 18.1 1단계: 재현성과 안전 gate

완료:

1. Doosan 드라이버를 사용자 fork submodule commit `8f4fa87`로 pin
2. 로컬 access-control patch와 RT topic key 변경을 commit

남은 항목:

1. Quest input freshness를 최종 streamer까지 전달
2. explicit arm deadman 추가
3. gripper launch 기본 dry-run으로 변경
4. Tool DO 극성을 하드웨어 시험으로 확정하고 문서화

### 18.2 2단계: 테스트

1. `quest_a0509_teleop` pure math를 별도 모듈로 분리
2. mapper quaternion/roll-only 테스트
3. safety freshness 테스트
4. streamer gate state-machine 테스트
5. prep planner 및 service failure 테스트
6. launch test로 dry-run에서 robot publisher 부재 확인

### 18.3 3단계: 구성 정리

1. `a0509_vr_teleop`과 `quest_a0509_teleop`의 역할을 명시하거나 통합
2. README를 현재 YAML과 동기화
3. namespace parameter 단일화
4. Quest2ROS 절대 경로 제거
5. 구조화 status/diagnostic message 도입
6. 빈 `quest_jrt_gripper` 제거 또는 실제 패키지로 완성

## 19. 다른 모델이 수정 작업을 시작할 때 읽을 파일 순서

주 텔레옵 수정:

1. `src/quest_a0509_teleop/config/xyz_position_only.yaml`
2. `src/quest_a0509_teleop/launch/a0509_full_bringup.launch.py`
3. `src/quest_a0509_teleop/quest_a0509_teleop/xyz_mapper_node.py`
4. `src/quest_a0509_teleop/quest_a0509_teleop/safety_guard_node.py`
5. `src/quest_a0509_teleop/quest_a0509_teleop/servol_rt_streamer_node.py`
6. `src/quest_a0509_teleop/quest_a0509_teleop/robot_prep_node.py`

그리퍼 수정:

1. `src/jrt_gripper_io/docs/jrt_gripper_io_test.md`
2. `src/jrt_gripper_io/jrt_gripper_io/gripper_logic.py`
3. `src/jrt_gripper_io/jrt_gripper_io/jrt_tool_io_driver_node.py`
4. `src/jrt_gripper_io/jrt_gripper_io/quest_inputs_ab_gripper_mapper_node.py`
5. `src/jrt_gripper_io/launch/jrt_gripper_robot_bringup.launch.py`

Doosan 연결 수정:

1. `src/doosan-robot2/dsr_description2/ros2_control/a0509.ros2_control.xacro`
2. `src/doosan-robot2/dsr_hardware2/src/dsr_hw_interface2.cpp`
3. `src/doosan-robot2/dsr_controller2/config/dsr_controller2.yaml`
4. `src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp`
5. `src/doosan-robot2/dsr_bringup2/launch/dsr_bringup2_rviz.launch.py`

## 20. 다른 AI 모델용 압축 컨텍스트

아래 블록은 토큰이 제한된 다른 모델에 우선 전달할 수 있는 요약이다.

```text
Project: ROS 2 Humble Meta Quest teleoperation for Doosan A0509 plus JRT gripper.
Workspace: /home/salim2001/ros2_ws
OS/Python: Ubuntu 22.04, Python 3.10

Primary arm pipeline:
/q2r_right_hand_pose PoseStamped(m, quaternion)
 -> quest_a0509_teleop/xyz_mapper_node
 -> /vr/target_posx [mm, deg]
 -> safety_guard_node
 -> /vr/safe_posx
 -> servol_rt_streamer_node
 -> /dsr01/servol_rt_stream ServolRtStream
 -> dsr_controller2 -> DRFL servol_rt()

Current mapping:
robot X <- +0.5 Quest Y
robot Y <- -0.5 Quest X
robot Z <- +0.5 Quest Z
orientation is roll-only:
robot ry <- -0.7 Quest relative roll; rx/rz locked to robot anchor.

Arm output gates:
dry_run must be false;
/vr/prepare_robot must succeed and /vr/teleop_ready must be true;
/vr/set_live_robot_output true must be called;
robot_state must be 1 or 2.

Safety:
XYZ box [250,-350,150]..[650,350,600] mm;
orientation delta currently +/-90 deg;
safety ramp 20 mm and 2 deg per 30 Hz tick;
stream ramp 7.5 mm and 3 deg per 10 Hz tick.

Critical issue:
Quest input timeout stops mapper publication, but safety_guard keeps republishing
the last target and streamer keeps sending the last safe pose. No explicit arm
deadman exists in the primary pipeline. Add freshness/deadman at final output gate.

Robot prep:
joint target [0,0,90,0,30,0] deg;
J5/J3 escape waypoints then <=10 deg joint steps;
not MoveIt collision-aware;
reads current TCP, publishes robot anchor, recenters Quest, sets teleop_ready.

Gripper pipeline:
/q2r_right_hand_inputs OVR2ROSInputs
 -> A button_lower=close, B button_upper=open, else/timeout=stop
 -> /jrt_gripper/cmd
 -> jrt_tool_io_driver_node
 -> /dsr01/io/set_tool_digital_output
DO1 close, DO2 open, code assumes 1=ON and 0=OFF.
The dsr_msgs2 .srv comment says the opposite, so verify electrically.
Default pulse: opposite OFF, wait 0.05 s, requested ON 0.20 s, requested OFF.

Repositories:
root base is main HEAD 28edfb3;
upload branch is agent/publish-latest-workspace.
src/doosan-robot2 is a submodule targeting
https://github.com/kim-salim/doosan-robot2.git.
The gitlink pins branch agent/a0509-runtime-changes commit 8f4fa87,
based on doosan-robotics/doosan-robot2 humble commit ec92425.
That commit adds access-control requests on controller activation/TP init
and changes the RT topic key to actual_tcp_position;
RT topic publishing remains disabled.

Validation on 2026-07-24:
custom packages and dsr_controller2 build successfully;
19 tests pass (10 a0509_vr_teleop, 9 jrt_gripper_io);
quest_a0509_teleop has zero tests;
no ROS nodes/services currently running;
robot IP 192.168.137.100 did not answer ping;
host is 192.168.0.142 and has no direct 192.168.137.0/24 interface.
```

## 21. 사실과 추론의 구분

확인된 사실:

- 파일, Git diff, package manifest, launch/config, source code에서 직접 확인한 값
- 현재 빌드/테스트 명령 결과
- 현재 ROS graph와 host network 명령 결과

추론:

- `quest_a0509_teleop`이 주 운영 파이프라인이라는 판단은 기능 완성도와 통합 launch를 근거로 함
- access-control patch가 현장 권한 문제 대응이라는 해석
- robot subnet 미설정 때문에 현재 연결 가능성이 낮다는 판단
- input timeout 후 지속 ServoL을 last-pose hold 정책으로 볼 수 있다는 해석

실기에서만 확정 가능한 항목:

- Tool DO 실제 전압 극성
- JRT pinout과 동작 mode
- A0509 controller 권한/firmware 상태
- RT host routing
- workspace가 실제 작업 셀에 안전한지 여부
- prep joint 경로의 충돌 여부
- Quest 축 방향과 사용자 체감 방향

## 22. 최종 상태

현재 워크스페이스는 소프트웨어 구조상:

- Doosan driver와 A0509 model을 포함하고,
- Quest2ROS pose를 task-space ServoL RT로 변환하며,
- 실제 출력에 다중 gate를 적용하고,
- Quest input으로 JRT Tool DO 그리퍼를 제어하는,
- 빌드 가능한 ROS 2 Humble 통합 작업 공간이다.

그러나 실제 로봇 운용 준비 완료로 판단하려면 최소한 다음 세 가지가 먼저 해결되어야 한다.

1. Quest input freshness/deadman을 최종 출력 gate에 연결
2. JRT Tool DO 극성과 배선 검증
3. prep joint 경로와 `rt_host` 연결 동작의 실기 검증
