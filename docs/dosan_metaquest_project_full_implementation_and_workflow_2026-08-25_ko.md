# Doosan A0509–MetaQuest–LeRobot 프로젝트 통합 구현 및 전체 흐름

## 문서 정보

- 문서 목적: 별도의 GPT/LLM이 이 저장소와 실험 흐름을 단독으로 분석할 수 있도록 현재 구현, 데이터, 모델, 검증 결과, 한계와 다음 단계까지 하나의 문서에 통합한다.
- 기준 시점: 2026-08-25, Asia/Seoul
- 기준 저장소: /home/rvlab/Dosan-MetaQuest-teleoperation-project
- 기준 실행 장치: Jetson Thor, Ubuntu 24.04 arm64, ROS 2 Jazzy
- 기준 LeRobot 환경: /home/rvlab/venvs/lerobot, LeRobot 0.6 계열, Python 3.12
- 작성 원칙: 현재 파일과 로컬 데이터/모델 메타데이터를 우선하고, 과거 문서의 실험 결과는 날짜와 검증 범위를 함께 표시한다.
- 보안 원칙: 서버 비밀번호, 개인 인증 정보, 토큰은 의도적으로 제외했다.
- 변경 원칙: 이 문서를 만들면서 기존 코드·데이터셋·모델은 수정하지 않았다.

> 중요: 이 저장소의 현재 작업 트리는 clean 상태가 아니다. 확인 시점에 tracked modified 20개, staged added 1개, untracked 57개가 있었다. 이 문서는 커밋된 main 브랜치만이 아니라 현재 로컬 working tree의 구현을 설명한다. 따라서 재현 가능한 기준점을 만들려면 검증 후 커밋·태그·체크섬 정리가 필요하다.

---

## 0. GPT 분석용 핵심 요약

이 프로젝트는 다음 여섯 계층으로 구성된다.

1. MetaQuest를 이용한 Doosan A0509 실시간 원격조작
2. JRT 그리퍼 Tool Digital Output 제어
3. MUX-first 안전 제어와 명시적인 Live 권한 관리
4. MetaQuest teacher action과 세 카메라를 이용한 LeRobot 데이터 수집
5. 서버에서 ACT 정책 학습 후 Jetson에서 비동기 30 Hz 정책 실행
6. 여러 정책을 잇는 A→Bridge→B Task-C와, T1~T6 demonstration의 semantic-only 분할

현재 가장 중요한 상태는 다음과 같다.

- MetaQuest → A0509 실기 제어 baseline은 물리 시험을 거쳐 고정된 파라미터가 있다.
- JRT 그리퍼 매핑은 DO1=OPEN, DO2=CLOSE로 검증되어 있다.
- LeRobot shadow recording은 13차원 state, 7차원 action, RGB 카메라 3대를 30 Hz로 기록한다.
- T1~T6 데이터셋과 각 ACT 최종 체크포인트가 Jetson에 존재한다.
- 여섯 ACT 모델은 동일한 모델 구조를 쓰고, 학습 step만 40k/50k/80k로 다르다.
- 기존 Task-C는 두 개의 초기 pick/transport 정책을 Cartesian Bezier Bridge로 연결했고, 고정된 작업 셀에서 V0 전체 실행 2회, representative V1 전체 실행 1회의 물리 성공 기록이 있다.
- T1~T6 semantic V3 분석은 gripper event와 Cartesian trajectory를 이용해 완료되었고 모든 산출물 체크섬이 통과했다.
- semantic V3는 의미 구간만 표현한다. 전환 적합도, 전환 phase, 구형 경계, Bridge, 상위 플래너의 조합 결정은 의도적으로 포함하지 않는다.
- T1~T6 semantic 그래프를 실제 N-skill 실행기로 연결하는 상위 SkillGraphExecutor는 아직 구현되지 않았다.
- TASK_DESCRIPTION은 현재 ACT의 언어 입력이 아니다. 데이터셋 메타데이터이자 외부 semantic/planner 해석 정보다.
- T1~T6 전체 작업의 반복 성공률은 아직 체계적으로 검증되지 않았다. 체크포인트 존재와 정책 로드는 실제 task success를 뜻하지 않는다.
- Diffusion 경로도 별도로 구현되어 실기 10초/40초 실행까지 했지만, 실제 rollout에서 close를 예측하지 못한 문제가 있어 현재 주력은 ACT다.

### 시스템 상태를 한 문장으로 표현하면

MetaQuest로 안전하게 수집한 multi-view demonstration을 ACT whole-task 정책으로 학습하고, 기존에는 두 정책 사이의 free-transport 구간을 물리적으로 연결했으며, 현재는 여섯 whole-task dataset을 gripper event 기반 semantic subgoal graph로 구조화한 단계다. 다음 핵심 과제는 semantic segment를 실제 skill policy와 검증 가능한 상위 실행기로 연결하는 것이다.

---

## 1. 프로젝트의 목표와 발전 과정

### 1.1 최초 목표

최초 목표는 MetaQuest 오른손 pose와 버튼을 이용하여 Doosan A0509와 JRT 그리퍼를 실시간으로 조작하는 것이었다.

팔 경로:

~~~text
Quest 오른손 pose
→ Quest2ROS/TCP endpoint
→ 좌표·자세 mapper
→ calibration/prepare gate
→ command MUX
→ workspace safety guard
→ ServoL RT streamer
→ Doosan A0509
~~~

그리퍼 경로:

~~~text
Quest A/B
→ MetaQuest gripper mapper
→ command MUX
→ JRT gripper driver
→ Doosan Tool Digital Output
→ JRT gripper
~~~

### 1.2 LeRobot 확장

원격조작이 안정화된 뒤 다음 확장이 이루어졌다.

- MetaQuest의 accepted target을 teacher action으로 기록
- 실제 joint/TCP/gripper state를 observation으로 기록
- front, side, ZED left RGB 영상을 동기화해 기록
- ACT 정책을 서버에서 학습
- Jetson에서 policy_dry_run 및 policy_live 실행
- MetaQuest와 LeRobot이 같은 하위 안전 계층을 공유하도록 MUX 도입

### 1.3 Task-C 정책 조합

다음 단계에서는 Task A와 Task B를 단순히 끝까지 연속 실행하지 않고, A의 grasp 이후 운반 구간과 B의 release 이전 운반 구간을 연결했다.

~~~text
Task C
= A retained prefix
+ feasible Cartesian Bridge
+ B retained suffix
~~~

이 과정에서 다음이 구현되었다.

- gripper close/open 기반 semantic candidate 검출
- closed-transport 구간 추출
- velocity-matched cubic Bezier
- 전체 C 경로 길이 기반 후보 선택
- 두 ACT 모델 GPU 상주
- fresh B inference
- endpoint settle
- atomic handoff
- tracking backpressure
- fail-closed external gate

### 1.4 Semantic skill 분할

현재 단계에서는 whole-task T1~T6를 그리퍼 이벤트와 task 문장에 따라 의미 구간으로 나눈다.

중요한 설계 결정:

- semantic event는 의미 anchor다.
- semantic event가 곧 policy 전환점인 것은 아니다.
- 지금 산출물은 semantic 정보만 담당한다.
- 어느 semantic span의 중간에서 다음 정책으로 전환할지는 향후 상위 planner가 판단한다.
- 따라서 현재 semantic V3에는 구형 경계, 전환 적합도, 가장 유사한 실제 episode, Bridge 비용을 넣지 않는다.
- 프레임 번호나 영상을 평균하지 않는다.
- Cartesian 경로만 semantic span별 arc-length phase로 표준화한다.

---

## 2. 시간순 구현 이력

| 날짜 | 주요 내용 | 현재 의미 |
|---|---|---|
| 2026-08-07 | MetaQuest 실기 accepted baseline 고정 | 현재 teacher recording과 하위 제어 기준 |
| 2026-08-08 | JRT gripper DO 매핑 재검증 | DO1 OPEN, DO2 CLOSE 확정 |
| 2026-08-09 | 에피소드 reset/review orchestrator 설계 | T1~T6 수집에 사용된 반복 수집 기반 |
| 2026-08-11 | Diffusion 독립 학습·런타임 경로 구축 | 실험적 대안, ACT와 분리 |
| 2026-08-13 | Task-C bounded live 개발 및 실패 원인 축적 | tracking/ramp/freshness 설계 근거 |
| 2026-08-14 | Task-C V0 전체 물리 실행 2회 성공 | 고정 장면에서 A→Bridge→B 실증 |
| 2026-08-18 | representative Bridge V1 구축 | 30개 episode 표준화 기반 경계 |
| 2026-08-19 | V1 endpoint-boundary 물리 실행 성공 | fresh B endpoint handoff 실증 |
| 2026-08-20~24 | T1~T5 데이터 수집·ACT 학습 | whole-task skill library 확장 |
| 2026-08-23~25 | T1~T6 semantic-only V3 생성 | 현재 semantic catalog의 기준 |
| 2026-08-24~25 | T6 수집·50k 학습·로컬 복사·semantic 분석 | 최신 추가 task |

---

## 3. 실행 환경과 하드웨어 계약

### 3.1 Jetson Thor

- OS: Ubuntu 24.04
- Architecture: arm64
- ROS: ROS 2 Jazzy
- Python: 3.12
- LeRobot venv: /home/rvlab/venvs/lerobot
- Diffusion 전용 venv: /home/rvlab/venvs/lerobot-diffusion
- 프로젝트: /home/rvlab/Dosan-MetaQuest-teleoperation-project
- 데이터셋: /home/rvlab/lerobot_datasets
- ACT 모델: /home/rvlab/lerobot_models/models
- legacy Task-C 모델: /home/rvlab/lerobot_models 아래의 기존 bs16 모델

### 3.2 Doosan A0509

- 모델: a0509
- ROS namespace: /dsr01
- controller: dsr_controller2
- 실기 controller 주소 기본값: 192.168.137.100
- port: 12345
- controller update rate: 100 Hz
- ServoL target rate: 30 Hz
- ServoL topic: /dsr01/dsr_controller2/servol_rt_stream

### 3.3 MetaQuest

- 입력 pose: /q2r_right_hand_pose
- 입력 buttons: /q2r_right_hand_inputs
- Quest endpoint는 TCP port 10000을 사용한다.
- XY/+X calibration은 세션 단위로 필수다.
- 각 episode 시작 때는 전체 calibration을 다시 하지 않고 /vr/recenter로 상대 원점만 갱신한다.

### 3.4 카메라

| key | 장치 | 기록 크기 | FPS | 비고 |
|---|---|---:|---:|---|
| front | Logitech C920 | 640×480 RGB | 30 | MJPG capture |
| side | Logitech C920, serial 947C90BF | 640×480 RGB | 30 | MJPG capture |
| zed_rgb | ZED2 left RGB | 672×376 RGB | 30 | 1344×376 UVC stereo frame의 left half |

원본 ACT 데이터에는 ZED depth, disparity, point cloud, right RGB가 포함되지 않는다.

### 3.5 학습 서버

- x86_64 GPU server
- 마지막 운용 기준 GPU 0과 GPU 1을 독립 학습에 사용할 수 있었다.
- 사용 GPU는 CUDA_VISIBLE_DEVICES=0 또는 CUDA_VISIBLE_DEVICES=1로 격리했다.
- 데이터 경로의 컨테이너 관점: /workspace/data
- 출력 경로의 컨테이너 관점: /workspace/outputs
- 저장소의 기존 compose.yaml은 GPU 0만 고정하는 과거 단일-GPU 계약을 담고 있다. 현재의 두 GPU 병렬 운용은 직접 CUDA_VISIBLE_DEVICES를 지정한 학습 명령으로 수행되었으며 compose 문서와 완전히 동기화되어 있지 않다.
- 서버 접속 자격 증명은 이 문서에 포함하지 않는다.

---

## 4. 저장소 구성

~~~text
Dosan-MetaQuest-teleoperation-project/
├── src/
│   ├── quest2ros/
│   │   └── Quest message interface
│   ├── ros_tcp_communication/
│   │   └── Unity/Quest TCP endpoint
│   ├── quest_a0509_teleop/
│   │   ├── MetaQuest pose mapping
│   │   ├── calibration/recenter
│   │   ├── command MUX
│   │   ├── safety guard
│   │   ├── ServoL streamer
│   │   └── robot prepare
│   ├── jrt_gripper_io/
│   │   ├── Quest A/B mapping
│   │   ├── JRT driver
│   │   └── Tool DO readback/diagnostics
│   ├── lerobot_robot_doosan_a0509/
│   │   ├── LeRobot robot/teleoperator plugin
│   │   ├── recording reset/encoder/scheduler
│   │   ├── ACT async rollout
│   │   ├── Diffusion async rollout
│   │   └── Task-C shadow/live rollout
│   └── doosan-robot2/
│       └── official Doosan ROS 2 Jazzy source
├── scripts/
│   ├── recording wrappers
│   ├── ACT/Diffusion run wrappers
│   ├── live gates
│   ├── Task-C wrappers
│   └── scheduling/realtime tools
├── offline_tools/
│   ├── task_c_bridge_v0/
│   ├── task_c_bridge_v1/
│   └── semantic_segmentation/
├── docs/
│   ├── accepted baseline
│   ├── MUX architecture
│   ├── reset workflow
│   ├── Task-C validation
│   ├── semantic analysis report
│   └── artifacts/t1...t6 semantic V3
└── docker/lerobot_training/
    └── x86_64 server training environment
~~~

### 현재 주력에서 제외된 경로

- src/a0509_vr_teleop은 과거 UDP 기반 구현이다.
- dh_robot_rviz는 현재 Jetson 실기 빌드의 핵심 경로가 아니다.
- Gazebo/MoveIt은 현재 실기 bringup 기본 경로에 포함되지 않는다.

---

## 5. 전체 제어 아키텍처

### 5.1 공통 명령 그래프

~~~mermaid
flowchart LR
    MQ[MetaQuest mapper] -->|/control/metaquest/target_posx| MUX
    ACT[LeRobot ACT] -->|/control/lerobot/target_posx| MUX
    MUX[DISABLED/METAQUEST/LEROBOT MUX] -->|/vr/target_posx| SAFE[Workspace & orientation guard]
    SAFE -->|/vr/safe_posx| STREAM[ServoL RT streamer]
    STREAM -->|/vr/commanded_posx| ACK[Command acknowledgement]
    STREAM -->|30 Hz ServolRtStream| ROBOT[Doosan A0509]

    MQG[MetaQuest A/B] -->|/control/metaquest/gripper_cmd| MUX
    ACTG[Policy gripper 0..1] -->|/control/lerobot/gripper_target| MUX
    MUX -->|/jrt_gripper/cmd| JRT[JRT Tool I/O driver]
    JRT -->|DO1/DO2| GRIP[JRT gripper]
~~~

### 5.2 권한 분리

정책 프로세스가 직접 로봇 출력 권한을 소유하지 않는다.

~~~text
정책 프로세스
- 모델 load
- camera/state observation
- target proposal
- MUX 선택 권한 없음
- Live ON 권한 없음

외부 gate
- 시작 pose와 freshness 검증
- MUX를 LEROBOT으로 선택
- Live ON
- 실행 중 watchdog
- 종료/예외 시 Live OFF + DISABLED
~~~

이 분리는 단일 ACT 실행과 Task-C 실행 모두의 핵심 안전 원칙이다.

---

## 6. MetaQuest 원격조작 구현

### 6.1 좌표 mapping

accepted configuration의 위치 mapping:

| 항목 | 값 |
|---|---|
| scale_xyz | [0.85, 0.85, 0.85] |
| axis map | robot X ← Quest Y, robot Y ← Quest X, robot Z ← Quest Z |
| axis sign | [1, -1, 1] |
| publish rate | 30 Hz |
| raw input timeout | 0.5 s |
| tracking invalid | 0.3 s |
| max position jump | 0.20 m |
| max rotation jump | 45 deg |

현재 orientation은 Doosan posx의 intrinsic ZYZ 표현을 사용한다. 일반적인 RPY로 간주하면 안 된다.

accepted 설정에서는 실제로 robot Ry에 대응하는 제한된 orientation component만 scale 0.7로 활성화되어 있다.

### 6.2 pose resampling

Quest packet은 평균적으로 빠르지만 burst 형태로 들어올 수 있다. mapper는 다음을 수행한다.

- receive-time pose buffer
- 1초, 최대 128 sample
- 30 Hz 고정 tick에서 resample
- 50 ms interpolation delay
- 20 ms까지 full prediction
- 80 ms까지 prediction decay
- 이후 hold
- low-pass filter

목적은 Quest packet burst를 그대로 robot command jitter로 전달하지 않는 것이다.

### 6.3 준비자세와 anchor

준비 joint pose:

~~~text
[0, 0, 90, 0, 60, 0] deg
~~~

기본 준비 속도/가속도:

~~~text
30 deg/s
30 deg/s²
~~~

/vr/prepare_robot이 성공하면:

1. robot이 준비 joint pose로 이동한다.
2. 실제 도달한 TCP로 robot anchor를 갱신한다.
3. teleop_ready가 true가 된다.
4. 이후 mapper/recenter가 이 anchor를 기준으로 동작한다.

### 6.4 calibration

- MetaQuest source는 XY/+X calibration VALID가 필요하다.
- calibration window: 2초
- 최소 이동: 40 mm
- calibration은 세션 전체의 방향 정렬이다.
- /vr/recenter는 각 episode의 controller 상대 원점을 현재 pose로 맞추는 작업이다.
- calibration invalid가 되면 MUX는 METAQUEST를 DISABLED로 내리고 Live를 끄도록 fail-closed한다.

---

## 7. MUX와 안전 제어

### 7.1 source state

MUX source:

~~~text
DISABLED
METAQUEST
LEROBOT
~~~

불변조건:

- startup source는 반드시 DISABLED다.
- Live가 켜진 상태에서는 active source 간 직접 변경을 거부한다.
- source 변경 시 새 target이 들어오기 전까지 gripper 출력도 허용하지 않는다.
- timeout이면 source를 DISABLED로 변경하고 Live OFF/hold를 요청한다.

### 7.2 freshness

| source | timeout |
|---|---:|
| METAQUEST | 1.0 s |
| LEROBOT | 0.3 s |
| selected heartbeat → streamer | 1.0 s |

MetaQuest freshness는 valid pose heartbeat를 사용한다. LeRobot freshness는 최근 target 수신 시각을 사용한다.

### 7.3 arm target 통과 조건

선택된 target이 하위 제어로 통과하려면 다음이 모두 필요하다.

- 요청 source가 현재 선택 source와 동일
- source가 DISABLED가 아님
- teleop_ready=true
- source heartbeat/target fresh
- MetaQuest이면 calibration VALID
- target이 finite 6D posx

### 7.4 gripper 통과 조건

그리퍼 출력은 arm target보다 더 엄격하다.

- 현재 source와 입력 source가 동일
- Live=true
- teleop_ready=true
- fresh source
- source 선택 이후 새 arm target이 이미 통과
- MetaQuest이면 calibration VALID

즉 팔의 유효한 새 target 없이 그리퍼만 단독으로 움직일 수 없도록 되어 있다.

### 7.5 safety guard

accepted workspace:

| 축 | 최소 | 최대 |
|---|---:|---:|
| X | 50 mm | 650 mm |
| Y | -350 mm | 350 mm |
| Z | 하한 비활성 | 600 mm |

orientation delta limit은 anchor 기준 각 Doosan orientation component 90 deg다.

Safety Guard는 /vr/target_posx를 받아 workspace/orientation을 제한한 /vr/safe_posx를 30 Hz로 발행한다.

### 7.6 ServoL streamer

Streamer가 최종으로 소유하는 조건:

- robot state 허용값 1 또는 2
- robot state freshness 1.0 s
- teleop_ready
- Live gate
- MUX selected heartbeat
- 30 Hz fixed output
- linear ramp 6.67 mm/tick
- rotation ramp 1.0 deg/tick
- ServoL horizon 0.1 s

6.67 mm/tick × 30 Hz는 축별 약 200.1 mm/s다.

Streamer가 실제로 전송한 target은 /vr/commanded_posx에 발행된다. Task-C와 live gate는 policy target 자체가 아니라 이 acknowledged command를 기준으로 transition/tracking을 검증한다.

---

## 8. JRT 그리퍼 구현

### 8.1 물리 매핑

| 명령 | Tool DO |
|---|---:|
| OPEN | DO1 |
| CLOSE | DO2 |

Quest mapping:

| Quest 입력 | 의미 |
|---|---|
| A / button_lower | CLOSE |
| B / button_upper | OPEN |

### 8.2 driver sequence

기본값:

- mode: pulse
- pulse: 0.50 s
- interlock: 0.05 s
- debounce: 0.30 s
- readback polling: 0.01 s
- readback timeout: 0.50 s
- startup/shutdown all-off: true

OPEN/CLOSE 과정:

1. 반대 방향 DO를 OFF
2. readback 확인
3. 50 ms interlock
4. 목표 DO ON
5. ON readback 확인
6. 0.50초 유지
7. 목표 DO OFF
8. 최종 readback 확인
9. completed_command 발행

### 8.3 상태 토픽의 의미

| 토픽 | 의미 |
|---|---|
| /jrt_gripper/accepted_command | driver가 명령을 수락 |
| /jrt_gripper/completed_command | DO sequence와 readback 완료 |
| /jrt_gripper/commanded_state | 마지막으로 완료한 논리 상태, open=0, close=1 |
| /jrt_gripper/driver_busy | 명령 실행 중 |
| /jrt_gripper/last_command_ok | 마지막 명령 성공 여부 |

commanded_state는 실제 jaw encoder가 아니다. 이미 열린 상태에서 OPEN을 보내면 육안 움직임이 없을 수 있다.

### 8.4 LeRobot gripper 값

ACT action의 7번째 값은 연속값 0..1이다.

MUX hysteresis:

~~~text
value < 0.3  → OPEN
value > 0.7  → CLOSE
0.3..0.7     → HOLD, 새 명령 없음
~~~

이미 commanded_state가 목표 상태이면 중복 명령을 억제한다.

### 8.5 recording gripper initializer

recording 전 초기 OPEN은 단순히 commanded_state latch만 보지 않는다.

이미 준비된 것으로 인정하는 조건:

~~~text
commanded_state == expected
driver_busy == false
last_command_ok == true
~~~

새 명령이 필요한 경우:

~~~text
fresh accepted_command
+ fresh busy=true 전이
+ fresh completed_command
+ expected commanded_state
+ busy=false
+ last_command_ok=true
~~~

이 조건이 충족되지 않으면 기록을 시작하지 않는다.

### 8.6 로그 해석

다음 warning은 보통 adapter가 completed_command의 stop을 open/close 상태로 사용하지 않아서 발생한다.

~~~text
ignored invalid gripper completed command: 'stop'
~~~

이 warning만으로 그리퍼 고장을 뜻하지 않는다. 실제 판단은 accepted/completed open 또는 close, busy, last_command_ok, Tool DO readback을 함께 봐야 한다.

---

## 9. LeRobot A0509 plugin

### 9.1 패키지

src/lerobot_robot_doosan_a0509는 LeRobot 0.6의 plugin prefix 규칙에 맞춘 Python package다.

등록 타입:

- robot.type=doosan_a0509_ros
- teleop.type=metaquest_a0509
- Diffusion용 별도 robot type
- Task-C용 custom rollout strategy

### 9.2 robot mode

| mode | observation | command publisher | send_action 동작 |
|---|---|---|---|
| shadow_record | ROS state + cameras | 만들어질 수 있으나 실행 명령 없음 | action을 그대로 반환, publish 0 |
| policy_shadow | ROS state + cameras | command/debug publisher 자체를 만들지 않음 | publish 금지 |
| policy_dry_run | ROS state + cameras | debug only | /control/lerobot/debug_action |
| policy_live | ROS state + cameras | live target/gripper | Live/ready/state gate 후 publish |

shadow_record에서 teacher action은 MetaQuest adapter가 만들며 robot adapter는 절대로 두 번째 robot command를 발행하면 안 된다.

### 9.3 observation schema

13차원 observation.state:

~~~text
0  joint_1_rad
1  joint_2_rad
2  joint_3_rad
3  joint_4_rad
4  joint_5_rad
5  joint_6_rad
6  tcp_x_mm
7  tcp_y_mm
8  tcp_z_mm
9  tcp_o1_deg
10 tcp_o2_deg
11 tcp_o3_deg
12 gripper_commanded_state
~~~

추가 visual observation:

- observation.images.front
- observation.images.side
- observation.images.zed_rgb

### 9.4 action schema

7차원 action:

~~~text
0 target_x_mm
1 target_y_mm
2 target_z_mm
3 target_o1_deg
4 target_o2_deg
5 target_o3_deg
6 gripper_target
~~~

### 9.5 rollout alias

LeRobot 0.6 rollout context가 scalar feature name의 .pos suffix를 기대하는 제약 때문에 policy mode에서 내부 alias를 사용한다. 원본 dataset schema와 ROS topic contract는 바꾸지 않는다.

### 9.6 Doosan orientation canonicalization

Doosan은 같은 물리 orientation을 다음 두 ZYZ family로 표현할 수 있다.

~~~text
[A, B, C]
[A+180, -B, C+180]
~~~

학습 데이터가 주로 B≥0 family를 사용하므로 policy observation에서는 quaternion을 거쳐 positive-B canonical family로 바꾼다. 실제 자세는 바뀌지 않지만 scalar OOD jump를 줄이는 목적이다.

---

## 10. 데이터 수집 파이프라인

### 10.1 정상 수집 전체 흐름

~~~mermaid
flowchart TD
    B[Full bringup] --> P[Prepare robot]
    P --> C[MetaQuest XY/+X calibration]
    C --> PF[Recording preflight]
    PF --> R1[Episode reset before first]
    R1 --> REC[30 Hz shadow recording]
    REC --> SAFE[Live OFF + DISABLED + gripper idle]
    SAFE --> REVIEW[n/r/q review]
    REVIEW -->|n| COMMIT[Commit episode]
    REVIEW -->|r| DISCARD[Discard buffer]
    REVIEW -->|q| STOP[Commit and stop]
    COMMIT --> NEXT[Prepare/recenter/open/preflight]
    DISCARD --> NEXT
    NEXT --> REC
~~~

### 10.2 새 데이터셋 기록 wrapper

scripts/record_lerobot_shadow_pilot.sh의 역할:

1. recorder CPU affinity 적용
2. nice priority 확인
3. accepted config checksum 확인
4. ROS/LeRobot 환경 source
5. gripper 초기 상태 검증
6. shadow adapter와 teacher adapter preflight
7. 새 dataset root만 허용
8. multiprocess streaming encoder와 deadline scheduler 설치
9. LeRobot record 실행

이 wrapper는 기존 root를 덮어쓰지 않는다. root가 존재하면 오류로 종료한다. 즉 새 수집 전용 wrapper다.

예시:

~~~bash
cd ~/Dosan-MetaQuest-teleoperation-project

DATASET_REPO_ID="local/a0509_blue_block_v6" \
DATASET_ROOT="$HOME/lerobot_datasets/a0509_blue_block_t6" \
TASK_DESCRIPTION="Stack the blue block on top of the other blue block on the black table." \
NUM_EPISODES=30 \
EPISODE_TIME_S=30 \
./scripts/record_lerobot_shadow_pilot.sh
~~~

### 10.3 recording scheduling

기본 CPU 분리:

| 역할 | CPU | nice |
|---|---|---:|
| robot/ROS control tree | 0–5 | 0 |
| recorder main + cameras | 6–9 | 10 |
| shared-memory encoder worker | 10–13 | 10 |

기본 encoder:

- H.264 NVENC
- preset 12
- CRF 30
- streaming encoding true
- image writer process/thread 0
- shared memory slot 32 per camera
- worker start method spawn

shared-memory ring이 가득 차거나 queue가 포화되거나 worker 실패/frame mismatch/drop이 생기면 episode를 실패시킨다. 조용히 영상과 Parquet row가 어긋난 데이터셋을 만들지 않는 것이 설계 목표다.

### 10.4 absolute-deadline recording

일반적인 sleep-after-work loop 대신 30 Hz absolute deadline을 사용한다.

로그 예:

~~~text
ticks=900
late_ticks=0
missed_deadlines=0
max_lateness_ms=0
~~~

30초 episode에서 900 tick, 60초 episode에서 1800 tick이 정상값이다.

### 10.5 episode reset state machine

~~~text
STOPPING
→ SAFE
→ PREPARING_ROBOT
→ WAITING_FOR_CALIBRATION, 필요한 경우
→ RECENTERING
→ INITIALIZING_GRIPPER
→ PREFLIGHT
→ READY
→ RECORDING
~~~

각 다음 episode 전에:

1. Live OFF
2. MUX DISABLED
3. gripper STOP, idle 확인
4. operator가 작업 공간 정리를 확인
5. /vr/prepare_robot
6. 물체 배치와 Quest 중립 pose 확인
7. Quest pose stable 0.5 s
8. calibration VALID 확인
9. /vr/recenter
10. fresh heartbeat
11. gripper OPEN 완료
12. METAQUEST source 선택
13. selected target이 anchor의 50 mm/10 deg 이내인지 확인
14. Live ON
15. recording 시작

### 10.6 episode review

자연 종료 후:

| 입력 | 결과 |
|---|---|
| n 또는 Right | 저장하고 다음 episode |
| r 또는 Left | 폐기하고 같은 index 재녹화 |
| q 또는 Esc | 저장하고 전체 종료 |

완료된 buffer를 commit한 다음 다음 reset을 시작한다. 따라서 다음 reset에서 취소하거나 예외가 나도 직전 저장 episode는 유지된다.

### 10.7 resume의 정확한 의미

LeRobot 0.6 record loop는 resume 때 recorded_episodes를 0에서 다시 센다.

따라서:

~~~text
--resume=true
--dataset.num_episodes=N
~~~

에서 N은 최종 총 episode 수가 아니라 이번 재개 실행에서 추가로 녹화할 episode 수다.

예:

~~~text
현재 저장 18개
최종 목표 30개
이번 --dataset.num_episodes = 12
~~~

로그의 Recording episode K에서 K는 dataset에 이미 저장된 zero-based 다음 index다.

- Recording episode 30은 31번째 episode를 시도한다.
- Recording episode 31은 이미 31개가 저장되어 있고 32번째를 시도한다.

T2가 31개가 된 이유와 직접 연결되는 중요한 동작이다.

재개 전 계산:

~~~bash
DATASET_ROOT="$HOME/lerobot_datasets/a0509_blue_block_t6"
CURRENT_EPISODES=$(jq -r '.total_episodes' "$DATASET_ROOT/meta/info.json")
TARGET_EPISODES=30
REMAINING_EPISODES=$((TARGET_EPISODES - CURRENT_EPISODES))

printf 'current=%s target=%s remaining=%s\n' \
  "$CURRENT_EPISODES" "$TARGET_EPISODES" "$REMAINING_EPISODES"
~~~

현재 wrapper에는 안전한 RESUME 옵션이 없으므로 direct record_entrypoint 명령을 정확히 재구성해야 한다. 향후 가장 먼저 개선할 운영 도구 중 하나는 새 수집과 resume를 모두 지원하면서 metadata를 읽어 remaining count를 자동 계산하는 wrapper다.

### 10.8 TASK_DESCRIPTION 수정

TASK_DESCRIPTION은 dataset의 meta/tasks.parquet에 저장된다.

수집 후 수정은 기술적으로 가능하지만 다음 원칙이 필요하다.

- 원본 dataset 전체가 같은 실제 작업을 표현하는 경우에만 변경
- backup 생성
- task_index와 tasks.parquet 일관성 유지
- 학습 전에 최종 문장 확정
- semantic spec의 expected 문장도 같이 갱신
- 이미 학습된 weight는 metadata 문장만 바꾼다고 달라지지 않음

T4는 수집 중 문장을 다음 최종 문장으로 정리했다.

~~~text
Open the drawer, pick up the blue block, place it inside the drawer, and then close the drawer
~~~

현재 /home/rvlab/lerobot_datasets/a0509_blue_block_t4_task_backup_20260823 백업 경로가 남아 있다.

---

## 11. 데이터 수집 장애 해석

### 11.1 no message received for gripper_commanded_state

의미:

- JRT gripper driver/bringup이 실행되지 않음
- driver node가 죽음
- ROS graph/domain/source 환경 불일치
- startup에서 아직 latched state가 발행되지 않음

해결 우선순위:

1. 중복 bringup이 없는지 확인
2. 실기 bringup에 start_gripper=true인지 확인
3. /jrt_gripper/driver_busy와 last_command_ok가 존재하는지 확인
4. OPEN initializer를 실행
5. recording 재시작

단순히 recorder timeout만 늘리는 것은 driver가 없는 문제를 해결하지 않는다.

### 11.2 stale topic joint_positions: age≈0.52 s, max_age=0.50 s

의미:

- joint state는 왔지만 strict freshness 0.5초를 조금 넘김
- bringup 직후 scheduling/ROS discovery/camera connect와 겹쳤을 수 있음
- control graph가 불안정하거나 중복 process가 있을 수 있음

우선 새 bringup이 안정된 후 재시도하고 control scheduling을 확인한다. threshold 완화는 마지막 수단이다.

### 11.3 safe target did not converge to the recentered robot anchor

이 오류는 다음 sequence의 preflight 실패다.

~~~text
prepare 성공
→ recenter 성공
→ selected MetaQuest target이 anchor 50 mm/10 deg 이내로 수렴해야 함
→ 제한 시간 안에 수렴하지 못함
~~~

주요 원인:

- Quest controller가 중립 pose에서 움직임
- calibration/pose heartbeat가 불안정
- 오래된 target이 남음
- anchor와 Quest recenter 시점 불일치
- 중복 bringup/MUX 상태
- robot actual pose가 아직 settle되지 않음

이 오류는 fail-closed이며 episode recording을 시작하지 않는다.

### 11.4 gripper did not complete verified open

예시 상태:

~~~text
accepted='open'
completed='stop'
commanded_state=1.0
busy=true
last_command_ok=false
~~~

이 경우 OPEN 요청은 접수됐지만 readback을 포함한 정상 완료가 확인되지 않은 것이다. bringup/driver를 정리하고 DO/readback 상태부터 확인해야 한다.

### 11.5 policy가 load됐지만 robot이 움직이지 않음

정상일 가능성이 높다.

run_a0509_act_policy_live.sh는 의도적으로:

- 시작 때 Live OFF
- MUX DISABLED
- policy load/warmup
- target hold
- 스스로 LEROBOT 선택 안 함
- 스스로 Live ON 안 함

따라서 별도의 a0509_act_live_trial_gate.py가 필요하다.

### 11.6 shell 복사 오류

반복해서 발생했던 오류:

- 긴 MODEL_PATH 문자열 내부 줄바꿈
- backslash 뒤 공백
- 한국어 설명 문장을 shell에 함께 붙여넣음
- Python -c import를 두 줄로 나눔
- false 대신 falsee 같은 오타

운영 명령에서는 긴 경로를 MODEL_DIR과 MODEL_NAME으로 나누고, Python -c는 한 줄로 실행하는 것이 안전하다.

---

## 12. T1~T6 데이터셋 현황

아래 수치는 2026-08-25에 각 meta/info.json, meta/tasks.parquet, episode metadata를 직접 읽은 결과다.

| Task | 로컬 dataset | TASK_DESCRIPTION | Episodes | Frames | 길이 |
|---|---|---|---:|---:|---|
| T1 | a0509_blue_block_t1_throw_away_inthe_container | Open the White Container throw away the blue block | 30 | 36,000 | 40 s × 30 |
| T2 | a0509_blue_block_t2_table | bring the blue block to black table | 31 | 27,899 | 30 s 수준, 899/900 frame |
| T3 | a0509_blue_block_t3 | Move the blue block off the black table | 30 | 27,000 | 30 s × 30 |
| T4 | a0509_blue_block_t4 | Open the drawer, pick up the blue block, place it inside the drawer, and then close the drawer | 30 | 54,000 | 60 s × 30 |
| T5 | a0509_blue_block_t5 | Open the drawer, take out the blue block, place it on the black table, and close the drawer. | 30 | 54,000 | 60 s × 30 |
| T6 | a0509_blue_block_t6 | Stack the blue block on top of the other blue block on the black table. | 30 | 27,000 | 30 s × 30 |

### T2의 31개 episode

- episode index는 0..30이다.
- 길이는 899 또는 900 frame이다.
- resume 실행에서 num_episodes를 남은 총량이 아니라 추가 수량으로 지정한 결과 31개까지 commit되었다.
- 이 상태 자체는 LeRobot dataset으로 읽을 수 있지만 다른 30개 task와 비교할 때 split과 weighting이 달라진다.

### 데이터 공통 계약

- codebase version: v3.0
- robot_type: doosan_a0509_ros
- FPS: 30
- state: 13
- action: 7
- task 수: 1
- video: front/side/zed_rgb
- original dataset은 semantic 분석에서 수정하지 않음

---

## 13. ACT 학습 구성

### 13.1 여섯 모델의 공통 설정

실제 각 pretrained_model/config.json과 train_config.json을 대조했으며 여섯 모델이 동일했다.

| 항목 | 값 |
|---|---|
| policy | ACT |
| n_obs_steps | 1 |
| chunk_size | 100 |
| n_action_steps | 100 |
| vision backbone | ResNet18, ImageNet pretrained |
| model dim | 512 |
| attention heads | 8 |
| feedforward dim | 3200 |
| encoder layers | 4 |
| decoder layers | 1 |
| VAE | true |
| latent dim | 32 |
| VAE encoder layers | 4 |
| dropout | 0.1 |
| KL weight | 10 |
| optimizer LR | 1e-5 |
| backbone LR | 1e-5 |
| weight decay | 1e-4 |
| normalization | visual/state/action MEAN_STD |
| AMP | false |
| device | CUDA |
| batch size | 32 |
| workers | 8 |
| eval split | 0.1 |
| eval steps | 5000 |
| save freq | 20000 |
| log freq | 200 |
| seed | 1000 |
| W&B | disabled |
| Hub push | disabled |

100 action은 30 Hz에서 약 3.33초다.

### 13.2 train/eval split

LeRobot 0.6은 task별 마지막 ceil(N×eval_split) episode를 eval로 hold out한다. random split이 아니다.

| Dataset | 총 episode | train | eval |
|---|---:|---:|---:|
| T1 | 30 | 27 | 3 |
| T2 | 31 | 27 | 4 |
| T3 | 30 | 27 | 3 |
| T4 | 30 | 27 | 3 |
| T5 | 30 | 27 | 3 |
| T6 | 30 | 27 | 3 |

eval 3개를 덜 학습하는 것은 데이터 손실이 아니라 held-out validation을 위한 의도적인 분리다. 다만 30개가 작은 데이터셋이므로 실제 robot success rate는 eval loss만으로 판단할 수 없다.

### 13.3 모델별 학습 결과

| Task | train repo_id | Steps | 로컬 모델 폴더 | checkpoint |
|---|---|---:|---|---|
| T1 | rvlab/a0509_blue_block_t1_throw_away_inthe_container | 50,000 | act_a0509_blue_block_t1_throw_away_inthe_container_bs32_50k_20260820_2323 | 050000 |
| T2 | rvlab/a0509_blue_block_t2_table | 40,000 | act_a0509_blue_block_t2_table_bs32_40k_20260821 | 040000 |
| T3 | rvlab/a0509_blue_block_t3 | 40,000 | act_a0509_blue_block_t3_bs32_40k_20260822 | 040000 |
| T4 | rvlab/a0509_blue_block_v4 | 80,000 | act_a0509_blue_block_t4_bs32_80k_20260823 | 080000 |
| T5 | rvlab/a0509_blue_block_v5 | 80,000 | act_a0509_blue_block_t5_bs32_80k_20260824 | 080000 |
| T6 | rvlab/a0509_blue_block_v6 | 50,000 | act_a0509_blue_block_t6_bs32_50k_20260824 | 050000 |

각 model.safetensors는 206,732,508 bytes로 존재한다.

학습 step 선택의 운영적 경향:

- 30초의 비교적 단순한 pick/place task: 40k 또는 50k
- 40초 T1 다중 조작: 50k
- 60초 drawer multi-stage T4/T5: 80k
- 30초 stacking T6: 50k

이것은 절대적인 최적값 증명이 아니라 당시 task 길이와 복잡도를 반영한 실험 설정이다.

### 13.4 서버 학습 명령의 공통형

~~~bash
CUDA_VISIBLE_DEVICES=1 lerobot-train \
  --dataset.repo_id="rvlab/..." \
  --dataset.root="/workspace/data/..." \
  --dataset.eval_split=0.1 \
  --policy.type=act \
  --policy.device=cuda \
  --output_dir="/workspace/outputs/..." \
  --job_name="..." \
  --batch_size=32 \
  --steps=50000 \
  --eval_steps=5000 \
  --num_workers=8 \
  --wandb.enable=false \
  --policy.push_to_hub=false
~~~

GPU 0과 1에서 서로 다른 run을 동시에 할 때 각 process에 서로 다른 CUDA_VISIBLE_DEVICES를 지정한다. 한 process가 두 GPU를 자동으로 사용하는 distributed training은 현재 학습 명령의 구조가 아니다.

### 13.5 TASK_DESCRIPTION과 ACT의 관계

현재 ACT config의 input feature는 다음뿐이다.

- observation.state
- observation.images.front
- observation.images.side
- observation.images.zed_rgb

언어 embedding input은 없다.

따라서 TASK_DESCRIPTION은:

- dataset metadata
- 사람에게 작업 의미를 설명하는 label
- semantic 분할의 의미 힌트
- 향후 LLM/VLM planner 입력
- runtime traceability 문자열

이다.

TASK_DESCRIPTION을 바꾸는 것만으로 ACT weight나 behavior가 바뀌지 않는다. 서로 다른 task를 같은 ACT checkpoint로 자연어만 바꿔 실행해도 새로운 skill이 생기지 않는다.

### 13.6 T1 폴더명 traceability

T1 로컬 폴더는 통일성을 위해 blue_block 문자열을 포함하도록 정리되었지만, 내부 train_config.output_dir에는 서버 학습 당시의 이전 run 이름이 남아 있다. weight에는 영향이 없지만 provenance를 분석할 때 로컬 폴더명과 내부 output_dir이 다를 수 있다.

---

## 14. ACT Jetson 실행기

### 14.1 비동기 FIFO

LeRobot 0.6의 RTC worker를 ACT에 맞게 non-prefix-guided FIFO로 사용한다.

고정 인자:

~~~text
--inference.type=rtc
--inference.rtc.enabled=false
--policy.n_action_steps=100
~~~

ACT는 prefix-guided RTC를 구현하지 않으므로 rtc.enabled=true는 명시적으로 거부한다.

### 14.2 warmup과 GPU arbiter

- 기본 warmup inference 2회
- warmup output은 모두 폐기
- 첫 executable chunk만 queue에 들어감
- process-wide ACT GPU arbiter로 같은 process의 ACT-A/ACT-B 동시 CUDA 호출 방지

### 14.3 queue와 overlap

기본값:

| 항목 | 값 |
|---|---:|
| queue threshold | 20 |
| overlap steps | 15 |
| action steps | 100 |
| FPS | 30 |

새 chunk가 도착하면 inference 동안 이미 소비된 delay prefix를 제거한다. 남은 old tail과 new chunk의 처음 15 step을 smoothstep으로 blend한다.

- XYZ: 선형 blend
- orientation: quaternion SLERP 후 Doosan ZYZ 변환
- gripper: 연속 blend하지 않고 old discrete decision 유지

### 14.4 Live generation isolation

Live rising edge마다 generation을 증가시킨다.

1. 이전 queue 즉시 clear
2. 이전 generation에서 계산 중이던 inference 결과 폐기
3. 최신 observation으로 새 chunk 생성
4. fresh queue가 준비되기 전까지 actual TCP hold target 발행
5. policy_queue_ready=true 후에만 model action 소비

이 때문에 policy process를 gate 전에 띄워도 과거 pre-Live chunk가 실행되지 않는다.

### 14.5 CPU 분리

기본 ACT live:

| thread | CPU |
|---|---|
| ROS executor | 6 |
| main/camera | 7–8 |
| ACT inference | 9–13 |
| 전체 process affinity | 6–13 |

### 14.6 single-policy live wrapper

scripts/run_a0509_act_policy_live.sh:

- checkpoint 존재 확인
- Live OFF + DISABLED 강제
- policy load
- camera 연결
- async engine 시작
- 실제 target hold
- MUX/Live를 자동으로 켜지 않음
- 종료 trap에서 Live OFF + DISABLED

### 14.7 external live gate

scripts/a0509_act_live_trial_gate.py 기본 검증:

| 항목 | 제한 |
|---|---:|
| startup timeout | 45 s |
| initial position delta | 50 mm |
| initial orientation delta | 10 deg |
| target freshness | 0.30 s |
| state freshness | 0.50 s |
| arming settle | 0.5 s |

순서:

~~~text
force Live OFF/DISABLED
→ actual TCP와 policy hold target 비교
→ gripper monitor 시작
→ select LEROBOT
→ fresh selected target/safe target 확인
→ Live ON
→ fresh policy queue 확인
→ first model target delta 확인
→ bounded RUNNING
→ watchdog
→ COMPLETED 또는 ABORTED
→ gripper summary
→ Live OFF/DISABLED
~~~

### 14.8 실제 실행의 세 터미널

~~~text
Terminal 1: control bringup
Terminal 2: policy load and async inference
Terminal 3: bounded live gate
~~~

DURATION_S=900은 policy process 유지시간이다. 실제 robot motion 시간은 gate의 --duration-sec 값이다.

### 14.9 일반 모델 실행 template

~~~bash
cd ~/Dosan-MetaQuest-teleoperation-project
source /opt/ros/jazzy/setup.bash
source ~/venvs/lerobot/bin/activate
source install/setup.bash

MODEL_DIR="/home/rvlab/lerobot_models/models"
MODEL_NAME="act_a0509_blue_block_t6_bs32_50k_20260824"

export MODEL_PATH="$MODEL_DIR/$MODEL_NAME/050000/pretrained_model"
export TASK_DESCRIPTION="Stack the blue block on top of the other blue block on the black table."
export DURATION_S=900

test -f "$MODEL_PATH/model.safetensors" && echo "CHECKPOINT_OK"
./scripts/run_a0509_act_policy_live.sh
~~~

다른 task는 MODEL_NAME, checkpoint step, TASK_DESCRIPTION만 바뀐다. bringup, 준비자세, gripper 초기화, gate 구조는 동일하다.

### 14.10 현재 실기 검증 범위

- T1 checkpoint는 Jetson에서 load, camera connect, async warmup, fresh chunk 생성이 확인되었다.
- 5초 bounded gate에서 arm target stream이 실행되었고 policy close target 및 MUX close 명령이 관측되었다.
- T1~T6 각각의 전체 task 성공률을 반복 측정한 정식 결과는 아직 없다.
- 모델 파일 존재, loss, inference 성공은 물체 조작 성공을 보장하지 않는다.
- task별 최소 10회 reset-and-repeat success test가 필요하다.

---

## 15. 기존 A+B→C Task-C 알고리즘

### 15.1 Task-C에서 사용한 A/B

현재 Task-C reference는 T1~T6 whole-task 모델이 아니라 더 이른 두 pick/transport 데이터셋과 bs16 30k ACT 모델을 사용한다.

| 역할 | Dataset | 의미 |
|---|---|---|
| A | a0509_blue_block_v1_20260810_170226 | table의 blue block pick |
| B | a0509_blue_block_v2_20260810_191044 | under-table blue block task의 transport/release suffix |

legacy checkpoints:

- /home/rvlab/lerobot_models/act_a0509_blue_block_bs16_30k_20260810_181114/030000/pretrained_model
- /home/rvlab/lerobot_models/act_a0509_blue_block_v2_bs16_30k_20260810_200240/030000/pretrained_model

### 15.2 semantic candidate

각 episode에서:

~~~text
open→closed = close frame
closed→open = open frame
~~~

closed-transport 후보:

~~~text
close + 30 frames
≤ candidate
≤ open - 30 frames
~~~

30 Hz에서 앞뒤 1초를 제외한다. grasp 직후와 release 직전 접촉을 피하려는 heuristic이다.

transport floor:

~~~text
max(grasp TCP Z, release TCP Z) + 50 mm
~~~

주의: gripper closed를 holding=true로 간주하는 heuristic일 뿐 실제 holding sensor는 없다.

### 15.3 V0 exhaustive optimizer

A cut, B entry, duration을 전수 조사한다.

Bezier boundary:

~~~text
P0 = pA
P1 = pA + T/3 × vA
P2 = pB - T/3 × vB
P3 = pB
~~~

검사 항목:

- path length
- vector/axis velocity
- acceleration
- curvature
- jerk
- integrated squared jerk
- backtracking
- workspace
- transport floor
- semantic compatibility

목적함수:

~~~text
L_C
= A retained prefix length
+ Bridge length
+ B retained suffix length
~~~

가장 짧은 Bridge가 아니라 가장 짧은 feasible 전체 C를 선택한다. 최단값 0.5% 안의 후보만 smoothness tie-break를 적용한다.

reference V0 선택:

- A episode 5, frame 312
- B episode 5, frame 450
- Bridge 3.0 s
- Bridge length 362.37 mm
- total C 1300.55 mm
- max velocity 161.38 mm/s
- max acceleration 213.54 mm/s²

### 15.4 dual-ACT runtime

~~~text
NEW
→ ACT-A/ACT-B load
→ warmup discard
→ fresh ACT-A queue
→ semantic A cut
→ ACT-A queue invalidate
→ initial Bridge
→ endpoint settle
→ endpoint observation capture
→ fresh ACT-B execution_refresh
→ connector/direct/alignment validation
→ exact B first target
→ atomic B queue activation
→ rolling ACT-B
→ release verification
→ COMPLETE
~~~

A 시작 속도는 predicted action이 아니라 최근 actual TCP의 causal regression을 사용한다. B는 아직 실행되지 않았으므로 fresh postprocessed action chunk의 속도를 사용한다.

### 15.5 runtime safety additions

실기 실패를 통해 다음이 추가되었다.

- command-time progression, wall-clock catch-up 금지
- /vr/commanded_posx acknowledgement
- one-command tracking backpressure
- actual→candidate 14 mm/9 deg admission
- hard tracking 15 mm/10 deg
- hold action 재발행으로 MUX freshness 유지
- endpoint-stop
- fresh B endpoint inference
- per-axis direct handoff 판단
- alignment Bridge fallback
- generation role 분리
- stale/failure 시 fail-closed

### 15.6 V0 물리 검증

2026-08-14 고정 scene에서 두 번 연속 전체 성공:

| run | 총 시간 | ACT-A commands | Bridge | ACT-B |
|---|---:|---:|---:|---:|
| 1 | 34.371 s | 417 | 163 | 410 |
| 2 | 34.721 s | 424 | 176 | 408 |

공통 결과:

- grasp/close latch 성공
- Bridge 완료
- fresh B handoff
- B rolling refresh 5회
- release driver confirmation
- final home settle
- guard/streamer intervention 0
- cleanup Live OFF, MUX DISABLED

이 결과는 고정된 scene과 계약에서의 성공이며 일반 장면 성공률이 아니다.

### 15.7 representative V1

V1은 A/B 각 30 episode의 closed transport를 arc-length phase 101점으로 정렬한다.

대표값:

- component median XYZ
- median velocity/acceleration
- covariance
- MAD
- episode residual/support

검색:

- coarse 21×21×13 at 30 Hz
- 0.01 phase local refine
- final/robust validation at 60 Hz
- 전체 C 길이 우선
- 40 mm support ≥80%
- 20 mm support ≥50%

선택된 대표 경계:

| 항목 | 값 |
|---|---:|
| A phase | 0.02 |
| A center | [435.753, 204.728, 415.287] mm |
| B phase | 0.94 |
| B entry | [414.468, -156.555, 418.762] mm |
| Bridge | 3.5 s |
| Bridge length | 385.317 mm |
| total C estimate | 1501.716 mm |

### 15.8 V1 live 40/20 mm 계약

- 40 mm: read-only prearm, pure planning만 수행
- 20 mm: direction/closed/semantic 조건을 연속 3 frame 확인 후 commit
- 경계는 actual TCP로 판단
- Bridge 시작은 acknowledged commanded_posx
- 시작 속도는 recent actual TCP causal regression
- initial Bridge는 B representative endpoint에서 zero velocity로 정지
- 정착 후 fresh ACT-B execution_refresh
- feasible connector 우선
- 각 XYZ가 6.67 mm 안이면 direct
- 아니면 shortest feasible zero-to-zero alignment Bridge
- 모두 실패하면 fail-closed

### 15.9 V1 물리 검증

2026-08-19 endpoint-boundary run 1회 전체 성공:

- A prearm 31.891 mm
- commit 14.618 mm
- endpoint actual error 0.054 mm
- endpoint hold 0.268 s
- fresh B 첫 target max axis delta 3.242 mm
- direct handoff limit 6.67 mm/tick 통과
- ACT-A 368, Bridge 133, ACT-B 372 commands
- gripper open driver confirmation
- COMPLETE
- cleanup Live OFF/DISABLED

0.6초 zero-to-zero alignment fallback은 exact prior endpoint replay에서 command-free로 검증되었지만, 실제 물리 실행에서 강제로 발생시켜 성공시킨 것은 아니다.

### 15.10 Task-C의 현재 한계

- V0 Cartesian candidate ranking은 orientation을 사용하지 않는다.
- gripper closed=holding은 heuristic이다.
- object identity sensor가 없다.
- payload swept collision/IK의 일반 인증이 없다.
- fixed scene 밖 distribution shift를 보장하지 않는다.
- 현재 상태기는 기본적으로 A/B 두 정책 전용이다.
- whole-task T1/T4/T5처럼 여러 close/open span이 있는 정책을 일반 의미 스킬로 자동 연결하지 않는다.
- 생성 manifest는 계속 robot_executable=false, dry_run_only=true를 유지하고, 실기는 별도 명시적 acknowledgement와 gate를 통해서만 수행했다.

---

## 16. T1~T6 semantic-only V3

### 16.1 현재 설계 범위

semantic V3가 포함하는 것:

- TASK_DESCRIPTION
- observation.state의 gripper_commanded_state
- open/close event ordinal
- TCP XYZ trajectory
- semantic object/state label
- semantic span별 0..1 phase
- 전체 episode의 phase별 통계
- hierarchy와 world-state graph

semantic V3가 포함하지 않는 것:

- 정책 전환 적합도
- 전환 phase 선택
- 20/40 mm runtime sphere
- Bridge cost
- upper planner task 조합
- 실제 대표 episode 선택
- frame number 평균
- image pixel 평균
- robot executable manifest

### 16.2 이벤트 검출

dataset loader는 observation.state names가 정확한 13D schema인지 먼저 검사한다. positional field를 추측하지 않는다.

gripper state 이진화:

~~~text
observation.state[12] >= 0.5 → closed
otherwise                   → open
~~~

episode 내부 전이:

~~~text
open → closed = C1, C2, ...
closed → open = O1, O2, ...
~~~

각 task의 expected grammar와 실제 episode grammar가 정확히 다르면 분석을 실패시킨다.

### 16.3 semantic span 표준화

각 span:

~~~text
START/Cn/On/END anchor
→ TCP XYZ cumulative Cartesian arc length
→ 0..1 phase
→ 101 points interpolation
~~~

동일 semantic phase의 전체 episode에서 계산:

- component median XYZ
- mean XYZ
- covariance
- MAD
- residual p50/p90/p95
- path length median/mean/std/min/max

representative 경로는 실제 한 episode가 아니라 합성 component-median trajectory다.

semantic_phase_core.py에는 과거 실험용 representative_episode_medoid 함수가 남아 있지만 semantic-only V3 analyzer는 이를 호출하지 않으며 individual_episode_selected=false를 기록한다.

### 16.4 의미 정보의 근거

현재 의미 label은 자동 vision model이 산출한 것이 아니다.

근거:

- TASK_DESCRIPTION
- event 순서
- gripper state
- TCP motion
- 기존 multi-camera review
- T4/T5 같은 inverse task의 Cartesian anchor correspondence

semantic_graph_v3.json은 model_used_for_semantic_extraction=false를 기록한다.

따라서 현재 결과는 audited rule-based semantic annotation이다. 향후 VLM/API를 사용할 수 있지만 현재 산출물 자체가 VLM 자동 인식 결과라고 해석하면 안 된다.

### 16.5 T1 semantic graph

Task:

~~~text
Open the White Container throw away the blue block
~~~

Grammar:

~~~text
C1 → O1 → C2 → O2
~~~

| ID | Semantic label | Gripper 의미 | 대상 | 대표 길이 | p90 residual 평균 |
|---|---|---|---|---:|---:|
| S1 | approach_white_container_handle | open, C1에서 close | white_container_handle | 197.8 mm | 28.9 mm |
| S2 | manipulate_white_container_open | closed, O1에서 open | white_container_handle | 479.2 mm | 36.5 mm |
| S3 | approach_blue_block_on_black_table | open, C2에서 close | blue_block | 209.0 mm | 34.5 mm |
| S4 | lift_transport_and_align_blue_block | closed, O2에서 open | blue_block | 437.2 mm | 41.3 mm |
| S5 | retract_after_blue_block_release | open | none | 252.2 mm | 38.1 mm |

world state:

~~~text
container closed, block on black table, gripper open
→ container open, block on black table
→ block held
→ block in white container
→ retract
~~~

### 16.6 T2 semantic graph

Task:

~~~text
bring the blue block to black table
~~~

Grammar:

~~~text
C1 → O1
~~~

| ID | Semantic label | Gripper 의미 | 대상 | 대표 길이 | p90 residual 평균 |
|---|---|---|---|---:|---:|
| S1 | approach_blue_block_in_source_region | open, C1 close | blue_block | 490.4 mm | 50.4 mm |
| S2 | lift_transport_and_align_blue_block_to_black_table | closed, O1 open | blue_block | 583.2 mm | 65.6 mm |
| S3 | retract_after_placing_blue_block_on_black_table | open | none | 197.9 mm | 56.6 mm |

문장에 source surface가 없으므로 source_region으로 geometry-neutral하게 유지한다.

### 16.7 T3 semantic graph

Task:

~~~text
Move the blue block off the black table
~~~

Grammar:

~~~text
C1 → O1
~~~

| ID | Semantic label | Gripper 의미 | 대상 | 대표 길이 | p90 residual 평균 |
|---|---|---|---|---:|---:|
| S1 | approach_blue_block_on_black_table | open, C1 close | blue_block | 272.5 mm | 29.8 mm |
| S2 | lift_transport_and_align_blue_block_off_black_table | closed, O1 open | blue_block | 511.9 mm | 41.5 mm |
| S3 | retract_after_releasing_blue_block_off_black_table | open | none | 405.9 mm | 49.7 mm |

### 16.8 T4 semantic graph

Task:

~~~text
Open the drawer, pick up the blue block, place it inside the drawer, and then close the drawer
~~~

Grammar:

~~~text
C1 → O1 → C2 → O2
~~~

| ID | Semantic label | Gripper 의미 | 대상 | 대표 길이 | p90 residual 평균 |
|---|---|---|---|---:|---:|
| S1 | approach_drawer_handle | open, C1 close | drawer_handle | 430.7 mm | 44.8 mm |
| S2 | manipulate_drawer_open | closed, O1 open | drawer_handle | 182.2 mm | 50.2 mm |
| S3 | approach_blue_block_on_black_table | open, C2 close | blue_block | 469.7 mm | 63.0 mm |
| S4 | lift_transport_and_align_blue_block_in_drawer | closed, O2 open | blue_block | 507.0 mm | 70.3 mm |
| S5 | close_drawer_and_retract_with_open_gripper | open throughout | drawer_front | 1115.7 mm | 81.8 mm |

서랍 닫기는 O2 이후 gripper open 상태에서 진행되므로 추가 gripper event가 없다. 근거 없는 내부 경계를 만들지 않고 S5 하나로 유지했다.

### 16.9 T5 semantic graph

Task:

~~~text
Open the drawer, take out the blue block, place it on the black table, and close the drawer.
~~~

Grammar:

~~~text
C1 → O1 → C2 → O2
~~~

| ID | Semantic label | Gripper 의미 | 대상 | 대표 길이 | p90 residual 평균 |
|---|---|---|---|---:|---:|
| S1 | approach_drawer_handle | open, C1 close | drawer_handle | 376.8 mm | 34.8 mm |
| S2 | manipulate_drawer_open | closed, O1 open | drawer_handle | 175.7 mm | 49.9 mm |
| S3 | approach_blue_block_in_drawer | open, C2 close | blue_block | 370.5 mm | 57.1 mm |
| S4 | lift_transport_and_align_blue_block_on_black_table | closed, O2 open | blue_block | 465.5 mm | 54.7 mm |
| S5 | close_drawer_and_retract_with_open_gripper | open throughout | drawer_front | 1009.2 mm | 63.5 mm |

T5는 T4의 inverse object transfer에 해당한다. C2/O2 anchor가 T4의 in-drawer release/black-table grasp 위치와 대응하는 것을 semantic label 근거로 사용했다.

### 16.10 T6 semantic graph

Task:

~~~text
Stack the blue block on top of the other blue block on the black table.
~~~

Grammar:

~~~text
C1 → O1
~~~

| ID | Semantic label | Gripper 의미 | 대상 | 대표 길이 | p90 residual 평균 |
|---|---|---|---|---:|---:|
| S1 | approach_moving_blue_block_in_source_region | open, C1 close | moving_blue_block | 227.5 mm | 30.9 mm |
| S2 | lift_transport_align_and_stack_blue_block_on_support_block | closed, O1 open | moving_blue_block | 576.3 mm | 41.9 mm |
| S3 | retract_after_stacking_blue_blocks | open | none | 518.4 mm | 50.7 mm |

TASK_DESCRIPTION이 두 blue block의 역할을 구분한다.

- moving_blue_block: 집어서 옮기는 블록
- support_blue_block: black table 위에 고정된 다른 블록
- O1: support block 위에 release해 stack assembled

moving block의 source surface는 문장에 없으므로 source_region으로 유지한다.

### 16.11 semantic 산출물

각 task directory에 다음이 있다.

~~~text
README.md
semantic_graph_v3.json
semantic_segment_catalog.csv
semantic_event_catalog.csv
representative_semantic_phase.csv
standardized_semantic_phase_trajectories.npz
*_semantic_only_graph.png/svg
*_semantic_hierarchy.png/svg
checksums.sha256
~~~

경로:

- docs/artifacts/t1_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t2_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t3_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t4_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t5_semantic_only_graph_v3_2026-08-24
- docs/artifacts/t6_semantic_only_graph_v3_2026-08-25

2026-08-25 확인 시 여섯 directory의 checksums.sha256 검증이 모두 통과했다.

### 16.12 semantic graph와 runtime graph의 차이

현재 semantic graph:

~~~text
무슨 의미의 동작인가?
어떤 object/state가 전후로 바뀌는가?
30개 demonstration의 분포는 어떤가?
~~~

향후 upper planner/runtime:

~~~text
현재 어떤 semantic phase에서 다음 skill로 옮길 것인가?
두 policy의 pose/visual/state envelope가 호환되는가?
Bridge가 필요한가?
안전하게 실행 가능한가?
~~~

두 문제를 의도적으로 분리했다.

---

## 17. 현재 semantic 조합 구상의 현실성

### 17.1 가능한 부분

현재 데이터로 높은 현실성이 있는 작업:

- gripper event 기반 1차 semantic span 구분
- drawer/container handle interaction과 block transport 구분
- T4/T5 inverse relation 표현
- 동일한 blue block acquire/deliver primitive 후보 통합
- 상위 planner가 structured state graph를 읽는 것
- 기존 Task-C fresh inference/atomic handoff 재사용

### 17.2 현재 바로 할 수 없는 부분

- TASK_DESCRIPTION만 보고 unseen motion 생성
- whole-task ACT를 임의 semantic phase부터 안정적으로 시작
- gripper closed만으로 held object identity 확정
- frame/영상 평균으로 대표 장면 생성
- semantic event 자체를 안전 전환점으로 사용
- T1~T6 그래프를 자동 조합해 실기에 바로 실행
- AI API가 30 Hz Cartesian command를 직접 생성

### 17.3 권장 계층

~~~mermaid
flowchart TD
    NL[Natural-language instruction] --> PL[LLM/VLM high-level planner]
    PL --> VAL[Deterministic schema & plan validator]
    VAL --> CAT[Whitelisted skill catalog]
    CAT --> EX[Local SkillGraphExecutor]
    EX --> DET[Precondition/effect detectors]
    EX --> TR[Bridge or certified reposition]
    EX --> POL[Skill ACT policies]
    POL --> MUX[MUX / Live / Safety / ServoL]
    DET --> EX
~~~

AI는 저주기 의미 해석과 계획에만 사용하고 robot command 권한은 갖지 않는다.

### 17.4 권장 primitive

너무 작은 move-left/close/lift보다 다음 크기가 적합하다.

- acquire(object, source)
- deliver(object, target)
- open_drawer
- close_drawer
- open_white_container
- stack(object, support)

contact 중인 drawer/container handle interaction은 중간에서 다른 정책으로 전환하지 않는 것이 안전하다.

### 17.5 Bridge 허용 조건

~~~text
current contact mode = free_transport
next entry contact mode = free_transport
held object identity 동일
gripper state 동일
object state 동일
next preconditions 충족
pose/orientation/visual envelope 안
~~~

다음에는 Bridge 금지:

- drawer handle 접촉 중
- container handle 접촉 중
- release 직전 surface contact
- held object 불명
- object identity 불일치
- scene OOD
- IK/collision 검증 실패

---

## 18. Diffusion 실험 경로

Diffusion은 기존 ACT 환경을 건드리지 않도록 분리되었다.

- 전용 venv
- 전용 Docker image
- 전용 파생 dataset
- 전용 camera adapter
- 전용 async rollout
- 기존 Doosan send_action/MUX/Safety/ServoL 재사용

정책 설정:

| 항목 | 값 |
|---|---|
| n_obs_steps | 2 |
| horizon | 16 |
| training n_action_steps | 8 |
| runtime extended action steps | 15 |
| vision backbone | ResNet18 |
| separate camera encoders | true |
| scheduler | DDIM |
| train timesteps | 100 |
| inference steps | 5 |
| AMP | true |
| batch | 16 |
| steps | 50k |

확인된 결과:

- 서버 50k 학습 완료
- Jetson benchmark 약 168 ms
- 실제 카메라 dry-run 약 30 Hz
- 10초 bounded live 성공
- 40초 full live 30.002 Hz
- MUX/stale/safety 중단 0
- 하지만 gripper close 미발생

원인 분리:

- dataset에는 정상 open/close 존재
- normalization 정상
- send_action/MUX/driver 정상
- 학습 close frame 입력에서는 close 재현
- 실제 rollout scene에서는 gripper output 최대 약 0.20022
- 즉 hardware/MUX가 아니라 policy가 close state를 인식하지 못함
- 가장 가능성 높은 원인은 누적 trajectory 오차로 인한 visual/TCP OOD

따라서 threshold를 낮추는 방식은 쓰지 않았고, 현재 ACT가 주력이다.

---

## 19. End-to-end 운영 흐름

### 19.1 데이터 수집

~~~text
1. 중복 ROS/record process 정리
2. accepted full bringup 시작
3. control scheduling 적용
4. 작업 공간 안전 확인
5. MetaQuest calibration 준비
6. record wrapper 시작
7. preflight에서 gripper OPEN 검증
8. 첫 episode reset
9. prepare/recenter/calibration/OPEN/preflight
10. 30 Hz record
11. n/r/q review
12. 목표 episode까지 반복
13. dataset metadata/frame/video 검증
14. dataset freeze/backup
~~~

### 19.2 서버 학습

~~~text
1. finalized dataset만 서버로 복사
2. meta/data/videos tree 확인
3. LeRobot dataset load check
4. GPU 0 또는 1 명시
5. batch 32, eval_split 0.1
6. task 복잡도에 따라 40k/50k/80k
7. train_config와 final checkpoint 확인
8. 전체 pretrained_model directory를 Jetson으로 복사
9. model.safetensors뿐 아니라 config/processors/train_config 보존
~~~

### 19.3 단일 ACT 실기 검증

~~~text
1. policy-only control bringup
2. Live OFF/DISABLED
3. prepare robot
4. gripper OPEN verified
5. task 초기 scene 배치
6. policy load
7. ACT async chunk ready 확인
8. 5초 bounded gate
9. reset
10. episode 길이만큼 full bounded gate
11. success/failure와 gripper summary 기록
12. Live OFF/DISABLED 확인
~~~

### 19.4 semantic 분석

~~~text
1. dataset schema/TASK_DESCRIPTION 확인
2. 모든 episode gripper grammar audit
3. semantic anchors 결정
4. span별 Cartesian arc-length phase 정렬
5. component median/covariance/MAD/residual 계산
6. hierarchy/world state 생성
7. CSV/JSON/NPZ/graph 저장
8. checksums 생성
9. 원본 dataset과 model 무변경 확인
~~~

### 19.5 향후 skill composition

~~~text
1. semantic-only graph를 catalog로 등록
2. raw frame boundary와 clip reference를 별도 index로 보존
3. skill별 segment dataset 생성
4. skill-specific ACT 재학습
5. precondition/effect detector 구현
6. N-skill executor 구현
7. command-free replay
8. policy shadow
9. bounded physical trial
10. natural-language planner 연결
~~~

---

## 20. 현재 완료도

| 구성 | 상태 | 검증 범위 |
|---|---|---|
| MetaQuest A0509 teleop | 구현·실기 accepted | 고정 mapping/workcell |
| MUX-first 안전 제어 | 구현 | unit/launch/실기 흐름 |
| JRT gripper | 구현·물리 매핑 검증 | DO1 OPEN, DO2 CLOSE |
| shadow recording | 구현·실사용 | T1~T6 수집 |
| reset/review | 구현·실사용 | 반복 수집 로그 존재 |
| multiprocess encoder | 구현·실사용 | episode frame/drop metrics |
| ACT server training | 완료 | T1~T6 final checkpoint |
| ACT Jetson async | 구현 | load/warmup/chunk/live gate |
| T1~T6 task 성공률 | 미완료 | 반복 실기 benchmark 필요 |
| Task-C V0 | 구현·물리 성공 | 고정 scene 2회 연속 |
| Task-C V1 | 구현·물리 성공 | endpoint direct branch 1회 |
| V1 alignment fallback | offline만 | 물리 발생 성공 미확인 |
| semantic V3 | 완료 | T1~T6 checksum 검증 |
| semantic→runtime 자동 조합 | 미구현 | 설계 단계 |
| SkillGraphExecutor | 미구현 | 설계 단계 |
| AI API planner | 미구현 | 역할/안전 계약만 정의 |
| Diffusion | 실험 구현 | arm 실행 성공, grasp 실패 |

---

## 21. 알려진 기술 부채와 재현성 문제

### 21.1 accepted baseline checksum 불일치

현재 실제 파일 SHA-256:

~~~text
856c62c933c12258f396d4d7777583d0df3cc40b977393a987e1d8e03ac23efa
~~~

scripts/record_lerobot_shadow_pilot.sh도 이 hash를 사용하므로 현재 수집 wrapper와 config는 일치한다.

그러나 docs/a0509_metaquest_accepted_baseline_2026-08-07.md 상단과 freeze verification에는 과거 hash가 남아 있다.

~~~text
46dc5d022db4221009a13848b0b514d4b366aecae72158c0db362815ae02866e
~~~

현재 config의 max_vr_jump_m은 0.20이며 과거 0.15에서 수정되었다. accepted snapshot을 immutable이라고 부르면서 파일 자체를 수정한 상태이므로 다음 중 하나로 정리해야 한다.

1. 새 accepted config 이름/날짜를 만들고 현재 hash를 문서화
2. 기존 2026-08-07 snapshot을 원상 보존하고 변경분을 새 파일로 분리

현재 상태에서는 pilot script는 동작하지만 historical freeze 문서 재현성은 깨져 있다.

### 21.2 dirty working tree

핵심 live gate, Task-C, Diffusion, semantic 파일 상당수가 untracked 또는 미커밋이다.

위험:

- 다른 clone에서 현재 기능이 사라짐
- source/build/install 간 drift
- 어떤 실험이 어떤 commit에서 수행됐는지 불명
- .orig 파일이 분석 대상을 혼동시킴

권장:

- .orig는 별도 backup 위치로 이동하거나 ignore
- 기능별 commit
- 실증 tag
- dataset/model/manifest checksum과 git commit 연결

### 21.3 resume wrapper 부재

새 수집 wrapper는 기존 root를 거부하지만, resume는 긴 direct CLI를 수동으로 입력해야 한다. 이 과정에서 num_episodes 오해와 오타가 반복됐다.

권장 새 script:

~~~text
resume_lerobot_shadow_record.sh
- dataset root에서 기존 metadata 자동 로드
- target total episode 입력
- remaining 자동 계산
- original fps/cameras/codec/task 검증
- dry summary 출력
- 명시적 확인 후 --resume=true
~~~

### 21.4 task metadata의 비정규성

예:

- T1 문장이 문법적으로 모호
- T2 source surface 누락
- T3 destination 영역 불명확
- 대소문자/마침표 불일치
- train repo_id와 local dataset name에 vN/tN 혼재

자연어만 고치지 말고 structured metadata를 추가해야 한다.

~~~yaml
objects:
  - blue_block
  - drawer
initial_state:
  drawer: closed
  blue_block: on_black_table
ordered_goals:
  - drawer_open
  - holding_blue_block
  - blue_block_in_drawer
  - drawer_closed
~~~

### 21.5 실제 state sensor 부족

현재 없는 정보:

- jaw width/encoder
- motor current
- force/contact
- actual held object
- object pose
- drawer/container actual state
- collision geometry

따라서 gripper closed=holding은 안전한 사실이 아니다.

### 21.6 semantic span과 실행 경계의 분리 미구현

semantic V3가 올바르게 의미와 전환 판단을 분리했지만, 다음 runtime layer가 아직 없다.

- span 내부 transition candidate evaluation
- current/next policy visual compatibility
- held object identity
- contact mode
- entry/exit envelope
- upper planner decision
- bridge/reposition selection

### 21.7 모델 평가 부족

현재 train/eval loss, checkpoint existence, inference load는 확인되어도 task success rate는 없다.

필요한 task별 지표:

- 10회 이상 success count
- grasp success
- release success
- drawer/container terminal state
- stack stability
- collision/intervention
- gripper driver confirmation
- final Live OFF/DISABLED
- failure taxonomy

---

## 22. 권장 다음 구현 순서

### P0: 재현성 고정

1. accepted config를 새 버전 파일로 분리
2. checksum 문서 수정
3. 현재 핵심 소스 commit
4. dataset/model/semantic artifact manifest 생성
5. 모든 실증 결과에 git commit 기록

### P1: task별 실제 정책 검증

T1~T6 각각:

1. 5초 bounded motion
2. 전체 episode duration
3. 10회 reset-and-repeat
4. 성공 predicate 수동/자동 기록
5. gripper target과 driver state 비교
6. 실패 episode 카메라/target trace 저장

### P2: semantic index 확장

현재 aggregate graph 외에 원본 episode boundary index를 만든다.

권장 schema:

~~~text
dataset_id
episode_index
segment_id
start_frame
end_frame
intent_event_frame
observed_event_frame
entry_gripper_state
exit_gripper_state
held_object
contact_object
source_location
target_location
preconditions_json
effects_json
clip_refs_json
confidence
label_source
human_approved
~~~

frame은 평균 대표값이 아니라 각 episode의 raw reference로만 유지한다.

### P3: skill dataset 생성

우선 T4/T5를 다음 primitive로 분해한다.

- open_drawer
- close_drawer
- acquire_blue_from_black_table
- acquire_blue_from_drawer
- deliver_blue_to_drawer
- deliver_blue_to_black_table

같은 원본 episode에서 나온 segment가 train/eval에 동시에 들어가지 않도록 원본 episode 단위 split을 사용한다.

### P4: skill-specific ACT

whole-task checkpoint 중간 진입보다 분할 dataset으로 skill별 정책을 새로 학습한다.

각 skill manifest에:

- preconditions
- effects
- contact mode
- held object
- gripper rule
- entry/exit pose envelope
- visual envelope
- success detector
- failure action

을 둔다.

### P5: SkillGraphExecutor

기존 Task-C coordinator를 N-skill로 일반화한다.

~~~text
WAITING_FOR_START
→ LOAD_SKILL
→ VERIFY_PRECONDITIONS
→ RUN_SKILL
→ VERIFY_EFFECTS
→ SELECT_TRANSITION
→ PLAN_BRIDGE_OR_REPOSITION
→ SETTLE
→ FRESH_NEXT_POLICY_INFERENCE
→ ATOMIC_HANDOFF
→ COMPLETE
~~~

실패:

~~~text
FAILED_HOLD
→ queue invalidate
→ last safe target hold
→ Live OFF
→ MUX DISABLED
→ operator notification
~~~

### P6: upper planner

LLM/VLM은 자연어를 whitelisted skill DSL로 변환한다.

~~~json
{
  "goal": ["blue_block_in_drawer", "drawer_closed"],
  "skills": [
    {"id": "open_drawer"},
    {"id": "acquire_blue_from_black_table"},
    {"id": "deliver_blue_to_drawer"},
    {"id": "close_drawer"}
  ]
}
~~~

deterministic validator가 다음을 확인한다.

- catalog에 skill 존재
- precondition/effect 연결
- object identity
- scene state
- 누락 primitive
- 금지된 contact transition

LLM/VLM은 safety threshold 변경이나 30 Hz command 생성 권한을 갖지 않는다.

---

## 23. GPT가 후속 분석할 때 지켜야 할 사실 경계

### 확인된 사실

- T1~T6 dataset과 final ACT checkpoint가 로컬에 존재한다.
- 여섯 ACT architecture/training common config는 동일하다.
- semantic V3 checksums는 모두 통과했다.
- Task-C V0는 고정 scene에서 두 번 전체 성공했다.
- representative V1은 endpoint direct branch로 한 번 전체 성공했다.
- Diffusion은 실제 arm rollout을 수행했지만 grasp close를 예측하지 못했다.
- MUX/Live는 disabled-by-default다.

### 추론 또는 설계

- T1~T6를 분할 skill로 재학습하면 일반 조합 성능이 좋아질 것이라는 판단
- upper planner가 semantic span 내부의 최적 전환을 선택할 수 있다는 구상
- visual embedding/OOD detector가 entry compatibility를 보완할 것이라는 제안
- 40k/50k/80k가 각 task의 최적 step이라는 주장

### 아직 증명되지 않은 주장

- T1~T6 모든 whole-task policy가 안정적으로 성공
- semantic label만으로 policy를 arbitrary phase에서 실행 가능
- gripper closed이면 물체를 실제로 holding
- Task-C가 새로운 물체/장면에 일반화
- AI API가 자동으로 안전한 novel task를 구성
- V1 alignment fallback의 실제 물리 성공
- semantic graph가 현재 robot command를 직접 생성

---

## 24. 핵심 파일 안내

### 제어

- src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml
- src/quest_a0509_teleop/launch/a0509_full_bringup.launch.py
- src/quest_a0509_teleop/launch/a0509_full_bringup_with_gripper.launch.py
- src/quest_a0509_teleop/quest_a0509_teleop/xyz_mapper_node.py
- src/quest_a0509_teleop/quest_a0509_teleop/a0509_command_mux_node.py
- src/quest_a0509_teleop/quest_a0509_teleop/command_mux_core.py
- src/quest_a0509_teleop/quest_a0509_teleop/safety_guard_node.py
- src/quest_a0509_teleop/quest_a0509_teleop/servol_rt_streamer_node.py
- src/quest_a0509_teleop/quest_a0509_teleop/robot_prep_node.py

### 그리퍼

- src/jrt_gripper_io/jrt_gripper_io/gripper_logic.py
- src/jrt_gripper_io/jrt_gripper_io/jrt_tool_io_driver_node.py
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/recording_gripper_initializer.py
- docs/jrt_gripper_mapping_revalidation_2026-08-08.md

### LeRobot와 recording

- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/doosan_a0509_ros.py
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/metaquest_a0509.py
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/episode_reset_orchestrator.py
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/multiprocess_streaming_encoder.py
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/deadline_record_loop.py
- scripts/preflight_lerobot_shadow_record.py
- scripts/record_lerobot_shadow_pilot.sh

### ACT live

- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/act_async_rollout.py
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/rollout_entrypoint.py
- scripts/run_a0509_act_policy_live.sh
- scripts/a0509_act_live_trial_gate.py
- scripts/run_task_c_control_bringup.sh

### Task-C

- offline_tools/task_c_bridge_v0
- offline_tools/task_c_bridge_v1
- src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/task_c_live_rollout.py
- scripts/run_task_c_live_candidate.sh
- scripts/run_task_c_live_gate.sh
- docs/task_c_full_live_validation_2026-08-14.md
- docs/task_c_representative_bridge_v1_2026-08-18.md

### semantic

- offline_tools/semantic_segmentation/semantic_phase_core.py
- offline_tools/semantic_segmentation/analyze_a0509_semantic_only_v3.py
- offline_tools/semantic_segmentation/analyze_t1_semantic_only_v3.py
- docs/artifacts/t1_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t2_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t3_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t4_semantic_only_graph_v3_2026-08-23
- docs/artifacts/t5_semantic_only_graph_v3_2026-08-24
- docs/artifacts/t6_semantic_only_graph_v3_2026-08-25

### 기존 상세 보고서

- docs/a0509_metaquest_accepted_baseline_2026-08-07.md
- docs/lerobot_a0509_mux_architecture.md
- docs/lerobot_episode_reset_workflow_2026-08-09.md
- docs/a0509_semantic_skill_composition_analysis_2026-08-23_ko.md
- docs/a0509_diffusion_progress_2026-08-11.md
- ROS2_WS_ANALYSIS_REPORT_KO.md

---

## 25. 주요 ROS topic과 service

### Topic

| Topic | 역할 |
|---|---|
| /q2r_right_hand_pose | Quest hand pose |
| /q2r_right_hand_inputs | Quest buttons |
| /control/metaquest/target_posx | MetaQuest arm proposal |
| /control/metaquest/gripper_cmd | MetaQuest gripper proposal |
| /control/metaquest/valid_pose_heartbeat | MetaQuest freshness |
| /control/lerobot/target_posx | policy arm proposal |
| /control/lerobot/gripper_target | policy gripper 0..1 |
| /control/lerobot/policy_queue_ready | fresh Live generation queue |
| /control/source | selected source |
| /control/selected_command_valid | selected target validity |
| /control/selected_command_heartbeat | downstream freshness |
| /vr/target_posx | MUX-selected target |
| /vr/safe_posx | safety output |
| /vr/commanded_posx | streamer acknowledged command |
| /vr/live_robot_output_enabled | Live state |
| /vr/teleop_ready | prepare state |
| /vr/metaquest_calibration/valid | calibration state |
| /rt_topic/actual_tcp_position | actual TCP |
| /rt_topic/robot_state | robot state |
| /dsr01/joint_states | joint state |
| /jrt_gripper/cmd | gripper command |
| /jrt_gripper/commanded_state | logical last completed state |
| /jrt_gripper/driver_busy | driver activity |
| /jrt_gripper/last_command_ok | driver result |
| /jrt_gripper/completed_command | completed open/close |

### Service

| Service | 역할 |
|---|---|
| /vr/prepare_robot | 준비자세 이동/anchor 갱신 |
| /vr/recenter | Quest relative origin 갱신 |
| /vr/calibrate_xy_yaw_to_x_plus | session calibration |
| /vr/set_live_robot_output | Live OFF/ON |
| /control/select_disabled | source DISABLED |
| /control/select_metaquest | source METAQUEST |
| /control/select_lerobot | source LEROBOT |
| /vr/hold_servol | 현재 안전 pose hold |

---

## 26. 분석용 구조화 snapshot

~~~yaml
project:
  name: Dosan-MetaQuest-teleoperation-project
  robot: Doosan A0509
  gripper: JRT JEGB via Tool DO
  runtime_host: Jetson Thor
  ros: Jazzy
  lerobot: 0.6
  control_rate_hz: 30
  mux_default: DISABLED
  live_default: false

observation:
  state_dim: 13
  cameras:
    front: [480, 640, 3]
    side: [480, 640, 3]
    zed_rgb: [376, 672, 3]

action:
  dim: 7
  arm: Doosan posx XYZ plus O1/O2/O3
  gripper: continuous_0_to_1

act:
  models: 6
  batch_size: 32
  chunk_size: 100
  n_action_steps: 100
  eval_split: 0.1
  async_fifo: true
  live_generation_isolation: true

datasets:
  t1: {episodes: 30, frames: 36000, duration_s: 40}
  t2: {episodes: 31, frames: 27899, duration_s: approximately_30}
  t3: {episodes: 30, frames: 27000, duration_s: 30}
  t4: {episodes: 30, frames: 54000, duration_s: 60}
  t5: {episodes: 30, frames: 54000, duration_s: 60}
  t6: {episodes: 30, frames: 27000, duration_s: 30}

semantic_v3:
  tasks: [t1, t2, t3, t4, t5, t6]
  event_source: observation_state_gripper_commanded_state
  phase_axis: per_segment_cartesian_arc_length_0_to_1
  phase_points: 101
  representative: synthetic_component_median_xyz
  representative_episode_selected: false
  frame_average: false
  image_average: false
  transition_fitness: false
  spherical_boundary: false
  bridge_planning: false
  robot_executable: false

task_c:
  form: A_prefix_plus_Bezier_Bridge_plus_B_suffix
  v0_physical_full_successes: 2
  v1_endpoint_physical_full_successes: 1
  generalized_skill_graph_executor: not_implemented

current_priority:
  - freeze_reproducible_source_and_config
  - measure_t1_to_t6_physical_success_rates
  - create_episode_level_semantic_segment_index
  - train_skill_specific_policies
  - implement_deterministic_skill_graph_executor
  - connect_high_level_language_planner_without_command_authority
~~~

---

## 27. 최종 판단

현재 프로젝트는 세 가지 중요한 기반을 이미 확보했다.

1. 실제 robot에서 사용할 수 있는 fail-closed MetaQuest/LeRobot 공통 제어 계층
2. multi-view demonstration 수집과 ACT 학습·Jetson 실행 파이프라인
3. 두 정책을 fresh inference와 안전 Bridge로 실제 연결한 경험

T1~T6 semantic V3는 이 기반 위에 의미 구조를 추가했다. 다만 현재 결과는 어디까지나 semantic annotation graph다. 실제로 새 작업을 조합하려면 whole-task checkpoint를 임의로 자르는 것보다 semantic segment dataset을 만들고 skill별 정책을 재학습한 뒤, deterministic precondition/effect 검증과 기존 Task-C의 안전 handoff를 일반화해야 한다.

가장 중요한 설계 원칙은 다음과 같다.

> 의미를 이해하는 계층, 정책을 실행하는 계층, 로봇 출력 권한을 갖는 안전 계층을 분리한다.

상위 AI는 무엇을 할지 제안할 수 있지만, 실제 30 Hz target과 gripper command는 검증된 local policy/executor와 MUX/Live/Safety 계층을 통해서만 실행되어야 한다.

