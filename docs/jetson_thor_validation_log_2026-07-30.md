# Jetson Thor 이관 및 하드웨어 검증 기록

- 최종 갱신: 2026-07-30 (Asia/Seoul)
- 호스트: `jetson-thor-02`
- 작업공간: `~/Dosan-MetaQuest-teleoperation-project`
- 대상 로봇: Doosan A0509
- 소프트웨어 환경: Ubuntu 24.04, arm64, ROS 2 Jazzy

이 문서는 기존 노트북에서 Jetson Thor로 프로젝트를 이관한 뒤 실제로
확인한 USB 카메라, 로봇 Ethernet, Doosan 드라이버, 그리퍼, MetaQuest,
준비자세 및 실시간 스케줄링 설정을 기록한다. 확인하지 않은 항목은
성공으로 간주하지 않고 별도로 표시한다.

## 전체 상태 요약

| 항목 | 상태 | 확인 내용 |
| --- | --- | --- |
| ZED2 USB/RGB | 통과 | USB 3.x 5 Gbps 링크와 RGB UVC 접근 확인 |
| ZED2 depth | 사용 안 함 | 현재 프로젝트 범위에서 의도적으로 제외 |
| Logitech C920 | 통과 | LeRobot 기본 카메라 경로로 640x480 캡처 확인 |
| ZED2+C920 동시 캡처 | 통과 | 두 카메라 동시 프레임 획득 실패 없음 |
| Doosan Ethernet | 통과 | 전용 NIC, route, ping, TCP 12345 확인 |
| Doosan DRCF/RT 드라이버 | 통과 | 실제 컨트롤러 연결과 ROS 서비스 확인 |
| JRT 그리퍼 | 통과 | OPEN/CLOSE 물리 동작과 DO 매핑 확인 |
| MetaQuest 통신 | 통과 | TCP 연결, pose/input 토픽과 주기 확인 |
| MetaQuest 좌표 변환 | 통과 | 축, 부호, 0.5 위치 배율, 손목 롤 확인 |
| MetaQuest 세션 회전 보정 게이트 | 소프트웨어 통과 | 전용 GUI/mapper 상태/MUX/LeRobot 연동 및 격리 dry-run 확인 |
| 로봇 준비자세 | 통과 | 실제 이동, 최종 관절/TCP, 정지 상태 확인 |
| realtime 영구 설정 | 통과 | 재로그인 후 4개 verifier PASS, FIFO50 생성 확인 |
| MetaQuest 실제 live 제어 | 30초 통과 | 무제한 세션 30.008초, 실제 TCP `153.126 mm` 이동, 종료 후 추가 명령 0건 확인 |
| MetaQuest A/B 그리퍼 제어 | 통과 | B→OPEN, A→CLOSE 원시 입력부터 Tool DO readback 및 실제 물리 동작까지 확인 |

## 1. 카메라 및 USB

### ZED2

장치 열거 결과:

```text
Bus 001 Device 028: ID 2b03:f781 STEREOLABS ZED-2 HID INTERFACE
Bus 002 Device 005: ID 2b03:f780 STEREOLABS ZED 2
```

- 최종 연결 상태에서 USB 3.x 5 Gbps 링크 확인
- 연장 케이블 재배치 후에도 장치 인식 및 RGB 접근 확인
- 현재는 ZED SDK depth를 사용하지 않고 RGB 영상만 사용
- depth 품질, 동기화 및 장시간 depth 스트레스 테스트는 수행하지 않음

### Logitech C920

- 안정적인 장치 식별 경로:

```text
/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0
```

- LeRobot 경로에서 640x480 실제 프레임 캡처 성공
- ZED2 RGB와 C920을 동시에 열었을 때 프레임 실패 없음
- 짧은 동시 캡처는 통과했으나 장시간 USB 대역폭 스트레스 테스트는 별도

## 2. 네트워크 구성

### Doosan 전용 Ethernet

- 인터페이스: `enP2p1s0`
- Jetson 주소: `192.168.137.20/24`
- 로봇 주소: `192.168.137.100`
- 로봇 제어 포트: TCP `12345`
- 링크 상태: `UP`, `LOWER_UP`

확인된 route:

```text
192.168.137.100 dev enP2p1s0 src 192.168.137.20
```

확인 결과:

- `ping -I enP2p1s0 192.168.137.100` 성공
- `192.168.137.100:12345` TCP 연결 성공
- launch의 `host`와 `rt_host`는 Jetson 주소가 아니라 로봇 주소

### MetaQuest 및 일반 네트워크

- Jetson Wi-Fi: `192.168.0.128/24`
- MetaQuest: `192.168.0.197`
- ROS-TCP-Endpoint: Jetson TCP `10000`
- Wi-Fi는 Quest/일반 트래픽, `enP2p1s0`은 로봇 전용으로 분리

## 3. Doosan 드라이버

실제 A0509 컨트롤러를 대상으로 다음을 확인했다.

- DRCF 및 RT 연결 성공
- driver activation 후 로봇 상태 `STANDBY` 확인
- 서비스 namespace:

```text
/dsr01/dsr_controller2/...
```

- Tool Digital Output 서비스:

```text
/dsr01/dsr_controller2/io/set_tool_digital_output
```

- 드라이버 activation 자체로 계획되지 않은 관절 이동은 발생하지 않음
- 기존 실행에서는 `SCHED_FIFO` 권한 부족 경고가 있었으며, 이를 해소하기
  위한 realtime 설정을 설치한 상태

## 4. 그리퍼 물리 매핑

실제 펄스 동작으로 확인한 매핑:

| 기능 | Tool DO | 활성 값 | 펄스 |
| --- | ---: | ---: | ---: |
| OPEN | 1 | 1 | 0.20 s |
| CLOSE | 2 | 1 | 0.20 s |

- `1=ON`, `0=OFF`
- OPEN과 CLOSE를 각각 실제 동작으로 확인
- 최종 검증은 `OPEN → 1초 대기 → CLOSE` 순서
- 펄스 종료 뒤 DO1/DO2 readback 모두 `0`
- 프로젝트 기본값을 `open=1`, `close=2`로 정리
- 잘못되어 있던 Tool DO 서비스 주석도 `0=OFF, 1=ON`으로 수정
- 관련 패키지 build 성공 및 관련 테스트 39개 통과

## 5. MetaQuest 통신 및 좌표 dry-run

### 연결과 토픽

- Quest `192.168.0.197`에서 Jetson `192.168.0.128:10000` 연결 성공
- 오른손 pose:

```text
/q2r_right_hand_pose
```

- 오른손 버튼/입력:

```text
/q2r_right_hand_inputs
```

- pose 입력 주기: 약 72 Hz
- dry-run 좌표 변환 출력 주기: 약 30 Hz
- 각 입력 토픽 publisher 1개 확인
- 버튼 메시지 필드 수신 확인

### 안전한 독립 좌표 테스트

좌표 테스트에서는 다음만 실행했다.

- ROS-TCP-Endpoint
- `xyz_mapper_node`

다음은 실행하지 않았다.

- `controller_manager`
- Doosan 드라이버
- ServoL streamer
- 그리퍼 driver

변환 출력은 전용 토픽으로 분리했다.

```text
/control/metaquest/dryrun_target_posx
```

이 토픽의 subscriber 수는 `0`이었으므로 로봇으로 명령이 전달되지 않았다.

### 확인된 좌표 변환

프로젝트 설정:

```text
robot X =  0.5 * Quest Y
robot Y = -0.5 * Quest X
robot Z =  0.5 * Quest Z
```

실제 로그 예:

```text
Quest delta:  [-69.63, +371.07, +8.81] mm
Robot delta: [+185.54,  +34.81, +4.40] mm
```

위 결과는 축 순서, Y축 부호 및 0.5 배율과 일치한다.

30초 재측정에서 관측된 target 범위:

```text
X: 약 110 mm
Y: 약 208 mm
Z: 약  63 mm
```

손목 롤은 로봇 `Ry`에만 반영되었고 측정 구간에서 약 15도 범위를
확인했다. `Rx`와 `Rz`는 고정되었다.

### 추적 점프 보호

첫 독립 실행에서 Quest의 초기 pose와 추적 정상화 후 pose 사이에 약
0.8 m 차이가 발생했다. `max_vr_jump_m=0.15` 보호 로직이 이를 거부하여
target을 기준점에 유지했다. 이후 `/vr/recenter`를 호출해 최신 정상 pose를
기준으로 채택하고 jump filter를 리셋한 뒤 재측정했다.

실제 통합 준비자세 흐름에서는 robot anchor 갱신 시 Quest recenter가 함께
수행되도록 설정되어 있다.

### 종료 시 참고 사항

Endpoint를 `Ctrl+C`로 종료할 때 Python daemon thread의 stdout lock 관련
종료 메시지와 exit code `-6`이 한 번 발생했다. 실행 중 통신에는 문제가
없었고 종료 후 다음을 확인했다.

- TCP 10000 listener 없음
- `UnityEndpoint`/`xyz_mapper_node` 없음
- Doosan/ServoL/그리퍼 관련 프로세스 없음

이 문제는 live 제어와 별개인 종료 경로의 기술 부채로 추적한다.

## 6. 실제 준비자세 이동

준비자세 파라미터는 원래 속도로 복구했다.

```text
prepare_vel_deg_per_sec = 30
prepare_acc_deg_per_sec2 = 30
prepare_max_step_deg = 10
```

목표 관절각:

```text
[0, 0, 90, 0, 60, 0] deg
```

실제 최종 관절각:

```text
[0.0372, -0.0654, 90.0000, 0.0791, 60.0000, 0.0110] deg
```

최종 관절 속도는 모두 0이었고 로봇 상태는 `STANDBY`였다.

준비자세 완료 후 측정 TCP:

```text
[428.3789, 0.8424, 457.4031, 0.2785, 149.9342, 0.2897]
```

준비자세 완료 시 `teleop_ready=true`가 되었지만 다음 안전 상태는 유지했다.

```text
control source = DISABLED
streamer dry_run = true
live_robot_output_enabled = false
```

따라서 Quest target은 로봇으로 출력되지 않았다.

## 7. Realtime 스케줄링

### 설정 전 진단

```text
kernel: 6.8.12-tegra PREEMPT
kernel.sched_rt_runtime_us: 950000
rvlab realtime group: 없음
ulimit -r: 0
chrt --fifo 50 true: Operation not permitted
```

커널과 RT runtime은 준비되어 있었고 사용자 권한만 빠져 있었다.

### 설치 완료 상태

실행한 명령:

```bash
sudo bash scripts/setup_realtime_permissions.sh
```

확인 결과:

```text
realtime:x:2003:rvlab
/etc/security/limits.d/99-realtime.conf: root:root 644
```

설치된 limits:

```text
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```

설치 직후 기존 로그인 셸에서는 아직 `ulimit -r=0`이었지만, 재로그인
후 아래 8절과 같이 권한과 실제 드라이버를 모두 재검증했다.

## 8. 재로그인 후 realtime/드라이버 재검증

`scripts/verify_realtime_permissions.sh`를 일반 사용자 `rvlab`로 실행했고
다음 네 항목이 모두 통과했다.

1. 현재 세션의 `realtime` 그룹
2. realtime priority 99
3. locked memory `unlimited`
4. `SCHED_FIFO` priority 50 프로세스 생성

이어서 `doosan_a0509_real.launch.py`만 실행해 실제 컨트롤러를 다시
검증했다.

- DRCF 연결 성공
- authority 획득 및 로봇 상태 `STANDBY`
- RT 연결 성공
- `dsr_controller2`와 `joint_state_broadcaster` active
- RT 진단 약 99.98 Hz
- 측정 관절/TCP 속도 모두 0
- 계획되지 않은 로봇 이동 없음
- 종료 code 0, 관련 프로세스 잔류 없음

종료 중 controller deactivate 순서 관련 경고가 있었지만 실행 중 RT 연결,
상태, 주기 및 무이동 검증에는 영향을 주지 않았다.

## 9. 소프트웨어 안전 보강 및 무GUI 통합 dry-run

실제 연결 전에 다음 네 경로를 보강했다.

1. 준비 동작 중 `/vr/stop_robot`이 preparation lock 뒤에서 대기하지 않고
   취소 token을 세운 뒤 `MoveStop`과 live-disable을 수행한다.
2. 그리퍼 실행 중 명령을 버리지 않고 한 개 pending 명령으로 합치며,
   `stop`은 이후 `open`/`close`로 덮어쓸 수 없다. 각 비동기 Tool DO 호출은
   응답 deadline을 가지며 실패/timeout 시 남은 plan을 폐기하고 양 출력
   OFF를 시도한다.
3. 수동 `DISABLED`와 source timeout은 동일하게 gripper `stop`, 즉시
   live-disable, ServoL hold를 요청한다. 이미 `DISABLED`여도 이 안전 동작을
   다시 요청한다.
4. MUX의 MetaQuest freshness는 raw pose가 아니라 mapper가 유효성 및
   jump 검사를 통과시킨 `/control/metaquest/valid_pose_heartbeat`만 사용한다.

소프트웨어 검증 결과:

- 핵심 안전 테스트: 34개 통과
- 관련 패키지 전체 회귀 테스트: 71개 통과
- Python `compileall`: 통과
- `jrt_gripper_io`, `quest_a0509_teleop` symlink build: 통과
- 무GUI 통합 launch의 8개 노드 정상 기동 및 정상 종료
- 초기 상태: source `DISABLED`, selected valid `false`, live `false`, gripper
  busy `false`
- 반복 `DISABLED` 요청: gripper stop, live-disable, hold 로그 확인
- 정상 합성 Quest pose에서 `METAQUEST` 선택 유지 확인
- 정상 pose를 끈 뒤 1 m jump pose만 20 Hz로 계속 발행했을 때 mapper가
  jump를 거부하고 입력 timeout을 유지했으며, MUX가 0.3초 watchdog으로
  `DISABLED` 전환 후 stop/live-disable/hold 실행
- 종료 뒤 관련 launch/node/publisher 프로세스 잔류 없음

이 통합 검증은 `start_robot_bringup=false`, `start_endpoint=false`,
`start_gui=false`, `dry_run=true`, `teleop_ready=false`로 수행했다. 따라서
Doosan ServoL 또는 실제 Tool I/O 명령은 전송되지 않았다.

### 안전 보강 후 실제 준비자세 재검증

안전 보강 코드를 포함한 전체 통합 launch를 실제 A0509에 연결하되
`start_gui=false`, `dry_run=true`, 초기 source `DISABLED`로 실행했다.
거의 0도인 초기 관절 상태에서 `/vr/prepare_robot`을 호출했고 다음 순서의
실제 이동이 정상 완료됐다.

1. J5를 약 30도까지 회피
2. J3를 약 20도까지 회피
3. 최종 목표 `[0, 0, 90, 0, 60, 0] deg`로 이동

완료 직후 `/dsr01/joint_states`에서 확인한 관절각은 다음과 같다.

```text
[0.0000, 0.0069, 90.0000, 0.0014, 60.0000, 0.0019] deg
```

새로 설정된 TCP 기준점은 다음과 같다.

```text
[428.7419, 0.4207, 456.8098, 0.1077, 150.0136, 0.1263]
```

사후 독립 측정 결과 관절 속도와 TCP 6축 속도는 모두 0,
`teleop_ready=true`, `live_robot_output_enabled=false`였으며
`/dsr01/dsr_controller2/servol_rt_stream` publisher는 0개였다. 따라서
준비 동작만 실제로 수행됐고 MetaQuest의 ServoL 출력은 발생하지 않았다.

재기동 뒤 Quest 앱이 아직 연결되지 않아 `/vr/recenter`는
`no Quest pose received yet`로 보류됐다. mapper는 다음 첫 유효 Quest
pose에서 현재 TCP 기준점에 맞춰 자동으로 VR 기준점을 설정하도록 대기
중이다.

### Quest 재연결 후 실제 드라이버 연동 dry-run

Quest 앱 재실행 후 `192.168.0.197`에서 Jetson TCP 10000 포트 연결과
right-hand pose/input publisher 등록을 확인했다. 첫 pose 자동 기준 설정도
성공했다.

첫 기준 설정 직후 Quest tracking frame이 약 0.82 m 재배치됐지만 mapper의
안전 jump 필터가 해당 pose들을 거부하고 freshness timeout을 유지했다.
`latest_raw_vr_pose_m` 복구 경로를 사용하는 명시적 `/vr/recenter` 호출은
성공했고 유효 pose 입력이 즉시 재개됐다.

측정 주기:

- `/q2r_right_hand_pose`: 약 72 Hz
- `/q2r_right_hand_inputs`: 약 72 Hz
- `/control/metaquest/valid_pose_heartbeat`: 약 72 Hz

source를 `DISABLED -> METAQUEST`로 전환해 실제 Quest 입력이 mapper, MUX,
safety guard, streamer까지 전달되는 것을 확인했다. 손 위치가 기준에서 크게
이동했을 때 safety guard는 작업영역 밖의 목표 X를 250 mm로 clamp했고,
streamer는 해당 목표를 `dry_run safe_posx`로만 처리했다.

검증 직후 source를 `METAQUEST -> DISABLED`로 복귀했다. 사후 측정에서 준비
자세 관절각은 변하지 않았고 관절/TCP 속도는 모두 0이었다.
`live_robot_output_enabled=false`와 ServoL publisher 0개도 다시 확인했다.

실제 live 전환 전에는 컨트롤러를 편한 중립 자세에 둔 상태에서 다시
recenter하고 초기 목표가 TCP 기준점 근처인지 확인해야 한다. 현재 dry-run에서
확인한 clamp된 목표를 그대로 live 출력으로 전환하지 않는다.

## 10. 보정 전용 GUI 및 세션 게이트 구현

기존 GUI의 좌표 보정 계산은 원래부터 `xyz_mapper_node` 서비스가 소유하고
있었다. 이 계산과 현재 YAML 변환값은 변경하지 않고, GUI 의존성을 제거한
세션 상태와 안전 게이트를 추가했다.

유지한 정적 변환:

```text
Robot X = +0.5 * Quest delta Y
Robot Y = -0.5 * Quest delta X
Robot Z = +0.5 * Quest delta Z
Robot Ry = -0.7 * Quest relative roll
Robot Rx/Rz = locked
```

mapper가 소유하는 상태:

```text
UNCALIBRATED -> CALIBRATING -> VALID
                         \-> INVALID
```

- mapper 시작 시 기본 `UNCALIBRATED/false`
- 보정 시작 즉시 `CALIBRATING/false`
- 충분한 +X 이동 측정 성공 시 `VALID/true`
- 측정 실패, reset, 추적 jump, 입력 timeout 시 `INVALID/false`
- `/vr/recenter`는 완료된 회전 보정값을 유지하고 위치/자세 anchor만 갱신
- 보정값은 GUI가 아니라 mapper 메모리에 있으며 mapper 재시작 시 재보정 필요

추가된 인터페이스:

```text
/vr/metaquest_calibration/valid
/vr/metaquest_calibration/status
/vr/calibrate_xy_yaw_to_x_plus
/vr/reset_xy_yaw_calibration
```

`metaquest_calibration_gui`는 Calibrate, Recenter, Reset과 상태/품질 표시만
제공한다. Prepare, RT, live-enable, stop, gripper 동작은 포함하지 않는다.
전체 launch에서는 `start_calibration_gui:=true`로 선택 실행하며 mapper가 이미
동작 중이면 `metaquest_calibration_gui.launch.py`만 별도로 실행할 수 있다.

안전 연동:

1. mapper는 보정 전 MetaQuest target 발행을 보류한다.
2. MUX는 보정 전 `METAQUEST` 선택을 거절한다.
3. MetaQuest 선택 중 보정이 무효화되면 MUX가 즉시 `DISABLED`로 전환하고
   gripper stop, live-disable, ServoL hold를 요청한다.
4. LeRobot `MetaQuestA0509` teacher는 mapper-accepted heartbeat와 유효한 보정을
   요구한다. LeRobot policy의 `/control/lerobot/*` 경로는 영향을 받지 않는다.

검증 결과:

- 새 Python 파일 전체 `py_compile` 통과
- Quest teleop 회귀 테스트 33개 통과
- 실제 LeRobot 0.6 가상환경 플러그인 테스트 13개 통과
- `quest_a0509_teleop`, `lerobot_robot_doosan_a0509` symlink build 통과
- `colcon test-result`: 오류 0, 실패 0
- 시스템 Python에서 LeRobot 테스트 1개 모듈은 의존성 부재로 skip되지만,
  지정된 `~/venvs/lerobot` 환경에서는 13개 모두 실행·통과

로봇과 분리한 `ROS_DOMAIN_ID=77` 통합 dry-run에서는
`start_robot_bringup=false`, `start_endpoint=false`, `start_gripper=false`,
`dry_run=true`, `teleop_ready=false`를 사용했다.

- 초기 보정 false에서 `/control/select_metaquest` 거절 확인
- 합성 Quest +Y 0.12 m 이동으로 mapped +X 약 0.0594 m 측정
- 상태 `VALID`, correction `0.0 deg`, 정적 mapping fingerprint 발행 확인
- fresh pose가 계속 입력되는 동안 `DISABLED -> METAQUEST` 선택 성공
- 즉시 reset했을 때 MUX의 `metaquest_calibration_invalidated` 이벤트,
  `METAQUEST -> DISABLED`, gripper stop, live-disable, hold 확인
- 격리 launch의 모든 노드 정상 종료 및 잔류 프로세스 없음

이 검증은 합성 Pose와 dry-run만 사용했으므로 실제 로봇 이동과 Tool I/O는
발생하지 않았다. Tk 8.6 import와 GUI 실행 엔트리 build는 확인했지만 실제
디스플레이에서의 버튼/레이아웃 확인과 실제 Quest 손 이동 보정은 아직
수행하지 않았다. 기존 실제 로봇 연결 세션도 계속 `dry_run=true`, source
`DISABLED`, live output false로 유지했으며 재시작하지 않았다.

## 11. 실제 장치 연결 상태에서 보정 GUI 재기동 검증

실제 Doosan 드라이버와 MetaQuest endpoint를 다시 연결하되 다음 안전 상태로
통합 launch를 재기동했다.

```text
dry_run=true
initial_control_source=DISABLED
start_calibration_gui=true
start_gui=false
start_rviz=false
```

- 실제 A0509 DRCF 및 RT control stream 연결 성공
- MetaQuest Unity endpoint 연결 및 오른손 pose/input publisher 등록 성공
- mapper 첫 pose anchor 설정 성공
- 재시작으로 `teleop_ready=false`, MUX source `DISABLED`, live output false 확인
- `/dsr01/dsr_controller2/servol_rt_stream` publisher count 0 확인

첫 GUI 기동에서는 `rclpy.node.Node.clients` 읽기 전용 속성과 GUI의
`self.clients` 딕셔너리 이름이 충돌하여 프로세스가 exit 1로 종료됐다.
내부 이름을 `self._service_clients`로 변경하고 패키지를 재빌드한 뒤 GUI만
독립 launch로 다시 실행했다. GUI 프로세스 생존과 상태 토픽 수신을
확인했으며, 동일 회귀를 잡는 ROS 노드 생성 테스트를 추가했다.

실제 MetaQuest 이동으로 수신된 보정 결과:

```text
state=VALID
distance_m=0.1326962661
observed_angle_deg=-11.71240972
xy_yaw_correction_deg=+11.71240972
mapping_fingerprint=d4fc94064541b420
```

보정 완료 후에도 MUX 상태는 `source=DISABLED`, `live_enabled=false`,
`teleop_ready=false`로 유지했다. `/control/select_disabled` 재호출도
`source already DISABLED; safety hold requested`로 성공했다. 따라서 이 단계에서
실제 준비자세 이동, ServoL 명령 발행, gripper Tool I/O는 발생하지 않았다.

보정은 유효하지만 `teleop_ready=false`인 상태에서 소스 선택 동작도 추가로
확인했다. 현재 역할 분리에 따라 MUX는 `DISABLED -> METAQUEST` 선택 자체는
허용하지만 mapper가 target과 accepted-pose heartbeat를 발행하지 않는다.
그 결과 약 3.8초 후 MUX source timeout이 발생해 자동 `DISABLED` 복귀,
live-disable, hold, gripper stop이 실행됐다. 전체 구간에서 live output은
false였고 ServoL publisher는 0개였다.

최종 상태 확인 시 Quest tracking jump가 검출되어 보정 상태가 sequence 7,
`INVALID/false`로 자동 전환됐다. 마지막 correction `+11.71240972 deg`는
진단용으로 유지되지만 target 발행과 MetaQuest 제어에는 사용할 수 없다.
MUX는 계속 `DISABLED`, live output false, ServoL publisher 0개였으며 다음
실험 전 GUI에서 XY +X 보정을 다시 수행해야 한다. 이는 추적 좌표계의 큰
변화를 이전 보정으로 계속 사용하는 것을 막는 의도한 fail-safe 동작이다.
이후 `/vr/recenter`로 최신 raw Quest pose를 새 anchor로 받아 jump filter를
정상 reset했으며, MUX는 `DISABLED`로 재확인했다. 보정 상태는 의도대로
`INVALID`인 채 유지되어 다음 GUI Calibrate 입력을 기다린다.

수정 후 Quest teleop 회귀 테스트는 34개 모두 통과했다.

## 12. 실제 Live 단기 시험과 Live-off 지연 수정

통합 launch를 다음 조건으로 다시 기동했다.

```text
dry_run=false
initial_control_source=DISABLED
start_calibration_gui=true
start_gui=false
start_rviz=false
```

시작 직후 `/vr/live_robot_output_enabled=false`와 MUX `source=DISABLED`를
확인했다. 첫 실제 Live 시도에서는 source 선택 직후 Quest pose가 약
0.873 m 점프해 mapper가 보정을 `INVALID`로 전환했고 MUX가 자동으로
`DISABLED` 복귀했다. 이 시도에서는 Live가 활성화되지 않았고 ServoL 명령도
발행되지 않았다. 이후 독립 호출에서 Doosan `read_data_rt`와
`get_current_posx`가 정상 응답해 컨트롤러 서비스 자체의 지속 장애는 아닌
것을 확인했다.

전체 스택을 깨끗하게 재기동한 뒤 실제 Quest 보정을 다시 수행했다.

```text
state=VALID
distance_m=0.1257218476
observed_angle_deg=-73.28715633
xy_yaw_correction_deg=+73.28715633
```

`/vr/prepare_robot`의 motion plan은 `phases=0`, `waypoints=0`이었으므로
물리 이동 없이 현재 자세를 준비자세로 인정하고 TCP/Quest anchor와
`teleop_ready=true`만 갱신했다. 준비 TCP는 다음과 같았다.

```text
[428.9525452, 0.3993194, 454.5524597,
 0.0867771, 150.2782135, 0.0998209]
```

사용자가 대기 중 컨트롤러를 내려놓아 발생한 큰 좌표 변화와 heartbeat
공백은 사용자 동작으로 확인되어 tracking 결함 판정에서는 제외했다. 실제
조작 자세에서 accepted heartbeat가 약 70~74 Hz로 복구된 뒤 다음 순서로
시험했다.

```text
Recenter
-> DISABLED -> METAQUEST
-> /vr/set_live_robot_output true
-> Live-off 요청
-> METAQUEST -> DISABLED
```

Live enable은 성공했다. 시작 시 streamer의 실제 TCP는 준비 TCP였고 첫
`requested_safe`는 다음 값으로, anchor와 위치 기준 약 5 mm 이내였다.

```text
[433.5562439, 2.3423429, 455.1386565,
 0.0867771, 150.2782135, 0.0998209]
```

따라서 Recenter 이후 절대 Quest 좌표를 직접 명령한 것이 아니라 기존 설계와
동일하게 `robot anchor + 보정된 Quest 상대변위`를 사용했음을 실제 로그로
확인했다. 이후 컨트롤러 상대 이동에 따라 robot target이 변경되고 실제
로봇이 이를 추종했다. 정지 후 TCP는 다음 값으로 두 번 연속 동일했다.

```text
[457.5241089, 18.0200481, 445.8045349,
 0.0843176, 147.0625610, 0.0996187]
```

준비 TCP 대비 변화는 약 `(+28.57, +17.62, -8.75) mm`, `Ry -3.22 deg`다.
Live와 source는 각각 `false`, `DISABLED`로 재확인했고 로봇은 위 TCP에서
정지했다.

이 시험에서 2초 후 Live-off를 요청했지만 streamer 로그 기준 Live enable
`1785409047.529`부터 disable 완료 로그 `1785409052.688`까지 약 5.16초가
걸렸다. 원인은 `ReentrantCallbackGroup` 하나와 executor thread 2개를
공유하던 10 Hz timer 콜백들이 Doosan 상태 서비스 응답을 기다리며 겹쳐
실행되어 Live-off 서비스 콜백이 스케줄링되지 못한 것이었다.

상대좌표/보정/축 매핑은 변경하지 않고 streamer 실행 구조만 수정했다.

- stream timer를 전용 `MutuallyExclusiveCallbackGroup`으로 분리해 중복 tick 방지
- Live/hold 서비스, 상태 subscription, 로봇 service client를 별도 그룹으로 분리
- executor thread를 2개에서 4개로 확장
- Live generation과 lock을 추가해 disable 이후 진행 중 tick의 추가 publish 차단
- hold 서비스도 로봇 TCP 조회 전에 Live를 먼저 false로 전환
- Live enable의 로봇 서비스 조회 후 heartbeat와 `teleop_ready`를 재검사

실제 로봇 토픽과 분리된 `/test/isolated_servol`에서 subscriber 0을 확인하고,
각 로봇 서비스에 의도적으로 0.2초 지연을 넣어 매 tick 상태 조회가 발생하는
격리 부하 검증을 수행했다.

```text
Live false 상태 도달: 0.001340 sec
Live-off 서비스 응답: 0.374465 sec
Live-off 응답 후 추가 command: 없음
```

최종 Quest teleop 회귀 테스트는 35개 모두 통과했고
`quest_a0509_teleop` symlink build도 성공했다.

수정 후 실제 장치 stack을 `DISABLED`, Live false로 재기동하고 GUI에서 다시
보정했다.

```text
state=VALID
distance_m=0.0905259594
xy_yaw_correction_deg=+62.04644867
```

이전 시험 종료 위치에서 `/vr/prepare_robot`을 실행했다. motion plan은
`phases=1`, `waypoints=1`이었고 준비 완료 후 현재 TCP를 robot anchor로 다시
설정했다.

```text
robot anchor =
[430.9750671, -2.8684671, 466.4339294,
 177.1697235, -148.6953888, 176.7336426]
```

지속 연결 클라이언트로 최종 Recenter, fresh safe target 검증, 실제 Live 2초,
Live-off, source disable을 한 번에 수행했다. Live enable 전 safe target 오차는
위치 norm `0.8831 mm`, 회전 `0 deg`로 제한 10 mm/3 deg 안이었다.

```text
Live duration from enable response = 2.000198 sec
Live false state latency          = 0.002296 sec
Live-off service response latency = 0.054817 sec
commands after disable response   = 0
final live state                  = false
final source                      = DISABLED
```

시험 전 TCP는 anchor와 같았고 시험 후 최종 TCP는 다음 값으로 두 번 연속
동일해 로봇 정지를 확인했다.

```text
[430.6550598, -2.2650526, 467.3954468,
 177.1620331, -148.7036133, 176.7318878]
```

즉 수정 전 약 3초였던 Live-off 스케줄링 지연이 실제 장치에서도 약 2.30 ms의
false 상태 전환으로 개선됐고, 요청한 2초 Live 구간과 종료 후 무추가명령을
확인했다. 현재 실제 stack과 보정 GUI는 계속 실행 중이지만 MUX는
`DISABLED`, Live는 false인 안전 상태다.

### 12.1 제한된 20 cm 연속 이동 1차 시도

Quest tracking frame이 기존 승인 pose에서 최대 약 `0.68 m`, `48 deg` 바뀌어
jump 필터가 입력을 거부했고 XY yaw 보정도 설계대로 자동 무효화됐다.
`/vr/recenter`로 최신 raw pose를 새 기준으로 승인한 뒤 GUI에서 보정을 다시
`VALID`로 완료해 heartbeat를 복구했다.

현재 TCP를 robot anchor로 갱신하고 Quest recenter, fresh target preflight를
거친 뒤 Quest 이동 norm `20 cm`에 해당하는 robot target norm `100 mm`와
rotation `10 deg` 감시 한계를 적용해 실제 Live를 실행했다. Live는 약
`9.80 sec` 동작한 뒤 target rotation이 `10.0319306 deg`가 되어 감시
클라이언트가 자동 중단했다. streamer 로그상 위치 target은 사실상 anchor에
머물렀고 `Ry`만 약 `+9.12 deg` 변했다.

```text
abort reason = rotation envelope exceeded: 10.031930633466374 deg
final live   = false
final source = DISABLED
final TCP    =
[430.6557922, -2.2293835, 467.4024658,
 177.1679993, -139.5798950, 176.7329712]
```

따라서 자동 중단과 종료 안전성은 확인했지만 요청한 20 cm 위치 이동 검증은
아직 완료되지 않았다. 재시도에서는 사용자 결정에 따라 시험용 rotation
감시 한계를 `45 deg`로 조정하고 위치 한계 `100 mm`는 유지한다.

회전 한계 조정 후 재시도에서는 preflight 오차가 사실상 0이었지만 Live 시작
약 `1.03 sec` 뒤 selected-command heartbeat age가 `0.665 sec`가 되면서 기존
streamer timeout `0.350 sec`에 의해 자동 중단됐다. robot target 위치 norm은
최대 `0.000019 mm`, 회전은 `0 deg`여서 실제 이동은 없었다. 종료 후 Live
false와 source `DISABLED`를 재확인했다.

Live 없이 15초간 raw/selected heartbeat를 동시 계측한 정상 구간은 다음과
같았다.

```text
raw:      count=940, max_gap=0.074907 sec, p99=0.064093 sec
selected: count=451, max_gap=0.037480 sec, p99=0.034829 sec
```

즉 평상시 주기는 충분하지만 실제 통합 실행 중 단발성 지터가 기존 0.3/0.35초
감시값을 넘었다. 사용자 결정에 따라 MetaQuest 유효 입력 timeout과 streamer
selected-command heartbeat timeout을 모두 `1.0 sec`로 변경했다. LeRobot
timeout은 `0.3 sec`로 유지한다.

변경 후 Quest teleop 회귀 테스트 `36개`와 `quest_a0509_teleop` symlink build가
통과했다. 실제 stack을 다시 시작해 파라미터 서비스에서 MUX
`metaquest_timeout_sec=1.0`, streamer `mux_heartbeat_timeout_sec=1.0`을 각각
확인했다. 재시작한 stack은 Live false, source `DISABLED`이며 보정과
`teleop_ready`는 예상대로 false로 초기화됐다.

GUI 보정을 다시 VALID로 완료하고 `/vr/prepare_robot` 성공 후 제한된 실제
teleop을 재실행했다. 1초 timeout에서는 heartbeat 오차 없이 요청한 10초
구간을 완료했고 Live-off 응답 후 추가 명령은 없었다.

```text
Live duration                    = 10.009132 sec
Live false state latency         = 0.001268 sec
Live-off service response latency= 0.053421 sec
commands after disable response  = 0
max robot target rotation        = 6.525733 deg
final live/source                = false / DISABLED
```

다만 max robot target position norm은 `0.0000163 mm`로 사실상 0이었다. 실제
TCP도 위치는 약 `0.065 mm` 이내에서 유지되고 `Ry`만 약 `+3.37 deg` 변했다.
이 구간에서는 사용자가 처리 대기 중 컨트롤러를 다른 곳에 놓았고 의도적인
평행 이동을 하지 않았다고 사후 확인했다. 따라서 raw position이 고정된 것은
정상적인 시험 입력이며, 이 결과만으로 position tracking 유실을 추정할 수
없다. 이 시험은 1초 heartbeat와 회전 전달만 검증했고 요청한 20 cm 위치 이동
검증은 수행하지 않은 것으로 정정한다.

같은 Quest 연결의 XY yaw 보정 1~3회도 2초간 position delta가 약 `1e-8 m`에
불과해 실패했지만, 당시 사용자가 보정 구간에 맞춰 이동하지 않은 결과일 수
있다. 4회째에는 raw position이 `[-0.2104, 0.3370, -0.3996] m`에서
`[-0.2593, 0.4580, -0.4481] m`로 변해 planar distance `0.06525 m` 보정에
성공했다.

원격 pre-MUX 코드와 현재 코드의 위치 경로도 비교했다. GitHub 원격 `main`의
기준 커밋 `84df505`와 현재 작업본 모두
`latest_vr_pose_m - vr_anchor_m` 후 동일한 `axis_map`, `axis_sign`,
`scale_xyz=0.5`를 적용한다. MUX 적용은 mapper 출력을
`/control/metaquest/target_posx`로 분리한 뒤 승인된 값을 기존
`/vr/target_posx`로 전달하는 라우팅/gate 추가이며 위치 수식은 바뀌지 않았다.
ROS TCP PoseStamped 변환기도 매 패킷의 position/orientation을 직접 변환하고
MUX 전후 동일하다.

이를 실제로 분리 검증하기 위해 Live false, source `DISABLED` 상태에서
`/q2r_right_hand_pose`와 `/control/metaquest/target_posx`를 동시에 수집했다.
첫 수동 이동에서는 raw Quest position이 최대 `0.312439 m` 변해 Quest 6DoF
position tracking이 정상임을 확인했다. 이때 mapper 출력이 0건이었던 이유는
Quest를 껐다 켠 동안 input timeout으로 XY yaw 보정이 INVALID가 됐고, 재접속
pose가 마지막 승인 pose보다 약 `0.22 m`, `96 deg` 달라 `0.15 m`, `45 deg`
jump 필터에 계속 거부됐기 때문이다.

`/vr/recenter`로 현재 raw pose를 승인하자 valid-pose heartbeat가 약
`60~75 Hz`로 즉시 복구됐다. GUI 보정 첫 재시도는 다시 tracking jump로
취소됐지만 현재 자세에서 한 번 더 recenter한 후 재시도해
`xy_yaw_correction_deg=73.834554`와 `VALID` 상태를 얻었다. valid 신호를
자동 감시해 3초 뒤 수행한 두 번째 수동 매핑 계측 결과는 다음과 같다.

```text
calibration valid final / dropped = true / false
raw / mapper samples              = 865 / 360
raw max displacement              = 0.063788 m
raw axis range                    = [0.061780, 0.113328, 0.040553] m
mapper max displacement           = 30.329453 mm
mapper axis range                 = [29.306130, 57.002770, 19.546941] mm
raw result / mapper result        = PASS / PASS
```

raw 최대 상대 이동 `63.788 mm`에 `scale_xyz=0.5`를 적용한 기대 norm은 약
`31.894 mm`이며 실제 mapper 최대 상대 이동은 `30.329 mm`였다. 개별 축 범위는
`axis_map=[1,0,2]`, `axis_sign=[1,-1,1]` 적용 후 유효 XY yaw 보정
`73.834554 deg`로 다시 회전되므로 단순 축 교환값과 일대일 비교하지 않는다.
이 결과로 Quest position → 상대 anchor delta → 축/스케일/회전 보정 → mapper
target 발행 경로가 MUX 앞까지 정상임을 확인했다. MUX는 source `DISABLED`라
이 target을 의도대로 거부했고 실제 로봇은 움직이지 않았다.

현재 PoseStamped에는 `isTracked`나 `trackingState(Position)`가 없으므로 향후
실제 tracking 유실을 ROS에서 구분하려면 Quest 앱 쪽 tracking-state 전송을
추가해야 한다. 다만 이번 현상의 직접 원인은 tracking 유실이나 MUX 회귀가
아니라 Quest 재접속 후 의도된 jump 필터와 calibration gate 차단이었다.

위 수동 검증 후 실제 XYZ 이동을 최종 확인했다. Quest를 내려놓은 동안 다시
tracking frame이 바뀌어 jump 필터가 차단됐으므로 Live false, source
`DISABLED`를 재확인하고 현재 Quest pose로 recenter했다. GUI 보정을 다시
VALID로 만든 다음 `/vr/prepare_robot`으로 준비자세 이동과 robot anchor 갱신을
성공시켰다. 준비자세가 고정된 상태에서 마지막 recenter와 GUI 보정을 수행한
뒤 mapper target 약 `30 Hz`를 확인했다.

실제 Live 감시 클라이언트는 현재 TCP를 다시 anchor로 설정하고 Quest를
recenter한 뒤 MetaQuest source를 선택했다. Live-enable 직전 preflight 위치
오차 norm은 `1.130232 mm`, 회전 오차는 `0 deg`로 허용 범위 안이었다.

```text
live elapsed before envelope stop = 5.137990 sec
max robot target position norm    = 100.206522 mm
max robot target rotation         = 3.890696 deg
anchor TCP                         =
[436.150146, 2.525118, 461.143341,
 0.368681, 149.429108, 0.875725]
final TCP                          =
[528.196289, 20.731625, 433.773163,
 0.377549, 145.841080, 0.869104]
actual TCP XYZ delta               =
[92.046143, 18.206506, -27.370178] mm
actual TCP XYZ norm                = 97.739940 mm
```

사용자가 Quest controller를 약 20 cm 이동하자 0.5 scale의 robot target이
설정한 `100 mm` envelope를 `0.206522 mm` 넘어 감시 클라이언트가 설계대로
즉시 중단했다. 예외 cleanup과 명시적 재호출로 최종 Live false, source
`DISABLED`를 확인했으며 이후 `/vr/commanded_posx`는 2초 동안 0건이었다.
실제 TCP가 anchor에서 `97.739940 mm` 이동했으므로 Quest 상대 위치 → 보정 →
MUX → safety guard → streamer → 실제 Doosan XYZ 경로가 정상임을 확인했다.
10초가 끝나기 전에 위치 경계에 도달한 것은 안전 한계의 의도된 동작이며,
경계 이내 10초 유지 시험이 필요하면 Quest 이동을 약 18 cm 이하로 제한한다.

### 12.2 GUI 클릭 없는 보정과 30초 전체 Live 세션

GUI는 상태 표시용으로 유지하되 버튼을 누르지 않고 ROS 서비스로 보정을
자동 실행했다. 고정 카운트다운 방식은 사용자가 움직이는 시점과 2초 측정
창이 맞지 않아 두 번 `distance too small`로 실패했다. 이후 controller가
기준점에서 10 cm 이동한 순간 `/vr/calibrate_xy_yaw_to_x_plus`를 호출하는
움직임 감지 방식을 사용했다. 최종 보정 결과는 다음과 같다.

```text
state                     = VALID
distance_m                = 0.1018155437
observed_angle_deg        = -34.75422008
xy_yaw_correction_deg     = +34.75422008
mapping_fingerprint       = d4fc94064541b420
```

30초 실전 실행 전 두 가지 preflight 문제를 발견해 안전 상태에서 수정했다.

1. 실행기가 Enter를 기다리는 동안 ROS callback을 spin하지 않아 실제 Quest
   스트림이 약 65~74 Hz로 정상이어도 내부 heartbeat가 stale로 판정됐다.
   시작 입력 후 fresh pose/input callback을 다시 받을 때까지 기다리도록 수정했다.
2. 새 TCP anchor 갱신 뒤 safety guard의 이전 `last_safe`가 회전 축을 틱당
   2 deg로 램핑하고 있었지만 실행기가 0.35초의 첫 샘플을 즉시 검사했다.
   Live false 상태에서 위치 10 mm, 회전 3 deg 안으로 최대 5초간 수렴을
   기다린 뒤 검사하도록 수정했다. 기존 workspace와 ramp 제한은 풀지 않았다.

수정 후 `ament_flake8`, symlink build와 Quest 패키지 회귀 테스트 47개가 모두
통과했다. 실제 30초 세션 결과는 다음과 같다.

```text
requested / actual Live duration = 30.0 / 30.007595 sec
preflight position error norm    = 0.537300 mm
preflight rotation error         = [0, 0, 0] deg
max target position norm         = 155.035702 mm
max target rotation              = 31.993223 deg
actual TCP XYZ delta             = [115.244568, 8.516446, -100.468079] mm
actual TCP XYZ norm              = 153.126337 mm
actual TCP Ry delta              = -23.072113 deg
safe samples                     = 901
commands after disable response  = 0
final live/source                = false / DISABLED
```

따라서 임시 100 mm/45 deg 실험 envelope 없이도 기존 runtime workspace,
ramp, heartbeat, calibration gate를 유지한 30초 arm teleoperation은 통과했다.

그리퍼는 세션 동안 mapper와 driver의 `open/close` 명령이 모두 0건이었다.
팔을 Live false로 유지한 15초 단독 시험에서도 driver가 받은 명령은 초기
`stop`뿐이었다. mapper/driver 프로세스와 `/q2r_right_hand_inputs` 토픽은
살아 있었고 확인 시점의 `button_lower`, `button_upper` 값은 모두 false였다.
따라서 기존 직접 Tool DO OPEN/CLOSE 물리 통과 결과는 유지하지만,
MetaQuest A/B → mapper → MUX → 실제 gripper 경로는 통과로 기록하지 않는다.
사용자가 A/B를 실제로 누른 구간의 raw 입력을 동시 기록해 버튼 필드 전달부터
재검증해야 한다.

### 12.3 A/B 그리퍼 포함 재실전

같은 30초 supervisor로 arm과 gripper 동시 실전을 다시 시작했다. 첫 10초에는
그리퍼 명령이 없었지만 사용자가 A와 B를 각각 명확히 누른 뒤 실제 driver가
다음 순서를 모두 수락했다.

```text
close -> stop -> open -> stop
```

따라서 `button_lower/button_upper` 원시 입력, mapper와 MUX까지의 전달은
확인했다. 그러나 사용자의 실제 관찰에서 그리퍼는 움직이지 않았으므로
물리 open/close 종단 경로는 통과가 아니다. driver 로그를 다시 확인한 결과
close는 DO2 ON 호출 뒤 약 `0.163 sec`, open은 DO1 ON 호출 뒤 약 `0.155 sec`
만에 버튼 release의 `stop`으로 선점됐다. 두 명령 모두
`preempted by stop request`로 중단되어 설정된 `0.20 sec` pulse를 완료하지
못했다. `accepted_command`는 접수 시점 진단이므로 물리 완료 근거로 사용할
수 없다는 점도 확인했다.

전날 직접 Tool DO 시험에서는 동일 하드웨어와 `DO1 OPEN`, `DO2 CLOSE`,
`0.20 sec` pulse가 실제로 정상 동작했다. 따라서 하드웨어, 배선, DO 번호가
아니라 MetaQuest 버튼 release-stop과 pulse driver 사이의 타이밍 결함이다.

약 20초 시점에는 mapper가 Quest tracking jump를 거부하면서 보정이 다음과
같이 자동 무효화됐다.

```text
reason       = Quest tracking jump rejected; repeat XY +X calibration.
state/valid  = INVALID / false
last yaw     = +34.75422008 deg
```

supervisor와 MUX는 즉시 Live false, source DISABLED, gripper stop으로 정리했고
driver의 `last_command_ok=true`를 확인했다. 이번 시도는 팔과 그리퍼 동시
물리 경로 및 jump fail-safe는 통과했지만, 추적 jump로 남은 약 10초가
중단됐으므로 30초 동시 완주로 기록하지 않는다. 재완주 전에는 현재 tracking
frame으로 recenter와 XY +X 보정을 다시 수행해야 한다.

### 12.4 Tool DO readback 동기화 및 A/B 물리 재검증

release `stop` 선점 방지 뒤에도 서비스 호출 완료만으로는 그리퍼가 움직이지
않았다. MUX와 driver를 우회한 직접 시험에서 SET 응답 직후 readback은 이전
값이었고, OFF 요청 직후에는 뒤늦게 ON 값이 관찰됐다. 즉 Doosan SET 서비스
성공 응답과 실제 플랜지 출력 반영 사이에 지연이 있었다.

readback을 기준으로 다시 측정한 결과는 다음과 같다.

```text
DO1 OPEN  : ON 반영 0.076 s, OFF 반영 0.099 s
DO2 CLOSE : ON 반영 0.046 s, OFF 반영 0.093 s
```

실제 ON readback부터 `0.20 sec`를 유지한 직접 OPEN/CLOSE는 사용자가 두 동작
모두 물리적으로 확인했다. 이에 driver에
`get_tool_digital_output`, `readback_timeout_sec=0.5`,
`readback_poll_sec=0.01`을 추가했다. 각 SET 뒤 목표 readback을 확인하고,
ON 확인 뒤에만 pulse timer를 시작하며, 최종 OFF readback 뒤에만
`completed_command`를 발행한다. readback timeout은 기존 실패 경로를 통해
failsafe stop으로 이어진다.

수정 driver 직접 시험도 다음과 같이 통과했다.

```text
OPEN  : ON readback 0.094 s, 0.20 s 유지, OFF readback 0.099 s
CLOSE : ON readback 0.074 s, 0.20 s 유지, OFF readback 0.103 s
```

사용자가 실제 OPEN/CLOSE를 확인했으며, 이어서 팔 Live를 계속 false로 둔
MetaQuest 종단 시험에서 다음을 확인했다.

```text
raw input              : B/button_upper 1회, A/button_lower 1회
mapper                  : stop -> open -> stop -> close
driver accepted         : open -> stop -> close
driver readback complete: stop -> open -> stop -> close
physical                : B OPEN, A CLOSE 모두 사용자 확인
live                    : 전체 구간 false
cleanup                 : source DISABLED, DO1=0, DO2=0
```

따라서 MetaQuest A/B → mapper → MUX → readback 동기 driver → 실제 JRT
그리퍼 경로는 통과다. 변경 후 `jrt_gripper_io`와
`quest_a0509_teleop` 회귀 테스트 `77개` 및 두 패키지 build도 통과했다.

### 12.5 재보정 후 arm+gripper 30초 동시 재시도

현재 Quest tracking frame에서 XY +X 보정을 다시 수행했다. 보정 결과는
다음과 같았고 정상 MUX의 calibration gate가 `VALID`를 수신한 상태에서만
full supervisor를 시작했다.

```text
state / valid                 = VALID / true
XY yaw correction            = +19.822188 deg
mapped planar displacement   = 74.427928 mm
Quest pose / input rate      = 약 72 / 70 Hz
initial robot state          = STANDBY
initial Tool DO1 / DO2       = 0 / 0
```

30초 Live 구간에서 arm은 실제 이동했고 B OPEN은 mapper와 readback 동기
driver의 완료까지 확인됐다.

```text
requested / actual duration  = 30.0 / 30.004160 sec
max target position norm     = 92.626422 mm
max target rotation          = 21.085178 deg
actual TCP XYZ delta         = [44.215027, -12.995554, -26.698059] mm
actual TCP XYZ norm          = 53.260111 mm
actual TCP Ry delta          = +17.983551 deg
gripper mapper               = open -> stop
gripper driver accepted      = open -> stop
gripper readback complete    = open -> stop
commands after disable       = 0
```

A CLOSE는 Live 구간 안에 mapper까지 입력되지 않아 open/close 동시 검증
판정은 `false`였다. 이번 실행은 arm과 B OPEN 종단 경로의 부분 통과이며,
arm+gripper 30초 동시 완주로 기록하지 않는다.

supervisor 종료 후 `live=false`, `source=DISABLED`,
`gripper_busy=false`, `gripper_last_command_ok=true`였고, 별도 재확인에서도
로봇은 `STANDBY`, Tool DO1/DO2는 모두 `0`이었다.

### 12.6 기존 실전 ServoL 운용값 복원

사용자가 이전 실전에서 사용한 제어 조건을 기준으로 streamer 선형 ramp를
`7.5 mm/tick`에서 `20.0 mm/tick`으로 복원했다. 나머지 주기와 회전 조건은
기존 설정과 같아 유지했다.

```text
제어 주기       = 10 Hz
주기 시간       = 100 ms
ServoL time     = 0.5 s
선형 ramp       = 20.0 mm/tick = 200 mm/s
회전 ramp       = 3.0 deg/tick = 30 deg/s
```

MUX, calibration gate, workspace clamp, Live gate, heartbeat watchdog 및
그리퍼 안전 로직은 변경하지 않았다. 이 설정은 다음 실제 세션에서 재시작 후
적용되며, 변경된 선형 ramp의 물리 추종성은 별도 확인 대상으로 남긴다.

### 12.7 20 mm/tick 적용 후 재보정 및 30초 arm 실전

Quest tracking frame jump를 현재 frame으로 recenter한 뒤 XY +X 보정을 다시
수행했다.

```text
state / valid                 = VALID / true
XY yaw correction            = -23.051479 deg
mapped planar displacement   = 94.383232 mm
post-correction Y error      = 약 0 mm
```

streamer 런타임 파라미터가 `10 Hz`, `ServoL time 0.5 s`,
`20.0 mm/tick`, `3.0 deg/tick`인 것을 서비스로 확인한 뒤 30초 full
supervisor를 실행했다. arm 경로는 중단이나 tracking jump 없이 완주했다.

```text
requested / actual duration  = 30.0 / 30.005081 sec
max target position norm     = 241.292041 mm
max target rotation          = 22.686159 deg
actual TCP XYZ delta         = [77.101624, 17.416012, -8.463562] mm
actual TCP XYZ norm          = 79.495973 mm
actual TCP Ry delta          = -19.044281 deg
safe samples                 = 935
commands after disable       = 0
```

세션 중 A/B 입력이 없어 mapper와 driver의 gripper command는 모두 0건이었다.
따라서 이번 실행은 새 선형 ramp의 arm 30초 완주로 기록하고, arm+gripper
동시 완주로는 기록하지 않는다. 종료 후 supervisor가
`live=false`, `source=DISABLED`로 정리했으며, 별도 확인에서도 로봇은
`STANDBY`, Tool DO1/DO2는 모두 `0`이었다.

### 12.8 30 Hz 저지연 프로필과 GetRobotState 비동기 분리

10 Hz / ServoL time 0.5초 조건에서 로봇이 컨트롤러를 늦게 따라오는 문제를
줄이기 위해 다음 1차 저지연 프로필을 적용했다. 초당 목표 변화율은 기존과
같게 유지했다.

```text
ServoL publish rate              = 30 Hz
ServoL time                      = 0.1 s
linear ramp                      = 6.67 mm/tick (약 200 mm/s)
rotation ramp                    = 1.0 deg/tick (30 deg/s)
```

이 조건의 실제 시험에서 로봇이 이동하기는 했으나 동작이 주기적으로 끊겼다.
토픽과 프로세스 로그를 분리 측정한 결과는 다음과 같다.

```text
/vr/safe_posx                    = 평균 30.49 Hz, p99 약 34 ms, 최대 약 37.5 ms
Quest pose/input                 = 평균 약 72.5 Hz이나 burst 형태
Quest interval                  = p95 약 52 ms, 최대 약 67 ms
별도 관찰된 Quest gap            = 약 0.36~0.376 s 1회
streamer 1초 상태 로그 간격       = 평균 1.117 s, 최대 1.817 s
idle GetRobotState response      = 중앙값 약 0.996 ms, 최대 약 7.997 ms
```

Quest/TCP 입력 burst도 별도 지연 요인이지만, streamer의 30 Hz `_tick()` 안에서
`GetRobotState`를 동기 호출하고 20 ms 간격으로 완료를 기다리는 구조는 ServoL
발행 callback 자체를 멈출 수 있었다. 서비스의 idle 응답이 빠르더라도 controller
callback 경합이나 일시 지연이 그대로 ServoL 주기 지터로 전파되는 구조였다.

이에 `GetRobotState`를 다음과 같이 분리했다.

```text
state poll timer                 = 5 Hz, 별도 callback group
동시 요청                        = 최대 1개, 완료 전 다음 요청 생략
응답 처리                        = future callback에서 상태와 수신 시각을 cache
ServoL 30 Hz tick               = cache 읽기만 수행
state stale timeout             = 1.0 s
안전 상태                        = robot_state 1 또는 2
```

첫 상태가 없거나 cache가 1초 이상 갱신되지 않거나 상태가 1/2가 아니면 Live
enable을 거부하며, Live 중에는 기존 hold/disable 안전 경로로 전환한다. 따라서
상태 감시는 유지하면서 ServoL tick 내부의 서비스 호출, 완료 대기, sleep은 모두
제거됐다.

변경 후 `quest_a0509_teleop` 테스트 `57개`와 symlink build가 통과했다. 실제
스트리머 재기동에서도 `robot_state_poll_rate_hz=5.0`과 비동기 cache
`state=1(STANDBY)` 수신을 확인했다. 최종 상태는 streamer 1개,
`live=false`, MUX `DISABLED`이다. 이 구조에서의 arm+gripper 30초 물리 추종성
재시험은 아직 수행하지 않았다.

### 12.9 비동기 robot-state 조건의 30초 arm+gripper 완주

사용자가 GUI에서 XY +X 보정을 완료한 뒤 ROS 유지 상태에서
`valid=true`, 보정 이동 거리 `137.587 mm`, `teleop_ready=true`를 확인했다.
스트리머 재기동 뒤에는 `teleop_ready` publisher가 transient-local인데 streamer
subscriber가 volatile이어서 유지된 `true`를 받지 못하는 문제를 발견했다.
streamer 구독도 transient-local로 맞추고 테스트 `57개` 및 build를 다시
통과시켰다. 재기동 후에는 별도 준비 자세 이동 없이 다음 두 상태를 모두
정상 수신했다.

```text
teleop_ready cache               = true
asynchronous robot-state cache  = 1 (STANDBY)
```

이후 기존 fail-safe supervisor로 30초 arm+gripper 세션을 실행했다.

```text
requested / actual duration      = 30.0 / 30.003557 sec
safe samples                     = 900 (30 Hz 전 구간)
max target position norm         = 156.837220 mm
max target rotation              = 14.438676 deg
actual TCP XYZ delta             = [69.572937, -5.053093, -1.812347] mm
actual TCP XYZ norm              = 69.779739 mm
actual TCP Ry delta              = +10.219818 deg
gripper mapper                   = open -> stop -> close -> stop
gripper driver accepted          = open -> close
gripper driver completed         = open -> close
gripper validation               = true
commands after disable           = 0
Live=false state latency         = 0.002143 sec
Live disable response latency    = 0.055944 sec
```

Live 구간의 1초 streamer 상태 로그 간격은 최대 약 `1.064초`로, 동기
GetRobotState 구조에서 관측한 최대 `1.817초`보다 안정됐다. robot-state cache는
동작 전환 시 `2`를 관찰한 뒤 `1`로 돌아왔고 stale/unsafe 중단은 없었다.

종료 후 `live=false`, MUX `DISABLED`, robot `STANDBY`, gripper driver
`busy=false`, `last_command_ok=true`, Tool DO1/DO2=`0/0`을 확인했다. 소프트웨어
및 Tool DO 경로의 open/close 완주는 통과했으며, 사용자의 그리퍼 실제 기구 동작
및 팔의 체감 부드러움 평가는 별도로 확인한다.

### 12.10 최신 pose 우선 처리와 Safety 타이머 단계 제거

비동기 robot-state 적용 후 사용자는 이전보다 훨씬 부드러워졌지만 남은 버벅임과
지연을 체감했다. 추가 측정에서는 Quest pose가 평균 약 71 Hz이면서도 짧은 burst와
긴 gap을 반복했고, pose subscriber 관측 간격은 p95 약 `54.4 ms`, p99 약
`64.3 ms`, 최대 약 `345.8 ms`였다. 기존 파이프라인에는 Mapper, Safety Guard,
Streamer의 독립 30 Hz 타이머도 있어 각 단계의 위상에 따라 추가 대기가 생겼다.

밀린 과거 pose 재생과 Safety 단계의 독립 타이머 대기를 줄이기 위해 다음을
적용했다.

```text
Mapper Quest pose subscription  = KEEP_LAST depth 1
reliability / durability        = BEST_EFFORT / VOLATILE
Safety target subscription      = depth 1
Safety processing               = target callback에서 즉시 clamp/ramp/publish
Safety control timer            = 제거
Safety publish_rate_hz          = 제거
```

Safety의 workspace clamp, anchor-relative orientation clamp, XYZ/RPY ramp 값은
변경하지 않았다. 정상 MetaQuest 경로에서는 Mapper가 계속 30 Hz로 최신 목표를
발행하므로 명목상 초당 ramp 제한도 유지된다. 콜백이 밀려 중간 목표가 폐기되는
경우에는 ramp step 횟수가 줄어들 뿐 더 공격적인 명령이 생성되지는 않는다. 입력이
끊겼을 때 Safety가 마지막 목표를 독립적으로 반복 발행하지 않으며, 기존 MUX와
Streamer heartbeat watchdog이 중단 경로를 계속 담당한다.

구조 회귀 테스트 두 개를 추가했고 전체 `quest_a0509_teleop` 테스트는
`59 passed`로 통과했다. `colcon build --symlink-install --packages-select
quest_a0509_teleop`도 성공했다. 실제 로봇과 분리한 ROS domain에서 두 노드를
각각 기동해 Safety의 `processing_mode=immediate_on_target`과 Mapper의
`pose_qos=KEEP_LAST(depth=1, BEST_EFFORT, VOLATILE)` 시작 로그도 확인했다. 실행
중인 기존 bringup 노드는 재기동 전까지 이전 Python 코드를 사용하므로, 이 변경의
토픽 주기와 물리적 체감 효과는 bringup 재기동 후 별도 실전 시험에서 확인한다.

### 12.11 Safety 중복 ramp 제거와 30 Hz 비동기 clamp 복원

12.10의 즉시 처리 Safety와 `KEEP_LAST(depth=1)` Mapper를 실제 30초 세션에
적용한 결과, 사용자는 12.9보다 동작이 더 버벅인다고 평가했다. 세션은 약
`20.8초`에 Streamer fail-safe로 중단됐으며 직접 중단 원인은 다음 로그였다.

```text
Live ServoL RT stopped: asynchronous robot-state sample is stale:
age=1.742s timeout=1.000s.
```

중단 전 그리퍼 close/stop/open/stop은 모두 실제 동작했다. 종료 정리도
`live=false`, MUX `DISABLED`, robot `STANDBY`, gripper stop으로 완료됐다.
Live 중 Streamer의 1초 상태 로그 간격은 최대 약 `2.17초`까지 늘어났고,
프로세스 CPU 사용량은 대략 `ros2_control 213%`, `Streamer 63.6%`,
`MUX 8.9%`, `endpoint 11.1%`, `Mapper 7.9%`, `Safety 0.5%`였다. 따라서
Safety 직접 계산이 다중 초 정지의 주원인이라고 단정할 근거는 없으며,
실제 자동 중단은 비동기 robot-state cache의 stale 조건에서 발생했다. 다만
Safety ramp를 target 도착 콜백에서 실행하면 목표 변화율이 고정 제어주기가 아닌
도착 지터에 종속되므로 제어 구조상 제거가 필요했다.

최종 구조는 다음과 같이 정리했다.

```text
Mapper Quest pose queue           = KEEP_LAST depth 1, BEST_EFFORT, VOLATILE
Safety 입력                       = 최신 target 1개만 저장
Safety 처리/출력                  = 독립 30 Hz timer에서 clamp 후 publish
Safety가 유지하는 제한            = finite, workspace, anchor-relative orientation
Safety XYZ/RPY ramp               = 제거
Streamer 처리                     = 독립 30 Hz ServoL + 유일한 XYZ/RPY ramp
Streamer ramp                     = 6.67 mm/tick, 1.0 deg/tick
```

즉 Safety는 이전과 같은 비동기 30 Hz 안전 envelope 역할로 복원하되, 중복
ramp는 두지 않는다. 실제 변화율 제한의 단일 소유자는 Streamer다. Mapper와
Safety 재시작 시 준비자세 anchor를 놓치지 않도록 `robot_prep_node`의
`/vr/robot_anchor_posx` publisher도 `RELIABLE + TRANSIENT_LOCAL + depth 1`로
맞췄다. 구독 측도 같은 QoS를 사용한다.

변경 후 `quest_a0509_teleop` 전체 테스트는 `60 passed`, Python 구문 검사,
`git diff --check`, symlink build가 모두 통과했다. 실제 ROS 그래프에서는 새
Safety의 시작 로그로 `publish_rate_hz=30.0`,
`processing_mode=fixed_rate_clamp_only`, `ramp_owner=streamer`를 확인했다.
준비자세 서비스 노드도 새 빌드로 재기동했으며, 현재 안전 상태는
`teleop_ready=false`, `live=false`, MUX `DISABLED`다. 다음 물리 시험 전에
`/vr/prepare_robot`을 호출해 현재 TCP anchor를 retained 발행하고 GUI 보정을
다시 완료해야 한다. Streamer의 robot-state stale/실행 지연 원인은 별도 추적
항목으로 남는다.

### 12.12 Robot-state RT 토픽 전환과 30초 arm+gripper 완주

12.11 직후 30초 재시험은 Live 시작 약 3초 뒤 다음 조건으로 자동 중단됐다.

```text
Live ServoL RT stopped: asynchronous robot-state sample is stale:
age=1.200s timeout=1.000s.
```

실행 중인 Safety CPU는 약 `2.2%`였고 직접 중단 원인은 다시 5 Hz
`GetRobotState` service future의 갱신 공백이었다. 안전 timeout을 늘리는 대신,
Doosan controller가 이미 제공하는 RT 상태 토픽을 확인했다.

```text
topic                         = /rt_topic/robot_state
type                          = std_msgs/msg/Float64MultiArray
data                          = [1.0] (STANDBY)
controller configuration      = rt_timer_ms: 10
observed rate                 = 약 99.84~100.04 Hz
latest preflight measurement  = 99.996 Hz
observed QoS                  = RELIABLE + TRANSIENT_LOCAL
```

Streamer에서 반복 `GetRobotState` client, 5 Hz timer, in-flight future를
제거하고 위 토픽을 `KEEP_LAST(depth=1)`, `RELIABLE`, `TRANSIENT_LOCAL`로
구독하도록 변경했다. callback은 첫 값을 유한한 정수 상태 코드로 검증한 뒤
watchdog의 상태와 monotonic 수신 시각만 갱신한다. 기존 허용 상태 `[1, 2]`와
stale timeout `1.0초`는 그대로 유지했다. 따라서 ServoL/Safety 30 Hz 제어주기와
안전 기준은 느슨해지지 않았다. `robot_prep_node`의 준비 이동 전 단발
`GetRobotState` 확인은 별도 목적이므로 유지했다.

실제 첫 30초 시험은 stale 중단 없이 다음과 같이 완주했다.

```text
duration                      = 30.004799 sec
safe samples                  = 904
commands after disable        = 0
state-stale stop              = 없음
target/TCP/gripper activity   = 모두 0
```

이 첫 시험은 상태 감시 경로에는 통과했지만 동작 시험은 아니었다. Mapper 로그의
`latest_raw_vr_pose_m`과 quaternion이 전 구간 동일했고, A/B raw input도 없었다.
즉 Live는 켜졌지만 anchor와 같은 목표만 전송돼 로봇이 의도치 않게 정지 유지했다.

XY 보정을 reset한 뒤 GUI에서 다시 보정했다. Live 시작 전 4초 pose 확인에서는
`176개` 샘플과 축별 이동폭 `[0.127176, 0.137182, 0.046555] m`를 확인해 Quest
입력이 실제로 변하는 상태임을 검증했다. 이어서 30초 arm+gripper 세션을 다시
실행했고 정상 완주했다.

```text
requested / actual duration   = 30.0 / 30.000959 sec
safe samples                  = 907
max target position norm      = 98.948625 mm
max target rotation           = 31.600264 deg
actual TCP XYZ delta          = [25.039185, -87.067200, -15.678955] mm
actual TCP XYZ norm           = 91.942850 mm
actual TCP rotation delta     = [-0.002849, 9.708389, 0.004308] deg
gripper mapper                = close -> stop -> open -> stop -> close -> stop
gripper accepted/completed    = close -> open -> close
gripper validation            = true
commands after disable        = 0
Live=false state latency      = 0.257151 sec
Live disable response latency = 0.572649 sec
```

상태 토픽의 정상 `1(STANDBY) ↔ 2(MOVING)` 전환은 ServoL 명령 중 빈번하므로
매 전환을 INFO로 남기지 않게 했다. cache는 모든 샘플에서 계속 갱신하되 최초
상태와 safe/unsafe 경계 전환만 로그로 남긴다. 최종 Streamer 로그에는 30초 동안
state stale/unsafe 중단이 없었고, 종료 후 `live=false`, MUX `DISABLED`, robot
`STANDBY(1)`, gripper idle을 확인했다. 전체 테스트 `64 passed`, Python 구문
검사, `git diff --check`, symlink build도 통과했다. 사용자의 체감 부드러움과
실제 그리퍼 기구 동작 평가는 별도 확인한다.

### 12.13 Streamer executor starvation 계측, 30 Hz 복구, gripper Live gate 보완

12.12 이후 사용자가 실제 이동은 되지만 심하게 끊기고 지연된다고 평가해,
통신 경로를 추측하는 대신 동일 15초 세션에서 rosbag, Streamer tick ring buffer,
프로세스/스레드 `/proc` 샘플을 함께 기록했다. Streamer trace는 Live 중 메모리에만
저장하고 Live 해제 후 별도 writer thread가 CSV로 쓰도록 했다. 프로세스 모니터는
100 ms 간격으로 CPU tick, processor, nice, scheduler policy, RT priority, context
switch를 기록한다.

로봇 전원을 다시 켠 첫 Doosan 연결은 DRCF와 STANDBY까지 성공한 뒤 DRFL이 임의
로컬 RT client port로 `420`을 선택해 실패했다. 시스템의
`net.ipv4.ip_unprivileged_port_start=1024` 조건과 충돌했고 드라이버는
`I/O error: Permission denied`, `free(): invalid pointer` 후 반쯤 살아 있었다.
이 실행만 종료하고 재연결하자 임의 port `33580`이 선택돼 DRCF, RT stream,
두 controller가 모두 정상 활성화됐다. 이전 성공 로그의 임의 port도
`13748`, `22271`, `29794` 등이어서 이번 `420`은 드문 DRFL port 선택 예외였고
텔레옵 지연의 지속 원인은 아니었다.

수정 전 정지 유지 15초 시험에서는 Mapper, Safety, RT feedback이 각각 명목
30/30/100 Hz를 유지했지만 실제 `servol_rt_stream`과 `/vr/commanded_posx`는 bag
전체에서 각각 39개뿐이었다. Live tick trace는 25행, 10.823초 span으로 다음과
같았다.

```text
effective Streamer rate         = 2.217 Hz
tick interval p50 / p95 / max   = 211.638 / 984.065 / 4720.824 ms
tick callback p50 / max         = 0.402 / 520.815 ms
interval > 100 ms               = 18 / 24
Streamer process CPU            = 102.1%
Streamer main thread CPU         = 98.1%
ros2_control aggregate CPU       = 217.0%
Live disable response latency    = 7.531987 sec
Live=false observation latency   = 3.324180 sec
```

Safety clamp 계산은 30 Hz로 들어왔고 Streamer의 checks/ramp 자체도 보통 수십
microseconds에 불과했다. 반면 `MultiThreadedExecutor.spin()` dispatcher main
thread가 한 core를 계속 사용하며 Python GIL을 거의 놓지 않았고 timer/service
worker callback이 굶었다. 이는 safety ramp, 목표 변화율 또는 ServoL time보다
앞단에서 실제 명령 발행률을 2 Hz대로 무너뜨린 직접 원인이었다.

Streamer executor를 4 worker로 유지하되 `spin_once(timeout_sec=0.1)` 뒤 1 ms
bounded yield를 주도록 변경했다. callback group 분리는 그대로 유지했다.
진단 모니터에서 process aggregate와 main thread가 같은 숫자 PID/TID를 사용해
CPU baseline이 충돌하던 문제도 `(kind, owner_pid, tid)` key로 수정했다. Live OFF
상태에서 Streamer CPU는 약 `102% -> 14.4%`로 감소했고, 지속 client로 측정한
Live OFF service 응답은 4회 `54~56 ms`, 1회 `244 ms`였다.

수정 후 15초 저속 X/Y/Z 이동 시험 결과는 다음과 같다.

```text
timing rows / rate              = 450 / 29.995 Hz
tick interval p50 / p95 / max   = 33.326 / 34.899 / 36.140 ms
interval > 40 ms                = 0
tick callback p50 / p99 / max   = 0.244 / 2.013 / 2.693 ms
safe target age p50 / max       = 20.085 / 22.436 ms
Streamer CPU mean / p95         = 19.4 / 30.0%
Live disable response latency   = 56.968 ms
Live=false observation latency  = 1.901 ms
actual TCP XYZ norm             = 50.655 mm
commands after disable          = 0
```

즉 기존 39개였던 bag의 ServoL 명령은 hold를 포함해 463개로 증가했고, Live 본
구간은 정확히 `15 sec * 30 Hz = 450`개였다. 30 Hz timer jitter도 한 tick을
놓치지 않는 범위로 회복됐다.

최종 arm+gripper 결합 15초 시험도 정상 완주했다.

```text
timing rows / rate              = 450 / 29.997 Hz
tick interval p50 / p95 / max   = 33.292 / 34.399 / 40.868 ms
interval > 40 / 50 ms           = 1 / 0
tick callback p50 / p95 / max   = 0.310 / 1.297 / 6.397 ms
safe target age p50 / max       = 19.792 / 27.247 ms
Streamer CPU mean / p95         = 18.4 / 30.0%
gripper driver sequence         = close -> stop -> open -> stop -> close -> stop -> open
gripper validation              = true
actual TCP XYZ delta            = [96.526855, 36.411261, -116.367310] mm
actual TCP XYZ norm             = 155.513872 mm
actual TCP rotation delta       = [-0.084630, 3.015869, -0.081798] deg
Live disable response latency   = 65.455 ms
Live=false observation latency  = 2.444 ms
commands after disable          = 0
```

gripper event는 Mapper `/control/metaquest/gripper_cmd`에서 MUX
`/jrt_gripper/cmd`까지 약 0~3 ms, driver accepted까지 약 3~6 ms에 전달됐다.
그리퍼 동작 중에도 arm stream은 29.997 Hz를 유지했다.

추가로 Live 종료 0.56초 뒤 사용자가 누른 `close`가 gripper에 전달된 것을 bag에서
발견했다. MUX 우회 publisher는 없었다. 세션이 arm Live를 먼저 끈 뒤 0.75초간
추가 arm 명령을 검사하고 나서 source를 DISABLED로 전환하는데, 기존 MUX의
gripper 허용 조건에 `live_enabled`가 없어 이 quiet window에서만 버튼이 계속
허용된 것이 원인이었다. MUX core를 다음과 같이 보완했다.

```text
Live true -> false              = 즉시 gripper stop 1회
Live false                      = MetaQuest/LeRobot open/close 거부
gripper 허용                    = selected source + teleop ready + calibration
                                  + fresh arm target + Live true
```

전체 `quest_a0509_teleop` 테스트는 새 회귀 테스트를 포함해 `66 passed`, Python
구문 검사와 symlink build가 통과했다. 수정된 MUX도 `initial source=DISABLED`,
`live=false`로 재기동했다. 현재 로봇 출력은 Live OFF, source DISABLED이며 다음
실험 전에는 Quest input timeout으로 무효화된 GUI 보정을 다시 해야 한다.

### 12.14 Quest 수신 burst 완화를 위한 30 Hz pose 재표본화

12.13의 최종 arm+gripper bag에서 `/q2r_right_hand_pose`는 평균만 보면
`72.066 Hz`였지만 receive timestamp 간격은 균일하지 않았다.

```text
receive interval p50 / p90      = 3.017 / 50.490 ms
receive interval p95 / p99      = 53.919 / 62.460 ms
receive interval max            = 79.682 ms
interval < 5 ms                 = 1682 / 2988
interval > 30 ms                = 601 / 2988
```

정지 bag도 p50 `2.723 ms`, p90 `50.398 ms`, max `75.118 ms`로 같은 형태였다.
즉 평균 72 Hz와 달리 Jetson에는 여러 pose가 약 3 ms 간격으로 몰린 뒤
50~80 ms 공백이 생겼다. 기존 Mapper는 입력 callback마다 `alpha=0.4` 필터를
한 번씩 적용했기 때문에 한 burst에서 필터가 빠르게 여러 번 전진한 뒤 공백 동안
정지하는 시간 의존 동작이었다. 현재 독립 Quest 앱의 `PoseStamped` header는
Jetson endpoint에서 수신 시각으로 채워지며 원본 생성 시각과 tracking-state는
제공되지 않으므로, 이번 변경은 Jetson monotonic receive time만 사용한다.

별도 ROS timer/node를 추가하지 않고 Mapper의 기존 30 Hz timer에 다음 receive-time
재표본화를 통합했다.

```text
callback                         = validate + bounded buffer push only
control query                    = now - 50 ms
position                         = linear interpolation
orientation                      = quaternion SLERP
full bounded prediction          = 20 ms
prediction velocity decay        = 20~80 ms
interpolation/prediction hold     = 80 ms 이후
tracking stale / calibration off = 300 ms
existing input timeout           = 500 ms
buffer                           = max 128 samples / 1 sec
```

수신 callback에서 하던 position/quaternion low-pass도 30 Hz tick으로 이동했다.
기존 72 Hz 기준 `alpha=0.4`의 시간상수를 유지하도록 tick alpha를
`0.70653048`로 환산한다. recenter, robot-anchor 갱신, calibration 시작/완료/reset,
teleop-ready 전환, Live enable, 300 ms stale에서 buffer를 비워 새 anchor에 과거
pose가 재생되지 않게 했다. heartbeat freshness는 보간 출력이 아니라 계속 실제
accepted receive callback만 갱신한다.

같은 최종 이동 bag의 ServoL 기록 구간 약 15.98초를 오프라인으로 기존 callback
필터와 새 재표본화기에 통과시킨 결과는 다음과 같다. 위치 수치는 실제 설정의
`scale_xyz=0.5`를 적용한 로봇 mm 단위다.

```text
                                      callback filter   fixed-tick resampler
step p95 / max [mm]                   3.288 / 5.876     2.588 / 4.043
speed p95 [mm/s]                      98.642            77.645
acceleration p50 [mm/s^2]             450.542           236.255
acceleration p95 [mm/s^2]             3052.257          1356.339
acceleration max [mm/s^2]             5288.629          2591.533
jerk p95 [mm/s^3]                     177396.190         74542.664
exact repeated output ticks           126               0
```

재표본화 mode는 `interpolate=467`, `extrapolate=12`, `decay=1`이었고 hold는
없었다. 50 ms 의도 지연으로 최신 raw pose와의 위치 차이는 p50 `0.665 mm`,
p95 `4.084 mm`, max `5.999 mm`였다. 이는 시간 지연과 움직임 중 위치차를 함께
나타내는 수치이며 실제 로봇 추종 지연은 다음 물리 시험에서 별도 확인해야 한다.

Jetson에서 72,000 push와 36,000 SLERP 포함 sample을 실행한 microbenchmark는
sample 평균 `10.34 us`, 입력 한 회당 결합 평균 `9.19 us`였다. 실제 72/30 Hz
부하는 한 core의 약 0.1% 미만 수준이다. 새 순수 코어 테스트 7개와 통합 계약
테스트를 포함해 `74 passed`, `colcon test-result` 전체는 `102 tests, 0 errors,
0 failures, 1 skipped`였고 symlink build와 Mapper 단독 기동도 통과했다. 실제
로봇 출력은 이 단계에서 실행하지 않았다.

빌드 후 기존 02:38 시작 Mapper만 SIGINT로 종료하고 같은 topic override로 새
Mapper를 재기동했다. runtime parameter는 `enable_pose_resampling=true`,
`pose_interpolation_delay_sec=0.05`, `pose_tracking_invalid_after_sec=0.3`으로
확인했다. 시작 직후 실제 Quest 입력에서 한 번 `age_sec=0.3126` stale이 발생해
target hold와 buffer reset이 실행됐고 다음 입력에서 정상 복구됐다. 당시와 최종
상태는 calibration `false`, Live `false`, MUX source `DISABLED`였으므로 실제
로봇 명령은 발행되지 않았다. 다음 물리 시험 전 GUI 보정이 필요하다.

## 13. 아직 수행하지 않은 항목

- 새 50 ms Quest pose 재표본화 적용 후 실제 로봇 체감 부드러움·추종 지연 확인
- 재표본화 이후 남는 가속도 불연속에 대한 Streamer acceleration/jerk limiter 검토
- RT robot-state 토픽 기반 Streamer의 장시간 연속 운전 및 stale 감시 검증
- 새 preparation 취소 경로의 실제 이동 중 `/vr/stop_robot` 물리 검증
- 그리퍼 서비스 지연/무응답을 주입한 실제 stop 선점 및 timeout 검증
- ZED2 depth 처리
- 두 카메라 장시간 USB 대역폭 스트레스 테스트
- ROS-TCP-Endpoint 종료 시 exit `-6` 원인 수정

## 14. 관련 파일

- `README.jetson.md`
- `config/realtime/99-realtime.conf`
- `scripts/setup_realtime_permissions.sh`
- `scripts/verify_realtime_permissions.sh`
- `src/quest_a0509_teleop/config/xyz_position_only.yaml`
- `src/quest_a0509_teleop/launch/doosan_a0509_real.launch.py`
- `src/quest_a0509_teleop/launch/a0509_full_bringup_with_gripper.launch.py`
- `src/jrt_gripper_io/`

현재 작업 트리에는 기존 작업을 포함한 미커밋 변경사항이 있으므로 이후
작업에서도 관련 없는 변경을 reset하거나 덮어쓰지 않는다.


## 15. ServoL RT automatic velocity/acceleration conditions (2026-07-31)

The Streamer now supports a reversible `servol_use_auto_velocity_acceleration`
parameter. The code default remains `false` for compatibility, while the active
experiment profile sets it to `true`.

When enabled, every `ServolRtStream` message carries:

```text
vel = [-10000, -10000, -10000, -10000, -10000, -10000]
acc = [-10000, -10000, -10000, -10000, -10000, -10000]
```

`-10000` is Doosan `DR_COND_NONE`; the controller derives the endpoint velocity
and acceleration conditions from the streamed target pose. Setting the parameter
to `false` restores the previous explicit zero velocity/acceleration fields.

Verification:

- `pytest -q src/quest_a0509_teleop/test`: 76 passed
- `colcon test --packages-select quest_a0509_teleop`: 76 passed
- workspace `colcon test-result --verbose`: 104 tests, 0 errors, 0 failures, 1 skipped
- runtime parameter: `servol_use_auto_velocity_acceleration=True`
- runtime `servol_time_sec=0.1`
- post-restart safety state: `Live=false`, source `DISABLED`

The implementation is active in the restarted real Streamer, but no Live robot
motion has been executed with the new conditions yet. The next step is a bounded
A/B motion test with the existing one-axis orientation mapping unchanged.


### 15.1 First bounded physical test with automatic conditions

A 10.0 second MetaQuest Live session completed with the restarted Streamer and
`servol_use_auto_velocity_acceleration=true`.

- Live samples: 300 (30 Hz)
- final safety state: `Live=false`, source `DISABLED`
- commands after disable response: 0
- target position norm maximum: 60.422 mm
- target rotation maximum: 23.345 deg (Ry-only mapping)
- actual TCP start-to-end translation: 38.971 mm
- actual TCP start-to-end Ry change: -6.183 deg
- Streamer warnings/stops: none
- tick period mean/p95/p99/max: 33.330/34.640/35.276/35.815 ms
- safe target age p50/p95/p99/max: 4.858/6.053/6.699/7.149 ms
- ramp-limited rows: 4/300; maximum rotation lag: 0.171 deg
- gripper was not part of this comparison and no A/B command was accepted

The automatic condition change did not disturb the 30 Hz command path or the
fail-safe shutdown. The timing trace cannot observe the controller-internal
interpolation, so subjective smoothness and/or high-rate actual TCP velocity and
acceleration recording are still required for a strict 0/0 versus -10000/-10000
motion-quality comparison.

## 16. Accepted teleoperation baseline freeze (2026-08-07)

After the subsequent arm-and-gripper sessions were accepted by the operator,
the active parameter set was frozen for LeRobot teacher recording. The canonical
copy is:

```text
src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml
SHA-256: 46dc5d022db4221009a13848b0b514d4b366aecae72158c0db362815ae02866e
```

The source, accepted snapshot, and installed snapshot checksums matched. Runtime
parameter dumps for the Mapper, Safety Guard, Streamer, MUX, preparation node,
Quest A/B mapper, and Tool I/O driver were checked against the frozen values.
The accepted headline values are:

- position scale `0.85`, Ry-only orientation scale `0.7`
- receive-time pose resampling at 30 Hz with 50 ms interpolation delay
- Safety Guard and ServoL Streamer at 30 Hz
- ServoL `time=0.1` with automatic `-10000` velocity/acceleration conditions
- approximately 200 mm/s linear and 30 deg/s rotational target ramps
- workspace X minimum 50 mm and Z minimum limit disabled
- one-second MetaQuest and selected-command heartbeat limits
- A=close, B=open; Tool DO1=open, DO2=close; 0.50-second pulse mode

The full baseline, exact physical launch command, per-session calibration rules,
and LeRobot `shadow_record` contract are documented in
`docs/a0509_metaquest_accepted_baseline_2026-08-07.md`.

Creating and installing the snapshot did not restart the running ROS nodes,
select a MUX source, enable Live output, or issue a robot/gripper command.

## 17. LeRobot shadow-record preflight (2026-08-07)

The Jetson LeRobot 0.6 environment and the editable A0509 plugin were checked
without changing the MUX source or enabling Live output.

- `lerobot==0.6.0`, `torch==2.11.0+cu130`
- `lerobot_robot_doosan_a0509==0.1.0`
- `lerobot-record` registered `doosan_a0509_ros` and `metaquest_a0509`
- all command-line fields used by the pilot script were present
- plugin regression tests: `15 passed`

The selected C920 path resolved to `/dev/video4`. Thirty frames were read at
640x480 with a measured 30.22 FPS and non-constant pixel content.

The first strict robot-adapter connection exposed an incorrect assumption:
`gripper_commanded_state` is transient-local state published when a command
completes, not a periodic stream. Treating it as stale after 0.5 seconds made
recording fail even though its last value was valid. The adapter was changed so
non-commanding `shadow_record`/`policy_dry_run` connection waits only for the
observation inputs it consumes: fresh joint/TCP streams and an available latched
gripper state. Observation reads use the latched gripper value.

The safety scope was deliberately kept narrow:

- `policy_live` connection and action retain the original full freshness gate
  for joint, TCP, robot state, solution space, Live, gripper, and teleop-ready.
- The default MetaQuest teacher path retains freshness checks for target,
  heartbeat, gripper, and teleop-ready.
- The pilot later added an explicit non-commanding shadow-record option for
  latched gripper/teleop-ready state; target and heartbeat remain freshness-gated.
- `shadow_record.send_action()` still validates and returns the teacher action
  without publishing it.

The corrected actual connection produced:

```text
robot shadow connection             = OK
scalar observations                 = 13
front camera                        = 480x640x3
shadow action validation            = OK
/control/lerobot target publishes   = 0
/control/lerobot debug publishes    = 0
```

The strict MetaQuest preflight rejected the current idle session because no
fresh `/control/metaquest/target_posx` was present. This is the intended gate;
the next recording session must prepare and calibrate first.

Reusable entry points were added:

- `scripts/preflight_lerobot_shadow_record.py`
- `scripts/record_lerobot_shadow_pilot.sh`

The shell script performs the preflight before creating a timestamped dataset
directory, records one 30-second 30 FPS episode by default, and never uploads to
the Hugging Face Hub unless a later workflow explicitly changes that policy.

## 18. First physical LeRobot shadow-record episode (2026-08-07)

The first synchronized physical MetaQuest Live session and local LeRobot
shadow-record episode completed. No dataset was uploaded to the Hub.

Before the run, the week-old Doosan bringup was found in `STATE_SAFE_OFF`.
Its log showed that it had originally received
`MONITORING_ACCESS_CONTROL_GRANT`, later reported an Ethernet disconnect, and
never reacquired authority. Restarting only the real Doosan bringup produced:

```text
MANAGE_ACCESS_CONTROL_FORCE_REQUEST
MONITORING_ACCESS_CONTROL_GRANT
CONTROL_SERVO_ON
STATE_STANDBY
```

The robot then moved to the accepted preparation pose, updated its anchor, and
the operator repeated the XY +X calibration before recording.

Two LeRobot compatibility defects were fixed before the successful attempt:

- Draccus 0.11 on Python 3.12 could not decode the robot `Literal` mode field;
  it is now a CLI-decodable string with the same closed-set validation.
- Transient-local gripper and teleop-ready values were incorrectly treated as
  0.3-second streams. An explicit
  `allow_latched_state_in_shadow_record` option is enabled only by the
  non-commanding pilot/preflight path. Motion target and heartbeat freshness
  were initially limited to 0.3 seconds; Section 21 aligns the recorder-side
  gate with the accepted 1.0-second MUX/streamer watchdog while remaining strict.

Plugin regression result after the changes: `18 passed`.

The supervised physical session result was:

```text
requested/actual Live duration       = 30.0 / 30.003 s
safe target samples                  = 900
maximum target translation norm      = 431.913 mm
maximum target rotation magnitude    = 24.927 deg
actual TCP start-to-end translation  = 331.946 mm
gripper accepted commands            = open, close
gripper validation                   = pass
commands after Live disable response = 0
final Live/source                    = false / DISABLED
```

The local dataset is:

```text
/home/rvlab/lerobot_datasets/a0509_metaquest_pick_place_pilot_20260807_213300
```

It contains one task and one episode, 7 action values, 13 scalar observation
values, and the C920 front image. Parquet and AV1 video cross-checks both
reported exactly 854 frames. The video is 640x480 at nominal 30 FPS and
28.4667 seconds; the Parquet timestamp range is 0.0 through 28.4333 seconds.
The gripper action contains both 0 and 1, with 17 nonzero frames. Total dataset
size is approximately 13 MiB.

The physical Live interval was 30.003 seconds, so 854 stored samples correspond
to an effective recording rate of approximately 28.47 FPS rather than the
requested 30 FPS. This pilot is structurally valid, but camera/record-loop
timing should be inspected before starting a large demonstration campaign.

After encoding and dataset validation, the independently queried final state
was `Live=false`, MUX source `DISABLED`, and robot state `1 (STANDBY)`.

## 19. Three-RGB-camera LeRobot registration (2026-08-07)

The default A0509 observation camera set was expanded from one C920 to three
RGB inputs:

```text
front    = C920 /dev/video4, 640x480 @ 30 FPS, MJPEG
side     = C920 serial 947C90BF /dev/video2, 640x480 @ 30 FPS, MJPEG
zed_rgb  = ZED2 /dev/video0 stereo 2560x720 @ 30 FPS, YUYV
           -> left RGB crop 1280x720
```

The stable `/dev/v4l/by-id` paths are stored in the plugin configuration. A
custom `zed_left_opencv` camera adapter validates the full stereo frame,
converts BGR to RGB, takes only the contiguous left half, and exposes it as
`observation.images.zed_rgb`. No ZED SDK depth pipeline, depth image, right
image, disparity, or point cloud is enabled.

Both C920 streams use MJPEG to reduce shared USB 2.0 bandwidth. The ZED2 remains
on its separate USB 3.0 bus.

Verification completed without robot motion or Live output:

- LeRobot camera factory created `front`, `side`, and `zed_rgb`.
- All three physical devices opened simultaneously.
- Captured shapes were `(480, 640, 3)`, `(480, 640, 3)`, and
  `(720, 1280, 3)` respectively.
- Every returned image was `uint8` and contiguous.
- ZED left-crop/RGB conversion unit test passed.
- Plugin regression result: `20 passed`.
- `lerobot-record --robot.type=doosan_a0509_ros --help` parsed successfully.

The cameras were disconnected after the smoke test. The next step is a bounded
three-camera performance run measuring achieved record-loop FPS, per-camera
frame updates, CPU load, USB errors, and dropped/mismatched video frames before
collecting more demonstrations.

## 20. Three-RGB-camera acquisition performance (2026-08-07)

A reusable read-only benchmark was added at
`scripts/benchmark_lerobot_three_cameras.py`. It opens the default LeRobot
camera set without connecting the robot, selecting a MUX source, enabling Live,
or writing a dataset. It validates shapes and measures capture timestamps,
frame age, loop timing, CPU, and memory.

The first 30 Hz observation runs appeared to show varying source rates from
approximately 23.7 to 30 FPS. Individual and paired camera checks showed that
the affected camera changed between runs, including cameras on separate USB
buses. A 60 Hz timestamp-only observer then proved this was sampling-phase
aliasing: all three sources produced exactly 450 new frames in 15 seconds.

```text
observer rate                         = 59.9997 Hz
front / side / zed_rgb source rate    = 29.9998 / 29.9998 / 29.9998 Hz
deadline misses                       = 0
timestamp-read p99 / maximum          = 0.0225 / 0.0657 ms
process / system CPU                  = 35.5 / 27.9 percent
RSS                                   = 656.7 MiB
```

The final test copied all three RGB arrays at the intended LeRobot 30 Hz loop
rate for 30 seconds:

```text
requested / actual duration           = 30.0 / 30.0001 s
sample ticks / achieved loop rate      = 900 / 29.9999 Hz
deadline misses                       = 0
frame-read mean / p99 / maximum        = 0.0529 / 0.0986 / 0.1370 ms
tick-period p99 / maximum              = 33.3982 / 33.9048 ms
process / system CPU                  = 40.9 / 28.6 percent
RSS                                   = 655.9 MiB
front observed unique / duplicates     = 881 / 19
side observed unique / duplicates      = 900 / 0
zed_rgb observed unique / duplicates   = 899 / 1
```

The 30 Hz unique-frame count must not be interpreted as source frame loss:
source and observer both run at nominally 30 Hz, so their independent clock
phase can occasionally reuse one latest frame and skip the next source frame.
The independent 60 Hz timestamp observation confirmed every source at 30 FPS.

During repeated rapid open/close diagnostics, the ZED2 UVC interface once
remained enumerated but stopped returning frames. LeRobot and direct FFmpeg
both saw zero frames, so this was below the plugin layer. Reconnecting/resetting
the ZED2 USB device restored output immediately. The post-reconnect 15-second
source-rate and 30-second copy tests both passed, and the kernel log showed no
USB/UVC errors during either test. Stable `/dev/v4l/by-id` addressing survived
the device-number change.

Camera acquisition itself therefore has sufficient timing and compute margin
for the configured three-camera 30 FPS observation set. The next performance
boundary is a short end-to-end three-camera `lerobot-record` episode, followed
by exact Parquet/video frame-count and effective-FPS checks; that test includes
dataset encoding and ROS observation/action work that this acquisition-only
benchmark intentionally excludes.

## 21. End-to-end three-RGB LeRobot record and physical Live trial (2026-08-07)

The ZED2 adapter was changed from an OpenCV reader to a bounded FFmpeg/V4L2
subprocess reader. FFmpeg captures the full 2560x720 YUYV stereo frame, crops
the left 1280x720 image, converts it to RGB24, and feeds a latest-frame
background buffer. A stale-frame recovery wait is bounded to 500 ms; depth and
the right image remain disabled.

End-to-end shadow-record tests established the stable writer configuration as
one image-writer process and two writer threads per camera. The record script
now uses those values by default. Before the physical trial, non-Live records
completed for 10 seconds (291 rows/frames) and 35 seconds (995 rows/frames).

The first physical attempt stopped safely when the LeRobot teacher freshness
gate observed `valid_pose_heartbeat age=0.314s` against the old 0.300-second
limit. Live was immediately disabled and the MUX changed from METAQUEST to
DISABLED. This gate is a recorder-side validation, not the ServoL watchdog.
The default and CLI record value were aligned with the accepted MUX/streamer
heartbeat allowance at 1.0 second. Plugin regression result: `20 passed`.

The successful transport trial used:

```text
dataset root       = /home/rvlab/lerobot_datasets/a0509_metaquest_pick_place_pilot_20260807_230927
requested window   = 50 s
stored samples     = 1420
timestamp range    = 0.0 through 47.3 s
effective rate     = 30 FPS
front video        = 640x480, 1420 frames, 47.3333 s
side video         = 640x480, 1420 frames, 47.3333 s
zed_rgb video      = 1280x720, 1420 frames, 47.3333 s
physical Live      = 30 s
final Live/source  = false / DISABLED
final robot state  = 1 (STANDBY)
```

Parquet rows, frame indices 0 through 1419, and all three AV1 video frame
counts match exactly. After encoding, joint/TCP/robot-state streams were again
observed live, and the final robot state was independently read as STANDBY.

This episode passes the three-camera transport and structural dataset test,
but it is not yet accepted as a behavioral demonstration. Its source-specific
teacher action is constant for all 1420 rows:

```text
[808.3388, -90.6034, 305.8444, 0.1056, 119.1184, 0.1209, 1.0]
```

The streamer trace independently shows a constant safety target
`[650.0, -90.6034, 305.8444, 0.1056, 119.1184, 0.1209]` for all 1009 Live
ticks. The commanded pose changed only while the streamer ramped from the
initial TCP to that fixed, X-clamped target. The observation therefore moved,
but the controller-derived action and gripper state did not vary. This is
consistent with a stationary/frozen Quest pose during the active interval and
must be distinguished from a task demonstration in which the operator moves
the controller and toggles A/B.

The next bounded trial should explicitly require observable variation on the
MetaQuest target axes and both gripper states before the resulting episode is
promoted to training data. Constant-action capture should remain preserved as
a transport diagnostic rather than silently accepted or overwritten.

## 22. JRT gripper state-aware mapping revalidation (2026-08-08)

An arm+gripper trial reported no visible A/B motion even though the recorded
raw Quest input, mapper output, MUX forwarding, driver acceptance, and Tool DO
readback completion were present. The gripper's manual OPEN/CLOSE buttons also
worked. Live was then kept false and the MUX source DISABLED while the gripper
path was isolated.

The first topic-level diagnostic contained a discovery race: a short-lived
`ros2 topic pub --once` CLOSE message was recorded by rosbag but did not reach
the driver. Later tests also initially treated a same-direction command at a
finger end position as a failed command. Neither condition represents a
physical mapping change.

The decisive test established known physical start states and activated only
one Tool DO at a time. From a known open state, DO2 produced CLOSE. From the
resulting known closed state, DO1 produced OPEN. Each controller readback
showed the requested ON value and final OFF value. The accepted contract is
therefore unchanged:

```text
A / button_lower -> close -> Tool DO2
B / button_upper -> open  -> Tool DO1
active value     -> 1
inactive value   -> 0
pulse mode       -> 0.50 s, timed after matching ON readback
```

The source, launch defaults, direct scripts, and accepted baseline already
matched this contract. An end-to-end regression now locks the A/B semantic
mapping together with the physical DO plan. The package runbook was updated to
require an opposite known finger state before judging motion, to state that
`completed_command` is output readback rather than finger feedback, and to
avoid using an unverified one-shot topic publisher while a recorder is also a
subscriber. The standalone recovery record is
`docs/jrt_gripper_mapping_revalidation_2026-08-08.md`.
