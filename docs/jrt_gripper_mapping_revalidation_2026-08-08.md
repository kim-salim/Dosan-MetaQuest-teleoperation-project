# JRT gripper mapping revalidation — 2026-08-08

이 문서는 A0509 + JRT JEGB 그리퍼의 물리 매핑과 재검증 절차를 고정한다.
그리퍼가 움직이지 않는 것처럼 보일 때 DO 번호를 임의로 바꾸지 말고 이 문서를
먼저 확인한다.

## 확정된 불변 조건

| 입력/명령 | 의미 | 실제 Tool DO |
|---|---|---:|
| Quest A / `button_lower` | CLOSE | DO2 |
| Quest B / `button_upper` | OPEN | DO1 |
| `close` | 닫기 | DO2 |
| `open` | 열기 | DO1 |

추가 고정값:

```text
active_value                  = 1
inactive_value                = 0
command_mode                  = pulse
pulse_sec                     = 0.50
interlock_sec                 = 0.05
readback_poll_sec             = 0.01
readback_timeout_sec          = 0.50
metaquest_gripper_min_pulse   = 1.50
```

드라이버는 반대 방향 출력을 OFF로 확인하고, 50 ms interlock 뒤 목표 DO의
ON readback을 확인한 시점부터 0.50초를 유지한다. 마지막 OFF readback까지
완료돼야 `/jrt_gripper/completed_command`를 발행한다.

## 2026-08-08 재검증 결과

처음에는 A/B 입력과 driver 완료 로그가 있었지만 그리퍼가 움직이지 않는 것처럼
보였다. 원인을 분리한 결과 두 가지 진단 혼선이 겹쳐 있었다.

1. 그리퍼에 물리 위치 센서가 연결되지 않았기 때문에 이미 열린 상태의 OPEN과
   이미 닫힌 상태의 CLOSE는 정상 명령이어도 육안 변화가 없다.
2. 한 번의 `ros2 topic pub --once` CLOSE 시험은 함께 실행한 rosbag recorder가
   먼저 구독자로 발견됐다. bag에는 `/jrt_gripper/cmd`가 남았지만 driver의
   `accepted_command`가 없었으므로 실제 driver 명령이 아니었다.

상태를 명확히 만든 뒤 Tool DO를 하나씩 직접 시험했다.

```text
알려진 OPEN 상태   -> DO2 단독 펄스 -> 실제 CLOSE 확인
알려진 CLOSED 상태 -> DO1 단독 펄스 -> 실제 OPEN 확인
최종 DO1 / DO2     -> 0 / 0 확인
```

따라서 물리 매핑은 바뀌지 않았으며 `DO1=OPEN`, `DO2=CLOSE`가 확정값이다.

## 세 경로의 차이

### MetaQuest 전체 경로

```text
Quest A/B
-> /control/metaquest/gripper_cmd
-> MUX
-> /jrt_gripper/cmd
-> jrt_tool_io_driver_node
-> Doosan Tool DO
```

이 경로는 METAQUEST source, `teleop_ready`, calibration VALID, fresh target,
heartbeat, Live가 모두 정상이어야 OPEN/CLOSE를 통과시킨다.

### 드라이버 직접 경로

```text
/jrt_gripper/cmd
-> jrt_tool_io_driver_node
-> Doosan Tool DO
```

이 경로는 MUX와 calibration VALID를 우회한다. 반드시
`/jrt_gripper/accepted_command`와 `/jrt_gripper/completed_command`를 함께
확인한다. rosbag을 동시에 쓸 때 `ros2 topic pub --once`만으로 전달 성공을
판정하지 않는다.

### Tool DO 직접 경로

```text
/dsr01/dsr_controller2/io/set_tool_digital_output
-> Doosan Tool DO
```

이 경로는 MetaQuest, MUX, calibration, Live, 그리퍼 드라이버를 모두 우회한다.
물리 신호 분리에만 사용한다. DO1과 DO2를 동시에 ON으로 만들지 않는다.

## 재현 가능한 진단 순서

1. 팔 Live를 false로 하고 MUX source를 `DISABLED`로 둔다.
2. DO1과 DO2가 모두 0인지 확인한다.
3. 수동 버튼 또는 이미 확인된 반대 명령으로 그리퍼의 시작 위치를 확실히 한다.
4. 알려진 OPEN 상태에서 CLOSE/DO2를 시험한다.
5. 실제 CLOSED를 확인한 다음 OPEN/DO1을 시험한다.
6. 각 단계에서 controller readback과 육안 동작을 별도로 기록한다.
7. 종료 시 DO1/DO2를 모두 0으로 만든다.

권장 직접 시험 명령은 다음과 같다.

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

src/jrt_gripper_io/scripts/gripper_close_pulse.sh
sleep 2
src/jrt_gripper_io/scripts/gripper_open_pulse.sh
src/jrt_gripper_io/scripts/gripper_all_off.sh
```

위 순서는 시작 상태가 OPEN일 때 가장 명확하다. 시작 상태가 CLOSED라면 OPEN을
먼저 실행해 알려진 OPEN 상태를 만든 뒤 CLOSE와 OPEN을 이어서 확인한다.

## 로그 해석 규칙

- `accepted_command`: driver가 명령 계획을 접수했다.
- `completed_command`: SET/GET readback 계획이 끝났다.
- `commanded_state=0.0`: 마지막으로 완료된 명령이 OPEN이다.
- `commanded_state=1.0`: 마지막으로 완료된 명령이 CLOSE다.
- 위 값들은 실제 손가락 위치 센서가 아니다.
- 같은 끝 위치로 다시 명령했을 때 무동작처럼 보이는 것은 정상일 수 있다.
- `/jrt_gripper/cmd`가 bag에 있다는 사실만으로 driver 수락을 증명하지 않는다.
- MetaQuest VALID가 false이면 MUX 앞 경로는 차단되지만 direct driver/Tool DO
  시험에는 영향을 주지 않는다.

## 코드 고정 장치

다음 파일들이 동일한 매핑을 사용한다.

- `jrt_tool_io_driver_node.py`: close=2, open=1
- `jrt_gripper_io.launch.py`: close=2, open=1
- `jrt_gripper_robot_bringup.launch.py`: close=2, open=1
- `a0509_full_bringup_with_gripper.launch.py`: close=2, open=1
- `gripper_close_pulse.sh`: close=2
- `gripper_open_pulse.sh`: open=1

`src/jrt_gripper_io/test/test_mapping_defaults.py`는 launch, driver, 수동
스크립트뿐 아니라 `A -> close -> DO2`, `B -> open -> DO1` 전체 계약을
회귀 테스트로 고정한다.

## 현재 판단

2026-08-08 사건은 그리퍼 전원 고장, MUX 매핑 변경, DO 번호 변경이 아니었다.
상태 의존적인 육안 판정과 일회성 publisher 발견 경쟁이 원인이었다. 향후에는
반드시 알려진 반대 끝 위치에서 한 방향씩 시험하고, driver 수락과 Tool DO
readback, 실제 손가락 동작을 서로 다른 증거로 취급한다.
