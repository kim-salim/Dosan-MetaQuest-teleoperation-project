# A0509 Diffusion 독립 구축 진행 기록 (2026-08-11)

## 범위와 불변 조건

- 기존 ACT 환경 `/home/rvlab/venvs/lerobot`은 수정하지 않는다.
- 기존 ACT 모델, `DoosanA0509Ros.send_action`, MUX, Safety Guard, ServoL 구현은 수정하지 않는다.
- Diffusion은 별도 Jetson venv, 별도 서버 Docker 이미지, 별도 데이터 파생본, 별도 모델 경로를 사용한다.
- 이 단계의 검증은 로봇 명령을 발행하지 않는 checkpoint benchmark까지만 수행한다.

## 독립 경로

- Jetson Diffusion venv: `/home/rvlab/venvs/lerobot-diffusion`
- 서버 Docker image: `lerobot-a0509-diffusion:0.6.0-torch2.11-cu130`
- 서버 원본 데이터: `/home/slkim/lerobot-a0509-training/data/a0509_blue_block_v1_20260810_170226`
- 서버 파생 데이터: `/home/slkim/lerobot-a0509-training/data/a0509_blue_block_v1_diffusion_rgb640_letterbox_v1`
- 서버 학습 출력: `/home/slkim/lerobot-a0509-training/outputs-diffusion/diffusion_a0509_blue_block_v1_rgb640_ddim5_bs16_50k_20260811_0226`
- Jetson 최종 모델 경로: `/home/rvlab/lerobot_models/diffusion_a0509_blue_block_v1_rgb640_ddim5_bs16_50k_20260811_0226/050000/pretrained_model`

## 데이터 계약

- LeRobot v3.0, 30 episodes, 31,500 frames, 30 FPS
- observation state: 13 dimensions
- action: 7 dimensions
- front RGB: `3x480x640`
- side RGB: `3x480x640`
- ZED left RGB: 원본 `3x376x672`, 파생본 `3x480x640`
- ZED 변환: 종횡비 유지 640x358 resize 후 위/아래 61 pixel 검정 letterbox
- depth 입력은 사용하지 않는다.

원본 전체 파일 manifest SHA-256:

```text
6ab1219d432e67abf65dbecca2573d54e0343ba878060869e6e861342e0d8657
```

파생 데이터 전체 파일 manifest SHA-256:

```text
e5bb975ecd1bd80a1b6b527b0614d023e64e68943fd66121bbaeedb4b2e2a871
```

LeRobot 실제 디코딩 검증에서 세 카메라의 표본 index 0, 15,750, 31,499가 모두 `3x480x640`으로 확인됐다.

## 정책 설정

```text
policy.type=diffusion
n_obs_steps=2
horizon=16
n_action_steps=8
drop_n_last_frames=7
vision_backbone=resnet18
use_separate_rgb_encoder_per_camera=true
noise_scheduler_type=DDIM
num_train_timesteps=100
num_inference_steps=5
use_amp=true
compile_model=false
batch_size=16
steps=50000
save_freq=10000
```

정책 크기는 293,091,943 parameters이다.

## 서버 검증

- 허용 GPU: GPU 0 한 장만 컨테이너에 노출
- GPU: NVIDIA RTX PRO 5000 Blackwell, 48,935 MiB
- 100-step 지속 smoke 결과:
  - 약 4.0 step/s
  - steady update 약 0.245 s/step
  - data 약 0.004 s/step
  - steady CUDA memory 약 17.17 GiB
- 10,000-step checkpoint의 model, processor, train config, optimizer, RNG, scheduler state 생성 확인
- 10,000-step pretrained manifest SHA-256:

```text
ec68592970e1d8df3b5ce052afadf1403a0a204667d8c8d3669d333f9216b71a
```
- 50,000-step 본 학습 완료:
  - 총 학습 시간 약 3시간 38분
  - 처리 속도 약 3.82 step/s, 61 samples/s
  - 마지막 기록 loss 약 0.002
  - steady CUDA memory 약 17.17 GiB
  - `End of training` 로그와 50,000-step checkpoint 생성 확인
- 50,000-step 최종 pretrained manifest SHA-256:

```text
1ca47cfe886958c951174e8fe65ffde0b5c94ac5db75bbd2923835ab78906ef5
```

## Jetson 실행 경로 검증

- Jetson 전용 venv:
  - torch `2.11.0+cu130`
  - lerobot `0.6.0`
  - diffusers `0.35.2`
  - CUDA device `NVIDIA Thor`
- 기존 ACT venv에서는 `diffusers`가 여전히 설치되지 않은 것을 확인했다.
- 100-step smoke pretrained model을 서버에서 Jetson으로 복사했고 양쪽 manifest가 일치했다.
- 실제 v1 두 프레임을 읽고 ZED만 런타임 letterbox한 benchmark:
  - action chunk shape `1x8x7`
  - mean latency 약 169.4 ms
  - p95 약 173.5 ms
  - peak CUDA memory 약 1.23 GiB
  - 모든 action finite 확인
- 50,000-step 최종 모델을 Jetson으로 복사했고 서버/Jetson manifest가 일치했다.
- 최종 모델 합성 입력 benchmark 20회:
  - action chunk shape `1x8x7`
  - mean latency 약 166.9 ms
  - p95 약 168.3 ms
  - p99/max 약 170.2 ms
  - peak CUDA memory 약 1.23 GiB
- 최종 모델 실제 v1 입력 benchmark 20회:
  - action chunk shape `1x8x7`
  - mean latency 약 168.5 ms
  - p95 약 170.7 ms
  - p99/max 약 170.7 ms
  - peak CUDA memory 약 1.23 GiB
  - 모든 action finite 확인
- benchmark와 어댑터는 로봇 action topic을 발행하지 않는다.

## 새 파일

- `scripts/prepare_a0509_diffusion_dataset.py`
- `scripts/validate_a0509_diffusion_dataset.py`
- `docker/lerobot_training/Dockerfile.diffusion`
- `docker/lerobot_training/compose.diffusion.yaml`
- `docker/lerobot_training/train_diffusion.sh`
- `scripts/benchmark_a0509_diffusion_policy.py`
- `scripts/benchmark_a0509_diffusion_policy.sh`
- `src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/diffusion_camera_adapter.py`
- `src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/config_doosan_a0509_diffusion_ros.py`
- `src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/doosan_a0509_diffusion_ros.py`
- `src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/diffusion_async_rollout.py`
- `src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/diffusion_rollout_entrypoint.py`
- `src/lerobot_robot_doosan_a0509/test/test_diffusion_async_rollout.py`
- `src/lerobot_robot_doosan_a0509/test/test_diffusion_camera_adapter.py`
- `scripts/benchmark_a0509_diffusion_async_runtime.py`
- `scripts/run_a0509_diffusion_policy_dry_run.sh`
- `scripts/run_a0509_diffusion_policy_live.sh`

Diffusion robot adapter는 ZED observation shape/이미지만 변경하고 기존 `DoosanA0509Ros.send_action` 메서드를 그대로 상속한다.

## Diffusion 전용 비동기 30 Hz 실행기

- 기존 ACT 비동기 실행기의 검증된 구조를 참고하되, Diffusion 전용 모듈과 전용 Live generation gate로 분리했다.
- ACT 실행 파일, ACT policy patch, ACT action queue 및 기존 두산 제어 경로는 변경하지 않았다.
- 카메라와 ROS 상태는 30 Hz control tick에서 최신 observation snapshot만 사용한다.
- Diffusion worker는 최근 observation 두 개를 시간축으로 쌓아 `n_obs_steps=2` 입력을 만든다.
- 첫 observation은 한 번 복제해 시작하고, worker backlog를 만들지 않고 최신 요청만 처리한다.
- 한 번에 8개 action을 생성하고, queue에 7개가 남으면 다음 추론을 요청한다.
- 새로 완료된 chunk는 오래된 tail 뒤에 누적하지 않고 최신 chunk로 교체한다.
- chunk 경계의 위치는 선형 보간하고, Doosan ZYZ 자세는 quaternion SLERP로 2 action 동안 overlap blending한다.
- gripper는 연속 보간하지 않고 새 chunk의 이산 상태를 유지한다.
- observation age 제한은 0.25 s, 완료 action chunk age 제한은 0.50 s이다.
- Live rising edge가 발생하면 이전 Live generation에서 계산된 chunk를 폐기한다.
- 실제 Live에서는 첫 유효 chunk가 준비될 때까지 기존 TCP hold를 발행하고 MUX 전환을 허용하지 않는다.

최종 50,000-step 모델을 사용한 Thor 오프라인 비동기 검증 결과:

```text
mode=offline_async_no_robot_commands
fps=30
steady_duration=5 s
initial_empty_ticks=38       # 모델 로드/2회 warm-up 중이며 로봇 명령 없음
actions_consumed=151
steady_empty_ticks=0         # 첫 유효 action 이후 queue underrun 없음
steady_stale_action_drops=0
steady_stale_inference_drops=0
last_inference_latency_ms=177.15
remaining_actions=8
```

시작 단계에서 0.50 s보다 오래 걸린 warm-up chunk 한 개는 의도대로 stale 폐기됐다. 첫 유효 action 이후에는 30 Hz 소비 중 queue 고갈과 stale 폐기가 모두 0건이었다.

검증 결과:

- Diffusion/카메라/기존 ACT 회귀 테스트 `16 passed`
- `colcon build --packages-select lerobot_robot_doosan_a0509 --symlink-install` 성공
- 설치된 factory가 `DoosanA0509DiffusionRos`를 생성함을 확인
- `DoosanA0509DiffusionRos.send_action is DoosanA0509Ros.send_action` 확인
- 최종 checkpoint 계약 검증: Diffusion, observation 2, horizon 16, action 8, DDIM 5, 3x `3x480x640`, state 13, action 7
- 새 Python 및 shell script 문법 검사 통과
- 이번 비동기 benchmark에서는 ROS robot action topic을 발행하지 않았다.

실제 bringup 및 세 카메라를 사용한 ROS `policy_dry_run`도 완료했다.

- 최종 50,000-step checkpoint와 실제 ROS state 및 세 카메라 입력 사용
- 20초, 10초, 계측용 6초 dry-run이 모두 정상 종료
- 6초 계측 run의 내부 발행 카운트: `debug=140`, `live=0`, `hold=0`
- 첫 유효 action 이후 debug action 발행률: 약 29.8 Hz
- control loop 30 Hz 미달 경고: 0건
- steady queue: 7~8 actions
- steady policy forward: 약 164~178 ms
- steady 전체 chunk 준비: 주로 약 181~198 ms
- 실제 `/control/lerobot/target_posx` 및 `/control/lerobot/gripper_target` 명령 발행: 0건
- 시작 warm-up 중 오래된 chunk 한 개만 의도대로 stale 폐기

dry-run 당시 bringup의 `robot_state=3`은 Live 허용 상태 1·2가 아니어서 실제 로봇 Live 시험을 수행하지 않았다. 이후 새 bringup부터 다시 시작해 아래의 bounded Live를 수행했다.

## 첫 실제 Diffusion Live 시험

기존 bringup을 Live OFF/MUX DISABLED로 정상 종료한 뒤 새 bringup부터 다시 수행했다.

- 새 bringup launch PID: `820904`
- robot state: `1`
- 준비자세 TCP: `[429.831, 0.412, 461.461, 0.101, 149.393, 0.118]`
- 그리퍼 초기 상태: open (`commanded_state=0.0`)
- 정책: 최종 50,000-step Diffusion checkpoint
- bounded Live 시간: 10 s
- 첫 hold target 차이: 위치 0.000 mm, 회전 0.000 deg
- 첫 fresh model target 차이: 위치 8.306 mm, 회전 0.230 deg
- Live action 발행: 314개, 약 31 Hz
- gate 상태: `RUNNING -> COMPLETED`
- steady queue: 7~8 actions
- steady policy forward: 주로 약 164~170 ms
- steady 전체 chunk 준비: 주로 약 177~198 ms
- MUX source 이탈, target/state stale, safety gate 중단: 0건
- 종료 상태: Live OFF, source DISABLED, robot state 1
- 종료 TCP: `[392.730, -10.335, 493.315, 0.682, 148.635, -0.122]`
- 종료 그리퍼: open (`commanded_state=0.0`)
- 정책 종료 후 세 카메라 모두 해제

Live rising 직후 첫 chunk는 source age 501.2 ms로 500 ms 제한을 1.2 ms 초과해 폐기됐고, startup underrun이 2회 기록됐다. 다음 fresh chunk로 즉시 회복한 뒤 10초 steady 구간에는 queue 고갈이 없었다.

그리퍼 출력 314개는 모두 open 영역이었다(`min=0.0`, `max=0.00546`). 따라서 이번 실행에서 그리퍼 미동작은 I/O 실패가 아니라 정책이 close action을 예측하지 않은 결과다.


## 40초 full Live 및 gripper 원인 분리

같은 최종 50,000-step Diffusion checkpoint로 새 준비자세부터 40초 bounded Live를 수행했다.

- 준비자세 TCP: `[430.635, 0.408, 464.901, 0.099, 148.936, 0.115]`
- 첫 hold target 차이: 위치 0.000 mm, 회전 0.000 deg
- 첫 fresh model target 차이: 위치 5.839 mm, 회전 0.466 deg
- 정책 Live 발행: 1,209개
- Streamer trace: 1,224 ticks / 40.764 s = 30.002 Hz
- Streamer 평균 주기 33.331 ms, p99 34.941 ms, 최대 41.228 ms
- MUX 전환: `DISABLED -> LEROBOT -> DISABLED`
- safety/streamer stale 또는 강제 중단: 0건
- 종료 TCP: `[506.480, 235.888, 298.560, 0.689, 154.782, -0.913]`
- 종료 상태: Live OFF, source DISABLED, robot state 1, gripper open
- 정책 종료 후 세 카메라 모두 해제

로봇 목표는 40초 종료까지 계속 변했고 XYZ command path length는 1,820.0 mm였다. 따라서 제어가 중간에 정지해서 파지를 못 한 것은 아니다. 목표 궤적은 약 25.50초에 학습 데이터의 한 close-onset TCP에서 9.09 mm까지 접근했지만 gripper close 명령은 발생하지 않았다.

그리퍼 경로를 단계별로 분리 검증했다.

1. 원본 v1 데이터는 31,500프레임이며 gripper action은 정확히 0 또는 1이다.
2. open은 19,273프레임, close는 12,227프레임(38.8%)이고, 30개 모든 에피소드에 open→close→open 전환이 있다.
3. checkpoint preprocessor/postprocessor의 action MIN_MAX 통계도 gripper min=0, max=1로 원본과 일치한다.
4. `DoosanA0509Ros.send_action()`은 복원된 7번째 값을 그대로 `/control/lerobot/gripper_target`에 발행한다.
5. MUX hysteresis는 `<0.3=open`, `>0.7=close`이며 기존 검증값과 일치한다.
6. 학습 데이터 episode 0의 close 직전 프레임 326~327을 checkpoint에 다시 입력하면 5개 seed 모두 뒤쪽 action에서 0.7을 초과했고 최대 0.9983을 출력했다.
7. 이미 닫힌 프레임 399~400에서는 5개 seed의 출력이 약 0.996~1.0이었다.
8. 실제 종료 장면에서 수행한 30초 물리 명령 없는 dry-run에서는 482개 gripper 출력이 모두 0.3 미만이었다. 최소 0.0, 중앙값 0.00422, p90 0.00854, 최대 0.20022였고 0.7 초과는 0건이었다.

따라서 데이터 기록, 정규화/역정규화, ROS 발행, MUX threshold, JRT driver는 이번 미파지의 원인이 아니다. 모델은 학습 프레임에서는 close를 재현하지만 실제 rollout 장면에서는 close 시점으로 분류하지 못했다. 가장 가능성이 높은 원인은 rollout 누적 오차로 인한 시각·TCP 상태 조합의 학습 분포 이탈이다. 단순히 MUX close threshold를 낮추는 것은 현재 최대값 0.20022와 정상 open 출력 사이의 안전 여유를 제거하므로 적용하지 않는다.

## 현재 상태

- 50,000-step 서버 학습 완료
- 최종 `pretrained_model` Jetson 복사 완료
- 서버/Jetson manifest 일치 확인 완료
- Thor 합성 입력 및 실제 v1 입력 benchmark 완료
- Diffusion 전용 최신-input 비동기 추론 worker와 30 Hz action queue 구현 완료
- 최종 모델 기준 5초 steady 30 Hz 오프라인 검증 완료: action 151개, steady underrun 0건
- 실제 세 카메라 ROS dry-run 완료: 약 29.8 Hz debug 발행, live/robot command 0건
- 새 bringup부터 10초 실제 Diffusion bounded Live 완료: 314 action, 안전 중단 0건
- 40초 full Live 완료: policy 1,209 action, Streamer 1,224 tick/30.002 Hz, 안전 중단 0건
- gripper 미파지 원인 분리 완료: 학습 프레임에서는 close 재현, 실제 rollout에서는 최대 0.20022로 close 미예측
- 기존 ACT venv, ACT 모델, Doosan 제어, MUX, Safety Guard, ServoL은 변경하지 않았다.
- 다음 단계는 MUX threshold를 바꾸지 않고 실제 rollout의 close 직전 이미지/TCP/action trace를 저장하여 학습 close-onset 샘플과 비교하고, 필요하면 실패 장면을 포함한 데이터 보강 또는 checkpoint 재학습을 수행하는 것이다.

## 횡방향 떨림 1·2순위 개선

40초 Live 계측에서 safety 출력과 Streamer 명령의 최대 차이는 0.021 mm였지만,
정책 목표의 Y 방향 반전은 초당 약 9.05회였다. 기존 구현은 약 190~230 ms가
걸린 새 chunk의 첫 action부터 다시 실행해, 이미 6~7 control tick 지난 예측을
현재 시각의 목표로 사용하고 있었다.

이번 변경은 Diffusion 전용 경로의 다음 두 항목에만 한정했다.

1. runtime `n_action_steps`를 8에서 15로 늘리고, observation source age를 30 Hz
   tick으로 환산해 이미 지난 action prefix를 제거한다. checkpoint의 학습 계약
   (`horizon=16`, `n_action_steps=8`)과 weight는 변경하지 않는다.
2. 새 chunk와 기존 미소비 tail을 같은 실행 시각끼리 맞춘 뒤 최대 3 tick만
   overlap blending한다. gripper는 연속 보간하지 않고 기존 이산 상태 정책을
   유지한다.

적용된 runtime 기준은 `n_action_steps=15`, `queue_threshold=9`,
`overlap_steps=3`이다. 실제 세 카메라 dry-run에서는 정상 추론마다 오래된 앞부분
6~7개를 제거하고 8~9개의 현재·미래 action을 유지했다. CUDA 초기 warm-up으로
34 tick 늦어진 첫 chunk 한 개는 의도대로 폐기했으며, 이후 정상 구간에는 queue
underrun이 없었다.

검증 결과:

- Diffusion queue, 카메라 adapter, 기존 ACT 회귀 테스트: `17 passed`
- 실제 세 카메라 10초 `policy_dry_run`: `debug=259`, `live=0`, `hold=0`
- 로봇 명령 없는 Thor 5초 GPU benchmark: action 151개, steady underrun 0건,
  stale action 0건, stale inference 0건
- 선택 패키지 `colcon build`: 성공
- 고정 noise, 추가 가속도/jerk filter, MUX, Safety Guard, ServoL은 변경하지 않았다.

이 검증은 스케줄러의 시간 정렬과 30 Hz 지속성을 확인한 것이다. 실제 로봇에서
횡방향 떨림이 줄었는지는 다음 bounded Live 시험에서 같은 계측 방식으로 확인한다.

## Live-generation noise 및 3-chunk temporal ensemble

지연 보정과 3-tick pairwise overlap을 적용한 40초 실제 시험은 30.000 Hz를
유지했고 MUX, stale action, Safety 중단이 없었다. 이전 시험보다 축 방향 반전은
줄었지만 Y/Z action step이 Streamer 한계인 6.67 mm/tick에 닿는 구간과 잔여
진동이 남았다. 통신 경로가 아니라 재추론 trajectory 분산을 다음 대상으로
분리했다.

Diffusion 전용 RTC 경로에 다음 두 항목을 추가했다.

1. 기본 `noise_correlation=1.0`으로 같은 Live generation 안에서는 동일한
   Diffusion prior를 재사용한다. Live rising edge 또는 rollout reset에서는 새
   독립 prior를 생성한다. 필요하면 실행 환경에서 0~1 사이 값으로 내려 상관
   noise를 사용할 수 있으며, 학습 noise에는 영향을 주지 않는다.
2. 지연 prefix를 제거한 각 raw chunk에 첫 실행 control tick을 기록한다. 최신
   chunk와 직전 두 chunk에서 동일 tick을 예측한 action만 최신순
   `1.0, 0.5, 0.25`로 결합한다. 위치는 물리 공간 가중 평균, 회전은 quaternion
   SLERP, gripper는 평균 없이 최신 이산 예측을 사용한다.

기본 실행 설정:

- `NOISE_CORRELATION=1.0`
- `ENSEMBLE_HISTORY_SIZE=3`
- `ENSEMBLE_WEIGHT_DECAY=0.5`
- 기존 `n_action_steps=15`, `queue_threshold=9`, 30 Hz 유지

Live generation이 바뀌면 action queue, temporal history, noise generation이
분리된다. MUX, Safety Guard, ServoL, ACT 경로는 변경하지 않았다.

검증 결과:

- Diffusion 전용 단위 테스트: `11 passed`
- ACT async, camera adapter, plugin 회귀 테스트: `40 passed`
- Python/Bash 문법 검사 및 `git diff --check`: 성공
- `ruff`는 현재 `lerobot-diffusion` 가상환경에 설치되어 있지 않아 미실행
- 빌드 후 Thor 합성 관측 5초 benchmark: action 151개, steady underrun 0건,
  stale action/inference 0건, ensemble 적용 tick 101개,
  최대 contributor 3개, 마지막 inference 174.7 ms

초기 CUDA warm-up 중 29 tick 늦은 chunk 한 개가 기존 delay-expired guard에 의해
폐기되고 startup underrun 1건이 기록됐지만, 첫 정상 chunk 이후에는 queue
고갈이 없었다.

다음 검증은 먼저 dry-run으로 로그의
`noise_correlation=1.000`, `ensemble_history=3`,
`ensemble_ticks`, `contributors`를 확인하고, 이후 bounded 10초와 40초
Live에서 축별 방향 반전율과 Streamer step 포화 횟수를 이전 trace와 비교한다.
