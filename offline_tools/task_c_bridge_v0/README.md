# Task-C Cartesian Bezier Bridge V0

이 디렉터리는 두 개의 ACT imitation-learning task를 Cartesian XYZ
trajectory 수준에서 조합하는 offline optimizer와 ROS-independent dual-ACT
runtime dry-run coordinator를 제공한다.

현재 구현은 다음 경로를 생성한다.

```text
ACT-A demonstration 시작
-> A 물체 grasp (gripper close)
-> A 물체를 든 운반 구간에서 cut
-> gripper closed 상태의 velocity-matched cubic Bezier bridge
-> ACT-B demonstration의 closed 운반 구간에 entry
-> ACT-B의 남은 운반 및 release trajectory
```

핵심 목적은 가장 짧은 Bridge를 찾는 것이 아니다. 다음 전체 조합 경로가
가장 짧은 feasible candidate를 찾는다.

```text
Task C = A retained prefix + Bezier Bridge + B retained suffix
```

Math/optimizer/coordinator는 ROS, command MUX, ServoL을 import하거나
실행하지 않는다. 별도 backend만 local LeRobot ACT checkpoint를 inference하며
robot adapter에는 연결되지 않는다. 생성물은 항상 `dry_run_only=true`,
`robot_executable=false`이고 로봇 명령을 publish하지 않는다.

---

## 1. 현재 검증 데이터와 ACT 정책

현재 reference 실행은 다음 두 LeRobot dataset을 사용했다.

| 역할 | Dataset | Task |
|---|---|---|
| ACT-A | `a0509_blue_block_v1_20260810_170226` | Pick up the blue block on the table |
| ACT-B | `a0509_blue_block_v2_20260810_191044` | Pick up the blue block under the table |

THOR에서 확인된 대응 policy checkpoint는 다음과 같다.

```text
ACT-A:
/home/rvlab/lerobot_models/
act_a0509_blue_block_bs16_30k_20260810_181114/
030000/pretrained_model

ACT-B:
/home/rvlab/lerobot_models/
act_a0509_blue_block_v2_bs16_30k_20260810_200240/
030000/pretrained_model
```

두 정책의 공통 contract:

```text
policy type: ACT
control/data frequency: 30 Hz
n_obs_steps: 1
chunk_size: 100
n_action_steps: 100
temporal_ensemble_coeff: null
state dimension: 13
action dimension: 7
images: front, side, zed_rgb
```

Offline optimizer는 demonstration trajectory로 전체 cut/entry/duration을
탐색한다. Runtime layer는 두 checkpoint를 동시에 resident로 두고 ACT-B를
bridge 중 최신 observation에서 재-inference하며, postprocessed action chunk
속도로 bridge tail을 다시 생성한다. 실제 robot command 연결은 구현하지 않는다.

---

## 2. Dataset schema

로더는 field를 추측하지 않는다. `meta/info.json`에서 다음 정확한 schema를
검증한 뒤 Parquet을 읽는다.

```text
observation.state[0:6]
  joint_1_rad ... joint_6_rad

observation.state[6:9]
  tcp_x_mm, tcp_y_mm, tcp_z_mm

observation.state[9:12]
  tcp_o1_deg, tcp_o2_deg, tcp_o3_deg

observation.state[12]
  gripper_commanded_state
```

행의 시간/episode identity:

```text
episode_index
frame_index
timestamp
```

현재 알고리즘에서 사용하는 값:

- TCP XYZ position, 단위 mm
- timestamp, 단위 second
- gripper commanded state
- episode/frame identity

Orientation O1/O2/O3는 별도 copy로 보존하지만 candidate generation,
Bezier generation, feasibility, ranking 어디에도 사용하지 않는다.

Dataset에는 다음 explicit field가 없다.

- object holding sensor
- contact state
- force/torque
- grasp success
- semantic subgoal annotation
- obstacle geometry

따라서 현재 V0는 다음 가정을 명시적으로 사용한다.

```text
gripper closed => holding=true (assumption)
gripper open   => holding=false (assumption)
```

이는 관측 사실이 아니라 heuristic이다.

---

## 3. A와 B를 비대칭적으로 처리하는 이유

A cut과 B entry는 같은 질문이 아니다.

### A cut

```text
이 frame에서 ACT-A를 중단해도
A 물체를 계속 든 상태로 Bridge에 진입할 수 있는가?
```

### B entry

```text
A 물체를 든 채 이 frame의 Cartesian 상태까지 이동한 후
ACT-B의 남은 transport/release 동작을 이어갈 수 있는가?
```

B의 원래 접근과 grasp는 Task C에서 제거된다. A에서 잡은 물체를 B의
transport/release 구간으로 전달하는 것이 목적이다.

---

## 4. Gripper transition과 closed-transport window

한 episode에서 다음 event를 찾는다.

```text
close_frame: gripper open -> closed
open_frame:  gripper closed -> open
```

현재 candidate 범위:

```text
close_frame + 30 <= candidate_frame <= open_frame - 30
```

30 Hz에서 30 frame은 약 1초다.

- close 이후 30 frame 제외: grasp 명령 직후의 불안정/접촉 구간 회피
- open 이전 30 frame 제외: release descent 및 접촉 구간 회피
- candidate stride 30 frame: 약 1초 간격으로 closed transport 탐색

이 조건은 `config_closed_holding.json`의 다음 값으로 설정된다.

```json
{
  "anchor": "closed_transport",
  "start_offset_frames": 30,
  "end_offset_frames": -30,
  "stride_frames": 30,
  "expected_gripper": "closed"
}
```

low-speed local minimum은 candidate 생성에 사용하지 않는다.

---

## 5. Transport floor heuristic

closed frame이라고 모두 free-space transport는 아니다. grasp 직후와
release 직전에는 gripper가 닫혀 있어도 작업면에 가까울 수 있다.

episode별 임시 transport floor:

```text
transport_floor = max(grasp_tcp_z, release_tcp_z) + 50 mm
```

Endpoint hard filter:

```text
A candidate Z >= A transport_floor
B candidate Z >= B transport_floor
```

Bridge 전체 hard filter:

```text
min(Bridge Z) >= max(A transport_floor, B transport_floor)
```

Endpoint가 높아도 endpoint velocity 때문에 cubic curve 중간이 아래로
처질 수 있으므로 Bridge sample 전체를 검사한다.

`+50 mm`는 robot-certified clearance가 아니다. TCP 높이를 기준으로 한
dataset heuristic일 뿐이며, payload geometry, gripper transform, table 및
obstacle geometry를 반영하지 않는다.

---

## 6. Velocity estimation

Velocity는 후보 유사도를 평가하는 cost가 아니다. 다음 두 handoff의
속도 연속성을 구성하는 boundary condition이다.

```text
ACT-A trajectory -> Bridge
Bridge -> ACT-B trajectory
```

각 candidate 주변 15 frame의 `(timestamp, XYZ)`에 독립 선형회귀를 한다.

```text
x(t) ~= ax + vx*t
y(t) ~= ay + vy*t
z(t) ~= az + vz*t
```

결과:

```text
velocity = [vx, vy, vz]  # mm/s
```

단일 frame difference는 사용하지 않는다. 설정:

```json
{
  "velocity_window_frames": 15,
  "velocity_smoothing_method": "linear_regression",
  "velocity_epsilon": 1e-9
}
```

`estimate_local_velocity()`는 timestamped recent TCP history에도 사용할 수
있어 향후 runtime A velocity estimator로 재사용할 수 있다.

---

## 7. Velocity-matched cubic Bezier

A candidate:

```text
pA = A candidate Cartesian position
vA = A local Cartesian velocity vector
```

B candidate:

```text
pB = B candidate Cartesian position
vB = B local Cartesian velocity vector
```

duration `T`에 대해 control point를 다음처럼 만든다.

```text
P0 = pA
P1 = pA + (T/3)*vA
P2 = pB - (T/3)*vB
P3 = pB
```

Cubic Bezier:

```text
B(u) =
    (1-u)^3 P0
  + 3(1-u)^2 u P1
  + 3(1-u) u^2 P2
  + u^3 P3

0 <= u <= 1
t = T*u
```

실제 시간에 대한 endpoint 조건:

```text
B(0) = pA
B(1) = pB
dB/dt at t=0 = vA
dB/dt at t=T = vB
```

따라서 position과 velocity가 연속인 C1 handoff를 만든다. A/B velocity의
방향이 서로 달라도 cubic curve가 두 방향을 연결한다.

---

## 8. Duration search

`T`는 속도만 바꾸는 값이 아니다. P1/P2가 T의 함수이므로 곡선 형태와
가속도도 바뀐다.

현재 exhaustive search:

```text
T = 2.0, 2.5, 3.0, ..., 8.0 seconds
```

설정:

```json
{
  "bridge_duration_min_s": 2.0,
  "bridge_duration_max_s": 8.0,
  "bridge_duration_step_s": 0.5
}
```

---

## 9. Bridge sampling과 metric

`config_closed_holding.json`의 기본 offline metric sampling은 2026-08-18부터
30 Hz다. 이는 실제 downstream command rate와 같으며 전체 후보의 coarse
geometry/dynamics 평가에 사용한다. 2026-08-14 두 번의 physical PASS에 사용한
기존 `runtime_transition_manifest.json`은 60 Hz로 생성된 검증 기준이므로
덮어쓰지 않고 유지한다.

동일 코드, dataset, 376,467개 후보, 전체 JSONL 기록 조건의 controlled
benchmark:

| Sampling | Wall time | Feasible candidates | Selected candidate |
|---:|---:|---:|---|
| 60 Hz | 116.983 s | 253,633 | A ep5/frame312, B ep5/frame450, 3.0 s |
| 30 Hz | 100.923 s | 253,671 | A ep5/frame312, B ep5/frame450, 3.0 s |

30 Hz는 16.060 s, 13.7% 단축됐고 최종 candidate와 Top-K set은 유지됐다.
다만 60 Hz에서 curvature violation 65건과 payload-floor violation 2건으로
관측된 failure reason이 30 Hz sample에서는 사라졌으며, 그중 38개 후보가
infeasible에서 feasible로 바뀌었다. 따라서 30 Hz는 연속 경로 안전의 증명이
아니며 새 manifest는 고주기/adaptive 최종 검증 전까지 dry-run-only다.
실제 command tick의 ramp/guard contract는 별도로 30 Hz에서 계속 검사한다.

동일 initial runtime planner 입력을 100회 반복한 microbenchmark에서는
260 candidates/call 기준 평균이 60 Hz의 61.281 ms에서 30 Hz의 51.236 ms로
10.044 ms, 16.4% 단축됐다. p95는 각각 61.574 ms와 51.434 ms였으며 선택된
B entry와 3.0 s duration은 동일했다.

### Path length

```text
L_bridge = sum(||p[k+1] - p[k]||)
```

### Velocity

```text
v(t) = dp/dt
max_velocity = max(||v(t)||)
```

### Acceleration

```text
a(t) = d2p/dt2
max_acceleration = max(||a(t)||)
```

### Curvature

```text
kappa(t) = ||v(t) x a(t)|| / max(||v(t)||^3, epsilon)
```

### Jerk

```text
j(t) = d3p/dt3
```

Cubic Bezier의 jerk vector는 시간에 대해 상수다. 최대 jerk와
integrated squared jerk를 모두 기록한다.

### Backtracking

```text
backtracking_ratio = sampled_bridge_length / endpoint_chord_length
```

과도하게 돌아가는 곡선을 제거한다.

---

## 10. 현재 feasibility threshold

`config_closed_holding.json` 기본값:

| Metric | Limit |
|---|---:|
| maximum velocity | 300 mm/s |
| maximum acceleration | 300 mm/s^2 |
| maximum curvature | 0.25 /mm |
| maximum jerk | 800 mm/s^3 |
| integrated squared jerk | 1,000,000 |
| backtracking ratio | 2.5 |

Workspace는 두 dataset의 observed XYZ envelope에 축별 75 mm margin을 둔
분석용 범위다. robot workspace/IK 인증이 아니다.

Candidate failure reason 예:

```text
workspace_violation
velocity_limit
acceleration_limit
curvature_limit
jerk_limit
integrated_squared_jerk_limit
backtracking_limit
payload_clearance_violation
```

---

## 11. 전체 Task-C objective

각 `(A_i, B_j, T)` 후보에 대해:

```text
L_A_retained = length(A[0:i])
L_bridge     = length(Bezier(A_i, B_j, T))
L_B_retained = length(B[j:end])
```

최종 cost:

```text
L_C = L_A_retained + L_bridge + L_B_retained
```

삭제되는 다음 구간은 Task C에 들어가지 않는다.

```text
A[i:end]
B[0:j]
```

Primary selection:

```text
argmin L_C over all feasible (i, j, T)
```

따라서 가장 짧은 Bridge가 아니라 가장 짧은 최종 조합 task를 선택한다.

---

## 12. Near-shortest smoothness tie-break

먼저 strict shortest length `L_min`을 구한다. 다음 범위만 near-shortest로
인정한다.

```text
L_C <= (1 + 0.005)*L_min
```

즉 최단보다 0.5% 이내다. 이 후보들에서 다음 순서로 선택한다.

1. maximum acceleration이 작은 후보
2. maximum curvature가 작은 후보
3. integrated squared jerk가 작은 후보
4. total C length가 작은 후보

Smoothness는 primary length objective를 대체하지 않는다.

---

## 13. Reference dry-run 결과

closed-holding reference run (phase-local representative B velocity 적용):

```text
A candidates: 197
B candidates: 147
semantic pairs: 28,959
duration-expanded candidates: 376,467
feasible candidates: 253,633
representative-velocity B candidates: 147
```

선택 결과:

```text
A: episode 5, frame 312
  XYZ=[435.88, 178.07, 426.65] mm
  local velocity=[-18.40, -22.10, 82.60] mm/s

B: episode 5, frame 450
  XYZ=[444.65, -157.01, 412.31] mm
  7-episode representative velocity=[6.11, -7.77, -35.04] mm/s

Bridge duration: 3.0 s
A retained: 538.53 mm
Bridge length: 362.37 mm
B retained: 399.65 mm
Task-C total length: 1300.55 mm
Bridge Z range: 412.31 ... 467.67 mm
maximum velocity: 161.38 mm/s
maximum acceleration: 213.54 mm/s^2
maximum curvature: 0.1432 /mm
maximum jerk: 135.06 mm/s^3
```

실제 최단 bridge 후보는 42.76 mm지만 retained A/B 때문에 total C가
1675.12 mm다. 선택 후보의 bridge는 362.37 mm로 더 길어도 total C는
1300.55 mm이므로 전체 조합 경로 목적함수가 의도대로 우선된다.

---

## 14. Source files

| File | Responsibility |
|---|---|
| `trajectory_states.py` | Trajectory, semantic state, candidate model |
| `dataset_io.py` | Strict LeRobot schema validation and read-only load |
| `semantic_candidates.py` | A/B semantic windows and compatibility |
| `velocity_estimation.py` | Windowed Cartesian velocity regression |
| `bezier_bridge.py` | Cubic Bezier and analytic derivatives |
| `bridge_metrics.py` | Geometry/dynamics metrics and feasibility reasons |
| `bridge_optimizer.py` | Exhaustive pair/duration search and selection |
| `serialization.py` | Auditable candidate JSON records |
| `visualization.py` | 3D composition and dynamics plots |
| `run_dry_run.py` | End-to-end offline exhaustive CLI |
| `runtime_policy.py` | Independent async policy generations/queues and chunk velocity |
| `runtime_bridge.py` | Live-state B-entry/duration replanner and manifest loader |
| `runtime_orchestrator.py` | Fail-closed A/Bridge/B handoff state machine |
| `lerobot_act_backend.py` | Local ACT inference backend without robot I/O |
| `run_dual_act_dry_run.py` | Two-resident-model warmup/inference validation |
| `run_model_conditioned_bridge.py` | Inject real ACT-B velocity into runtime planner |
| `config_closed_holding.json` | Current closed-holding/runtime experiment contract |
| `config.json` | Older open/open comparison contract |
| `test_task_c_bridge_v0.py` | 21 offline unit/contract tests |
| `test_task_c_runtime.py` | 13 runtime unit/contract tests |
| `REPOSITORY_AUDIT.md` | ACT/dataset/MUX/runtime audit |

---

## 15. THOR path and environment

Copied source location:

```text
/home/rvlab/Dosan-MetaQuest-teleoperation-project/
offline_tools/task_c_bridge_v0
```

THOR dataset paths:

```text
/home/rvlab/lerobot_datasets/a0509_blue_block_v1_20260810_170226
/home/rvlab/lerobot_datasets/a0509_blue_block_v2_20260810_191044
```

THOR preflight on 2026-08-13:

```text
/home/rvlab/venvs/lerobot/bin/python:
  numpy: available
  pyarrow: available
  matplotlib: missing

/usr/bin/python3:
  numpy: available
  pyarrow: missing
  matplotlib: available
```

따라서 pure math/unit tests는 LeRobot venv에서 즉시 실행 가능하다. 전체
Parquet-to-PNG CLI는 동일 Python environment에 `numpy`, `pyarrow`,
`matplotlib`이 모두 있어야 한다. shared LeRobot 환경을 임의로 수정하지
않기 위해 이번 복사 작업에서는 package를 설치하지 않았다.

### Unit tests

Project root에서:

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project/offline_tools
/home/rvlab/venvs/lerobot/bin/python \
  -m unittest task_c_bridge_v0.test_task_c_bridge_v0 -v
```

### Full offline run

세 dependency가 모두 준비된 Python에서:

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project/offline_tools

python -m task_c_bridge_v0.run_dry_run \
  --dataset-a /home/rvlab/lerobot_datasets/a0509_blue_block_v1_20260810_170226 \
  --dataset-b /home/rvlab/lerobot_datasets/a0509_blue_block_v2_20260810_191044 \
  --config task_c_bridge_v0/config_closed_holding.json \
  --output /home/rvlab/lerobot_datasets/task_c_bridge_v0_closed_holding
```

주의: `config.json`은 이전 open/open 연결 비교용이다. A 물체를 들고
B transport로 연결하려면 반드시 `config_closed_holding.json`을 사용한다.

---

## 16. Output artifacts

Full run은 다음을 생성한다.

```text
all_duration_candidates.jsonl
  모든 semantic-compatible (A, B, T)의 metric과 failure reason

selected_candidate.json
  최종 선택 후보와 P0/P1/P2/P3, metric, provenance

top_k_candidates.json
  feasible 후보 중 total C length 기준 Top-K

selected_bridge_samples.npz
  sampled position/velocity/acceleration/curvature/jerk

selected_task_c_xyz.csv
  A retained + Bridge + B retained의 XYZ review trajectory

selected_task_c_3d.png
  A retained/removed, Bridge, B removed/retained 3D plot

selected_bridge_dynamics.png
  position, velocity, acceleration, curvature, jerk plot

dry_run_report.json
  dataset/config/candidate count/selection/safety summary

runtime_transition_manifest.json
  offline B-entry Top-K와 realtime planner/handoff contract

dual_act_inference_dry_run.json
  실제 두 ACT checkpoint의 resident warmup/fresh inference/queue isolation 결과

model_conditioned_bridge_report.json
  fresh ACT-B velocity로 재계획한 후보·Top-K·endpoint contract

model_conditioned_bridge_samples.npz
  model-conditioned position/velocity/acceleration/curvature/jerk samples

model_conditioned_bridge_3d.png
model_conditioned_bridge_dynamics.png
  실제 ACT-derived velocity를 적용한 path와 dynamics plot


checksums.sha256
  output integrity
```

CSV의 Bridge row:

```text
gripper_closed=1
holding_assumption=gripper_closed_implies_holding
payload_state=A_OBJECT_HELD_ASSUMED
orientation_status=pending
robot_executable=false
```

B release 이후 row는 `NO_PAYLOAD_ASSUMED`로 바뀐다.

---

## 17. Tests

현재 offline 21개 + runtime 13개, 총 34개 test가 다음 contract를
검증한다. Runtime suite는 measured/model velocity source, warmup output
폐기, A/B queue isolation, stale generation drop, ACT-B tail replan/final
refresh와 fail-closed no-XYZ도 포함한다.

- Bezier 시작/끝 위치 exact match
- Bezier 시작/끝 velocity match
- 서로 반대 방향 endpoint velocity 처리
- duration에 따른 control point/acceleration 변화
- 먼 XYZ가 semantic 이유로 제거되지 않음
- 가까운 XYZ라도 holding mismatch면 제거
- low-speed boundary 없이 candidate 생성
- orientation이 ranking에 사용되지 않고 원본 copy가 보존됨
- path length, velocity, acceleration, curvature, jerk
- workspace 및 acceleration rejection
- shortest Bridge가 아니라 shortest total Task C 선택
- feasible candidate가 없을 때 명시적 failure
- closed transport candidate의 holding/clearance contract

---

## 18. Implemented dual-ACT runtime design

Runtime 코드는 여전히 ROS-independent command-proposal layer지만 model
lifecycle, warmup, generation isolation, model-derived velocity와 bridge-tail
replanning까지 구현돼 있다.

Velocity source는 의도적으로 비대칭이다.

```text
A bridge start:
  최근 실제 TCP history의 causal linear regression
  (ACT-A action velocity는 diagnostic intent일 뿐 boundary가 아님)

Initial B bridge end:
  동일 semantic phase의 episode-balanced robust demonstration velocity

Final B bridge end:
  최신 관측으로 ACT-B를 새로 inference한 postprocessed XYZ action chunk
  (여러 step linear regression; single-frame difference를 쓰지 않음)
```

A에서는 controller lag/tracking error까지 반영한 실제 실현 속도가 물리적
handoff boundary다. B는 handoff 전 아직 실행되지 않으므로 freshly inferred
physical-unit action chunk가 terminal model intent다. Demonstration B
velocity는 초기 계획 seed/fallback이며 valid fresh chunk가 있으면 최종
runtime boundary로 사용하지 않는다.

ACT-B 진입 검사는 두 구간을 분리한다. 최신 observation pose에서 ACT-B의
첫 target까지의 거리는 `first target jump`로만 검사하며 이 공간 간격은
Bridge가 연결한다. ACT-B의 predicted speed와 terminal velocity는 첫 target을
포함한 연속 postprocessed B target 사이에서만 계산한다. 따라서 Bridge 간격을
ACT-B의 1 control tick 속도로 환산하지 않는다. 기존 first-jump, Bridge ramp,
predicted-speed 제한값과 fail-closed 동작은 그대로 유지한다.

구현된 state flow:

```text
NEW
  -> load ACT-A and ACT-B resident
  -> warm each model; discard every warmup output
  -> independent fresh ACT-A generation/queue
  -> semantic A cut
  -> invalidate and clear ACT-A queue
  -> initial bridge: measured vA + planned zero terminal velocity
  -> complete the validated bridge without a mid-Bridge executable replan
  -> hold the exact endpoint until actual TCP position and velocity settle
  -> capture a fresh endpoint observation and request ACT-B execution_refresh
  -> verify chunk pose, internal velocity, generation, and age
  -> prefer a feasible velocity-matched connector (suffix search <= 15 steps)
  -> otherwise use stopped direct when every XYZ axis fits one ramp tick
  -> otherwise plan the shortest feasible zero-to-zero alignment Bridge
  -> reach the exact accepted B target and atomically activate execution_refresh
  -> ACT-B running
```

ACT-B generation에는 명시적인 역할이 있다. Live endpoint-stop 모드에서는
mid-Bridge `tail_seed`를 만들지 않는다. 종점에서 actual TCP가 3 mm 이내이고 최근
causal TCP 속도가 15 mm/s 이하로 정착한 뒤 만든 `execution_refresh`만 실행 후보가
된다. 정착은 최소 0.1 s hold를 요구하며 3.0 s 안에 만족하지 못하면
`bridge_endpoint_settle_timeout`으로 fail-closed한다. Velocity-matched suffix가
선택되면 최대 15 action까지만 순서대로 검색하고 건너뛴 prefix는 queue activation
직후 내부적으로 소비되어 robot command로 발행되지 않는다. Direct 또는
zero-to-zero alignment fallback은 fresh chunk의 첫 target을 유지한다.

### V1 B-boundary fresh inference with alignment fallback (2026-08-19)

기본/V0에는 새 opt-in이 모두 false로 남는다. Representative V1 wrapper는
`b_moving_overlap_primary_enabled=false`와
`b_stopped_endpoint_position_bridge_enabled=true`를 명시한다. 따라서 ACT-B는
Bridge 주행 중이 아니라 대표 B entry에 정착한 뒤 최신 observation으로 한 번
추론된다. 모델은 계속 GPU resident/warm 상태지만 warmup 출력은 재사용하지 않는다.

```text
40 mm A prearm -> 20 mm A commit
  -> 검증된 zero-terminal Bridge를 대표 B entry까지 완주
  -> actual/acknowledged position과 causal TCP speed 정착 확인
  -> endpoint observation으로 ACT-B execution_refresh 추론
  -> velocity-matched connector가 가능하면 사용
  -> 아니면 max(abs(delta_xyz)) <= 6.67 mm일 때 stopped direct
  -> 그것도 아니면 shortest-feasible zero-to-zero alignment Bridge
  -> exact ACT-B first target에서 queue를 atomic 활성화
```

23:53 실증에서 mid-Bridge seed는 B 대표 entry보다 너무 일찍 추론되어 모든
0..15 suffix가 `act_b_entry_position_inconsistent`로 거절됐다. 기존 stop Bridge는
그대로 완주했고 endpoint actual error는 `0.041 mm`였다. Endpoint fresh ACT-B
chunk는 정상(`max_predicted_velocity=83.354 mm/s`)이었지만 first target delta가
`[-3.100, -3.935, -7.512] mm`라 최대 축 간격 7.512 mm가 6.67 mm/tick direct
한계를 0.842 mm 넘었다. 명령은 B에 한 건도 발행되지 않았고 Live/MUX는 즉시
false/`DISABLED`로 복귀했다.

같은 endpoint와 target을 command-free로 재계산하면 기존 한계를 모두 유지한
zero-to-zero alignment Bridge의 shortest-feasible duration은 0.6 s다. 길이
9.029 mm, 최대 vector/axis 속도 22.555/18.766 mm/s, 최대 가속도
150.478 mm/s^2, 최대 jerk 501.595 mm/s^3, 곡률은 사실상 0이며 최소 TCP Z
411.250 mm는 runtime payload floor 409.693 mm보다 높다. Workspace, payload
floor, ramp, velocity, acceleration, jerk, curvature, backtracking 기준은 하나도
완화하지 않았다.

Stopped direct의 `6.67 mm/tick` 검사는 실제 ServoL streamer와 같은
`max(abs(delta_xyz))` 기준을 사용한다. Alignment fallback은 direct threshold를
높이는 대신 기존 runtime planner로 모든 duration을 검사하고 통과 후보 중 가장
짧은 것을 고른다. `shortest_feasible` 선택은 이 정지 endpoint fallback에만
opt-in되며 기존 near-shortest/smoothest 선택은 다른 Bridge에 그대로 유지된다.

Endpoint generation이 stale/invalid이거나, full 6D·workspace·ramp·tracking 검증에
실패하거나, alignment 후보가 하나도 없으면 계속 fail-closed한다.

2026-08-19 실증
`task_c_representative_v1_endpoint_alignment_physical_20260819_003139`은 이
endpoint-boundary 구성으로 full semantic `COMPLETE`에 도달했다. 대표 A boundary는
31.891 mm에서 prearm, 14.618 mm에서 commit했고, 3.5 s initial Bridge 종점 actual
error는 0.054 mm였다. 0.268 s settle 뒤 fresh ACT-B `execution_refresh`의 첫 target
delta는 `[-1.963, -3.242, -3.055] mm`였다. Velocity-matched tail은 기존 dynamics
검사에서 거절됐지만 최대 축 간격 3.242 mm가 6.67 mm/tick 이하여서 stopped-direct
full 6D 검증 후 atomic B handoff가 성공했다. ACT-A/Bridge/ACT-B command 수는
368/133/372, guard/stream exact match는 207/208, intervention은 각각 0이었다.
Release authorize -> open command -> driver confirmation -> home settle 순서를 모두
확인했고 gate는 종료 시 Live false와 MUX `DISABLED`를 복원했다. 이번 자연 표본은
direct branch를 사용했으므로 0.6 s zero-to-zero alignment fallback의 physical 실행
성공을 별도로 주장하지 않는다.

Task-A gripper는 Live 이후 open 상태가 안정적으로 관측되고 ACT-A가 처음
`gripper_target >= 0.7`을 요청한 순간 closed로 latch된다. 이후 semantic cut까지
ACT-A의 Cartesian 6D target은 계속 사용하지만 gripper channel만 `1.0`으로
고정한다. 이는 pick policy가 close 직후 다음 chunk에서 reopen을 반복해 physical
closed 안정 조건을 영원히 깨는 것을 막는다. 기존 15-frame closed, 1.0 s
post-close delay, transport Z 조건은 완화하지 않으며 latch scope는 ACT-A에만
한정된다.

Live tail 교체의 XYZ 시작 상태는 lag가 있는 실제 TCP가 아니다. 직전 Bridge
command와 일치하는 fresh `/vr/commanded_posx`를 tick boundary에서 확인한 뒤
그 pose를 `p0`, 교체 전 Bridge의 같은 logical tick tangent를 `v0`로 사용한다.
새 tail의 첫 command는 최대 한 control tick만 전진하며 기존 6.67 mm/tick
ramp와 속도 검사를 그대로 받는다. 실제 TCP는 계속 ACT-B observation과
15 mm tracking-error guard에 사용된다. Shadow/offline처럼 downstream
acknowledgement가 없는 실행만 actual TCP와 causal measured velocity fallback을
사용한다.

`AsyncPolicySession`은 A/B별 policy reset state, executor, generation
counter, ready chunk, activation state와 queue index를 각각 소유한다. 공유
가능한 것은 simultaneous CUDA forward를 막는 GPU inference mutex뿐이다.
이미 invalidate된 in-flight result는 generation 검사에서 폐기되므로 다시
active queue로 들어올 수 없다.

`TaskCRealtimeCoordinator`는 fail-closed다. Timeout, missing observation,
stale/wrong-generation chunk, 과도한 first target jump, infeasible tail,
endpoint tracking error, empty B queue가 발생하면 XYZ가 없는 proposal을
반환한다. 모든 proposal은 다음 선언을 강제한다.

```text
orientation_status = pending
orientation_bridge_generated = false
robot_executable = false
dry_run_only = true
```

Coordinator는 ROS, command MUX, Tool I/O, ServoL, Doosan adapter를 import
하거나 호출하지 않는다. Proposal을 hardware command로 연결하는 작업은
V0 범위에서 의도적으로 제외돼 있다.

### Runtime files

| File | Responsibility |
|---|---|
| `runtime_policy.py` | Warmup discard, async fresh generation, stale drop, separate queue, chunk velocity |
| `runtime_bridge.py` | Live measured p/v에서 offline B-entry 전체 및 duration 재탐색 |
| `runtime_orchestrator.py` | Fail-closed A-to-Bridge-to-B state machine과 final refresh |
| `lerobot_act_backend.py` | Local LeRobot ACT load/preprocess/infer/postprocess; robot I/O 없음 |
| `run_dual_act_dry_run.py` | 두 checkpoint resident inference와 queue isolation 검증 |
| `shadow_latency.py` | Append-only JSONL과 p50/p90/p95/p99, stale/deadline summary |
| `task_c_shadow_rollout.py` | Live snapshot, virtual bridge, dual-session latency strategy |
| `task_c_shadow_entrypoint.py` | Context build 전 read-only CLI invariant 검사 |
| `scripts/run_task_c_shadow_latency.sh` | Default checkpoint/manifest 기반 shadow 실행 |
| `runtime_transition_manifest.json` | Offline B-entry Top-K와 runtime contract |
| `test_task_c_runtime.py` | Velocity source, generation isolation, replan, refresh, failure tests |

Runtime manifest의 각 B entry는 나중에 여러 task를 잇는 directed task graph
edge template으로 재사용할 수 있다. Pure planner와 policy-session isolation은
N-task composition에서도 그대로 유지한다.

### Dual-checkpoint dry-run

Repository root에서 실행한다.

```bash
PYTHONPATH=/usr/lib/python3/dist-packages \
/home/rvlab/venvs/lerobot/bin/python -m \
  offline_tools.task_c_bridge_v0.run_dual_act_dry_run \
  --checkpoint-a /home/rvlab/lerobot_models/act_a0509_blue_block_bs16_30k_20260810_181114/030000/pretrained_model \
  --checkpoint-b /home/rvlab/lerobot_models/act_a0509_blue_block_v2_bs16_30k_20260810_200240/030000/pretrained_model \
  --dataset-a /home/rvlab/lerobot_datasets/a0509_blue_block_v1_20260810_170226 \
  --dataset-b /home/rvlab/lerobot_datasets/a0509_blue_block_v2_20260810_191044 \
  --a-episode 5 --a-frame 312 \
  --b-episode 5 --b-frame 450 \
  --output /tmp/task_c_dual_act_inference.json
```

이 명령은 PyAV로 recorded camera를 decode하고 두 모델을 동시에 resident로
유지하며 warmup chunk 폐기, fresh inference, postprocessed model-intent
velocity 추정, queue isolation을 검증한다. Robot command를 publish할 수
없는 코드 경로다.

### 2026-08-13 verified dual-ACT result

Reference output directory:

```text
/home/rvlab/lerobot_datasets/task_c_bridge_v0_closed_holding_20260813
```

실제 두 checkpoint를 NVIDIA Thor에 동시에 resident로 유지한 결과:

```text
CUDA allocated/reserved: 443,122,688 / 576,716,800 bytes
model load: ACT-A 1.275 s, ACT-B 0.607 s, total 1.882 s
fresh inference: ACT-A 68.52 ms, ACT-B 63.98 ms
warmup: policy마다 2회 수행, 모든 warmup output 폐기
A/B temporal ensemble shared: false
A/B action queue shared: false
```

ACT-A의 fresh postprocessed chunk에서 얻은 intent velocity는
`[-9.120, -21.975, 101.181] mm/s`였다. 그러나 실제 A-to-Bridge 물리
경계에는 controller tracking 결과를 포함하는 최근 causal TCP history를
사용한다. 같은 recorded observation replay에서 이 값은
`[-22.679, -17.269, 93.102] mm/s`였다. ACT-A intent는 미리 계산된
demonstration velocity를 대체하는 runtime 진단/예측 정보지만, 실제 로봇이
도달한 속도와 다를 때 이를 물리 경계로 강제하지 않는다.

ACT-B는 handoff 전에 아직 실행된 TCP history가 없으므로 최신 관측으로
fresh inference한 15-step postprocessed action chunk의 회귀 속도
`[-7.010, -24.829, -37.492] mm/s`를 최종 Bridge terminal boundary로
사용한다. Offline 7-episode representative B velocity
`[6.105, -7.769, -35.038] mm/s`는 초기 Bridge seed일 뿐이며 fresh B
chunk가 도착하면 tail을 현재 measured `p/v`에서 다시 전수 탐색한다.

Model-conditioned selected bridge:

```text
B entry: episode 5, frame 450
duration: 3.0 s
bridge length: 366.742 mm
total-C estimate: 1304.921 mm
max velocity: 159.184 mm/s
max acceleration: 217.403 mm/s^2
max curvature: 0.070121 /mm
max jerk: 130.600 mm/s^3
endpoint position error: 0.0 mm
endpoint velocity error: <= 3.52e-14 mm/s
```

Bridge 중 ACT-B prime 또는 final refresh로 tail을 재생성할 때는 이미
실행된 Bridge arc length를 `committed_bridge_prefix_length_mm`로 누적해
`A retained + committed prefix + new tail + B retained`를 ranking cost로
사용한다. 따라서 재계획 후에도 이미 이동한 거리가 total-C에서 사라지지 않는다.


---

## 19. Live observation read-only shadow runner

`task_c_shadow` rollout strategy는 기존 A0509 rollout의 observation processor와
feature schema를 그대로 사용한다. 별도 `policy_shadow` robot mode는 ROS state와
camera subscription만 만들며 target, gripper, debug action, policy-ready publisher를
하나도 만들지 않는다. Strategy도 `send_action()`을 호출하지 않으며 accidental
call을 즉시 예외로 막는 instance guard를 추가로 설치한다.

실행 전제:

```text
ACT-A: build_rollout_context가 한 번 load한 resident model 재사용
ACT-B: 별도 LeRobotACTBackend/session으로 한 번 load
A/B shared state: GPU inference lock만 공유
A/B policy state/action queue: 공유하지 않음
warmup output: 전부 폐기
robot commands: 0
```

실행:

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
./scripts/run_task_c_shadow_latency.sh
```

기본 출력은 timestamp가 붙은 새 디렉터리에 생성된다.

```text
/home/rvlab/lerobot_datasets/task_c_shadow_latency_<timestamp>/
  shadow_latency.jsonl
  shadow_latency.summary.json
```

기존 파일은 덮어쓰지 않는다. 기본 100 trial은 p99를 최소 100 sample로
계산하기 위한 값이다. 100개보다 적은 distribution은 summary의
`p99_sample_count_sufficient=false`로 표시된다.

### 실제 observation과 virtual bridge의 구분

Shadow mode에서는 실제 Cartesian command를 보내지 않으므로 물리 robot이
Bridge를 따라가지 않는다. 대신 다음처럼 명시적으로 두 상태를 분리한다.

```text
ACT policy input:
  매 tick 실제 camera + joint/TCP/gripper observation snapshot

Coordinator A boundary:
  실제 timestamped TCP history의 causal window regression velocity

Coordinator Bridge tracking:
  selected Bezier를 완벽히 추종한다고 가정한 virtual Cartesian state

ACT-B boundary:
  실제 live snapshot에서 ACT-B가 fresh inference한 postprocessed chunk velocity
```

따라서 이 실행은 compute latency/deadline 및 ACT-B observation distribution
문제를 찾는 실험이지, 실제 Bridge tracking 성능이나 robot feasibility
검증이 아니다. JSONL에는
`planning_state_during_bridge=perfect_tracking_virtual_shadow`가 매 trial
기록된다.

### JSONL timing chain

ACT-B generation마다 다음 monotonic timestamp를 기록한다.

```text
camera/state capture begin
-> capture complete
-> observation processor complete
-> unbatched ACT snapshot ready
-> async snapshot copy complete
-> executor submitted
-> worker started
-> shared GPU arbiter acquired
-> backend preprocess + ACT inference + postprocess complete
-> ACT-B chunk assessment
-> velocity-matched bridge-tail exhaustive planning complete
-> ready/fail-closed decision
```

주요 flat metric:

```text
capture_duration_ms
policy_input_build_ms
act_a_gpu_wait_ms
act_a_backend_inference_ms
act_b_gpu_wait_ms
act_b_backend_inference_ms
act_b_capture_to_gpu_acquired_ms
act_b_inference_to_tail_planning_ms
tail_planning_ms
tail_planning_to_ready_ms
capture_to_ready_ms
observation_age_at_ready_ms
deadline_slack_ms
deadline_missed
stale
```

Deadline은 ACT-B prime 순간의 `remaining_bridge_s`로 정의한다. Ready 시각이
virtual bridge end보다 늦으면 `deadline_missed=true`이다. Stale은 ACT-B가 본
capture부터 terminal ready/fail-closed decision까지의 age가
`--strategy.stale_after_s`를 넘었는지로 계산한다. Source TCP/joint/gripper
topic age도 각 capture record에 별도로 남으며, adapter freshness gate에서
거절된 snapshot은 `capture_failure` JSONL record가 된다.
여기서 `gpu_wait`는 별도 worker가 shared inference lock을 얻기까지의
GPU-arbiter wait이다. CUDA device queue 및 kernel 시간은 synchronize된
`backend_inference_ms` 안에 포함된다. Summary는 inference snapshot stale rate와
별도로 `source_capture_failure_rate`도 기록한다.

### Semantic trigger와 현재 한계

Trial은 실제 observation의 gripper commanded state가 설정한 frame 수만큼
closed일 때만 시작한다. Runner가 gripper를 닫지는 않는다. 현재 holding,
contact, object state는 sensor observation이 아니라 기존 V0 가정이며 모든
record에 `semantic_assumption_active=true`가 남는다. 실제 ACT-B가 A scene을
보고 예측한 첫 target이 offline B entry와 맞지 않으면 tail은 임의 fallback을
선택하지 않고 `failed_hold`로 기록된다.


---

## 20. Robot execution 전에 반드시 필요한 항목

현재 V0 결과를 실제 로봇에 실행하면 안 된다. 최소한 다음이 추가돼야
한다.

### Orientation bridge

- 현재 O1/O2/O3를 무시함
- quaternion 변환 및 continuous orientation interpolation 필요
- angular velocity/acceleration limit 필요

### IK and joint feasibility

- 모든 sampled Cartesian pose의 IK 존재 여부
- joint position/velocity/acceleration limit
- singularity proximity

### Collision and payload sweep

- table/under-table/fixture geometry
- robot self-collision
- gripper collision
- A object geometry and grasp transform
- full swept-volume check

### Holding verification

- gripper encoder/current/width
- force/contact or vision-based grasp confirmation
- Bridge 중 payload loss detection

### ACT-B distribution shift validation

- B가 원래 B 물체를 잡은 영상 대신 A 물체를 잡은 상태를 관측함
- Bridge 종료 observation이 B training distribution 안인지 검증 필요
- dry-run inference와 shadow mode 검증 필요

### Runtime safety

- MUX source ownership
- Live gate
- heartbeat/freshness
- emergency stop and hold
- bounded bridge command rate
- operator confirmation

---

## 21. Current safety declaration

현재 결과의 의미:

```text
orientation_status = pending
orientation_bridge_generated = false
collision_status = NOT_CHECKED_WITH_PAYLOAD
ik_status = NOT_CHECKED
holding_state = assumed_true_from_closed_gripper
robot_executable = false
dry_run_only = true
publish_robot_commands = false
```

이 선언은 실제 runtime 통합 전까지 완화하면 안 된다.

---

## 22. One-sentence summary

> Generate every semantically compatible closed-holding A-cut/B-entry cubic
> Bezier bridge, enforce Cartesian endpoint velocity continuity and V0 dynamic
> feasibility, then select the shortest complete A-prefix + Bridge + B-suffix
> Task-C trajectory, with smoothness used only as a near-shortest tie-break.
---

## 23. Bounded live A → Bridge → B candidate (2026-08-13)

The command-producing candidate is deliberately split into three processes so
no policy process owns robot-output authority:

1. `scripts/run_task_c_control_bringup.sh` starts the real observation/control
   graph with MetaQuest endpoint/input nodes and GUIs disabled.  It defaults to
   `CONTROL_DRY_RUN=true`, MUX `DISABLED`, and Live OFF.  It never calls Prepare
   Robot, selects LeRobot, or enables Live.
2. `scripts/run_task_c_live_candidate.sh` loads ACT-A and ACT-B resident on the
   GPU and waits in `WAITING_FOR_LIVE`.  It cannot select the MUX or enable Live.
3. `scripts/run_task_c_live_gate.sh` is preflight-only by default.  It compares
   the latched runner contract with the live ROS parameters before acquiring
   MUX/Live authority.  Real output requires both `EXECUTE=1` and the exact
   `MOTION_AUTHORIZATION=I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION` token.

The Bridge remains Cartesian position streaming at 30 Hz through the existing
path:

`/control/lerobot/target_posx → MUX → /vr/target_posx → safety_guard →
/vr/safe_posx → ServoL RT streamer → Doosan controller`.

The effective Bridge feasibility domain is the intersection of the offline
manifest and the accepted live control configuration:

- workspace: `[276.883, -318.318, 179.842]` to
  `[581.556, 338.293, 600.000]` mm;
- vector speed: at most `300 mm/s` (manifest);
- each Cartesian axis: at most `6.67 mm/tick × 30 Hz = 200.1 mm/s`
  (streamer pass-through);
- orientation: at most `1 degree/tick` during the Bridge and within the
  90-degree geodesic safety envelope;
- ServoL horizon `0.1 s`, controller-derived velocity/acceleration, MUX LeRobot
  freshness `0.3 s`.

The runner rejects a Bridge before publication if either the safety guard would
clamp it or the streamer would ramp it.  Every Bridge command and the first
ACT-B command are checked against a fresh `/vr/commanded_posx` sample—the pose
actually sent by the ServoL streamer, not merely the previous policy target.
The gate verifies the MUX/guard/streamer topic wiring, observes raw target, safe
target, and commanded target during Bridge, and aborts after three consecutive
non-matching samples.  ACT-A and ACT-B continue using the existing streamer
ramp internally; only both transition boundaries are required to be
pass-through exact.

Recorded no-command validation on 2026-08-13:

- actual two-camera and ROS/TCP connection succeeded;
- ACT-A and ACT-B loaded and warmed resident on CUDA;
- Live remained false, MUX remained DISABLED, ServoL and gripper drivers were
  dry-run;
- runner event artifact:
  `/tmp/task_c_live_command_disabled_20260813/task_c_live_events.jsonl`;
- event counters recorded zero commands in every phase;
- offline replay with the actual checkpoint observation artifact produced an
  initial 3.0 s Bridge (max axis speed `161.317 mm/s`, Z
  `412.306–472.307 mm`) and a fresh-B 1.0 s exact tail (max axis speed
  `138.257 mm/s`, max orientation step `0.061 degrees`); both passed without
  downstream clamp/ramp intervention;
- the observed physical pose `[506.766, 229.579, 297.094, 0.676, 154.269,
  -0.180]` was outside the Task-A demonstration start envelope, so it is not a
  valid later motion-test start pose;
- physical A → Bridge → B execution was still deferred at the time of this
  command-disabled artifact.

The first bounded physical trial reached the semantic cut and published 17
Bridge commands, then failed closed before a target whose Y step would have
been `6.758 mm` against the `6.670 mm/tick` streamer limit.  Live was disabled,
MUX returned to `DISABLED`, and no ACT-B command was sent.  The measured cause
was wall-clock catch-up after a scheduling gap, not a guard or streamer clamp.
The live coordinator now advances Bridge path time by at most one 30 Hz tick
per acknowledged command and waits for a matching new `/vr/commanded_posx`
sequence before advancing again.  Inference timeout and stale checks continue
to use real monotonic time.

After restarting the physical bringup, a later bounded trial completed the
semantic cut and sent 411 ACT-A plus 62 Bridge commands.  The first ACT-B
generation was valid: first-target jump `55.592 mm`, internal predicted speed
`76.601 mm/s`, internal max-axis speed `65.592 mm/s`.  Its velocity-matched tail
was also feasible.  No ACT-B command was sent, however, because that tail had
started at the lagging actual TCP; the next Y command was `7.117 mm` from the
last acknowledged streamer command and exceeded the unchanged `6.670
mm/tick` limit.  The external gate returned Live OFF and MUX `DISABLED`.

That intermediate live splice anchored both `tail_seed` and
`execution_refresh` tails at the last matching `/vr/commanded_posx` with the
old Bridge tangent. The actual `7.117 mm` lag case remains a regression fixture
for atomic command-space splicing. Live endpoint-stop mode supersedes the
mid-Bridge executable splice: its initial Bridge ends at zero velocity, and the
only executable tail starts from the settled endpoint with zero `v0`.

A subsequent bounded trial confirmed the acknowledged-command initial hold and
sent 39 unmodified Bridge commands, but the actual TCP fell `15.149 mm` behind
the last command and tripped the unchanged `15.0 mm` hard tracking guard.  The
guard and ServoL streamer recorded zero interventions; this was command
progress outrunning physical tracking, not a clamp or ramp failure.

The live runner therefore uses one-command tracking backpressure for both the
Bridge and ACT-B queue.  It may compute exactly one next command, but does not
consume another coordinator tick or policy action until that command is sent.
The command is admitted only when current actual TCP to candidate error is at
most `14.0 mm` and `9.0 degrees`, preserving a `1.0` unit margin inside the
unchanged hard limits of `15.0 mm` and `10.0 degrees`.  While waiting, the
runner republishes the complete previous seven-element action—including its
gripper value—so the MUX's `0.3 s` LeRobot freshness contract remains valid and
the robot continues tracking a fixed Cartesian target.  JSONL records
`tracking_backpressure_started`, `tracking_backpressure_released`, wait cycles,
hold-command count, wait duration, and candidate error maxima.  Hard tracking,
workspace, per-tick ramp, stale-source, and transition timeouts remain
fail-closed and were not relaxed.

The 2026-08-13 22:52 bounded trial confirmed the gripper close latch and sent
406 ACT-A plus 101 Bridge commands with zero guard/streamer intervention. It
failed closed before ACT-B because the 0.5 s early-refresh Bridge tangent and
fresh ACT-B velocity were `168.5 deg` apart; all 117 single-cubic tail candidates
failed the unchanged curvature limit. The gate returned Live OFF and MUX
`DISABLED`; final robot state was `STANDBY`. This trial is the direct evidence
for the endpoint-stop design above, not a successful A-to-B completion claim.

That pre-success checklist required the robot at the Task-A demonstration
start envelope, a reset work cell, gripper open, a passing preflight-only gate,
and fresh motion authorization. The checklist was satisfied on 2026-08-14 and
the full path was then validated twice as described below.

### 2026-08-14 full physical A → Bridge → B validation: PASS

Two consecutive physical trials reached semantic `COMPLETE`:

- ACT-A grasp and close latch succeeded;
- the endpoint-stop Bridge reached the settled fresh-ACT-B tail;
- the Bridge-to-B command boundary was committed atomically;
- ACT-B performed five fresh-observation rolling queue refreshes in each run;
- ACT-B release was authorized inside the demonstration envelope, sent once,
  and confirmed by the gripper driver;
- the open latch remained active through the final home-envelope settle;
- raw-to-safe and safe-to-commanded Bridge interventions were zero;
- no failure or rejected event occurred;
- cleanup returned Live OFF and MUX `DISABLED`.

The first trial completed in 34.371 s with 417 ACT-A, 163 Bridge, and 410 ACT-B
commands. The second completed in 34.721 s with 424 ACT-A, 176 Bridge, and 408
ACT-B commands. ACT-B used 407 and 406 policy steps respectively. Both runs
needed five rolling refreshes and zero refresh queue-hold cycles. Final measured
TCP speeds were 4.256 and 4.148 mm/s, below the unchanged 15 mm/s completion
limit.

Bridge pass-through evidence was 193/193 exact guard/stream matches in the first
trial and 200/200 in the second, with zero intervention. Rolling refresh
request-to-completion latency was 75.088–90.014 ms and 70.310–84.736 ms.
Maximum observation ages were 102.197 ms and 135.029 ms, below the configured
500 ms fail-closed limit.

The current validated status is therefore **two consecutive full physical
successes for the fixed scene and runtime contract**, not merely a successful
handoff. This supersedes the earlier “physical completion not yet claimed”
status while preserving the failed trials above as regression evidence.

The complete comparison, release/final poses, event counts, latency summary,
artifact paths, before/after images, and scope limits are recorded in
[Task-C Full Live Validation — 2026-08-14](../../docs/task_c_full_live_validation_2026-08-14.md).

### Additive representative-boundary V1

The exhaustive V0 implementation and its validated manifests remain preserved.
The optional single-representative A/B, coarse-to-fine shortest-path planner
and 40/20 mm live boundary are implemented separately in
[task_c_bridge_v1](../task_c_bridge_v1/README.md). V1 cannot activate through
the V0 live defaults; it requires both a V1 manifest and an explicit strategy
flag.
