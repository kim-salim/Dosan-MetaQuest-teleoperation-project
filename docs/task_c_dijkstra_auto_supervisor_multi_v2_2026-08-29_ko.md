# Dijkstra 기반 T4 → T2 → T4 자동 Multi-V2 개발 및 검증 보고서

작성일: 2026-08-29  
대상: `Dosan-MetaQuest-teleoperation-project`

## 1. 결론

현재 T1~T8 semantic operator catalog에서 다음 unseen task는 Dijkstra와 동일한 uniform-cost search로 계획되며, 기존 Task-C V2를 재설계하지 않고 현재 Multi-V2 runtime plan으로 컴파일할 수 있다.

```text
목표:
1. drawer 열기
2. floor의 blue block 집기
3. drawer 안에 넣기
4. drawer 닫기

operator plan:
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer

policy visits:
T4 → T2 → T4
```

검증 판정은 `LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE`이다. 심볼릭 goal 도달, exact semantic boundary handoff 존재, 실제 ACT checkpoint를 사용한 command-free prefix admission과 soft handoff까지 통과했다. 실제 로봇을 움직이지 않았으므로 `LEVEL_3_PHYSICALLY_VALIDATED`는 주장하지 않는다.

## 2. 이번 개발에서 확정한 원칙

### 2.1 자동 supervisor

사람이 각 policy 전환을 승인하지 않는다. 각 non-terminal policy visit은 actual TCP 기반 semantic phase/support supervisor가 전환 **요청**을 만든다. supervisor는 직접 명령을 publish하거나 ACT queue를 바꾸지 않는다.

```text
actual TCP/state
→ bounded phase/support tracking
→ gripper event latch
→ persistence 충족
→ transition mailbox 요청
→ 기존 V2 actual/ACK Bridge commit
```

그 뒤의 실행은 기존 V2 계약을 그대로 사용한다.

```text
ACT-A queue invalidate
→ actual/acknowledged state Bridge generation
→ precomputed Bridge queue 30 Hz 소비
→ handoff window
→ ACT-B async shadow inference
→ fresh B prefix admission
→ quintic soft crossfade
→ ACT-B takeover/rolling refresh
```

### 2.2 의미 판단과 파지 안정성

사용자 결정에 따라 Bridge가 물체 identity나 접촉 의미를 다시 판단하지 않는다. `semantic_authority=external_planner`로 기록하고, source/successor semantic 불일치는 후보의 진단 정보로 남긴다.

다만 자동 phase supervisor에서 쓰는 `closed_then_open`, `open_then_closed`는 유지한다. 이것은 파지 성공·힘·물체 유지 여부를 판정하는 안전기가 아니라, 초기 gripper 상태에서 phase가 너무 일찍 lock되는 것을 막는 최소 이산 이벤트 latch다.

### 2.3 Z minimum 비활성화

새 edge의 Bridge 검증에는 다음을 적용했다.

```text
workspace_min_limit_enabled = [true, true, false]
transport_floor_mm = null
```

Z 최소값과 별도 transport floor만 비활성화했다. 다음은 유지된다.

- X/Y workspace minimum
- X/Y/Z workspace maximum
- finite pose 검사
- Bridge 속도/가속도/jerk 및 orientation step 검사
- 기존 policy → MUX → safety guard → streamer → commanded/actual 경로
- Live/MUX 외부 gate 및 fail-closed

IK와 환경 충돌 검사는 현재 신뢰할 수 있는 구현이 없어 `false`로 명시했다. Z minimum 비활성화는 충돌 안전 보장을 의미하지 않는다.

### 2.4 최종 종료

final T4 visit에는 semantic completion box나 drawer-close detector를 추가하지 않았다. 현재 ACT checkpoint에는 학습된 done token이 없으므로, `policy_inference_end`는 설정된 inference/control horizon이 끝나는 시점이다.

현재 canonical plan:

```text
inference_end_steps = 1800
fps = 30
terminal horizon = 60 s
```

이는 “T4가 성공을 인지해 종료했다”는 뜻이 아니라 “final T4에게 60초 동안 연속 추론 권한을 준 뒤 종료한다”는 뜻이다.

## 3. 구현 구조

### 3.1 자동 source supervisor

신규 모듈:

```text
src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/
  task_c_handoff/stage_supervisor.py
```

주요 특성:

- semantic phase support artifact를 stage 시작 전에 preload
- 첫 lock은 vectorized global search
- 이후 이전 phase 주변 local search
- monotonic progression과 작은 backward tolerance
- nearest actual episode support 거리 사용
- median point는 진단용이며 mandatory waypoint가 아님
- 3 tick persistence
- ROS, 카메라, 파일 I/O, CUDA inference 없음

stage 계약:

| stage | boundary | phase window | gripper event |
|---|---|---|---|
| `visit_00_t4_open_drawer` | T4 S2 phase 1.00 | [0.95, 1.00] | closed → open |
| `visit_01_t2_acquire_from_floor` | T2 S2 phase 0.50 | [0.45, 0.55] | open → closed |
| `visit_02_t4_deliver...close` | terminal | 1,800 steps | 없음 |

supervisor가 ready가 되어도 Bridge를 control loop에서 탐색하지 않는다. 기존 V2 transition mailbox와 commit 경로가 actual/ACK snapshot에서 Bridge를 만들고 feasibility가 실패하면 기존 fallback/fail-closed 경로를 유지한다.

### 3.2 Multi-stage runtime 확장

수정 위치:

```text
task_c_handoff/multi_stage.py
task_c_multi_live_v2_rollout.py
```

추가 authority:

```text
phase_supervisor
policy_inference_end
```

기존 `external_planner`, `external_complete`는 하위 호환을 위해 남겼다. 같은 T4 checkpoint가 첫 visit과 세 번째 visit에서 다시 등장해도 runtime policy context를 재사용하도록 현재 Multi-V2 구조를 유지한다.

### 3.3 Dijkstra plan compiler

신규 모듈/CLI:

```text
interior_policy/multi_v2_compiler.py
scripts/compile_a0509_dijkstra_plan_to_multi_v2.py
```

compiler는 다음 순서로 fail-closed 검증한다.

1. 현재 T1~T8 catalog로 controlled UCS를 다시 실행한다.
2. 선택된 operator가 현재 catalog 객체와 정확히 같은지 확인한다.
3. 연속된 same-policy operator를 하나의 policy visit으로 묶는다.
4. 각 policy switch에 exact task/segment/phase handoff가 있는지 확인한다.
5. source gripper/holding contract를 확인한다.
6. `transport_floor_mm`가 남아 있으면 현재 구성에서는 거부한다.
7. 명시적 gripper event가 없으면 거부한다.
8. 실제 checkpoint를 쓴 command-free policy shadow가 takeover PASS인지 확인한다.
9. generation/age/prefix/fallback/deadline 조건이 통과해야 기존 Multi-V2 schema를 출력한다.

따라서 Dijkstra는 30 Hz thread에서 실행되지 않는다. episode 시작 또는 high-level replan 시점에만 plan을 만들고, 30 Hz thread는 준비된 queue에서 action 하나를 소비한다.

## 4. Dijkstra 결과

초기 상태:

```text
drawer=closed
white_container=closed
blue_block_location=floor
holding=none
gripper=open
contact_mode=free_space
```

goal predicate:

```text
drawer=closed
blue_block_location=drawer
holding=none
gripper=open
```

전체 T1~T8 catalog에서 선택된 최저 비용 계획:

```text
T4.open_drawer                 cost 1.00
T2.acquire_from_floor          cost 1.25  (policy switch +0.25)
T4.deliver_to_drawer           cost 1.25  (policy switch +0.25)
T4.close_drawer                cost 1.00  (same-policy continuation)
                              --------
total cost                     4.50
```

symbolic replay 최종 상태:

```text
drawer=closed
blue_block_location=drawer
holding=none
gripper=open
last_policy=T4
GOAL SATISFIED
```

forward-only cursor 때문에 `T4.open → T2 → T4.deliver/close`는 허용되지만, 늦은 T4 구간 실행 후 `T4.open`으로 되돌아가는 계획은 거부된다.

## 5. 실제 dataset 경계 후보

exact semantic phase만 사용해 episode support index를 만들고 4초 cubic Bezier 후보를 생성했다.

| edge | 후보 | hard-feasible | diversity 보존 |
|---|---:|---:|---:|
| T4 S2 phase 1.00 → T2 S1 phase 0.00 | 930 | 120 | 5 |
| T2 S2 phase 0.50 → T4 S4 phase 0.00 | 930 | 31 | 5 |

canonical representative:

### T4.open → T2.acquire

```text
handoff_id: h_454934a7c93e
source: T4 episode 22, frame 477
successor: T2 episode 21, frame 0
length: 245.364 mm
duration: 4.0 s / 120 targets
max velocity: 74.969 mm/s
max acceleration: 34.167 mm/s²
max orientation step: 0.167 deg/tick
```

### T2.acquire → T4.deliver

```text
handoff_id: h_dfa226e0e5fd
source: T2 episode 27, frame 428
successor: T4 episode 18, frame 945
length: 264.861 mm
duration: 4.0 s / 120 targets
max velocity: 84.040 mm/s
max acceleration: 95.673 mm/s²
max orientation step: 0.243 deg/tick
```

representative는 deterministic 기본 선택일 뿐 “가장 안전한 실제 경로”라는 의미가 아니다.

## 6. 실제 ACT checkpoint command-free shadow

두 successor checkpoint를 실제로 로드하고 저장된 RGB/state observation으로 inference했다. ROS/robot/MUX/Live는 사용하지 않았다.

| 지표 | T4→T2 | T2→T4 |
|---|---:|---:|
| terminal state | RUN_B | RUN_B |
| takeover | PASS | PASS |
| fallback | false | false |
| robot commands | 0 | 0 |
| first warmup | 441.92 ms | 403.09 ms |
| second warmup | 40.56 ms | 41.64 ms |
| first XYZ delta | 50.09 mm | 34.81 mm |
| first rotation delta | 1.52° | 4.95° |
| execution crossfade acceleration | 1803.57 mm/s² | 989.65 mm/s² |
| max crossfade XYZ axis step | 5.610 mm | 3.104 mm |
| control tick p99 | 3.042 ms | 2.938 ms |
| control tick max | 4.433 ms | 4.308 ms |
| deadline misses | 0 | 0 |

raw ACT prefix acceleration은 각각 1882.21, 1031.22 mm/s²이며 hard reject에는 사용하지 않는다. admission은 실제로 전송될 Bridge→crossfade command acceleration을 사용하고 현재 설정 4000 mm/s² 이하인지 확인한다.

warmup은 stage 실행 전에 policy context를 준비하는 비용이고, handoff-window inference는 async worker가 수행한다. control tick은 inference 완료를 기다리지 않고 Bridge queue를 계속 소비한다.

shadow의 제한:

- Bridge actual tracking을 perfect tracking으로 가정
- successor observation은 dataset에서 저장된 RGB/state
- IK 미검사
- 환경 collision 미검사
- 실제 물체/서랍 상태 변화 미검사

## 7. 30 Hz timing

실제 T4/T2 support bank에서 각 mode/stage 10,000회를 실행했다.

| supervisor | search | p50 | p95 | p99 | max |
|---|---|---:|---:|---:|---:|
| T4 S2 | global | 0.0811 ms | 0.0838 ms | 0.0876 ms | 0.1077 ms |
| T4 S2 | local | 0.0248 ms | 0.0256 ms | 0.0264 ms | 0.0574 ms |
| T2 S2 | global | 0.0840 ms | 0.0867 ms | 0.0908 ms | 0.1111 ms |
| T2 S2 | local | 0.0283 ms | 0.0289 ms | 0.0295 ms | 0.0609 ms |

이는 해당 Jetson의 command-free NumPy benchmark 결과이며 물리 rollout 전체 p99를 대신하지 않는다. 다만 phase tracker 자체가 33.33 ms tick을 blocking할 가능성은 현재 측정에서 매우 낮다.

## 8. 유지한 기존 불변조건

이번 변경에서 다음 코어는 재설계하지 않았다.

- ACT async successor worker 및 process-wide GPU arbitration
- ACT queue/generation isolation과 rolling refresh
- Bridge 전체 target 사전 생성 및 30 Hz queue 소비
- handoff window와 stale generation rejection
- fresh B prefix compatibility
- quintic soft crossfade와 quaternion orientation interpolation
- actual/acknowledged Bridge 시작 상태
- bounded ACK pipeline
- endpoint/reserve fallback
- MUX/Live/safety guard/ServoL command 경로
- gripper discrete command 및 hysteresis

30 Hz thread가 하지 않는 작업:

```text
Dijkstra search
전체 candidate 탐색
파일 읽기/쓰기
ACT CUDA 완료 대기
Thread.join/Future.result blocking
Bridge 전체 optimization
```

## 9. 생성된 canonical plan

기준 파일:

```text
docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/
  multi_stage_plan_shadow_validated.json
```

구조:

```text
LOAD T2, T4
  ↓
T4 visit: open_drawer
  └─ automatic S2 phase/support + closed→open latch
  ↓
actual/ACK Bridge 4 s
  ↓ async T2 prefix → admission → crossfade
T2 visit: acquire_from_floor
  └─ automatic S2 phase/support + open→closed latch
  ↓
actual/ACK Bridge 4 s
  ↓ async T4 prefix → admission → crossfade
T4 visit: deliver_to_drawer + close_drawer
  └─ 1,800-step inference horizon
  ↓
COMPLETE / external fail-closed shutdown
```

## 10. Fail-closed 상태와 실제 실행 전 남은 일

현재 두 episode manifest는 다음처럼 남겨 두었다.

```text
validation.dry_run_only = true
validation.robot_executable = false
validation.ik_checked = false
validation.collision_checked = false
```

이는 compiler/runtime 연결 실패가 아니라 물리 승인 상태의 구분이다. command-free 결과만으로 `robot_executable=true`를 자동 기입하지 않았다.

실제 연속 무인 policy 전환을 하기 전 최소 확인 항목:

1. 동일 물체/서랍 배치에서 command-free ROS shadow trace 확인
2. 각 edge별 Live OFF single-Bridge target/ACK 추적 확인
3. 각 edge별 bounded 5-second 물리 검증
4. T4→T2 단일 handoff 검증
5. T2→T4 단일 handoff 검증
6. 그 뒤 full T4→T2→T4 연속 실행

단계별 human transition approval는 최종 설계에 포함하지 않는다. 다만 전체 실험 시작과 비상 정지는 기존 외부 Live/MUX gate를 유지한다.

## 11. 테스트 결과

관련 회귀 테스트:

```text
93 passed in 7.41 s
```

포함 범위:

- T1~T8 Dijkstra/UCS planner와 symbolic replay
- forward re-entry / backward re-entry reject
- Dijkstra→Multi-V2 compiler fail-closed cases
- exact manifest 및 command-free shadow 필수 조건
- source phase/support tracker
- automatic stage supervisor
- terminal inference horizon
- async successor result routing/stale rejection
- handoff compatibility/crossfade/fallback
- multi-stage repeated policy visit/retry
- Z minimum axis mask

## 12. 재현 가능한 핵심 산출물

- 전체 판정: `docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/symbolic_probe_runtime_contract_2026-08-29.json`
- canonical runtime plan: `docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/multi_stage_plan_shadow_validated.json`
- edge registry: `docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/edge_registry.json`
- supervisor timing: `docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/phase_supervisor_timing_10000.json`
- T4→T2 shadow: `docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/policy_shadow_t4_to_t2_h_454934a7c93e.json`
- T2→T4 shadow: `docs/artifacts/task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/policy_shadow_t2_to_t4_h_dfa226e0e5fd.json`

작업 전 백업:

```text
/home/rvlab/project_backups/
  Dosan-MetaQuest-teleoperation-project_pre_auto_phase_supervisor_20260829.tar.gz
```

## 13. 최종 연구 해석

현재 결과가 증명하는 것은 다음이다.

```text
T1~T8 semantic operator catalog
→ Dijkstra/UCS symbolic plan
→ exact dataset boundary handoff
→ automatic phase-supervised policy visits
→ existing async V2 Bridge/handoff runtime plan
```

이 연결은 코드와 실제 checkpoint shadow 수준에서 성립한다. 즉 연구 가설은 구현 가능한 단계에 도달했다.

아직 증명하지 못한 것은 다음이다.

```text
실제 drawer/blue block 성공률
실제 RGB distribution에서 T4 재진입 안정성
실제 tracking p95/p99
실물 가속도/접촉 충격 감소
물체 유지
환경 충돌 안전성
final T4 horizon 안의 drawer-close 성공
```

따라서 현 시점의 정확한 결론은 **“심볼릭 계획과 현재 V2 runtime 계약 연결은 가능하며 command-free checkpoint shadow를 통과했다. 물리적 일반화 성공 여부는 다음 bounded 실증에서 검증해야 한다.”**이다.
