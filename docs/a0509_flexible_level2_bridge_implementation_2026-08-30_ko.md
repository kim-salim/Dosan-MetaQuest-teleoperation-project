# A0509 FLEXIBLE_LEVEL2 Bridge 구현 및 오프라인 검증 보고서

- 작성일: 2026-08-30
- 저장소: `/home/rvlab/Dosan-MetaQuest-teleoperation-project`
- 검증 범위: 정적 분석, 단위 테스트, command-free 오프라인 replay
- 실제 로봇 명령: 실행하지 않음
- Live ON / MUX LEROBOT / ServoL publish / Tool I/O: 실행하지 않음

## 1. 결론

기존 `STRICT_LEVEL2`는 그대로 유지하면서 opt-in 방식의
`FLEXIBLE_LEVEL2`를 추가했다.

새 모드는 30개 demonstration에서 얻은 Bridge를 실행해야 할 고정 궤적으로
간주하지 않는다. 해당 Bridge를 전체 이동 방향, 높이 경향, 자세 endpoint,
진행 순서를 제공하는 reference로 사용하고 다음과 같이 연결한다.

```text
ACT-A phase/support source collar
  → fresh actual/ACK snapshot
  → async bounded Bridge candidate search
  → newest ACK에서 entry 재검증
  → ACT-A queue invalidate
  → precomputed reference-guided Bridge 30 Hz 실행
  → handoff window
  → fresh ACT-B chunk 비동기 추론
  → bounded B[j] prefix 탐색
  → compatible B[j]와 soft crossfade
  → ACT-B 100% takeover 및 기존 rolling refresh

실패:
  → Bridge는 중단 없이 계속 실행
  → endpoint fallback 또는 fail-closed
```

T7의 drawer-top block acquire 이후 T3의 floor delivery로 직접 넘어가는 edge는
기존 STRICT에서는 900/900이 curvature 하나로 제거됐지만, FLEXIBLE의 오프라인
command-space 재평가에서는 900/900 episode-pair에 적어도 하나의 안전조건 통과
repair 후보가 존재했다.

다만 이 값은 물리 성공률이 아니다. nominal demonstration support state에서
precomputed command sequence가 현재 코드의 수치 제한을 통과한 비율이다.
ACT-B checkpoint shadow, 실제 actual/ACK residual, 물체 유지, IK 및 충돌은 아직
검증되지 않았다. 따라서 해당 edge의 상태는 `strict_verified`가 아니라
`flexible_semantic_candidate`이다.

## 2. 기존 T7→T3 direct edge가 사라진 정확한 이유

기존 offline evaluator는 각 source/successor support pair로 cubic Bézier를 만들고
`evaluate_bridge()` 결과를 `feasibility_reasons()`에 전달한다.

실제 제거 경로는 다음과 같다.

```text
30 T7 source supports × 30 T3 successor supports
  → 900 cubic candidates
  → evaluate_bridge
  → max curvature > 0.25 /mm
  → rejection_reason = ["curvature_limit"]
  → feasible = false
  → strict Level-2 registry에 direct edge가 없음
  → Dijkstra/UCS가 release detour를 선택
```

관련 source 위치:

- `offline_tools/cross_task_handoff/validate_handoff_candidates.py`
  - Bridge 평가: 약 278행
  - feasibility reason 결합: 약 288행
  - `feasible=not reasons`: 약 337행
- `offline_tools/task_c_bridge_v0/bridge_metrics.py`
  - `curvature_limit` 판정

현재 artifact 기준 STRICT 곡률 분포는 다음과 같다.

| 통계 | curvature `/mm` |
|---|---:|
| min | 0.3243439 |
| p50 | 1.3912492 |
| p95 | 2.9290597 |
| max | 5.6946252 |
| 기존 limit | 0.25 |

900개 모두 저장된 직접 탈락 reason이 `curvature_limit` 하나였다. semantic
diagnostic이나 acceleration이 이 900개 STRICT 후보를 직접 제거한 사유는 아니었다.

문제는 다음 곡률 식이 저속 endpoint에서 매우 민감하다는 점이다.

```text
kappa = ||p_dot × p_ddot|| / ||p_dot||^3
```

속도가 0에 가까워지면 분모가 세제곱으로 작아지므로, 실제 normal acceleration과
30 Hz command step이 작아도 sampled maximum curvature만 커질 수 있다.

## 3. STRICT와 FLEXIBLE의 차이

| 항목 | STRICT_LEVEL2 | FLEXIBLE_LEVEL2 |
|---|---|---|
| 기존 재현성 | 기존 동작 그대로 | 별도 opt-in |
| offline hard-pass 필요 | 필요 | reference-only 후보 허용 |
| 대표 Bridge 의미 | 실행할 nominal curve | 변형 가능한 reference |
| source 시작 | 기존 strict commit | actual/ACK에서 adaptive 시작 |
| candidate search | strict 한 개 | bounded priority search |
| curvature | global max hard reject | 저속 endpoint 및 moving p95 soft cost |
| cusp/reversal | 기존 검사 | hard reject |
| ACT-B 진입 | B[0] | bounded B[j] |
| planner edge | strict verified만 | semantic candidate도 보존 가능 |
| physical claim | dry-run artifact만 | 역시 physical claim 없음 |

`scripts/run_task_c_multi_stage_v2_candidate.sh`의 기본값은 여전히
`BRIDGE_ADMISSION_MODE=strict_level2`이다. FLEXIBLE은 명시적으로 선택해야 한다.

## 4. 대표 Bridge가 유지되는 부분과 변형되는 부분

### 4.1 도입부

fresh actual/ACK position, orientation, causal actual TCP velocity를 snapshot한다.
`ReferenceGuidedBridge`의 C2 quintic entry connector는 actual/ACK 경계조건에서
reference 중간부로 합류한다.

worker가 후보를 만드는 동안 ACT-A는 계속 실행된다. 후보가 준비됐다는 이유만으로
즉시 사용하지 않고, 가장 최신 ACK와 첫 command 사이의 position/orientation/
acceleration을 다시 검사한다. 이 rebase가 실패하면 ACT-A queue를 끊지 않는다.

### 4.2 중간부

대표 cubic의 중간부를 가장 강하게 유지한다. 기본 affine endpoint deformation이
command dynamics를 통과하면 reference shape RMS가 0인 후보가 우선될 수 있다.
그 후보가 실패하면 tangent-regularized, settle, alignment, lift-transport 순으로
repair한다.

### 4.3 종료부와 다른 ACT 추론 영역 진입

Bridge 생성 시에는 B demonstration support가 nominal attractor다. 실제 takeover는
해당 endpoint나 nominal B phase 도달로 결정하지 않는다.

handoff window에서 최신 RGB/state observation으로 ACT-B를 한 번 추론한 뒤, 같은
fresh 100-step chunk의 bounded initial indices를 검사한다.

기본 FLEXIBLE 설정:

```text
max splice index = 8
max candidates   = 6
prefix steps     = 15
crossfade steps  = 15
```

즉 B[0]만 맞추지 못해도 B[1..8] 중 제한된 후보를 비교할 수 있다. 각 B[j]에 대해
첫 XYZ/rotation 차이, prefix velocity, Bridge-tail velocity mismatch, 실제 생성될
crossfade의 command step과 command acceleration을 먼저 hard 검증한다. 통과 후보만
다음 score로 순위를 정한다.

```text
first XYZ distance
+ first rotation distance
+ 0.01 × Bridge/B velocity mismatch
```

이는 추가 GPU inference가 아니라 이미 받은 fresh chunk 내부의 작은 NumPy 연산이다.
선택된 `j`, candidate score, 모든 admission metric은 trace에 남는다.

이 기능의 범위는 “한 fresh chunk의 가까운 초기 추론 영역”이다. whole-task ACT에
semantic token이나 phase input이 없으므로 임의의 먼 semantic segment로 점프시키는
기능은 아니다. 더 먼 재진입은 해당 observation에서 ACT가 그 행동을 실제로 출력해야
하고, planner의 operator/phase supervisor contract도 별도로 충족해야 한다.

## 5. Candidate search 우선순위

bounded worker는 다음 순서로 후보 그룹을 평가한다. 앞 그룹에서 hard-safe 후보가
나오면 다음 그룹 전체를 불필요하게 탐색하지 않는다.

1. `reference_deformation`
2. `tangent_regularized_deformation`
3. `settle_reference_connector`
4. `alignment_reference_connector`
5. `lift_transport_reference_connector`
6. `generic_command_validated_ood`

generic connector는 기본적으로 꺼져 있고, IK/collision checker가 없는 현재 live
설정에서는 자동 활성화되지 않는다.

현재 tangent ratio 후보는 안정성 및 기존 구현 범위에 맞춰
`0.0, 0.05, 0.10, 0.20, 0.25`로 제한했다. duration multiplier는
`1.0, 1.25, 1.50`이다. 후보 수는 최대 64, search wall-time은 최대 0.50초이며
항상 single-worker에서 수행한다.

## 6. Hard condition과 soft condition

### 6.1 FLEXIBLE에서도 유지되는 command-space hard reject

- NaN/Inf
- workspace X/Y minimum 및 XYZ maximum
- velocity, axis velocity
- acceleration
- jerk 및 integrated squared jerk
- backtracking ratio
- cusp
- severe direction reversal
- sampled self-intersection
- 30 Hz axis position step
- quaternion 기준 orientation step
- reconstructed command acceleration/jerk
- bounded ACK pipeline의 2-step span
- fresh actual/ACK entry rebase
- ACT-B freshness 및 generation
- MUX/Safety/fail-closed 기존 경로

현재 offline validation 값:

| 항목 | 값 |
|---|---:|
| velocity | 300 mm/s |
| axis velocity | 200.1 mm/s |
| acceleration | 300 mm/s² |
| jerk | 800 mm/s³ |
| integrated squared jerk | 1,000,000 |
| backtracking ratio | 2.5 |
| position step | 6.67 mm/tick |
| orientation step | 1.0 deg/tick |
| ACK pipeline | 최대 1 target 선행 |

Crossfade command acceleration의 live 기본값 4000 mm/s²는 과거 command-free trace에서
도입한 provisional admission 값이며 두산 로봇의 공인 가속도 rating이 아니다.

### 6.2 soft cost / diagnostic

- representative shape RMS
- source support distance/OOD
- actual/reference velocity 차이
- 충분한 speed 구간의 curvature p95
- 저속 endpoint maximum curvature
- duration 및 path length
- 경미한 target/velocity/orientation mismatch
- B[j]별 fresh-prefix compatibility score

curvature threshold를 단순히 0.25에서 0.4로 올리지 않았다. 저속 maximum은 기록하고,
moving p95와 `a_normal = speed² × curvature`를 함께 해석한다. cusp와 실제 command
dynamics 위반은 계속 hard reject한다.

## 7. Planner 변경과 no-intermediate-release

edge registry v2는 다음 상태를 구분한다.

```text
strict_verified
flexible_semantic_candidate
temporarily_unavailable
```

STRICT planning은 `strict_verified`만 허용한다. FLEXIBLE planning은 명시적으로
등록된 `flexible_semantic_candidate`도 유지한다. runtime search가 실패하면 해당
transition은 실행되지 않으며 기존 fallback 또는 fail-closed로 간다.

`allow_intermediate_release=false`일 때 현재 goal과 다른 location에 block을 내려놓는
operator는 Dijkstra/UCS transition admission에서 거부된다. 따라서 direct T7→T3를
요청한 계획이 실패했다고 black table release detour를 자동 실행하지 않는다.

## 8. 30 Hz non-blocking 구조

```text
30 Hz control thread:
  prepared queue pop
  bounded B[j] compatibility/crossfade 연산
  target publish

Flexible Bridge worker:
  reference deformation
  bounded candidate generation
  full command-space validation

ACT-B inference worker:
  camera/state snapshot 처리
  GPU inference
  generation-tagged chunk 반환
```

`Future.result()`는 `future.done()` 확인 뒤 worker 결과를 가져올 때만 호출한다.
Bridge search가 pending인 동안 source ACT queue는 살아 있고, ACT-B가 pending인 동안
Bridge queue가 계속 소비된다.

single-edge와 multi-stage phase supervisor 모두 primary commit window보다 앞선
`prearmed` collar에서 FLEXIBLE worker를 poll/request한다. multi-stage의 다음 edge는
이때 미리 bind되지만 현재 active policy session과 queue는 유지된다. transition
mailbox 요청과 source queue invalidation은 persistence를 포함한 commit-ready와 최신
ACK revalidation을 모두 통과한 뒤에만 일어난다.

기존 50/100/200/500 ms artificial ACT-B latency 테스트는 모든 경우 Bridge command가
계속 나오고 coordinator tick maximum이 20 ms 미만임을 assert한다. 이는 synthetic
test 결과이며 실제 카메라/GPU p95/p99 측정값은 아니다.

## 9. T7→T3 900-pair before/after

입력 artifact:

```text
docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29/edges/
  t7_acquire_from_drawer_top__to__t3_deliver_to_floor/candidate_library.json
```

최종 평가:

| 항목 | 결과 |
|---|---:|
| episode pairs | 900 |
| STRICT feasible | 0 |
| STRICT curvature-only reject | 900 |
| FLEXIBLE command-safe pair | 900 |
| evaluated repair variants | 7,704 |
| hard-pass repair variants | 1,734 |
| variants per pair max | 15 |
| reference deformation selected | 483 |
| tangent regularized selected | 417 |
| selected duration | 전부 6.0 s |

search latency:

| 통계 | 시간 |
|---|---:|
| p50 | 30.07 ms |
| p95 | 157.05 ms |
| p99 | 157.60 ms |
| max | 158.83 ms |

이 latency는 prearm worker 시간이며 command tick latency가 아니다.

### 9.1 알려진 최소-curvature 후보 `h_6795bb74f713`

기존:

```text
source episode 12 → successor episode 22
legacy max curvature = 0.3243439 /mm
STRICT result = curvature_limit reject
```

FLEXIBLE repair 선택:

| 항목 | 값 |
|---|---:|
| generator | tangent_regularized_deformation |
| duration | 6.0 s |
| tangent ratio | 0.05 |
| max curvature | 0.5317200 /mm |
| curvature p95 at moving speed | 0.0112162 /mm |
| max curvature 위치 | u=0.00556 |
| 그 위치 speed | 2.0315 mm/s |
| max normal acceleration | 3.2679 mm/s² |
| max total acceleration | 67.4240 mm/s² |
| max command acceleration | 67.2583 mm/s² |
| max curve jerk | 477.0867 mm/s³ |
| max command jerk | 380.9919 mm/s³ |
| max axis step | 1.6928 mm/tick |
| max 2-step ACK span | 3.3856 mm |
| reference RMS deformation | 0.7656 mm |

max curvature 자체는 더 커졌지만 그것이 2 mm/s 부근 endpoint에서 발생한다. 실제
normal acceleration과 command dynamics는 limit 안이며 reference shape 변형도 작다.
이 사례가 global curvature maximum만 hard gate로 사용하면 잘못 제거할 수 있다는
핵심 증거다.

### 9.2 전체 library에서 선택된 reference `h_25132136158b`

- source episode 27 → successor episode 7
- runtime-selected repair duration: 6.0 s
- generator: `reference_deformation`
- reference RMS: 0.0 mm
- moving curvature p95: 0.1071506 /mm
- max curvature: 0.6716196 /mm at u=1.0, speed 4.5113 mm/s
- max normal acceleration: 33.7403 mm/s²
- max command acceleration: 33.5737 mm/s²
- max command jerk: 499.8851 mm/s³
- max axis step: 1.3587 mm/tick
- max 2-step ACK span: 2.7173 mm

manifest에는 원래 reference duration 4.0초가 보존된다. runtime search가 hard
command jerk를 만족시키기 위해 1.5배인 6.0초 후보를 선택한 것이다.

## 10. 생성한 artifact

- `docs/artifacts/t7_to_t3_flexible_level2_2026-08-30/evaluation.json`
  - 900 pair 및 실제 평가된 7,704 repair variant diagnostics
- `docs/artifacts/t7_to_t3_flexible_level2_2026-08-30/selected_flexible_reference_manifest.json`
  - 선택된 reference-only episode manifest
- `docs/artifacts/t7_to_t3_flexible_level2_2026-08-30/edge_registry_v2.json`
  - direct edge를 `flexible_semantic_candidate`로 등록

`evaluation.json`은 약 13 MB이며 각 candidate generator, tangent, duration,
curvature location/speed, dynamics, command step, ACK span, reference/OOD score,
hard rejection reason을 포함한다.

## 11. 수정 및 신규 파일

핵심 runtime:

- `task_c_handoff/models.py`
- `task_c_handoff/bridge_runtime.py`
- `task_c_handoff/flexible_bridge.py` (신규)
- `task_c_handoff/compatibility.py`
- `task_c_handoff/coordinator.py`
- `task_c_live_v2_rollout.py`
- `task_c_multi_live_v2_rollout.py`

Planner/web request contract:

- `interior_policy/multi_v2_compiler.py`
- `interior_policy/episode_control.py`

Config/tool:

- `scripts/run_task_c_multi_stage_v2_candidate.sh`
- `config/realtime/task_c_multi_stage_flexible_level2.yaml` (신규)
- `offline_tools/cross_task_handoff/evaluate_flexible_level2.py` (신규)

Tests:

- `test/test_task_c_flexible_bridge.py` (신규)
- `test/test_flexible_level2_planner.py` (신규)

## 12. 수행한 테스트

최종 회귀 명령:

```bash
source /home/rvlab/venvs/lerobot/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONPATH="src/lerobot_robot_doosan_a0509:.:$PYTHONPATH" \
python -m pytest -q \
  src/lerobot_robot_doosan_a0509/test \
  offline_tools/task_c_bridge_v0/test_task_c_bridge_v0.py \
  offline_tools/task_c_bridge_v0/test_task_c_runtime.py \
  offline_tools/task_c_bridge_v1/test_task_c_bridge_v1.py
```

결과:

```text
354 passed in 17.39s
```

포함 검증:

- STRICT 기존 동작 회귀
- high curvature / safe dynamics FLEXIBLE admission
- cusp 및 severe reversal hard reject
- command jerk hard reject
- actual offset adaptive entry
- multi-stage phase prearm 중 candidate worker 시작 및 source queue 유지
- stale ACK entry fail-closed
- Bridge worker non-blocking request/poll
- Bridge worker exception/stale result fail-closed
- B[0] 부적합, B[3] 적합 선택
- B[j] soft crossfade 후 실제 coordinator `RUN_B`
- ACT-B 50/100/200/500 ms 지연 중 Bridge 지속
- stale B / generation mismatch / timeout fallback
- FLEXIBLE planner direct T7→T3
- STRICT에서 flexible edge 오인 승인 금지
- no-intermediate-release detour 차단
- web/planner/multi-stage/source-trigger 회귀

`ruff`는 현재 venv에 설치되어 있지 않아 실행하지 못했다. 대신 수정 모듈
`py_compile`과 전체 pytest가 통과했다.

## 13. 아직 검증되지 않은 것과 위험

### 13.1 semantic/source collar 한계

선택된 T7 source artifact는 `gripper=closed`, `held_object=blue_block` precondition을
가지지만 contact label은 `contact_unknown_assumed`이고 semantic label도 실제
free-transport를 직접 센싱한 결과가 아니다. 현재 multi runtime은 사용자가 결정한
설계대로 `semantic_authority=external_planner`를 사용한다.

따라서 high-level planner가 semantic validity를 책임지고 Bridge runtime은 command
safety를 책임진다. 실제 held-object sensor가 없으므로 “블록을 정말 잡고 있다”를
runtime이 독립적으로 증명하지 못한다.

### 13.2 Z minimum / clearance

현재 validation config는 Z minimum을 비활성화했고 선택 manifest의
`transport_floor_mm`도 `null`이다. 이는 앞서 정한 실험 조건을 반영한 것이며,
collision clearance 보장을 뜻하지 않는다. lift connector는 Z를 올리는 기하학 후보일
뿐 환경 충돌 보장이 아니다.

### 13.3 B[j] 의미

B[j]는 fresh ACT 출력의 동역학적으로 연결 가능한 초기 index다. 올바른 subgoal을
선택했다는 semantic 보장은 아니다. 현재 whole-task ACT에는 task text, operator token,
semantic phase 입력이 없다.

### 13.4 물리 안전 미검증

- actual robot success
- 실제 camera/preprocess/GPU inference p95/p99
- actual/ACK residual을 포함한 source-collar success rate
- held object 유지
- object/environment collision
- IK feasibility
- 물리 acceleration/jerk 감소
- repeated-policy re-entry success

위 항목은 실제 bounded 검증 전까지 확인된 사실로 표현하면 안 된다.

## 14. 실제 로봇 적용 전 최소 검증 순서

1. 선택 manifest로 command-free T3 checkpoint shadow를 수행해 fresh B[j] admission을
   확인한다.
2. saved ACT-A actual/ACK trace 여러 개를 source snapshot으로 replay한다.
3. ROS graph를 Live OFF/MUX DISABLED 상태에서 dry-run하고 trace를 확인한다.
4. 사용자 수동 승인 후 5초 bounded motion으로 entry connector만 검증한다.
5. single Bridge와 gripper hold를 검증한다.
6. T7→T3 한 edge 전체를 검증한다.
7. 성공 trace를 검토한 뒤에만 multi-policy plan에 확대한다.

현재 구현 및 artifact만으로 `LEVEL 3 / physically validated`를 주장하지 않는다.
