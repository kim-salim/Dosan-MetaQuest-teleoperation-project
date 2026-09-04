# A0509 T1~T6 Interior-Policy Dijkstra 검증 보고서

- 작성일: 2026-08-27
- 대상 repository: `/home/rvlab/Dosan-MetaQuest-teleoperation-project`
- 검증 기준: 현재 working tree와 현재 local dataset/model/artifact
- 실제 로봇 동작: 수행하지 않음
- 최종 판정: **LEVEL 1 — SYMBOLICALLY_PLANNABLE**

## 1. 결론

현재 T1~T6 whole-task ACT의 semantic interval을 operator로 표현하면 다음 unseen composite task는 deterministic uniform-cost search로 계획할 수 있다.

```text
drawer를 연다
→ floor의 blue block을 집는다
→ drawer 안에 넣는다
→ drawer를 닫는다
```

Controlled mode와 T1~T6 full catalog mode 모두에서 최저비용 계획은 다음과 같았다.

```text
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer
```

연속된 같은 policy를 묶은 policy sequence는 다음과 같다.

```text
T4 → T2 → T4
```

마지막 두 operator는 같은 T4 visit 안에서 다음과 같이 연속 실행하는 표현이 가능하다.

```text
T4.deliver_to_drawer_and_close
```

다만 현재 working tree에는 다음 두 exact edge의 planner-reviewed episode manifest가 없다.

```text
T4.open_drawer → T2.acquire_from_floor
T2.acquire_from_floor → T4.deliver_to_drawer
```

두 edge 모두 현재 V2 code 구조에는 연결 가능하지만, exact Bridge hard-filter와 saved-observation ACT prefix shadow가 완료되지 않았다. 따라서 이번 결과는 LEVEL 1이며, LEVEL 2 또는 LEVEL 3으로 표현하지 않는다.

## 2. 검증 범위와 안전 원칙

이번 작업에서 수행한 것은 다음뿐이다.

- static source inspection
- T1~T6 `tasks.parquet` / `info.json` 검증
- semantic v3 JSON/CSV/NPZ checksum 및 schema 검증
- raw parquet에서 episode별 gripper event/frame 재검출
- ACT checkpoint config와 weight 파일 존재 검증
- symbolic Dijkstra/UCS
- symbolic replay
- current V2 source의 AST 기반 command-free interface audit
- unit/integration/regression tests

다음은 수행하지 않았다.

- Live ON
- MUX LEROBOT 선택
- ServoL command
- Doosan bringup 또는 real motion
- gripper Tool DO
- camera capture
- CUDA inference
- physical collision/IK 검증

새 planner package에는 ROS publisher, ROS service client, robot connection, camera, GPU worker가 없다. Dijkstra는 30 Hz loop에 포함되지 않는다.

## 3. 실제 T1~T6 data/model/artifact 확인

모든 dataset은 LeRobot v3, 30 Hz, `observation.state=13D`, `action=7D`, front/side/zed RGB 구조였다. 모든 checkpoint는 ACT, `chunk_size=100`, `n_action_steps=100`이고 input/output contract가 동일했다.

| Task | tasks.parquet의 실제 TASK_DESCRIPTION | Episodes | Frames | Semantic segments | Checkpoint |
|---|---|---:|---:|---:|---|
| T1 | `Open the White Container throw away the blue block` | 30 | 36,000 | 5 | `...t1.../050000/pretrained_model` |
| T2 | `bring the blue block to black table` | 31 | 27,899 | 3 | `...t2_table.../040000/pretrained_model` |
| T3 | `Move the blue block off the black table` | 30 | 27,000 | 3 | `...t3.../040000/pretrained_model` |
| T4 | `Open the drawer, pick up the blue block, place it inside the drawer, and then close the drawer` | 30 | 54,000 | 5 | `...t4.../080000/pretrained_model` |
| T5 | `Open the drawer, take out the blue block, place it on the black table, and close the drawer.` | 30 | 54,000 | 5 | `...t5.../080000/pretrained_model` |
| T6 | `Stack the blue block on top of the other blue block on the black table.` | 30 | 27,000 | 3 | `...t6.../050000/pretrained_model` |

전체 absolute path, schema, checksum, feature shape는 machine-readable result의 `artifact_audit.tasks`에 저장했다.

### 3.1 Semantic artifact의 증거 수준

각 v3 artifact는 다음을 명시한다.

```text
semantic_information_only=true
robot_executable=false
individual_episode_selected=false
representative=component median XYZ at each semantic phase
```

그러므로 component median 자체를 physical execution guarantee로 취급하지 않았다. 대신 `standardized_semantic_phase_trajectories.npz`의 episode별 support를 검증했다.

```text
각 segment:
episode_ids             [N]
episode_xyz_mm          [N, 101, 3]
median_xyz_mm           [101, 3]
covariance_mm2          [101, 3, 3]
MAD / p50 / p90 / p95
```

T2는 N=31, 나머지는 N=30이었다. semantic JSON과 phase NPZ의 `checksums.sha256`도 모두 일치했다.

### 3.2 Physical setup override

원본 semantic artifact는 수정하지 않았다.

| Task | 원본 artifact | Planner metadata | Authority |
|---|---|---|---|
| T2 | `blue_block=source_region` | `blue_block_location=floor` | operator-confirmed current setup |
| T3 | `blue_block=off_black_table` | `blue_block_location=floor` | operator-confirmed current setup |

이 두 mapping은 `physical_setup_override`로 별도 저장되며, source artifact를 덮어쓰지 않는다.

## 4. Operator catalog

Graph node는 T1~T6 policy가 아니라 immutable `WorldState`다. T1~T6의 interior semantic interval이 action/operator다.

총 17개 operator를 등록했다.

```text
T1.open_white_container
T1.acquire_from_black_table
T1.deliver_to_white_container

T2.acquire_from_floor
T2.deliver_to_black_table

T3.acquire_from_black_table
T3.deliver_to_floor

T4.open_drawer
T4.acquire_from_black_table
T4.deliver_to_drawer
T4.close_drawer

T5.open_drawer
T5.acquire_from_drawer
T5.deliver_to_black_table
T5.close_drawer

T6.acquire_moving_block
T6.stack_on_support_block
```

### 4.1 Artifact-derived interval과 raw frame 분포

아래 frame은 고정 실행 frame이 아니다. 각 episode의 raw gripper event를 다시 검출하고 Cartesian arc-length phase로 boundary를 찾은 결과의 `min / median / max`다.

| Operator | Semantic span | Entry→Exit phase | Start frame min/med/max | End frame min/med/max |
|---|---|---|---:|---:|
| T1.open_white_container | S1,S2 | S1@0.00→S2@1.00 | 0/0/0 | 396/472/570 |
| T1.acquire_from_black_table | S3 | S3@0.00→1.00 | 396/472/570 | 600/725/794 |
| T1.deliver_to_white_container | S4 | S4@0.00→1.00 | 600/725/794 | 859/1023/1112 |
| T2.acquire_from_floor | S1,S2 | S1@0.00→S2@0.50 | 0/0/0 | 386/448/562 |
| T2.deliver_to_black_table | S2 | S2@0.50→1.00 | 386/448/562 | 582/685/818 |
| T3.acquire_from_black_table | S1 | S1@0.00→1.00 | 0/0/0 | 218/243/330 |
| T3.deliver_to_floor | S2 | S2@0.00→1.00 | 218/243/330 | 540/602.5/738 |
| T4.open_drawer | S1,S2 | S1@0.00→S2@1.00 | 0/0/0 | 395/495.5/634 |
| T4.acquire_from_black_table | S3 | S3@0.00→1.00 | 395/495.5/634 | 683/824.5/945 |
| T4.deliver_to_drawer | S4 | S4@0.00→1.00 | 683/824.5/945 | 960/1120.5/1251 |
| T4.close_drawer | S5 | S5@0.00→1.00 | 960/1120.5/1251 | 1799/1799/1799 |
| T5.open_drawer | S1,S2 | S1@0.00→S2@1.00 | 0/0/0 | 395/447.5/542 |
| T5.acquire_from_drawer | S3 | S3@0.00→1.00 | 395/447.5/542 | 645/763.5/823 |
| T5.deliver_to_black_table | S4 | S4@0.00→1.00 | 645/763.5/823 | 951/1058.5/1158 |
| T5.close_drawer | S5 | S5@0.00→1.00 | 951/1058.5/1158 | 1799/1799/1799 |
| T6.acquire_moving_block | S1 | S1@0.00→1.00 | 0/0/0 | 159/205.5/345 |
| T6.stack_on_support_block | S2 | S2@0.00→1.00 | 159/205.5/345 | 522/585.5/762 |

### 4.2 T2 acquire prefix의 근거

`T2.acquire_from_floor`는 T2 전체 task가 아니다.

```text
S1: floor/source 접근→C1 grasp
+
S2: held/free-transport 구간의 phase 0.50까지
```

종료 boundary는 이미 T2→T3 V2 실증에 사용된 hard-filter-passed manifest에서 가져왔다.

```text
segment: S2
phase: 0.50
support episode: 27
support frame: 428
gripper: closed
held_object: blue_block
contact_mode: free_transport_assumed
```

따라서 `T2.acquire_from_floor`의 effect는 `block=held`이며 `block=black_table`이 아니다.

### 4.3 Contact skill을 쪼개지 않은 구간

T4 drawer open은 다음 두 segment를 하나의 operator로 묶는다.

```text
S1 approach/grasp drawer handle
→ S2 manipulate drawer open/release handle
```

T4 close는 artifact가 정의한 S5 `close_drawer_and_retract_with_open_gripper` 전체를 유지한다.

## 5. WorldState

`WorldState`는 frozen dataclass이며 hashable하다.

```text
drawer
white_container
blue_block_location
holding
gripper
contact_mode
support_blue_block_location
stack

cursor_t1 ... cursor_t6
last_policy
```

허용 값은 명시적으로 검증한다. `unknown`은 wildcard가 아니며 concrete precondition을 만족하지 않는다.

Operator effect는 명시한 field만 변경한다. 나머지는 frame axiom으로 유지한다. 예를 들어 T4 drawer open은 initial block이 floor인 상태에서 block location을 black table로 바꾸지 않는다.

## 6. Forward-only ACT 제약

각 policy operator는 `entry_order < exit_order`를 갖는다.

```text
T4.open:     10 → 20
T4.acquire:  20 → 30
T4.deliver:  30 → 40
T4.close:    40 → 50
```

적용 규칙은 다음과 같다.

```text
operator.entry_order < current_policy_cursor
→ reject
```

따라서 다음은 허용된다.

```text
T4.open (cursor T4=20)
→ T2.acquire
→ T4.deliver (entry 30 >= cursor 20)
```

반면 다음은 차단된다.

```text
T4.deliver (cursor T4=40)
→ other policy
→ T4.open (entry 10 < cursor 40)
```

## 7. Dijkstra / Uniform-Cost Search

외부 graph library 없이 `heapq`를 사용했다.

```text
node: WorldState
edge: InteriorPolicyOperator
goal: partial state predicate
```

Cost는 아직 physical success probability를 가장하지 않는다.

```text
operator base cost = 1.0
policy switch penalty = 0.25
same-policy continuation penalty = 0.0
Bridge cost hook = interface만 제공, 현재 0.0
```

Deterministic tie-break 순서는 다음이다.

```text
total cost
→ plan length
→ operator-id tuple
```

### 7.1 Initial state와 goal

```text
initial:
  drawer=closed
  white_container=closed
  block=floor
  holding=none
  gripper=open
  contact=free_space

goal:
  drawer=closed
  block=drawer
  holding=none
  gripper=open
```

### 7.2 Exact result와 cost

| Step | Operator | Base | Switch | Step cost | Cumulative |
|---:|---|---:|---:|---:|---:|
| 1 | T4.open_drawer | 1.00 | 0.00 | 1.00 | 1.00 |
| 2 | T2.acquire_from_floor | 1.00 | 0.25 | 1.25 | 2.25 |
| 3 | T4.deliver_to_drawer | 1.00 | 0.25 | 1.25 | 3.50 |
| 4 | T4.close_drawer | 1.00 | 0.00 | 1.00 | 4.50 |

Full search는 238 states를 expand하고 358 states를 generate했다. Best total cost는 4.50이다.

## 8. Symbolic replay

```text
STEP 0
drawer=closed, block=floor, holding=none, gripper=open

STEP 1  T4.open_drawer
drawer=open, block=floor, holding=none, gripper=open

STEP 2  T2.acquire_from_floor
drawer=open, block=held, holding=blue_block, gripper=closed

STEP 3  T4.deliver_to_drawer
drawer=open, block=drawer, holding=none, gripper=open

STEP 4  T4.close_drawer
drawer=closed, block=drawer, holding=none, gripper=open

GOAL SATISFIED
```

## 9. T5 alternative와 다른 task가 선택되지 않은 이유

### 9.1 T5 alternative

Full catalog에는 동일 비용 4.50의 대안이 실제로 존재한다.

```text
T5.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer
```

현재 cost에는 T4-open과 T5-open의 physical success 차이가 없다. Best plan이 T4를 선택한 이유는 operator ID lexical tie-break이며, T4가 물리적으로 더 좋다는 증거가 아니다.

또한 T5-open 학습 artifact는 `blue_block=in_drawer`, T4-open은 `blue_block=on_black_table` 장면이다. Target은 `block=floor`이므로 둘 다 visual-scene generalization이 필요하다. Symbolic contract에서는 drawer opening과 무관한 block location을 완화했지만, ACT physical success를 보장하지 않는다.

### 9.2 T1

White-container state를 바꾸지만 drawer goal predicate에는 기여하지 않는다. 모든 cost가 양수이므로 불필요한 T1 detour는 지배된다.

### 9.3 T3

T3 acquisition은 `black_table→held`, delivery는 `held→floor`다. Initial block은 floor이므로 바로 적용되지 않으며 목표 방향과 반대다.

### 9.4 T6

T6는 `moving_source`, support block, unassembled stack을 요구한다. Target initial state는 이를 확인하지 못하며 effect도 `stacked`이지 `drawer`가 아니다.

## 10. Current V2 integration audit

기존 V2 runtime은 수정하지 않았다. 실제 source에서 다음 mapping을 확인했다.

| 역할 | Current implementation |
|---|---|
| actual/ack snapshot과 runtime Bridge 준비 | `TaskCLiveV2Strategy._prepare_bridge_commit` |
| Bridge feasible 후 A queue invalidate | `TaskCLiveV2Strategy._begin_bridge` |
| precomputed actual-state Bridge | `bridge_runtime.instantiate_bridge_queue` |
| 30 Hz Bridge 소비 | `TaskCHandoffV2Coordinator.tick` |
| ACT-B async request/result | `AsyncSuccessorController` |
| fresh B prefix admission | `HandoffCompatibilityEvaluator.evaluate_prefix` |
| quintic soft crossfade | `soft_handoff.build_soft_handoff` |
| repeated policy visit T4→T2→T4 | `TaskCMultiLiveV2Strategy` + `MultiStagePlan` |
| external high-level stage request | `MultiStageCoordinator.try_request_next` |

현재 `config/realtime/task_c_t4_t2_t4_plan.example.yaml`도 T4→T2→T4 stage를 표현하지만, 두 `handoff_manifest`는 `/absolute/path/to/...` placeholder다.

### 10.1 Edge A — T4.open → T2.acquire

Symbolic state:

```text
gripper=open
holding=none
contact=free_space
```

판정:

```text
symbolic compatible                  PASS
source/successor semantic support    PRESENT
source/successor checkpoint          PRESENT
current V2 code components           PRESENT
external_planner authority           PRESENT
structural runtime mapping           PASS
exact edge episode manifest          MISSING
edge policy-prefix shadow            MISSING
runtime contract complete            NO
```

`BridgeRuntimeSnapshot.gripper_target`와 actual state 기반 queue는 open-gripper target을 표현할 수 있다. `external_planner` authority에서는 semantic mismatch가 runtime control authority를 갖지 않는다.

그러나 기존 offline `runtime_guarded` hard filter는 `free_transport`만 허용하고 `free_motion_assumed`를 reject한다. 따라서 이 edge는 external-planner 모드에서 별도 geometric/dynamic candidate validation을 거쳐야 한다. 코드 표현 가능성과 edge 검증 완료를 혼동하지 않는다.

현재 missing work:

```text
planner-reviewed T4 S2@1.0 → T2 S1@0.0 manifest
empty-gripper reposition candidate hard filter/dry run
saved-observation T2 fresh-prefix shadow
```

### 10.2 Edge B — T2.acquire → T4.deliver

Symbolic state:

```text
gripper=closed
holding=blue_block
contact=free_transport
```

판정:

```text
symbolic compatible                  PASS
T2 S2@0.50 source boundary           EXISTING/CERTIFIED
T4 S4 support bank                   PRESENT
current V2 code components           PRESENT
structural runtime mapping           PASS
exact T2→T4 manifest                 MISSING
T4 fresh-prefix shadow               MISSING
runtime contract complete            NO
```

이 edge는 기존 T2→T3 실증과 동일한 held/free-transport transition 종류이지만 successor가 T4 S4이므로 별개의 검증 대상이다. T2→T3 success를 T2→T4 success로 복사하지 않았다.

### 10.3 T4.deliver → T4.close

두 operator는 같은 policy의 forward continuation이다. 중간 V2 Bridge가 아니라 resident T4 session의 normal rolling refresh로 표현하는 것이 맞다.

## 11. LEVEL 판정

### LEVEL 1 — SYMBOLICALLY_PLANNABLE: PASS

- T1~T6 full catalog에서 plan 존재
- controlled mode exact T4→T2→T4
- symbolic replay goal 만족
- backward re-entry 차단

### LEVEL 2 — RUNTIME_CONTRACT_COMPATIBLE: 아직 보류

- V2 structural mapping은 두 edge 모두 PASS
- exact episode manifests가 없음
- exact Bridge validation/policy shadow가 없음

### LEVEL 3 — PHYSICALLY_VALIDATED: 주장하지 않음

이번 작업에서 robot을 움직이지 않았다.

## 12. 테스트 결과

새 test:

```bash
pytest -q src/lerobot_robot_doosan_a0509/test/test_interior_policy_planner.py
```

```text
21 passed
```

Task-C V0/V1/V2/multi-stage + planner 회귀:

```text
205 passed
```

LeRobot A0509 package 전체:

```text
218 passed
```

필수 test에는 다음이 포함된다.

- full-catalog target plan
- controlled exact plan
- goal replay
- T4 forward re-entry
- backward re-entry reject
- drawer-open precondition
- holding 상태 drawer-open reject
- T2 acquire prefix effect
- T3 방향
- irrelevant task exclusion
- no-plan condition
- T5 alternative
- frame axiom
- unknown non-wildcard
- actual artifact/checkpoint/checksum audit
- floor override/source preservation
- raw parquet frame interval reconstruction
- cost breakdown
- V2 exact-edge readiness 분류

## 13. 신빙성의 한계

다음 사실 때문에 symbolic success를 physical success로 확대 해석하면 안 된다.

1. ACT input은 13D state와 RGB이며 task description/semantic operator/phase token을 받지 않는다.
2. Whole-task ACT가 interior re-entry observation에서 의도한 subgoal을 선택한다는 보장은 없다.
3. T4/T5 drawer-open의 original block scene과 target floor scene이 다르다.
4. Held/contact state는 gripper event와 semantic annotation 기반이며 force sensing이 아니다.
5. Semantic NPZ는 XYZ support다. V2 candidate는 raw orientation을 quaternion으로 별도 사용해야 한다.
6. Exact T4→T2/T2→T4 edge의 IK/collision은 검사되지 않았다.
7. Existing semantic artifact 자체가 `robot_executable=false`다.
8. Fresh T2/T4 prefix가 Bridge tail과 compatible한지는 saved observation inference 전에는 알 수 없다.

## 14. 실제 로봇 검증 전에 필요한 최소 작업

순서는 다음이 적절하다.

1. T4 S2 exit와 T2 S1 entry의 real-episode phase index를 orientation/velocity 포함 형태로 생성한다.
2. `semantic_authority=external_planner`로 empty-gripper 후보를 만들되 workspace, ramp, velocity, acceleration, curvature, jerk 검사는 유지한다.
3. T2 S2@0.50 source와 T4 S4 support prototypes로 held-object 후보를 생성한다.
4. 각 edge에서 hard-filter-passed representative manifest를 선택한다.
5. `ik_checked=false`, `collision_checked=false`를 명시적으로 유지한다.
6. Saved RGB/state로 successor ACT prefix shadow를 실행한다.
7. first delta, rotation, prefix velocity, Bridge-prefix mismatch, prospective crossfade dynamics를 확인한다.
8. 두 manifest를 multi-stage plan template에 넣고 command-free dry-run을 통과시킨다.
9. 그 후에만 사용자가 수동으로 shadow→bounded single edge→bounded full sequence 순서의 physical test를 수행한다.

현재 exact manifest가 없으므로 이 보고서에는 곧바로 실행 가능한 Live command를 제공하지 않는다.

## 15. 산출물

- Catalog: `config/interior_policy/a0509_t1_t6_operator_catalog_v1.json`
- Contracts: `interior_policy/contracts.py`
- Artifact audit/catalog loader: `interior_policy/catalog.py`
- Dijkstra/UCS: `interior_policy/planner.py`
- Symbolic replay: `interior_policy/simulator.py`
- V2 command-free adapter: `interior_policy/v2_adapter.py`
- CLI: `scripts/run_a0509_interior_policy_symbolic_probe.py`
- Tests: `test/test_interior_policy_planner.py`
- Machine result: `docs/artifacts/interior_policy_symbolic_probe_t4_t2_t4_2026-08-27/symbolic_probe_result.json`

## 16. 재현 명령

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
source /opt/ros/jazzy/setup.bash
source ~/venvs/lerobot/bin/activate
source install/setup.bash

PYTHONPATH="$PWD/src/lerobot_robot_doosan_a0509:$PWD:$PYTHONPATH" \
python scripts/run_a0509_interior_policy_symbolic_probe.py \
  --output docs/artifacts/interior_policy_symbolic_probe_t4_t2_t4_2026-08-27/symbolic_probe_result.json
```

이 명령은 dataset/artifact/checkpoint를 읽고 symbolic search만 수행한다. ROS service, Live, MUX, robot command를 호출하지 않는다.
