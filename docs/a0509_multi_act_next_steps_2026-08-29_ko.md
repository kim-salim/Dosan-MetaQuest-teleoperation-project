# A0509 Multi-ACT 일반화를 위한 현재 상태와 후속 구현 계획

- 작성일: 2026-08-29
- 대상 프로젝트: `Dosan-MetaQuest-teleoperation-project`
- 기준: 현재 working tree의 Dijkstra/UCS, semantic operator catalog, Task-C Multi-V2 구현
- 대표 목표: `T4.open_drawer → T2.acquire_from_floor → T4.deliver_to_drawer → T4.close_drawer`
- policy-level composition: `T4 → T2 → T4`
- 이번 분석 중 실제 로봇 동작: 수행하지 않음
- Live ON / MUX LEROBOT / ServoL / 그리퍼 Tool DO: 수행하지 않음

## 1. 결론

새로운 Multi-ACT runtime을 다시 만들 필요는 없다. 현재 필요한 핵심 작업은 다음 세 가지다.

```text
1. Dijkstra가 선택한 operator 경계를 실제 V2 Bridge manifest로 변환
2. 상위 플래너가 실제 semantic 완료를 확인하고 stage 전환을 요청하도록 연결
3. 두 edge를 shadow → 단일 edge → 전체 T4→T2→T4 순서로 검증
```

현재 상태를 계층별로 구분하면 다음과 같다.

| 계층 | 현재 상태 |
|---|---|
| T1~T6 semantic operator catalog | 구현 및 검증됨 |
| Dijkstra/UCS 계획 | `T4→T2→T4` 생성 성공 |
| Multi-V2 stage/queue/policy 재진입 | 구현됨 |
| T4 모델 재사용 | 모델 재로딩 없이 가능 |
| T4→T2 Bridge manifest | 아직 없음 |
| T2→T4 Bridge manifest | 아직 없음 |
| Dijkstra→MultiStagePlan 자동 변환 | 아직 없음 |
| 실제 effect 확인→stage 전환 supervisor | 아직 없음 |
| 전체 물리 검증 | 아직 없음 |

현재 판정:

```text
LEVEL 1: SYMBOLICALLY_PLANNABLE
```

다음 목표:

```text
LEVEL 2: RUNTIME_CONTRACT_COMPATIBLE
```

실제 반복 실험까지 성공한 뒤에만 다음을 주장할 수 있다.

```text
LEVEL 3: PHYSICALLY_VALIDATED
```

## 2. 최종적으로 필요한 전체 구조

현재 구현을 유지하면서 아래 연결부를 완성하는 것이 적절하다.

```text
사용자 목표
  "서랍을 열고 floor 블록을 집어 drawer에 넣고 닫기"
                    ↓
현재 WorldState 구성
                    ↓
Dijkstra / UCS
                    ↓
semantic operator plan
  T4.open_drawer
  T2.acquire_from_floor
  T4.deliver_to_drawer
  T4.close_drawer
                    ↓
[아직 필요한 부분]
Plan Compiler + Edge Manifest Registry
                    ↓
MultiStagePlan
  T4 visit #1
  T2 visit #1
  T4 visit #2
                    ↓
현재 구현된 Multi-V2
  ACT → Bridge → async successor → crossfade → ACT
                    ↑
상위 Effect Observer / Planner Supervisor
```

각 계층의 역할:

- Dijkstra: 어떤 semantic operator를 어떤 순서로 사용할지 결정한다.
- Edge registry: operator 사이를 연결할 수 있는 검증된 Bridge manifest를 제공한다.
- Multi-V2: 선택된 정책과 Bridge를 30 Hz로 실행한다.
- 상위 플래너: 실제 semantic 동작이 끝났는지 확인하고 다음 stage를 승인한다.
- ACT: 각 stage 안의 실제 연속 동작을 추론한다.
- Bridge: 서로 다른 정책의 상태를 기하학적·동역학적으로 연결한다.

Dijkstra가 직접 궤적을 생성하거나 30 Hz command를 보내는 구조가 아니다.

## 3. 현재 구현되어 있는 핵심 기반

### 3.1 Dijkstra / Uniform-Cost Search

현재 planner 위치:

```text
src/lerobot_robot_doosan_a0509/
  lerobot_robot_doosan_a0509/
    interior_policy/
      contracts.py
      catalog.py
      planner.py
      simulator.py
      v2_adapter.py
```

핵심 기능:

- immutable/hashable `WorldState`
- semantic manipulation 단위 `InteriorPolicyOperator`
- operator precondition/effect
- frame axiom
- T1~T6 policy별 forward-only cursor
- deterministic `heapq` uniform-cost search
- policy switch penalty
- symbolic replay
- 현재 V2 edge readiness 정적 검사

### 3.2 Multi-stage V2

현재 Multi-V2 위치:

```text
src/lerobot_robot_doosan_a0509/
  lerobot_robot_doosan_a0509/
    task_c_multi_live_v2_rollout.py
    task_c_multi_live_v2_entrypoint.py
    task_c_handoff/
      multi_stage.py
      policy_registry.py
```

핵심 기능:

- ordered stage list
- 동일 policy의 여러 visit 지원
- unique policy당 모델 한 번 load
- visit별 queue/generation isolation
- stage를 떠날 때 이전 queue invalidate
- stale inference result 차단
- 각 edge에서 기존 V2 coordinator 재사용
- external high-level planner가 stage cut authority 소유
- 최종 stage만 external completion 허용
- 실패 시 fail-closed

### 3.3 기존 V2 edge runtime

각 stage transition에서는 다음 기존 구조를 그대로 사용한다.

```text
actual/acknowledged snapshot
→ runtime cubic Bridge regeneration
→ precomputed Bridge queue
→ 30 Hz Bridge execution
→ handoff window
→ async successor inference
→ fresh prefix admission
→ quintic soft crossfade
→ successor ACT takeover
→ rolling refresh
```

## 4. T4→T2→T4의 정확한 semantic 경계

### 4.1 Stage 1: T4로 서랍 열기

사용 구간:

```text
T4.S1 approach_drawer_handle
→ T4.S2 manipulate_drawer_open
```

종료 경계:

```text
T4 S2 phase 1.0
anchor = O1
drawer = open
gripper = open
holding = none
```

이 시점에서 원래 T4는 검은 테이블 블록으로 접근하려고 하지만, 상위 플래너가 T4를 중단한다.

### 4.2 Edge 1: T4.open → T2.acquire

```text
source:
  T4 S2 phase 1.0

successor reference:
  T2 S1 phase 0.0
```

상태 의미:

```text
source:
  drawer open
  gripper open
  holding none

successor:
  floor block에 접근하기 전
  gripper open
  holding none
```

symbolic transition type:

```text
empty-gripper / free-space reposition
```

### 4.3 Stage 2: T2로 floor 블록 파지

```text
T2.S1 approach_blue_block_in_source_region
→ grasp
→ T2.S2 lift_transport_and_align...
```

종료 경계:

```text
T2 S2 phase 0.5
gripper = closed
holding = blue_block
contact = free_transport
```

T2가 검은 테이블에 블록을 내려놓기 전에 중단한다.

### 4.4 Edge 2: T2.acquire → T4.deliver

```text
source:
  T2 S2 phase 0.5

successor reference:
  T4 S4 phase 0.0
```

상태 의미:

```text
source:
  floor 블록 파지 완료
  블록을 들고 있음

successor:
  drawer가 열린 상태
  블록을 들고 drawer로 운반하기 시작하는 T4 구간
```

symbolic transition type:

```text
held-blue-block / free-transport
```

### 4.5 Stage 3: T4로 drawer delivery 및 close

```text
T4.S4 lift_transport_and_align_blue_block_in_drawer
→ release
→ T4.S5 close_drawer_and_retract
```

다음 두 operator는 같은 T4 visit 안에서 연속 실행한다.

```text
T4.deliver_to_drawer
→ T4.close_drawer
```

따라서 중간에 추가 Bridge를 만들지 않는다.

## 5. 실제 T2/T4 dataset 경계 후보 분석

이번 분석에서는 실제 T2·T4 dataset을 읽어 command-free로 각 episode support 조합을 검사했다.

```text
T4: 30 episodes
T2: 31 episodes

각 edge의 exact boundary pair:
30 × 31 = 930 pairs
```

사용한 boundary:

```text
Edge 1:
  T4 S2 phase 1.0
  → T2 S1 phase 0.0

Edge 2:
  T2 S2 phase 0.5
  → T4 S4 phase 0.0
```

현재 validation 설정과 4초 cubic Bezier Bridge 결과:

| Edge | 전체 pair | 4초 feasible | Bridge 길이 중앙값 | 최대속도 중앙값 | 최대가속도 중앙값 |
|---|---:|---:|---:|---:|---:|
| T4.open→T2.acquire | 930 | 112 | 246.0 mm | 84.2 mm/s | 86.0 mm/s² |
| T2.acquire→T4.deliver | 930 | 29 | 296.9 mm | 98.2 mm/s | 124.1 mm/s² |

### 5.1 Edge 1 feasible 후보 범위

```text
Bridge length:
  230.5 ~ 298.7 mm

max velocity:
  73.7 ~ 100.9 mm/s

max acceleration:
  46.3 ~ 137.9 mm/s²

orientation step:
  0.035 ~ 0.197 deg/tick
```

### 5.2 Edge 2 feasible 후보 범위

```text
Bridge length:
  240.3 ~ 366.3 mm

max velocity:
  84.4 ~ 136.2 mm/s

max acceleration:
  94.3 ~ 167.8 mm/s²

orientation step:
  0.096 ~ 0.506 deg/tick
```

두 Edge 모두 현재 다음 제한 안에서 통과하는 후보가 존재한다.

```text
6.67 mm/tick XYZ ramp
1.0 deg/tick orientation ramp
300 mm/s² offline Bridge acceleration limit
```

`300 mm/s²`는 offline cubic Bridge 자체의 가속도 제한이다. 이전에 수정한 `4000 mm/s²`는 Bridge/B crossfade에서 재구성한 command acceleration 허용값이므로 서로 다른 검사다.

대부분의 후보가 탈락한 주된 이유는 속도나 step 간격이 아니라 `curvature_limit`이었다.

```text
4초 기준:
Edge 1: 112개
Edge 2: 29개
```

따라서 첫 실험에서 4초 설정을 유지할 수 있는 실제 후보가 존재한다. 당장 5~6초로 늘릴 필요는 없다.

이 결과가 아직 보장하지 않는 항목:

```text
IK feasibility
drawer/environment collision safety
camera observation에 대한 ACT의 semantic 반응
실제 물체 유지
실제 robot tracking
최종 physical success
```

즉 두 Bridge manifest를 만들 수 있다는 근거이지 물리 실행 승인 자체는 아니다.

## 6. 실제 boundary 위치 분포

### T4 S2 phase 1.0

```text
count: 30
XYZ median: [400.80, -179.98, 298.05] mm
XYZ min:    [333.70, -212.05, 256.45] mm
XYZ max:    [430.36, -160.73, 319.28] mm
```

### T2 S1 phase 0.0

```text
count: 31
XYZ median: [428.72, 0.42, 456.92] mm
XYZ min:    [419.08, -10.83, 442.21] mm
XYZ max:    [432.34, 12.17, 470.83] mm
```

### T2 S2 phase 0.5

```text
count: 31
XYZ median: [382.42, 123.61, 537.99] mm
XYZ min:    [331.65, 69.69, 487.72] mm
XYZ max:    [449.95, 142.70, 575.97] mm
```

### T4 S4 phase 0.0

```text
count: 30
XYZ median: [398.82, 2.12, 356.91] mm
XYZ min:    [317.25, -11.45, 290.77] mm
XYZ max:    [431.40, 18.96, 367.86] mm
```

## 7. T2→T4 payload transport floor

Edge 2는 블록을 잡은 상태이므로 payload clearance를 별도로 검토해야 한다.

| Transport floor | Feasible 후보 |
|---:|---:|
| 없음 | 29 |
| 250 mm | 29 |
| 275 mm | 29 |
| 300 mm | 29 |
| 325 mm | 0 |
| 350 mm | 0 |
| 400 mm | 0 |

T4 S4 entry가 drawer 쪽으로 내려가는 경계이므로 기존 T2→T3에서 사용한 `400 mm transport floor`를 그대로 재사용하면 모든 후보가 탈락한다.

Edge 2의 transport floor는 다음을 바탕으로 별도로 정해야 한다.

```text
drawer 입구 높이
drawer front/handle 위치
블록 크기
gripper 아래 payload clearance
실제 Bridge trajectory의 minimum Z
```

현재 dataset geometry만 보면 `300 mm 이하`에서 후보가 존재하지만, `300 mm`가 물리적으로 안전하다는 의미는 아니다.

## 8. 가장 먼저 만들어야 할 두 edge artifact

현재 예제 설정의 두 handoff manifest 경로는 placeholder다.

```text
config/realtime/task_c_t4_t2_t4_plan.example.yaml
```

필요한 최종 artifact 구조 예시:

```text
docs/artifacts/task_c_t4_t2_t4_multi_v2_<date>/
├── t4_phase_index.json
├── t2_phase_index.json
├── edge_t4_open_to_t2_acquire/
│   ├── candidate_library.json
│   ├── candidate_library_revalidated.json
│   ├── selection_summary.json
│   ├── trajectory_visualizations/
│   ├── episode_manifests/
│   └── policy_shadow_reports/
├── edge_t2_acquire_to_t4_deliver/
│   ├── candidate_library.json
│   ├── candidate_library_revalidated.json
│   ├── selection_summary.json
│   ├── trajectory_visualizations/
│   ├── episode_manifests/
│   └── policy_shadow_reports/
└── multi_stage_plan.yaml
```

첫 실험에서는 다음 순서가 적절하다.

```text
1. 가장 보수적인 후보 1개 선정
2. shadow를 통과한 후보 3~5개 확보
3. 단일 후보로 physical validation
4. 성공 후 episode-level diverse selection 활성화
```

초기 후보 선정 기준:

```text
낮은 Bridge acceleration
낮은 curvature
짧은 경로
작은 orientation 변화
실제 episode support
successor ACT shadow 결과
trajectory 육안 검토
```

farthest-point diversity에서 처음 선택됐다는 이유만으로 바로 실기 후보로 사용하면 안 된다.

## 9. task ID canonicalization 문제

semantic artifact의 task ID:

```text
t2
t4
```

Dijkstra와 예제 MultiStagePlan의 policy ID:

```text
T2
T4
```

현재 `MultiStagePlan`은 manifest task 문자열과 policy ID를 대소문자까지 동일하게 비교한다. offline generator가 생성한 소문자 manifest를 대문자 plan에 그대로 연결하면 validation이 실패할 수 있다.

권장 구조:

```json
{
  "artifact_task_id": "t4",
  "policy_id": "T4"
}
```

원본 semantic artifact를 변경하지 않고 planner-to-runtime compiler에서 canonical policy ID로 변환하며 provenance를 보존해야 한다.

## 10. Dijkstra Plan→MultiStagePlan compiler

현재 Dijkstra와 Multi-V2 사이를 자동으로 이어주는 compiler가 없다.

필요한 interface 예시:

```python
compile_plan(
    dijkstra_plan,
    edge_registry,
) -> MultiStagePlan
```

compiler 역할:

1. 연속된 동일 policy operator를 하나의 stage visit으로 묶는다.
2. policy가 변경되는 지점만 handoff edge로 만든다.
3. source operator의 exit boundary를 추출한다.
4. successor operator의 entry boundary를 추출한다.
5. 정확히 일치하는 검증된 edge manifest를 검색한다.
6. 계획에서 사용하는 unique checkpoint만 등록한다.
7. immutable MultiStagePlan YAML을 생성한다.
8. command-free plan validator를 실행한다.

T4→T2→T4에서는 다음과 같이 collapse한다.

```text
Operators:
  T4.open
  T2.acquire
  T4.deliver
  T4.close

Policy visits:
  T4 visit 1
  T2 visit 1
  T4 visit 2
```

마지막 `T4.deliver + T4.close`는 같은 policy 연속이므로 하나의 stage로 묶는다.

## 11. Dijkstra와 실제 edge availability 연결

현재 Dijkstra 비용:

```text
operator base cost = 1.0
policy switch penalty = 0.25
physical transition cost = 0.0
```

따라서 현재의 최적 경로는 다음 의미다.

```text
semantic operator 수가 적음
+
policy switch 수가 적음
```

아직 가장 빠르거나 가장 안전한 물리 경로라는 의미는 아니다.

### 권장 초기 방식

```text
Dijkstra plan 생성
→ 모든 policy-switch edge에 exact manifest가 있는지 검사
→ 하나라도 없으면 해당 edge를 금지
→ Dijkstra 재실행
```

### 이후 확장

검증된 edge에 대해 다음 transition cost를 사용할 수 있다.

```text
transition_cost =
    bridge duration
  + normalized bridge length
  + measured shadow failure penalty
  + measured physical failure penalty
```

물리 성공률이 아직 없으므로 가상의 risk 점수를 만들면 안 된다. 초기에는 다음 정도가 적절하다.

```text
manifest 존재 여부 = hard availability
Bridge duration = 실제 cost
policy switch = 기존 penalty
```

## 12. 상위 Effect Observer / Planner Supervisor

현재 Multi-V2는 semantic 완료를 스스로 판단하지 않는다. 이는 semantic 판단을 상위 플래너에 맡기는 설계와 일치한다.

필요한 실행 흐름:

```text
operator 실행
↓
Effect Observer가 실제 상태 확인
↓
effect confirmed
↓
WorldState 갱신
↓
request_next_stage
```

시간이 지났다는 이유만으로 symbolic effect를 적용하면 안 된다.

잘못된 예:

```text
T2 acquire 예상 시간이 지남
→ 실제 grasp 확인 없이 holding=blue_block 적용
```

### 첫 physical prototype

```text
T4 drawer open 눈으로 확인
→ operator complete 승인

T2 block grasp/lift 확인
→ operator complete 승인

T4 place/close 확인
→ episode complete 승인
```

### 이후 자동 확인 후보

T4.open:

```text
C1→O1 gripper event sequence
drawer-open vision
actual TCP progression
```

T2.acquire:

```text
gripper closed
driver command success
block-in-gripper vision
lift/free-transport 상태
T2 S2 phase 근처 actual support
```

T4 final:

```text
gripper open
holding none
block in drawer vision
drawer closed vision
```

이 semantic 판단은 Bridge 내부 hard gate가 아니라 상위 플래너에 둔다.

## 13. planner service의 idempotency 보강

현재 stage 전환 서비스:

```text
/control/task_c/request_next_stage
/control/task_c/complete_episode
```

coordinator 내부는 `expected_stage_id` 검사를 지원하지만 현재 `std_srvs/srv/Trigger` request에는 stage ID가 없다.

자동 planner에는 다음 필드가 필요하다.

```text
composition_id
expected_stage_id
expected_visit_id
request_id
```

예:

```yaml
composition_id: drawer_floor_to_drawer
expected_stage_id: t2_pick_blue_block_from_floor
expected_visit_id: 1
request_id: op_complete_002
```

runtime 검사:

```text
현재 stage와 expected_stage 일치
현재 visit_id 일치
이미 처리한 request_id가 아님
```

첫 수동 실험에서는 status를 확인하고 한 번만 호출하면 되지만, 일반화된 자동 planner에는 idempotent API가 필요하다.

## 14. Whole-task ACT observation 재진입 검증

이 항목을 Bridge semantic hard reject로 넣을 필요는 없지만 물리 성공률에 영향을 주므로 실험 항목으로 남겨야 한다.

현재 ACT 입력:

```text
13D robot state
front RGB
side RGB
zed RGB
```

semantic operator ID나 phase token은 입력되지 않는다.

### T4 visit 1

T4 학습에서는 블록이 검은 테이블 위에 있었지만 복합 task에서는 floor에 있다. 이 차이에도 T4가 drawer를 정상적으로 여는지 확인해야 한다.

### T2 visit

T2 학습 환경과 달리 drawer가 열린 상태일 수 있다. T2가 열린 drawer를 보고도 floor block으로 정상 접근하는지 확인해야 한다.

### T4 visit 2

T4는 블록을 들고 있고 drawer가 열린 observation을 받아야 후반 delivery 행동을 생성할 가능성이 높다.

fresh prefix compatibility는 다음을 판단한다.

```text
현재 Bridge motion과 T4 출력이 동역학적으로 연결 가능한가
```

그러나 다음까지 보장하지는 않는다.

```text
T4가 semantic하게 drawer delivery 행동을 선택했는가
```

구조를 막는 조건으로 쓰지는 않되 actual composed-scene shadow에서 반드시 확인한다.

## 15. successor policy shadow 검증

### 15.1 Dataset-based shadow

각 후보 manifest마다 다음을 검사한다.

```text
실제 successor ACT checkpoint 로딩
successor support frame 사용
async inference
freshness 확인
generation 일치
prefix compatibility
crossfade reconstruction
robot command count = 0
```

### 15.2 Composed-scene saved-state shadow

실제 로봇은 Live OFF 상태로 두고 camera/state snapshot만 저장한다.

필요한 snapshot:

```text
Snapshot A:
  drawer open
  floor block present
  gripper open
  → T2 shadow inference

Snapshot B:
  drawer open
  block held
  gripper closed
  → T4 shadow inference
```

확인 항목:

```text
first XYZ delta
first orientation delta
prefix velocity
Bridge tail↔prefix mismatch
prospective crossfade step
prospective command acceleration
T2가 floor 쪽으로 접근하는지
T4가 drawer 쪽으로 운반하는지
```

이 단계가 whole-task ACT 재진입 가능성을 실제 observation 기준으로 확인하는 핵심이다.

## 16. 모델 load와 queue 계약

현재 Multi-V2는 unique policy당 한 번만 모델을 로딩한다.

```text
T4 load: 1회
T2 load: 1회

T4 visit: 2회
T2 visit: 1회
```

이전에 수행한 command-free 측정:

```text
T4 load: 약 2.58 s
T2 load: 약 0.67 s
첫 warmup 포함 순수 policy 준비: 약 3.9 s
steady inference: 약 40 ms
```

이 시간은 Live 전 setup에서 발생하므로 Bridge command를 정지시키지 않는다.

두 모델이 올라간 뒤 측정된 PyTorch CUDA allocated memory는 약 `0.386 GiB`였다. 이는 전체 GPU 점유량이 아니라 PyTorch allocated 값이므로 T1~T8 전체 동시 residency를 의미하지 않는다.

일반화 원칙:

```text
Dijkstra plan을 먼저 결정
→ 해당 plan의 unique policy만 load
→ 모든 policy warmup
→ Live 시작
```

실행 중 아직 로딩되지 않은 정책이 필요하면 다음처럼 처리해야 한다.

```text
Live OFF
MUX DISABLED
새 정책 load/warmup
새 plan으로 재시작
```

30 Hz control 중 모델을 로딩하면 안 된다.

## 17. 현재 plan은 episode 시작 전에 고정됨

현재 `MultiStagePlan`은 immutable ordered stage list다.

```text
episode 시작 전 Dijkstra
→ 계획 확정
→ MultiStagePlan 생성
→ 그대로 실행
```

실행 도중 future stage를 자유롭게 바꾸는 기능은 아직 없다.

초기 recovery:

```text
fail-closed
→ 실제 WorldState 재확인
→ Dijkstra 재계획
→ 새 MultiStagePlan
→ 새 bounded trial
```

이후 안정화되면 safe stage boundary에서만 future stage queue를 교체할 수 있다. Dijkstra를 30 Hz command loop 안에서 실행하거나 동작 중 stage list를 변경하면 안 된다.

## 18. Multi-stage fallback

현재 Multi-V2는 endpoint fallback을 비활성화한다.

이유:

```text
T4→T2와 T2→T4는 서로 다른 endpoint를 필요로 함
한 개의 V1 fallback manifest를 공통 사용하면 잘못된 위치로 갈 수 있음
```

초기 실험에서 다음 실패는 fail-closed로 처리한다.

```text
handoff window 실패
B inference stale
generation mismatch
prefix incompatible
tracking error
ACK error
```

향후 fallback이 필요하면 edge별로 분리한다.

```text
Edge 1 전용 fallback manifest
Edge 2 전용 fallback manifest
```

## 19. Multi-ACT dataset 기록

기존 V2는 LeRobot Task-C recording을 지원한다.

Primary action contract:

```text
ACT policy 구간: 해당 ACT proposal
Bridge: Bridge target
Crossfade: 실제 blended target
successor ACT: successor proposal
```

일반화 연구를 위해 sidecar에 다음을 frame 단위로 추가하는 것이 좋다.

```text
timestamp
composition_id
operator_id
policy_id
stage_id
visit_id
transition_id
handoff_id
bridge_progress
crossfade_weight
planned_world_state
observed_world_state
effect_confirmation_source
```

현재 multi event와 control trace를 timestamp로 join할 수 있지만 처음부터 frame 단위로 기록하는 편이 분석 신뢰도가 높다.

실패 episode는 성공 demonstration dataset에 자동 포함하지 말고 별도로 보관·표시해야 한다.

## 20. 실제 검증 순서

### Gate 1 — Artifact/manifest

두 edge 모두 다음을 만족해야 한다.

```text
정확한 source segment/phase
정확한 successor segment/phase
hard_filter_passed=true
semantic_authority=external_planner
4초 Bridge
finite/workspace/ramp/dynamics PASS
trajectory 시각 검토
```

실제 manifest를 MultiStagePlan에 넣은 뒤 command-free validator가 성공해야 한다.

### Gate 2 — Command-free policy shadow

```text
robot_commands_published = 0
fresh result
generation match
prefix PASS
crossfade PASS
```

### Gate 3 — Composed-scene shadow

실제 scene camera/state snapshot에서 T2와 T4가 의도한 방향을 출력하는지 확인한다.

### Gate 4 — Edge 1 단독 bounded test

```text
T4 drawer open
→ T4 queue invalidate
→ Bridge #1
→ T2 takeover
→ 짧게 실행
→ Live OFF
```

검증:

```text
drawer가 열린 채 유지
gripper open 유지
무제어 command gap 없음
T2 fresh prefix 사용
```

### Gate 5 — Edge 2 단독 bounded test

```text
T2 block grasp/lift
→ T2 queue invalidate
→ Bridge #2
→ T4 takeover
→ 짧게 실행
→ Live OFF
```

검증:

```text
블록 유지
gripper 임의 open 없음
T4가 drawer 방향으로 움직임
```

### Gate 6 — 전체 T4→T2→T4

초기에는 상위 stage 완료를 사람이 승인한다.

```text
T4 drawer open 확인 → next
T2 stable grasp 확인 → next
T4 place/close 확인 → complete
```

### Gate 7 — 반복 평가

권장 최소 반복 수:

```text
Edge 1: 5회
Edge 2: 5회
전체 pilot: 10회
성공률 평가: 20~40회
```

성공 지표:

```text
T4 drawer open success
Edge 1 handoff success
T2 grasp success
Edge 2 handoff/payload retention success
T4 delivery success
drawer close success
overall success
```

## 21. 예상 실행 시간

실제 dataset semantic interval 시간을 기준으로 한 전체 동작:

```text
T4 drawer open
+ Bridge #1 4초
+ T2 acquire
+ Bridge #2 4초
+ T4 deliver
+ T4 close
```

전체 motion time 추정:

```text
median: 약 72.6초
p10~p90: 약 68.5~76.9초
```

추론과 crossfade는 Bridge handoff window 안에서 겹치므로 단순 추가 시간이 아니다.

준비 포함 예상:

```text
모델 load/warmup: 약 3.9초
camera/robot context 연결: 기존 관찰상 약 12~20초
준비자세 및 물체 배치: 사용자 작업 시간
실제 motion: 약 70~80초
```

첫 전체 실험은 준비를 포함해 약 2분 정도를 잡는 것이 현실적이다. `DURATION_S=300`은 예상 실행시간이 아니라 최대 실행 제한이다.

Dijkstra 자체는 약 3 ms 수준이므로 30 Hz 제어에 실질적인 영향을 주지 않는다.

## 22. T1~T8로 일반화

현재 planner contract는 T1~T6까지만 등록되어 있다.

T7/T8 semantic artifact와 모델은 존재하지만 아직 Dijkstra operator catalog에는 포함되지 않았다.

T4→T2→T4가 검증된 뒤 다음 순서로 확장한다.

1. T7/T8 operator contract 추가
2. operator pair별 edge registry 구축
3. 검증된 manifest가 있는 edge만 runtime-capable로 표시
4. Dijkstra가 runtime-infeasible edge를 선택하지 않도록 연결
5. plan에 실제 등장하는 모델만 resident load
6. episode마다 검증된 candidate set에서 manifest 선택

모든 operator pair를 무조건 연결할 필요는 없다.

```text
symbolic precondition/effect 연결 가능
+ 실제 support 존재
+ hard-filter-passed Bridge 존재
+ successor shadow PASS
```

인 edge만 registry에 넣는다.

semantic 판단은 상위 플래너가 담당하고 Bridge에는 다음 hard contract만 남긴다.

```text
finite pose
workspace
per-tick ramp
Bridge dynamics
actual/ack tracking
fresh policy output
generation isolation
crossfade dynamics
fail-closed
```

## 23. 권장 구현 순서

1. `t2/t4` artifact ID와 `T2/T4` policy ID의 canonical mapping 수정
2. 범용 Multi-V2 shared runtime safety manifest 정리
3. 정확한 두 operator 경계로 4초 candidate library 생성
4. Edge 2 transport floor를 drawer 환경 기준으로 결정
5. 각 edge에서 보수적인 manifest 3~5개 선택
6. trajectory PNG/SVG 생성 및 육안 검토
7. dataset-based successor shadow 실행
8. composed-scene saved-state shadow 실행
9. 두 실제 manifest를 MultiStagePlan에 연결
10. Dijkstra Plan→MultiStagePlan compiler 구현
11. effect-confirmation 기반 execution supervisor 구현
12. stage request에 expected stage/visit/request ID 추가
13. per-command multi-stage audit metadata 보강
14. 단일 edge bounded physical test
15. 전체 T4→T2→T4 수동 stage 승인 실험
16. 성공 후 episode-level diverse handoff 활성화
17. 이후 T1~T8 operator/edge registry로 확장

가장 먼저 구현해야 할 묶음:

```text
T4→T2 exact edge library
+ T2→T4 exact edge library
+ Dijkstra-to-MultiStagePlan compiler
+ 상위 stage completion supervisor
```

## 24. 최종 판단

확인된 사실:

```text
Dijkstra가 목표 operator sequence를 생성함
T4 forward re-entry가 symbolic하게 허용됨
Multi-V2가 동일 policy 재방문과 queue isolation을 지원함
두 exact edge 모두 실제 dataset support를 가짐
4초 cubic Bridge에서 두 edge 모두 feasible episode pair가 존재함
```

아직 필요한 사실:

```text
두 exact edge manifest 생성
successor ACT shadow PASS
composed-scene observation에서 올바른 ACT behavior 확인
edge-specific payload clearance 검토
실제 collision/IK 검토
bounded physical edge 검증
전체 T4→T2→T4 반복 성공률 측정
```

다음 단계는 Multi-V2를 다시 설계하는 작업이 아니라 현재 구현을 실제 T4→T2→T4 semantic 경계와 연결하여 `LEVEL 2`로 올리는 작업이다.

## 25. 관련 파일

```text
config/interior_policy/a0509_t1_t6_operator_catalog_v1.json
config/realtime/task_c_t4_t2_t4_plan.example.yaml
config/realtime/task_c_multi_stage_v2.yaml

src/lerobot_robot_doosan_a0509/
  lerobot_robot_doosan_a0509/
    interior_policy/contracts.py
    interior_policy/planner.py
    interior_policy/v2_adapter.py
    task_c_multi_live_v2_rollout.py
    task_c_handoff/multi_stage.py
    task_c_handoff/policy_registry.py

offline_tools/cross_task_handoff/
  build_episode_phase_index.py
  enumerate_handoff_candidates.py
  validate_handoff_candidates.py
  select_diverse_handoffs.py
  run_v2_policy_shadow.py

scripts/
  run_a0509_interior_policy_symbolic_probe.py
  validate_task_c_multi_stage_plan.py
  run_task_c_multi_stage_v2_candidate.sh
```

## 26. 분석 provenance

이번 추가 분석에서 생성한 command-free 임시 phase index:

```text
/tmp/codex_multi_act_t4_phase_index.json
/tmp/codex_multi_act_t2_phase_index.json
```

이 임시 파일은 정식 runtime manifest가 아니며 physical execution authority를 갖지 않는다.

이번 분석 과정에서는 다음을 수행하지 않았다.

```text
실제 robot bringup
Live ON
MUX LEROBOT 선택
ServoL command
gripper Tool DO
physical collision/IK 검사
```

