# A0509 T1~T8 Dijkstra Level 1 → Level 2 전체 확장 보고서

작성일: 2026-08-29  
대상: 현재 working tree의 T1~T8 semantic operator catalog 및 Task-C Multi-V2  
검증 방식: static audit, Bridge candidate hard filter, frozen successor ACT shadow, compiler/unit/browser test  
실제 로봇 명령: **0회**

## 1. 결론

현재 21개 interior-policy operator에서 성립 가능한 모든 직접 cross-policy
Level 1 edge를 전수 열거하고, 각 edge를 현재 V2 조건으로 검증했다.

| 구분 | 수량 |
|---|---:|
| 직접 cross-policy Level 1 edge | 128 |
| context-independent edge | 36 |
| persistent context가 필요한 edge | 92 |
| 4초 Bridge hard filter 통과 edge | 84 |
| strict command-free Level 2 edge | **80** |
| Bridge 후보 0개 | 44 |
| Bridge 후보는 있으나 ACT-B shadow 실패 | 4 |
| 검증하지 않고 남은 edge | 0 |

따라서 이번 확장으로 기존 6-edge 제한 registry가 아니라, 현재 선언된 T1~T8
operator contract 전체에서 확인된 **80개 Level 2 edge registry**를 사용할 수 있다.

단, 여기서 Level 2는 다음을 뜻한다.

```text
symbolic contract valid
+ actual episode support boundary
+ exact handoff manifest
+ 4 s cubic-Bezier Bridge hard filter
+ real frozen successor ACT shadow
+ fresh prefix dynamic admission PASS
+ soft crossfade simulation PASS
+ fallback 없음
+ control deadline miss 0
```

실제 다단 로봇 성공을 뜻하는 Level 3는 아니다.

## 2. “모든 조합”의 정확한 범위

이번 전수 검사는 무한한 자연어 task 전체가 아니라 다음 유한 계약을 대상으로 한다.

```text
8 frozen ACT policies
21 semantic interior operators
WorldState precondition/effect
forward-only per-policy cursor
direct cross-policy operator pair
```

두 operator `A → B`에 대해 source의 알려진 postcondition과 successor precondition이
충돌하지 않으면 Level 1 direct edge witness를 구성했다.

- `context-independent` 36개: source의 pre/effect만으로 B의 조건이 보장된다.
- `context-dependent` 92개: drawer/container/stack 등 source가 바꾸지 않는 외부 상태를
  명시적인 persistent context witness로 추가해야 한다.

`unknown`을 조건 충족으로 간주하지 않았다. context-dependent edge는 registry에
있다는 이유만으로 항상 적용되는 것이 아니며, 실행 당시 상위 planner의 WorldState가
그 context를 만족해야 한다.

같은 policy 안의 forward continuation은 Bridge가 필요한 cross-policy edge가 아니므로
128개 집계에서 제외했다. 연속 operator program은 이 direct edge들을 이어서 구성한다.

## 3. 검증 파이프라인

### 3.1 Level 1 inventory

각 pair에 대해 다음을 저장했다.

```text
source/successor operator
source exit segment/phase
successor entry segment/phase
witness state before source
witness state after source
required persistent context
transition type
```

### 3.2 Bridge candidate

현재 catalog의 exact semantic boundary와 episode phase support를 사용했다.

```text
generator: cubic Bezier
duration: 4.0 s
candidate total: 115,950
hard-filter feasible candidates: 6,030
diverse representatives selected: 357
```

안전/feasibility filter를 diversity selection보다 먼저 적용했다. 현재 workspace
계약을 유지했으며 Z minimum은 비활성화 상태다.

### 3.3 Successor ACT shadow

Bridge-feasible edge에 대해서만 실제 frozen successor checkpoint와 recorded
successor RGB/13D state를 사용해 command-free shadow를 수행했다.

```text
virtual Bridge: 30 Hz
handoff window: 24 steps
B prefix: 15 steps
crossfade: 15 steps
raw B acceleration hard reject: disabled
command acceleration limit: 4000 mm/s²
semantic authority: external_planner
robot commands published: 0
Live: false
MUX selected: false
```

strict PASS는 `terminal_state=RUN_B`, takeover 성공, fresh prefix valid, fallback false,
deadline miss 0을 모두 요구한다.

## 4. 전체 Level 2 edge 분포

| Source operator | L1 | L2 | Bridge 0 | Shadow 실패 |
|---|---:|---:|---:|---:|
| T1.acquire_from_black_table | 7 | 3 | 4 | 0 |
| T1.deliver_to_white_container | 2 | 2 | 0 | 0 |
| T1.open_white_container | 11 | 7 | 4 | 0 |
| T2.acquire_from_floor | 7 | 2 | 5 | 0 |
| T2.deliver_to_black_table | 7 | 6 | 1 | 0 |
| T3.acquire_from_black_table | 7 | 3 | 4 | 0 |
| T3.deliver_to_floor | 4 | 1 | 1 | 2 |
| T4.acquire_from_black_table | 7 | 5 | 2 | 0 |
| T4.close_drawer | 2 | 2 | 0 | 0 |
| T4.deliver_to_drawer | 2 | 2 | 0 | 0 |
| T4.open_drawer | 9 | 8 | 1 | 0 |
| T5.acquire_from_drawer | 7 | 3 | 4 | 0 |
| T5.close_drawer | 4 | 4 | 0 | 0 |
| T5.deliver_to_black_table | 4 | 3 | 1 | 0 |
| T5.open_drawer | 9 | 9 | 0 | 0 |
| T6.acquire_moving_block | 7 | 3 | 3 | 1 |
| T6.stack_on_support_block | 4 | 0 | 4 | 0 |
| T7.acquire_from_drawer_top | 7 | 1 | 5 | 1 |
| T7.deliver_to_black_table | 7 | 6 | 1 | 0 |
| T8.acquire_top_block_from_stack | 7 | 3 | 4 | 0 |
| T8.deliver_unstacked_to_black_table | 7 | 7 | 0 | 0 |

80개 중 16개는 context-independent이고 64개는 explicit persistent context가
필요하다.

## 5. 등록된 80개 edge

아래는 successor를 source별로 묶은 목록이다. 각 edge의 handoff ID, 후보 수,
context witness, manifest/shadow 경로는 `edge_coverage_matrix.csv`와
`edge_coverage_summary.json`에 모두 기록돼 있다.

- `T1.acquire_from_black_table` → T3.deliver_to_floor, T4.deliver_to_drawer, T6.stack_on_support_block
- `T1.deliver_to_white_container` → T4.open_drawer, T5.open_drawer
- `T1.open_white_container` → T2.acquire_from_floor, T3.acquire_from_black_table, T4.open_drawer, T5.open_drawer, T6.acquire_moving_block, T7.acquire_from_drawer_top, T8.acquire_top_block_from_stack
- `T2.acquire_from_floor` → T4.deliver_to_drawer, T8.deliver_unstacked_to_black_table
- `T2.deliver_to_black_table` → T1.acquire_from_black_table, T1.open_white_container, T3.acquire_from_black_table, T4.open_drawer, T5.close_drawer, T5.open_drawer
- `T3.acquire_from_black_table` → T1.deliver_to_white_container, T4.deliver_to_drawer, T6.stack_on_support_block
- `T3.deliver_to_floor` → T4.open_drawer
- `T4.acquire_from_black_table` → T1.deliver_to_white_container, T2.deliver_to_black_table, T3.deliver_to_floor, T6.stack_on_support_block, T8.deliver_unstacked_to_black_table
- `T4.close_drawer` → T1.open_white_container, T5.open_drawer
- `T4.deliver_to_drawer` → T1.open_white_container, T5.acquire_from_drawer
- `T4.open_drawer` → T1.acquire_from_black_table, T1.open_white_container, T2.acquire_from_floor, T3.acquire_from_black_table, T5.acquire_from_drawer, T6.acquire_moving_block, T7.acquire_from_drawer_top, T8.acquire_top_block_from_stack
- `T5.acquire_from_drawer` → T2.deliver_to_black_table, T4.deliver_to_drawer, T7.deliver_to_black_table
- `T5.close_drawer` → T1.acquire_from_black_table, T1.open_white_container, T3.acquire_from_black_table, T4.open_drawer
- `T5.deliver_to_black_table` → T1.acquire_from_black_table, T1.open_white_container, T3.acquire_from_black_table
- `T5.open_drawer` → T1.acquire_from_black_table, T1.open_white_container, T2.acquire_from_floor, T3.acquire_from_black_table, T4.acquire_from_black_table, T4.close_drawer, T6.acquire_moving_block, T7.acquire_from_drawer_top, T8.acquire_top_block_from_stack
- `T6.acquire_moving_block` → T1.deliver_to_white_container, T3.deliver_to_floor, T4.deliver_to_drawer
- `T7.acquire_from_drawer_top` → T5.deliver_to_black_table
- `T7.deliver_to_black_table` → T1.acquire_from_black_table, T1.open_white_container, T3.acquire_from_black_table, T4.open_drawer, T5.close_drawer, T5.open_drawer
- `T8.acquire_top_block_from_stack` → T1.deliver_to_white_container, T2.deliver_to_black_table, T4.deliver_to_drawer
- `T8.deliver_unstacked_to_black_table` → T1.acquire_from_black_table, T1.open_white_container, T3.acquire_from_black_table, T4.acquire_from_black_table, T4.open_drawer, T5.close_drawer, T5.open_drawer

`T6.stack_on_support_block`를 source로 하는 strict Level 2 edge는 현재 0개다.

## 6. Level 2로 승격하지 않은 48개

### 6.1 Bridge 후보가 없는 44개

44개 모두 현재 `curvature_limit=0.25 /mm`에서 모든 candidate가 탈락했다.
이는 semantic contract가 거짓이라는 뜻이 아니다. 현재 actual support tangent와
successor boundary를 **동일한 4초 cubic-Bezier generator 및 현재 hard limit**으로
연결할 feasible candidate를 찾지 못했다는 뜻이다.

threshold를 임의로 완화하지 않았고 registry에 넣지 않았다.

### 6.2 Bridge 후보는 있으나 shadow가 실패한 4개

현재 주요 admission threshold:

```text
bridge↔B prefix velocity mismatch: 75 mm/s
crossfade XYZ axis step: 6.67 mm/tick
crossfade velocity: 300 mm/s
crossfade command acceleration: 4000 mm/s²
```

| Edge | feasible / total | 선택 shadow | 실제 실패 |
|---|---:|---:|---|
| T3.deliver_to_floor → T2.acquire_from_floor | 1 / 930 | 1 | velocity mismatch 81.370 mm/s |
| T3.deliver_to_floor → T5.open_drawer | 4 / 900 | 4 | velocity mismatch 78.417~100.621 mm/s |
| T6.acquire_moving_block → T2.deliver_to_black_table | 10 / 930 | 5 | crossfade axis step 7.878~11.976 mm, 일부 velocity/mismatch 동시 초과 |
| T7.acquire_from_drawer_top → T2.deliver_to_black_table | 2 / 930 | 2 | mismatch 89.377/85.946 mm/s, 두 번째는 axis step도 초과 |

12개 shadow report 모두 `ENDPOINT_FALLBACK`으로 끝났고 strict registry에서
제외됐다. command acceleration은 모두 4000 mm/s² 아래였으므로 이번 네 edge의
직접 탈락 원인은 가속도 기준이 아니다.

## 7. Level 2 전용 Dijkstra 확장

기존 흐름은 다음이었다.

```text
Level 1 UCS가 최저비용 symbolic plan 선택
→ compiler가 각 policy switch를 registry에서 require
→ edge 하나라도 없으면 fail-closed
```

이는 안전하지만, 더 긴 **검증된 Level 2 우회 경로**가 있어도 검색 단계에서 찾지
못하는 문제가 있었다.

새 `uniform_cost_search_level2(...)`는 다음 규칙을 사용한다.

```text
first operator: 허용
same-policy forward continuation: 허용 (Bridge 불필요)
cross-policy transition: 80-edge strict registry에 있을 때만 허용
```

기본 `uniform_cost_search(...)`의 Level 1 의미는 바꾸지 않았다. 연구자는 두 결과를
명시적으로 비교할 수 있다.

또한 edge admission은 정확한 직전 operator pair에 의존하므로 Dijkstra dominance
key를 다음으로 보강했다.

```text
(WorldState, exact previous operator id)
```

이 처리가 없으면 동일 WorldState에 먼저 도달한 미호환 경로가, 실제 registry edge를
가진 경로를 잘못 지울 수 있다.

`compile_controlled_catalog_plan(...)`은 이제 Level 2-constrained UCS를 사용한 뒤
기존 exact manifest/shadow assertion을 다시 수행한다. 즉 검색과 compiler 양쪽에서
fail-closed다.

## 8. 구체 사례: drawer의 블록을 floor에 놓기

Level 1 최단경로는 다음과 같다.

```text
T5.acquire_from_drawer
→ T3.deliver_to_floor

cost = 2.25
```

하지만 이 direct edge는 900개 Bridge candidate가 모두 curvature limit에서 탈락했다.
따라서 Level 2 registry에는 없다.

Level 2 전용 Dijkstra는 이를 강제로 쓰지 않고 다음 경로를 찾았다.

```text
T5.acquire_from_drawer
→ T5.deliver_to_black_table
→ T3.acquire_from_black_table
→ T3.deliver_to_floor

policy sequence = T5 → T3
cost = 4.25
```

같은 T5 내부의 acquire→deliver는 forward continuation이므로 별도 Bridge가 없다.
정책 switch는 `T5.deliver_to_black_table → T3.acquire_from_black_table` 한 번이며,
이 edge는 strict Level 2 registry에 있다. 생성된 plan은 현재 Multi-V2 compiler도
통과했다.

이 결과는 “직접 서랍→바닥 운반”이 아니라 **검은 테이블에 한 번 놓고 T3로 다시
집어 바닥에 놓는 우회 program**이다. 목표 predicate에는 맞지만 사용자가 원하지 않는
중간 효과일 수 있으므로, 상위 planner가 ordered subgoal/금지 효과/cost를 지정하는
기능은 여전히 중요하다.

## 9. 기존 목표 T4 → T2 → T4

Level 2 전용 검색에서도 기존 목표 계획은 그대로 유지된다.

```text
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer

policy sequence = T4 → T2 → T4
```

두 cross-policy edge 모두 새 80-edge registry에 있으며 compiler가 exact manifest와
shadow evidence를 다시 확인한다.

## 10. GUI 확장

독립 GUI에 `Level 2 registry 전환만` 토글을 추가했다.

- OFF: 기존 Level 1 symbolic UCS
- ON: 동일 policy continuation + strict 80-edge policy switch만 사용
- 미등록 edge는 탐색 중 `level2_edge_unavailable`로 기록
- 결과 제목에서 Level 1/Level 2-constrained를 구분
- `Level 2 경로 · 서랍→검은 테이블→바닥` 비교 시나리오 추가

GUI는 static snapshot만 읽으며 ROS/ACT/robot 연결이 없다.

## 11. 산출물

### 전체 증거

```text
docs/artifacts/t1_t8_full_level2_coverage_2026-08-29/
  level1_edge_inventory.json
  edge_coverage_summary.json
  edge_coverage_matrix.csv
  edge_registry_all_level2.json
  edges/<128 edge ids>/...
```

### 재현 스크립트

```text
scripts/validate_a0509_full_level2_coverage.py
```

stage는 `inventory`, `candidates`, `shadows`, `summary`, `all`로 분리돼 있고
resumable하다. ROS node, camera, MUX 또는 robot connection을 생성하지 않는다.

### GUI snapshot

```text
/home/rvlab/a0509-semantic-dijkstra-gui/data/semantic_snapshot.json
```

## 12. 테스트 결과

```text
full Level 2 coverage/unit/compiler tests: 7 passed
관련 V2/Multi-V2/phase/supervisor 회귀: 105 passed
GUI snapshot/provenance: 10 passed
GUI Chromium browser planner core: 15 passed
80 registry edges exact compile: 80 / 80 PASS
physical robot tests: NOT RUN
```

80개 각각에 대해 registry load, manifest/shadow file 존재, exact operator boundary,
current Multi-V2 plan compilation을 확인했다.

## 13. 해석상 주의점

1. **Level 2는 Level 3가 아니다.** frozen ACT shadow에서 동역학적 연결 계약을
   통과했지만 물체 유지, 충돌, IK, 실제 시각 분포, 누적 multi-edge 성공은 미검증이다.
2. `ik_checked=false`, `collision_checked=false`다. 없는 기능을 안전 보장처럼
   표현하지 않는다.
3. 92개 context-dependent edge는 상위 planner의 정확한 persistent state가 필요하다.
4. 80개 edge를 모두 등록한 것은 모두 유용하다는 뜻이 아니다. 목표 predicate와 cost가
   불필요한 행동을 배제해야 한다.
5. 여러 개의 개별 Level 2 edge가 있다는 사실만으로 장시간 chain의 joint success
   probability가 증명되지는 않는다.
6. 실제 robot 실행 전에는 목표 plan 하나를 고정해 shadow → bounded single-edge →
   bounded full sequence 순으로 Level 3를 별도 검증해야 한다.

## 14. 최종 판정

```text
Level 1 direct search space: 전수 열거 완료 (128)
Bridge candidate coverage: 완료 (128/128)
successor ACT shadow coverage: 완료 (84/84 feasible edges)
strict Level 2 registry: 80 edges
Level 2-constrained Dijkstra: 구현 및 검증 완료
current Multi-V2 compiler 연결: 80/80 완료
physical validation: 미수행
```

현재 시스템은 Level 1에서 가능한 edge를 단순히 많이 등록한 상태가 아니라,
**검증된 Level 2 edge만 사용해 Dijkstra가 대체 경로를 다시 탐색하고, 선택된 모든
policy switch를 기존 Multi-V2 compiler가 fail-closed로 재검증하는 구조**로 확장됐다.
