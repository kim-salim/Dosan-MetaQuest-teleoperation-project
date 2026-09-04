# A0509 전체 Level-2 Semantic-local Bridge 갱신 보고서

작성일: 2026-09-03  
검증 범위: 오프라인 artifact 생성, recorded-observation ACT-B shadow, compiler 계약, unit/regression test  
물리 로봇 명령: 실행하지 않음

## 1. 목표

T2→T3와 T7→T3에서 검증한 것과 동일하게, 각 cross-policy edge의 Bridge reference를 다음 목적함수로 선택한다.

```text
L_total =
    source semantic interval의 시작부터 source 전환점까지 길이
  + Bridge 길이
  + successor 진입점부터 successor semantic interval 종료까지 길이
```

이는 Dijkstra의 symbolic operator 비용을 변경하는 것이 아니다. Dijkstra가 선택한 각 Level-2 edge 안에서 source 전환점과 successor 진입점을 고르는 data-derived local objective이다. 모든 후보는 기존 command-space hard safety를 먼저 통과해야 하며, 그 이후에만 `L_total`로 순위를 매긴다.

## 2. 실제 적용 범위

전체 registry edge는 127개다. 이 중 위 목적함수를 적용할 수 있는 source contract는 다음 8개다.

- `T1.acquire_from_black_table`
- `T2.acquire_from_floor`
- `T3.acquire_from_black_table`
- `T4.acquire_from_black_table`
- `T5.acquire_from_drawer`
- `T6.acquire_moving_block`
- `T7.acquire_from_drawer_top`
- `T8.acquire_top_block_from_stack`

각 source에서 다른 7개 policy의 successor delivery operator로 연결되는 56개 cross-policy edge가 적용 대상이다.

```text
기존 검토 결과 보존: 2개  (T2→T3, T7→T3)
새로 생성·검증·승격: 54개
적용 대상 합계:       56개
비적용 edge:          71개
registry 합계:       127개
```

71개는 생성 실패가 아니다. source가 물체 파지 후 `free_transport` 상태를 제공하는 acquisition tail이 아니므로, `source prefix + Bridge + successor suffix` 공식을 동일하게 적용할 의미 계약이 없는 edge다. 이들은 기존 baseline entry를 보존하고 `source_bridge_mode_is_not_held_object_free_transport` 근거를 기록한다.

## 3. 수정 사항

### 일괄 갱신 도구

`scripts/refresh_a0509_semantic_local_level2_registry.py`를 추가했다.

- 전체 eligible edge를 탐색한다.
- 기존 reviewed T2→T3/T7→T3 artifact는 보존한다.
- 나머지는 동일한 semantic-local reference builder로 생성한다.
- recorded observation으로 ACT-B fresh-prefix shadow를 실행한다.
- exact manifest와 shadow가 모두 PASS인 경우에만 `flexible_verified`로 승격한다.
- 실패 시 마지막 reviewed baseline을 유지한다.
- ROS node, Live, MUX, 로봇/그리퍼 command는 생성하지 않는다.

### 다중 gripper transition 처리

T1/T4/T5에는 손잡이 조작과 blue-block 조작이라는 두 개의 close→open span이 있다. 기존 trajectory standardizer는 항상 첫 번째 span(C1→O1)을 선택했기 때문에 block transport가 아니라 drawer/container handle 동작을 참조할 수 있었다.

다음을 수정했다.

- close event와 그 다음 open event를 순서대로 쌍으로 구성한다.
- semantic anchor `C2→O2`를 zero-based transition ordinal 1로 변환한다.
- operator의 실제 semantic segment에 맞는 transport span을 standardize한다.
- ordinal이 존재하지 않으면 조용히 첫 span을 쓰지 않고 실패한다.

### successor reference 높이 domain

- T2/T3/T6/T7/T8 계열 free-transport successor에는 기존 50 mm empirical transport clearance를 유지한다.
- T1/T4/T5 delivery는 container/drawer 내부로 내려가는 실제 demonstration이므로, event-relative 50 mm filter를 적용하면 support가 사라진다. 이 세 policy에는 offline reference-domain clearance를 0 mm로 두되, semantic segment, endpoint 30-frame margin, closed-gripper mask, 15 mm path margin은 유지한다.
- 이것은 runtime workspace Z safety를 비활성화하거나 변경한 것이 아니다.

## 4. 후보 선택과 hard safety

각 후보는 먼저 기존 FLEXIBLE_LEVEL2 command-space 검사를 통과해야 한다.

- finite pose/action
- semantic held-object/gripper/contact compatibility
- source/ACK freshness와 generation 계약
- position/orientation step
- velocity, acceleration, jerk
- Bridge/B prefix compatibility
- crossfade command step
- 기존 workspace 및 fail-closed 계약

그 뒤 `L_total`이 작은 후보 순으로 선택한다. 따라서 “짧기 때문에 위험한 후보”가 먼저 실행되는 구조가 아니다.

T8→T3에서는 rank 0 후보가 Bridge 자체는 통과했지만 crossfade XYZ axis step이 8.259 mm로 7.5 mm/tick 제한을 넘었다. rank 1과 rank 3도 통과하지 못했고, rank 11이 모든 검사와 ACT-B shadow를 통과했다.

```text
선택: rank 11
source phase: 0.165
successor T3 phase: 0.290
L_total: 824.791 mm
handoff_id: h_semantic_ca272d614254
```

깨끗한 artifact 재생성에서도 같은 rank 11이 자동 선택되도록 reviewed override를 기록했다. 이는 hard safety를 우회하는 것이 아니라 hard-safe 집합에서 검증된 다음 후보를 선택하는 것이다.

## 5. 산출물

주 registry:

```text
docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03/
  edge_registry_level2_spatial_v6.json
  semantic_local_refresh_summary.json
  semantic_local_edge_matrix.csv
  edges/<source>__to__<successor>/...
```

통계:

```text
semantic-local edge 수: 56
L_total 최소: 220.097 mm
L_total 평균: 498.027 mm
L_total 최대: 824.791 mm
```

GUI의 기본 registry도 v6 경로로 변경했다.

## 6. 검증 결과

```text
semantic-local/ordinal/기존 reviewed artifact 테스트: 19 passed
planner/FLEXIBLE/multi-stage V2 회귀 테스트:          79 passed
합계:                                                98 passed
compiler exact-edge + command-free shadow 순회:      56/56 PASS
T8→T3 clean rebuild rank 재현:                       rank 11 PASS
```

56개 shadow 모두 다음 조건을 만족했다.

- terminal state `RUN_B`
- takeover success `true`
- fallback required `false`
- fresh B prefix admission `valid=true`
- control deadline miss 0
- robot commands published 0
- Live disabled
- MUX not selected

## 7. 의미와 제한

이번 변경으로 해당 56개 Level-2 edge는 T2→T3/T7→T3와 동일한 semantic-local objective와 runtime contract를 사용한다. Dijkstra의 operator cost와 policy-switch cost는 그대로이므로 symbolic 최적 계획의 의미는 바뀌지 않는다.

다만 `flexible_verified`는 현재 단계에서 오프라인/recorded-observation runtime-contract 검증을 뜻한다. 54개 신규 edge의 실제 물체 보유, 환경 collision, ACT 재진입 성공률을 물리적으로 검증했다는 뜻은 아니다. 실제 로봇 적용 전에는 edge별 shadow → bounded single-edge → full composition 순서의 검증이 필요하다.

