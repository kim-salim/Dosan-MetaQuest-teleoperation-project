# A0509 T1–T8 경계 곡률 후보 복구 결과

작성일: 2026-08-29  
검증 범위: offline candidate hard filter + frozen ACT-B command-free shadow  
실기 검증: 수행하지 않음

## 1. 결론

기존 fixed cubic에서 후보가 하나도 없었던 44개 Level-1 edge만 별도로
재검증했다. 기존 곡률 제한 `0.25 /mm`를 올리지 않고, 저속 endpoint의
Bezier tangent handle이 전체 chord에 비해 지나치게 짧아지는 경우에만
opt-in 보정을 적용했다.

- 기존 fixed-cubic strict Level-2 registry: 80 edge
- 재검증 대상: 44 zero-candidate edge
- geometry/dynamics hard filter 복구: 10 edge, 19 candidate
- frozen ACT-B strict shadow 통과: 6 edge
- Bridge는 복구됐지만 ACT-B admission 실패: 4 edge
- 최종 확장 registry: 86 edge
- 실로봇 명령: 0
- Live/MUX 활성화: 없음

따라서 여기서 `복구 10개`는 Bridge candidate 복구를 뜻하고, 실제 확장
registry에 추가된 것은 ACT-B takeover까지 command-free로 통과한 6개다.

## 2. 기존 fixed cubic을 보존한 이유

기존 control point는 다음과 같다.

```text
P0 = source position
P1 = P0 + T * source velocity / 3
P2 = P3 - T * successor velocity / 3
P3 = successor position
```

이 방식은 endpoint 속도를 정확히 보존하지만, endpoint 속도가 매우 낮고
source-successor 거리가 멀면 `P0-P1` 또는 `P2-P3` handle이 수 mm로
축소된다. 그러면 긴 chord를 짧은 tangent에서 시작하거나 끝내야 하므로
endpoint 부근 곡률이 폭증한다.

기존 80개 edge의 재현성을 깨지 않기 위해 fixed generator의 기본 동작은
수정하지 않았다. 새 방식은 regularized manifest에만 적용된다.

## 3. 적용한 opt-in 보정

새 algorithm id:

```text
cubic_bezier_tangent_regularized_v1
```

계산:

```text
chord = ||P3 - P0||
minimum_handle = 0.04 * chord

source_handle = max(T * ||v_source|| / 3, minimum_handle)
successor_handle = max(T * ||v_successor|| / 3, minimum_handle)
```

방향은 원래 source/successor velocity 방향을 그대로 유지하고, handle의
크기만 늘린다. endpoint 방향이 정의되지 않으면
`source_direction_undefined` 또는 `successor_direction_undefined`로
fail-closed한다.

추가 hard reject:

```text
maximum_endpoint_speed_adjustment_mm_s = 12.0
```

즉 tangent를 늘리는 데 필요한 endpoint 속도 크기 변화가 12 mm/s를
넘으면 후보를 폐기한다. 그 뒤 기존 workspace, velocity, axis velocity,
acceleration, curvature, jerk, integrated jerk, backtracking, 30 Hz linear
step, orientation step 검사를 모두 다시 수행한다.

## 4. 파라미터 선택 근거

44개 edge의 모든 support pair를 스윕한 결과는 다음과 같다.

| 최소 handle/chord | geometry 복구 edge | feasible candidate | endpoint Δv ≤ 15 mm/s인 복구 edge |
|---:|---:|---:|---:|
| 0.02 | 0 | 0 | 0 |
| 0.03 | 2 | 2 | 2 |
| **0.04** | **10** | **19** | **10** |
| 0.05 | 18 | 70 | 18 |
| 0.06 | 25 | 267 | 24 |
| 0.075 | 30 | 1,754 | 26 |
| 0.10 | 43 | 11,713 | 30 |

0.05부터 후보가 급격히 늘기 때문에 이번 경계 복구 목적에는 0.04를
선택했다. 실제 19개 feasible candidate의 관측 범위:

```text
max curvature       = 0.247627 /mm  (limit 0.25 /mm)
max endpoint Δv     = 10.137264 mm/s (limit 12 mm/s)
```

## 5. geometry hard-filter 복구 edge

| Edge | 후보 수 | 곡률 범위 (/mm) | 최대 endpoint Δv (mm/s) | ACT-B shadow |
|---|---:|---:|---:|---|
| T1.acquire_from_black_table → T2.deliver_to_black_table | 5 | 0.082017–0.244159 | 4.652 | FAIL, endpoint fallback |
| T3.acquire_from_black_table → T8.deliver_unstacked_to_black_table | 1 | 0.193681 | 4.034 | PASS |
| T3.deliver_to_floor → T1.open_white_container | 5 | 0.143004–0.247627 | 6.715 | PASS |
| T5.acquire_from_drawer → T1.deliver_to_white_container | 1 | 0.106661 | 5.158 | PASS |
| T5.acquire_from_drawer → T3.deliver_to_floor | 1 | 0.143754 | 2.127 | PASS |
| T6.acquire_moving_block → T8.deliver_unstacked_to_black_table | 1 | 0.224144 | 7.043 | PASS |
| T6.stack_on_support_block → T4.open_drawer | 2 | 0.206275–0.217693 | 6.640 | FAIL, endpoint fallback |
| T6.stack_on_support_block → T5.open_drawer | 1 | 0.238591 | 6.939 | PASS |
| T6.stack_on_support_block → T8.acquire_top_block_from_stack | 1 | 0.240051 | 6.689 | FAIL, endpoint fallback |
| T8.acquire_top_block_from_stack → T7.deliver_to_black_table | 1 | 0.222101 | 10.137 | FAIL, endpoint fallback |

## 6. strict Level-2에 추가된 6개 edge

```text
T3.acquire_from_black_table
  → T8.deliver_unstacked_to_black_table

T3.deliver_to_floor
  → T1.open_white_container

T5.acquire_from_drawer
  → T1.deliver_to_white_container

T5.acquire_from_drawer
  → T3.deliver_to_floor

T6.acquire_moving_block
  → T8.deliver_unstacked_to_black_table

T6.stack_on_support_block
  → T5.open_drawer
```

이 분류는 `symbolic 의미가 자동으로 타당하다`는 뜻이 아니다. 현재 계약에
따라 semantic authority는 high-level external planner에 있으며, 여기서는
Bridge geometry와 fresh ACT-B output의 runtime 연결 가능성만 검증했다.

## 7. shadow 실패 4개를 등록하지 않은 이유

4개 edge는 regularized Bridge 자체는 통과했지만 fresh ACT-B prefix와의
동적 연결이 실패했다. 모든 선택 candidate에서 실패한 뒤 기존 V2의
`ENDPOINT_FALLBACK`으로 종료됐다.

집계된 admission failure reason:

```text
crossfade_xyz_axis_step          8회
bridge_prefix_velocity_mismatch  6회
crossfade_velocity               3회
```

곡률 보정을 했다는 이유로 B takeover 조건을 완화하지 않았다. 이 4개는
recovery registry에 들어가지 않는다.

## 8. non-blocking timing 결과

10개 복구 edge에 대해 선택된 총 15개 manifest shadow를 실행했다.

```text
control tick p99 range       3.043–4.976 ms
deadline miss total          0
ACT-B total preparation      36.784–105.441 ms
successful takeover progress 0.950–1.000
robot commands published     0
```

ACT-B 준비 시간 동안 Bridge command-free simulation은 계속 30 Hz tick을
진행했고, inference wait 때문에 control thread가 멈춘 사례는 없었다.

## 9. backward compatibility와 fail-closed

- 기존 manifest에 algorithm field가 없으면 `cubic_bezier_fixed`로 해석한다.
- 기존 fixed-cubic 후보의 handoff id 계산은 변하지 않는다.
- regularized 후보는 algorithm/ratio/speed limit를 id hash에 포함한다.
- offline candidate와 runtime actual-state regeneration이 동일 helper를 쓴다.
- runtime actual source velocity 방향이 정의되지 않으면 실행 전 실패한다.
- runtime actual state에서 endpoint Δv가 12 mm/s를 넘으면 실행 전 실패한다.
- 기존 0.25/mm 및 나머지 hard filter를 runtime에서 다시 검사한다.
- 기존 80개 registry 파일은 수정하지 않았다.

## 10. 산출물

기존 baseline:

```text
docs/artifacts/t1_t8_full_level2_coverage_2026-08-29/
```

신규 recovery evidence:

```text
docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29/
  recovery_inventory.json
  recovery_summary.json
  recovery_matrix.csv
  edge_registry_recovered_level2.json
  edge_registry_all_level2_extended.json
  edges/*/candidate_library.json
  edges/*/episode_manifests/*.json
  edges/*/policy_shadow_*.json
  edges/*/shadow_evaluation.json
```

확장 planner/runtime contract 검증에 사용할 registry:

```text
docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29/
  edge_registry_all_level2_extended.json
```

## 11. 테스트와 한계

실행된 핵심 테스트:

```bash
pytest -q \
  offline_tools/task_c_bridge_v0/test_task_c_bridge_v0.py \
  src/lerobot_robot_doosan_a0509/test/test_cross_task_handoff_v2.py \
  src/lerobot_robot_doosan_a0509/test/test_task_c_handoff_v2.py
```

결과:

```text
focused regularization/V2 tests: 54 passed
Bridge + A0509 package full regression: 327 passed
```

확인된 사실이 아닌 항목:

- 실물 Doosan의 성공 여부
- 물체 파지/유지 여부
- 환경 collision safety
- IK feasibility
- actual-state variation에서 10개 edge의 runtime acceptance rate

따라서 새 6개 edge는 `command-free LEVEL 2`이며, physical LEVEL 3로
표현하면 안 된다.
