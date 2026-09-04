# A0509 T7/T8 cross-policy Level 2 검증 보고서

> **범위 주의:** 이 문서는 T7/T8 관련 초기 16-edge probe 결과다. 이후 T1~T8 전체
> direct Level 1 edge 128개를 같은 계약으로 전수 검사했고 80개 strict Level 2 edge로
> 확장했다. 최신 통합 결과는
> `docs/a0509_t1_t8_full_level2_coverage_2026-08-29_ko.md`를 사용한다.

## 1. 판정

T1~T8 operator catalog에서 T7 또는 T8이 포함된 직접 cross-policy contract를
전수 열거한 결과는 다음과 같다.

```text
symbolically compatible edges: 16
4 s Bridge hard-filter에서 후보가 남은 edges: 5
fresh successor ACT shadow까지 통과한 Level 2 edges: 4
Level 1에 남은 edges: 12
physical robot validation: not performed
```

기존 T4→T2→T4의 2개 edge를 포함한 unified registry에는 현재 총 6개
command-free Level 2 edge가 들어 있다.

## 2. 검증 범위와 조건

모든 작업은 실제 robot command 없이 수행했다.

```text
robot_commands_published = 0
live_enabled = false
mux_selected = false
IK checked = false
environment collision checked = false
physical success claimed = false
```

공통 조건:

- 실제 T1~T8 LeRobot datasets
- 실제 semantic graph V3와 episode phase support
- 실제 frozen ACT successor checkpoints
- cubic Bezier Bridge 4.0 s
- 30 Hz virtual Bridge execution
- semantic authority: `external_planner`
- Z workspace minimum 비활성화, X/Y minimum과 XYZ maximum 유지
- raw B prefix acceleration hard reject 비활성화
- crossfade command acceleration limit: 4000 mm/s²
- candidate hard-filter 후에만 farthest-point diversity selection

Level 2 조건은 symbolic contract, 실제 support, exact manifest, Bridge hard
filter, real ACT successor shadow의 `RUN_B`, fresh-prefix admission,
fallback 없음, deadline miss 0을 모두 요구한다.

## 3. 16개 edge 전수 결과

| Edge | feasible / total | 최종 판정 | 주된 미통과 이유 |
|---|---:|---|---|
| T1.acquire → T7.deliver | 0 / 900 | Level 1 | curvature limit |
| T2.acquire → T7.deliver | 0 / 930 | Level 1 | curvature limit |
| T3.acquire → T7.deliver | 0 / 900 | Level 1 | curvature limit |
| T4.acquire → T7.deliver | 0 / 900 | Level 1 | curvature limit |
| T5.acquire → T7.deliver | 2 / 900 | **Level 2** | shadow PASS |
| T6.acquire → T7.deliver | 0 / 900 | Level 1 | curvature limit |
| T6.acquire → T8.deliver | 0 / 900 | Level 1 | curvature limit |
| T6.stack → T8.acquire | 0 / 900 | Level 1 | curvature limit |
| T7.acquire → T2.deliver | 2 / 930 | Level 1 | 두 shadow 모두 dynamic admission FAIL |
| T7.acquire → T3.deliver | 0 / 900 | Level 1 | curvature limit |
| T7.deliver → T3.acquire | 25 / 900 | **Level 2** | shadow PASS |
| T8.acquire → T2.deliver | 116 / 930 | **Level 2** | 5 shadows 중 1 PASS |
| T8.acquire → T3.deliver | 0 / 900 | Level 1 | curvature limit |
| T8.acquire → T6.stack | 0 / 900 | Level 1 | curvature limit |
| T8.acquire → T7.deliver | 0 / 900 | Level 1 | curvature limit |
| T8.deliver → T3.acquire | 220 / 900 | **Level 2** | shadow PASS |

`curvature limit`은 의미가 맞지 않는다는 뜻이 아니다. 선택한 실제 source
velocity tangent와 successor boundary를 4초 cubic Bezier로 연결했을 때 현재
`0.25 /mm` hard limit을 통과하지 못했다는 뜻이다. threshold는 임의로
완화하지 않았고 이 edge들은 registry에 넣지 않았다.

## 4. 신규 Level 2 edge

| Edge | Handoff | Bridge 길이 | first XYZ | first rotation | velocity mismatch | crossfade accel | tick p99 |
|---|---|---:|---:|---:|---:|---:|---:|
| T5.acquire_from_drawer → T7.deliver_to_black_table | `h_25f4b8e4b5f8` | 105.240 mm | 12.693 mm | 0.731° | 43.019 mm/s | 2400.436 mm/s² | 3.337 ms |
| T7.deliver_to_black_table → T3.acquire_from_black_table | `h_47001cfd34c7` | 180.555 mm | 27.331 mm | 0.585° | 45.316 mm/s | 876.643 mm/s² | 3.210 ms |
| T8.acquire_top_block_from_stack → T2.deliver_to_black_table | `h_00bbb036a5f8` | 305.067 mm | 35.541 mm | 0.943° | 41.026 mm/s | 2956.281 mm/s² | 3.672 ms |
| T8.deliver_unstacked_to_black_table → T3.acquire_from_black_table | `h_094cf79e9bbd` | 156.046 mm | 25.543 mm | 4.713° | 28.611 mm/s | 1244.299 mm/s² | 3.130 ms |

네 shadow 모두 다음을 만족했다.

```text
terminal_state = RUN_B
takeover_success = true
prefix_admission.valid = true
fallback_required = false
deadline_miss_count = 0
robot_commands_published = 0
```

## 5. Bridge 후보는 있었지만 Level 2가 아닌 edge

`T7.acquire_from_drawer_top → T2.deliver_to_black_table`은 930개 중
2개가 Bridge hard-filter를 통과했다. 하지만 실제 T2 ACT shadow에서는:

```text
h_29e424a8185c:
  bridge-prefix velocity mismatch = 89.377 mm/s
  threshold = 75 mm/s

h_4111f3dd33c8:
  bridge-prefix velocity mismatch = 85.946 mm/s
  crossfade XYZ axis step도 초과
```

두 경우 모두 `ENDPOINT_FALLBACK`으로 종료되어 registry에서 제외했다.
이는 V2 fail-closed가 의도대로 동작한 결과다.

## 6. T6 → T8에 대한 정확한 결론

`T6.stack_on_support_block → T8.acquire_top_block_from_stack`은
symbolic state 계약상 자연스럽지만 현재 4초/0.25 per-mm curvature 기준에서
900개 후보가 모두 탈락했다. 따라서 현재는 Level 1이다.

향후에는 threshold를 임의 완화하기보다 Bridge duration, endpoint tangent
regularization, low-speed boundary control-point minimum distance 또는 별도
alignment connector를 새 실험으로 비교해야 한다. 현재 결과에는 적용하지 않았다.

## 7. T8 object identity alias

T8 artifact의 실제 label은 `held_object=moving_blue_block`이고 planner는
`holding=blue_block`으로 정규화한다. 이를 임의 문자열 mismatch 무시로
처리하지 않고 catalog에 다음 machine-readable alias를 명시했다.

```json
{"held_object_aliases": {"moving_blue_block": "blue_block"}}
```

compiler는 source operator에 이 alias가 명시된 경우에만 exact held-object check를
통과시킨다. 등록되지 않은 다른 물체명 mismatch는 계속 fail-closed다.

## 8. Unified registry

`docs/artifacts/t7_t8_level2_edge_validation_2026-08-29/edge_registry_all_level2.json`에는:

```text
기존:
  T4.open_drawer → T2.acquire_from_floor
  T2.acquire_from_floor → T4.deliver_to_drawer

신규:
  T5.acquire_from_drawer → T7.deliver_to_black_table
  T7.deliver_to_black_table → T3.acquire_from_black_table
  T8.acquire_top_block_from_stack → T2.deliver_to_black_table
  T8.deliver_unstacked_to_black_table → T3.acquire_from_black_table
```

compiler command-free test는 `T5.acquire → T7.deliver`와
`T8.acquire → T2.deliver` two-policy plans를 실제 Multi-V2 schema로
변환했다. registry에 없는 edge는 compile 단계에서 계속 거부된다.

## 9. 산출물

- `phase_indices/t1.json` ~ `t8.json`: 실제 episode phase indices
- `candidates/*.json`: 16개 edge hard-filter 결과
- `edges/*/episode_manifests/`: feasible diversity manifests
- `edges/*/policy_shadow_*.json`: real successor ACT command-free 결과
- `edge_validation_summary.json`: machine-readable 전체 판정
- `edge_registry_all_level2.json`: passing evidence만 포함한 registry
- `scripts/summarize_a0509_t7_t8_level2_validation.py`: 재생성 도구

## 10. 회귀 및 정적 검증

실제 robot/ROS node를 실행하지 않고 다음 검사를 통과했다.

```text
planner + V2 + failure routing + multi-stage + source phase + supervisor:
  98 passed in 7.65 s

standalone semantic Dijkstra GUI snapshot:
  9 passed

JSON integrity:
  edge_validation_summary.json PASS
  edge_registry_all_level2.json PASS
```

planner test에는 신규 네 Level 2 edge의 registry lookup, Level 1 edge의
fail-closed, T5→T7 및 T8→T2 계획의 실제 Multi-V2 compilation이 포함된다.

## 11. 한계

Level 2는 실제 로봇 성공이 아니다. recorded successor observation과
perfect-tracking virtual Bridge를 사용했으며, 물체 유지·IK·환경 collision·실제
가속도·full composite physical success는 검증하지 않았다.

따라서 네 신규 edge의 정확한 표현은
`LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE`,
`physical_validation=false`다.
