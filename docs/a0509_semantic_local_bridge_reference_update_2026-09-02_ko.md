# A0509 Semantic-Local FLEXIBLE_LEVEL2 Bridge Reference 개편 보고서

작성일: 2026-09-02  
범위: `T7.acquire_from_drawer_top → T3.deliver_to_floor` 및 공통 Dijkstra→Multi-V2 compiler 계약  
검증 범위: 데이터/오프라인 geometry/policy shadow만 수행, 실제 로봇 명령 0회

## 1. 결론

기존 Dijkstra의 symbolic operator 비용은 바꾸지 않았다.

```text
T7.acquire_from_drawer_top = 1.0
T3.deliver_to_floor        = 1.0
policy switch              = 0.25
Dijkstra total             = 2.25
```

대신 planner가 승인한 하나의 semantic edge 안에서 물리 Bridge reference를 선택하는 목적함수를 다음과 같이 변경했다.

```text
J_local = T7 S2 시작→source cut 경로 길이
        + command-safe Bridge 길이
        + T3 successor reference→T3 S2 종료 경로 길이
```

따라서 Dijkstra는 여전히 어떤 semantic operator를 어떤 순서로 실행할지 결정하고, 새 selector는 그 edge를 실제로 연결할 source/successor reference를 고른다. 두 최적화 계층은 분리되어 있다.

## 2. 기존 문제

기존 T7→T3 manifest는 다음 한 점을 successor reference로 고정했다.

```text
T3 S2 phase = 0.000
position ≈ [409.77, 2.00, 354.12] mm
```

이 값은 T3 delivery semantic의 시작점이라 compiler에는 단순했지만, T7이 충분히 물체를 들어 올린 뒤에도 Bridge가 T3의 초기 운반 시작 위치로 되돌아가는 긴 우회를 만들었다. ACT-B takeover 자체는 이미 fresh policy output으로 결정되고 있었으므로, B phase 0 고정은 제어 권한 조건이 아니라 불필요하게 강한 기하학적 endpoint 제약이었다.

반대로 `Bridge + B suffix` 길이만 최소화하면 T3 release 직전 phase로 붙는 편향이 생긴다. 그래서 30개 demonstration의 empirical transport support와 내부 경로 여유를 동시에 사용했다.

## 3. 30개 T3 에피소드의 empirical support

기존 V1 transport standardization 계약을 재사용했다.

```text
close 이후 30 frames 제외
open 이전 30 frames 제외
각 episode의 grasp/release event 높이보다 50 mm 높은 연속 closed-gripper transport 구간
고정 absolute Z minimum = 사용하지 않음
```

각 episode의 이 구간 시작/끝을 T3 S2 semantic phase로 다시 투영한 뒤 다음 robust 공통 범위를 계산했다.

```text
low  = episode start phase의 95% upper quantile = 0.13
high = episode end phase의 5% lower quantile    = 0.68
```

이 범위는 successor candidate domain일 뿐이며 Live safety Z gate가 아니다.

추가로 medoid episode 경로에서 양쪽 support 경계로부터 최소 15 mm의 arc-length 내부 여유를 요구했다. 이에 따라 실제 successor 후보의 마지막 phase는 0.65가 되었고 phase 0.68 경계는 후보에서 제외됐다.

## 4. 새로 선택된 T7→T3 reference

```text
source
  task/segment: T7 / S2
  phase:        0.160
  episode:      15
  XYZ:          [590.4823, -179.3844, 447.4809] mm

successor
  task/segment: T3 / S2
  phase:        0.650
  episode:      23
  frame:        402
  XYZ:          [368.3908, -176.6889, 437.3741] mm

semantic-local length
  T7 prefix:    108.2765 mm
  Bridge:       302.6578 mm
  T3 suffix:    183.7758 mm
  total:        594.7101 mm
```

T3 medoid episode의 standardized trajectory 기준으로 phase 0.65는 release phase 1.0보다 약 148.3 mm 높다. empirical support end phase 0.68보다도 약 12.0 mm 높다. 다만 실제 invariant는 Z 차이가 아니라 support 끝까지 남은 경로 길이 15.6561 mm이다.

## 5. 실제 Bridge hard validation

총 275개 bounded source/successor 조합을 비교했다.

```text
pairs considered:  275
hard passed:       275
selected generator: reference_deformation
Bridge duration:   4.0 s
Bridge length:     302.6578 mm
```

선택된 reference의 주요 수치:

```text
max position axis step:          3.3327 mm/tick
max 2-step ACK span axis step:   6.5846 mm
configured positional limit:     7.5 mm
max orientation step:            0.0433 deg/tick
configured orientation limit:    1.25 deg/tick
max path acceleration:           91.1553 mm/s²
max command acceleration:        90.1727 mm/s²
configured acceleration limit:   4000 mm/s²
max command jerk:                1342.7845 mm/s³
configured jerk limit:           4000 mm/s³
hard rejection reasons:          none
```

Workspace, finite command, step, velocity, acceleration, jerk, reversal, cusp, self-intersection 및 기존 ACK pipeline 검사는 그대로 유지된다. `transport_floor_mm`는 여전히 `null`이다.

## 6. ACT-B policy shadow

새 successor real frame(T3 episode 23, frame 402)에서 T3 ACT checkpoint를 실제로 로드해 command-free policy shadow를 다시 수행했다.

```text
terminal state:                RUN_B
takeover:                      true
fallback:                      false
fresh prefix admission:       PASS
splice index:                 0
first B XYZ delta norm:       43.3445 mm
crossfade max axis step:       7.0083 mm/tick
crossfade max acceleration:    1431.9500 mm/s²
control tick p99:              24.2495 ms
control tick max:              24.5798 ms
30 Hz deadline misses:         0
robot commands published:      0
```

첫 B target 차이 43.3 mm를 한 번에 전송한 것이 아니다. 15-step soft crossfade가 실제 command step을 최대 7.0083 mm로 제한했고, 현재 7.5 mm/tick 계약을 통과했다.

## 7. B phase의 역할

새 manifest의 T3 phase 0.65는 다음 용도만 가진다.

```text
- nominal Bridge 방향/endpoint reference
- recorded observation을 이용한 offline shadow
- 연구용 provenance/사후 분석
```

다음 용도는 갖지 않는다.

```text
- ACT-B runtime control authority
- “phase 0.65에 도달해야 B 실행” 같은 gate
```

실제 takeover 조건은 기존 V2와 동일하다.

```text
Bridge handoff window
→ 최신 RGB/state로 ACT-B async inference
→ fresh B prefix와 Bridge tail의 compatibility
→ PASS 시 soft crossfade
→ ACT-B 100%
```

compiler는 successor support bank가 있는 FLEXIBLE_LEVEL2 manifest에 한해서 operator entry phase와 다른 interior reference를 허용한다. bank/manifest/sample/objective/margin이 불일치하면 compile 단계에서 fail-closed한다. 기존 STRICT 및 successor bank가 없는 manifest는 여전히 exact entry phase를 요구한다.

## 8. 30 Hz와 CPU 영향

semantic-local search는 artifact 재생성 시에만 수행된다. 30 Hz control loop에는 Dijkstra, 275-pair search, 데이터 파일 읽기 또는 path-length 계산이 추가되지 않았다.

Runtime에는 이미 계산된 하나의 manifest와 cached reference만 들어간다. 실제 전환 시 기존 방식대로 최신 actual/ACK에서 짧은 future-join adaptive entry만 worker에서 생성한다. ACT-B async inference, Bridge queue consumption, bounded ACK pipeline, MUX/Safety 경로는 변경하지 않았다.

## 9. 일반화 범위

공통 selector는 다른 planner-approved FLEXIBLE_LEVEL2 edge에도 재사용할 수 있도록 구현했다. 필요한 입력은 다음과 같다.

```text
source real-episode support bank
successor real-episode support bank
source/successor semantic phase trajectory
기존 command-space Bridge validator
empirical successor phase window
interior path margin
```

또한 registry refresh가 verified semantic-local overlay의 manifest와 policy-shadow handoff ID를 다시 검증한 뒤 적용하도록 확장했다. 따라서 전체 registry를 다시 요약해도 T7→T3가 과거 phase-0 artifact로 조용히 되돌아가지 않는다.

이번 작업에서 실제 30-episode 재선택과 ACT policy shadow까지 완료한 edge는 T7→T3 하나다. 다른 127개 direct edge가 자동으로 같은 성공률을 가진다고 주장하지 않는다. 각 edge는 해당 successor interval의 의미적 연속성, empirical support window, fresh policy shadow를 별도로 만들어야 한다.

## 10. 주요 코드와 artifact

코드:

- `interior_policy/semantic_local_reference.py`: bounded semantic-local selector
- `interior_policy/execution_tail_reference.py`: empirical support window와 successor bank 검증
- `task_c_handoff/models.py`: successor reference/objective manifest provenance
- `interior_policy/multi_v2_compiler.py`: metadata-only interior B reference 허용 및 fail-closed 검증
- `task_c_live_v2_rollout.py`: runtime trace 필드
- `task_c_multi_live_v2_rollout.py`: multi-edge binding trace 필드
- `scripts/rebuild_a0509_t7_t3_execution_tail_reference.py`: 30-episode 재생성
- `scripts/refresh_a0509_flexible_level2_registry.py`: generic verified overlay 적용
- `scripts/render_a0509_t7_t3_semantic_local_reference.py`: command-free 시각화

핵심 artifact:

- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/flexible_reference_manifest.json`
- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/t3_s2_empirical_transport_phase_window.json`
- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/semantic_local_reference_selection.json`
- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/geometry_evaluation.json`
- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/flexible_policy_shadow.json`
- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/level2_registry_overlay.json`
- `docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/t7_to_t3_semantic_local_selected_bridge_20260902.png`

## 11. 테스트 결과

```text
Targeted semantic-local/compiler tests: 6 passed
Relevant V2 regression bundle:          41 passed
All 119 flexible edge compile audit:     1 passed
Registry summary:                        119 verified / 9 semantic candidates / 0 unavailable
```

관련 V2 회귀 묶음에는 flexible Bridge, dynamic future join, multi-stage V2, Bridge retry 및 Dijkstra compiler 테스트가 포함됐다.

## 12. 아직 검증되지 않은 사항

다음은 이번 결과로 확인됐다고 말할 수 없다.

```text
- 실제 T7→Bridge→T3 물리 성공
- 물체가 실제로 계속 파지돼 있는지에 대한 센서 기반 보장
- payload 포함 collision safety
- IK 전 구간 검증
- 실제 tracking error에서 7.5 mm/tick 여유가 충분한지
- 실제 RGB가 episode 23 frame 402 support와 다를 때의 ACT-B 성공률
```

따라서 현재 판정은 `offline command-safe + recorded-observation policy-shadow PASS`이며, physical validation 전에는 물리 성공으로 승격하지 않는다.
