# T7 S2 → T3 S2 execution-tail Bridge 갱신 보고서

작성일: 2026-08-31  
검증 범위: dataset/artifact 분석, compiler, offline geometry, recorded-observation ACT-B shadow, saved actual/ACK replay, unit/regression tests  
실제 로봇 명령: **0회** (`Live ON`, MUX LEROBOT, ServoL publish, gripper I/O 미실행)

## 1. 적용한 계약

이번 갱신은 `T7.acquire_from_drawer_top → T3.deliver_to_floor`에서 symbolic operator 종료점과 실제 Bridge source를 분리한다.

- symbolic T7 acquire 종료: `S1 phase 1.0` (planner/audit 의미만 유지)
- runtime source tracking: `T7 S2`
- commit collar: `phase 0.16–0.21`
- nominal runtime source phase: `0.185`
- successor reference: `T3 S2 phase 0.0`
- ACT-A를 끊기 전 complete Bridge candidate가 worker에서 준비되어야 한다.
- Bridge 시작은 nominal S1 좌표가 아니라 최신 actual/ACK snapshot이다.
- 최신 ACK는 cached S2 reference에 투영한 뒤 반드시 그보다 미래인 join point로 연결한다.

## 2. 30-episode runtime source bank

`T7 S2 phase 0.16–0.21`의 실제 30개 episode support를 모두 보존했다. 각 episode의 해당 phase-window trajectory와 component median trajectory 사이 RMS를 계산하고, RMS가 가장 작은 **실제 episode**를 medoid reference로 선택했다. synthetic component median 자체는 robot command로 사용하지 않는다.

- episode 수: 30
- window phase sample 간격: 0.005
- 선택 방식: `minimum_window_rms_to_component_median_real_episode_v1`
- T7 medoid: episode `15`, frame `317`
- medoid window RMS: `6.601414988 mm`
- source XYZ: `[585.222168, -175.116028, 460.498688] mm`
- source velocity: `[-41.373160, 33.464925, 93.828305] mm/s`
- calibrated support threshold: `23.363041815 mm`
- semantic state: closed gripper / blue block held / free transport

T3 S2 entry 역시 같은 방식으로 실제 support medoid를 선택했다.

- T3 medoid: episode `17`, frame `228`
- T3 entry XYZ: `[409.771118, 2.000915, 354.114929] mm`

## 3. Compiler fail-closed 규칙

`source_execution_tail`이 존재할 때 다음을 적용한다.

1. manifest source task는 operator policy와 같아야 한다.
2. manifest source segment는 execution-tail `tracking_segment`와 반드시 같아야 한다.
3. manifest reference phase는 tail nominal phase보다 뒤일 수 없다. 실제 state가 reference의 미래로만 causal join할 수 있어야 한다.
4. T7→T3처럼 runtime source bank가 선언된 edge는 segment, phase-window, nominal phase, 30 episode, medoid episode/frame/pose/orientation/velocity까지 compile 단계에서 일치시킨다.
5. mismatch는 warning으로 넘기지 않고 `ValueError`로 real-motion plan compilation을 차단한다.

같은 segment의 더 이른 reference는 허용한다. 예를 들어 기존 T2 `S2 phase 0.50` reference와 execution-tail `S2 nominal 0.565`는 actual/ACK projection 후 미래 join할 수 있으므로 유지된다. 반면 기존 T7 `S1 phase 1.0` reference와 runtime tail `S2` 조합은 차단된다.

현재 registry 119개 flexible-verified edge 중:

- command-free compile 가능: 75개
- 그중 execution-tail source: T2 same-S2 7개 + 새 T7→T3 1개
- cross-segment old reference라 compile fail-closed: 44개

44개는 각 source operator의 tail segment reference bank/manifest를 재생성하기 전에는 runtime plan으로 승인되지 않는다.

## 4. Runtime actual/ACK future join

heavy reference-family search는 worker에서 한 번 수행하고 cached template로 보존한다. commit 시점 worker 작업은 다음으로 제한한다.

1. 최신 actual/ACK/causal TCP velocity snapshot
2. cached S2 reference의 early/middle(`≤0.60`)에 ACK 투영
3. 투영점보다 미래인 lookahead 후보 `0.05, 0.10, 0.15, 0.20, 0.30` 평가
4. join 상한 `0.80`, 후보 최대 6개
5. C2 adaptive entry connector와 이후 cached reference tail의 전체 command-space hard validation
6. 안전 후보 중 score가 가장 작은 future join 선택

이 탐색은 Bridge worker에서 실행되며 30 Hz command loop는 결과를 기다리거나 candidate search를 수행하지 않는다.

## 5. Offline geometry 결과

- handoff ID: `h_tail_e900a4ddfed8`
- manifest nominal duration: 4.0 s
- bounded duration search가 선택한 safe duration: 6.0 s
- evaluated candidates: 15
- hard-pass candidates: 4
- selected generator: `tangent_regularized_deformation`
- Bridge steps: 180 @ 30 Hz
- length: `354.388587 mm`
- max velocity: `107.867466 mm/s`
- max acceleration: `104.806911 mm/s²`
- max command acceleration: `104.704596 mm/s²`
- max jerk: `664.454261 mm/s³`
- max command jerk: `542.645230 mm/s³`
- max axis step: `3.123655 mm/tick`
- bounded ACK two-step span: `6.224593 mm` (`6.67 mm` 이내)
- max orientation step: `0.104203°/tick`

최대 curvature `1.360756/mm`는 저속 endpoint에서 발생했지만 moving p95 curvature는 `0.200280/mm`, normal acceleration은 `72.898531 mm/s²`였다. FLEXIBLE 규칙에 따라 curvature 하나로 reject하지 않았고 실제 command step/acceleration/jerk를 hard 검사했다.

## 6. Recorded-observation ACT-B shadow

T3 episode 17 frame 228 RGB/state로 command-free ACT-B inference와 handoff를 검증했다.

- warmup: `399.30 ms`, `33.62 ms`
- B total preparation latency: `101.90 ms`
- stale: false
- B first XYZ delta: `6.8974 mm`
- B first rotation delta: `0.4057°`
- Bridge↔B velocity mismatch: `28.5992 mm/s`
- reconstructed crossfade max axis step: `0.9208 mm`
- reconstructed crossfade max acceleration: `539.6473 mm/s²` (`4000` 이내)
- prefix admission: PASS
- terminal state: `RUN_B`
- takeover success: true
- deadline miss: 0
- published robot commands: 0

## 7. 최근 actual/ACK trace 재생

2026-08-31 최근 실패 trace의 adaptive rebase snapshot 5개를 새 S2 reference에 command-free로 재주입했다.

- 과거 fixed single-join runtime: 5개 모두 reject
- 새 bounded future-join: generation 6, 7에서 safe candidate 생성
- 최초 safe generation: 6
- reference projection fraction: `0.066667`
- selected future join fraction: `0.372222`
- future lookahead: `0.305556`
- actual↔ACK position error: `6.620828 mm`
- max ACK span: `6.256897 mm`
- max command acceleration: `135.417756 mm/s²`
- max command jerk: `453.914114 mm/s³`

초기 generation 3–5는 ACK lag/jerk/orientation hard limit 때문에 계속 reject되었다. 이는 의도한 동작이다. ACT-A queue를 먼저 끊지 않고 S2 안에서 계속 진행한 뒤 generation 6에서 안전한 미래 join이 준비될 수 있다.

## 8. 테스트

실행 결과:

```text
pytest -q src/lerobot_robot_doosan_a0509/test
311 passed in 22.36s
```

포함된 핵심 검증:

- T7 S2 30-support medoid compile validation
- S1/S3 source ↔ S2/S4 tail mismatch compile rejection
- same-segment earlier reference phase의 causal future join 허용
- future reference phase rejection
- recorded actual/ACK generation 6 future-join 재생
- cached middle/tail 보존
- StagePhaseSupervisor S2 tracking 및 S1 symbolic audit 분리
- ACT-B shadow fresh prefix admission / soft crossfade / takeover
- 기존 web T4→T2→T4 compile 회귀

## 9. 남은 한계

- IK checker: 없음 (`ik_checked=false`)
- environment collision checker: 없음 (`collision_checked=false`)
- 실제 held-object sensor 확인: 없음
- 실제 로봇 tracking, payload 유지, physical success: 미검증
- offline shadow는 perfect Bridge tracking 가정
- 선택된 6초 duration의 실제 체감 속도와 tracking p95/p99는 bounded physical test 전에는 확정할 수 없음

따라서 이 결과는 command-space 및 compiler/runtime-contract 검증이며 physical validation 성공을 의미하지 않는다.
