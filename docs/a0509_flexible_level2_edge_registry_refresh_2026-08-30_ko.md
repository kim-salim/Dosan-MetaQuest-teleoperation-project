# A0509 FLEXIBLE_LEVEL2 전체 Edge Registry 개편 및 오프라인 검증 보고서

- 작성일: 2026-08-30
- 범위: T1~T8 semantic operator 사이의 Level-1 direct edge 128개
- 검증 범위: 정적 분석, bounded geometry search, frozen ACT successor shadow, Dijkstra/Multi-V2 compile
- 실제 로봇 동작: 수행하지 않음
- Live/MUX/ServoL/Tool I/O: 사용하지 않음

## 1. 결론

기존 STRICT registry를 삭제하지 않고 비교 기준으로 보존한 상태에서, 실제 운용 기본 registry를 FLEXIBLE_LEVEL2 기준으로 다시 만들었다.

최종 결과는 다음과 같다.

| 구분 | Edge 수 | 의미 |
|---|---:|---|
| Level-1 semantic direct edge | 128 | symbolic contract상 가능한 전체 cross-policy 경계 |
| FLEXIBLE geometry command-safe | 128 | representative deformation queue가 오프라인 command hard check 통과 |
| `flexible_verified` | 119 | geometry + fresh frozen ACT-B prefix + crossfade + `RUN_B` shadow 통과 |
| `flexible_semantic_candidate` | 9 | geometry는 통과했지만 fresh ACT-B takeover hard check 실패 |
| `temporarily_unavailable` | 0 | geometry 단계에서 안전 후보가 전혀 없었던 edge |

따라서 현재 FLEXIBLE registry의 command-free Level-2 admission 비율은 `119 / 128 = 92.97%`이다. 기존 STRICT 확장 registry는 `86 / 128 = 67.19%`였다. 이는 물리 성공률이 아니라 **오프라인 Level-2 runtime contract 통과율**이다.

FLEXIBLE Dijkstra는 119개 verified edge만 실행 계획에 사용한다. 9개 candidate는 분석과 후속 repair를 위해 registry에 남지만 실기 계획으로 컴파일되지 않는다.

## 2. 개편된 admission 구조

```text
Level-1 semantic inventory: 128 edges
              |
              v
bounded representative shortlist (최대 16)
              |
              v
actual/ACK nominal support snapshot 기반
reference-guided FLEXIBLE queue search
              |
              +-- command hard fail --> temporarily_unavailable
              |
              v
command-safe flexible manifest
              |
              v
frozen ACT-B fresh inference
+ adaptive B[j] splice + crossfade shadow
              |
        +-----+------+
        |            |
      PASS          FAIL
        |            |
flexible_verified   flexible_semantic_candidate
        |            |
Dijkstra admission  audit/repair only
```

실행 가능성을 다음과 같이 분리했다.

- `STRICT_LEVEL2`: 기존 strict registry의 `strict_verified`만 허용한다.
- `FLEXIBLE_LEVEL2`: 새 V3 registry의 `flexible_verified`만 허용한다.
- `flexible_semantic_candidate`: 어느 모드에서도 자동 실행하지 않는다.
- 한 Multi-V2 plan 안에 STRICT/FLEXIBLE manifest를 섞으면 compiler에서 fail-closed한다.
- 웹/CLI가 요청한 bridge mode와 manifest mode가 다르면 compiler에서 거부한다.

기존 STRICT artifact와 runtime code는 삭제하지 않았다. STRICT 회귀 실험은 이전 registry와 `strict_level2`를 명시해 재현할 수 있다.

## 3. STRICT 대비 FLEXIBLE 변화

| 비교 | Edge 수 |
|---|---:|
| STRICT와 FLEXIBLE 모두 verified | 82 |
| STRICT에는 없었으나 FLEXIBLE에서 새로 verified | 37 |
| STRICT verified였으나 이번 FLEXIBLE exact shadow에서는 candidate | 4 |
| 순증가 | 33 |

4개 regression은 기존 STRICT evidence가 사라졌다는 뜻이 아니다. 새 FLEXIBLE manifest와 실제 fresh successor crossfade 조합을 별도로 검사했을 때 hard limit를 통과하지 못했다는 뜻이다. STRICT baseline은 그대로 남아 있다.

## 4. Representative Bridge를 실제로 사용한 방식

대표 Bridge는 고정 궤적이 아니라 reference trajectory로 사용한다.

- 도입부: actual/ACK position, orientation, velocity를 반영한다.
- 중간부: representative Bridge의 방향, 높이, 자세 변화, 진행 순서를 가장 강하게 보존한다.
- 종료부: 선택된 successor prefix `B[j]` 쪽으로 유연하게 연결한다.
- 저속 endpoint의 큰 curvature 하나는 soft cost로만 사용한다.
- cusp, severe reversal, self-intersection, workspace, step, velocity, acceleration, jerk, ACK-span 위반은 hard reject한다.

전체 128개에서 선택된 generator는 다음과 같다.

| Generator | Edge 수 |
|---|---:|
| `reference_deformation` | 110 |
| `tangent_regularized_deformation` | 17 |
| `settle_reference_connector` | 1 |

Generic free connector는 이번 verified registry의 주된 생성기로 사용하지 않았다. representative 특성을 유지하는 후보를 먼저 선택했다.

## 5. 실제 hard limit와 soft metric

이번 전체 refresh에서 사용한 주요 geometry hard limit는 다음과 같다.

| 항목 | 값 |
|---|---:|
| control rate | 30 Hz |
| per-axis position step | 6.67 mm/tick |
| orientation step | 1.0 deg/tick |
| Cartesian speed | 300 mm/s |
| per-axis speed | 200.1 mm/s |
| Bridge acceleration | 300 mm/s² |
| Bridge jerk | 800 mm/s³ |
| bounded ACK lag | 1 step |

Fresh ACT-B admission/crossfade의 주요 hard limit는 다음과 같다.

| 항목 | 값 |
|---|---:|
| first B axis delta | 75 mm |
| bridge-prefix velocity mismatch | 75 mm/s |
| crossfade position step | 6.67 mm/tick |
| crossfade orientation step | 1.0 deg/tick |
| crossfade velocity | 300 mm/s |
| crossfade reconstructed command acceleration | 4000 mm/s² |

Raw ACT-B prefix acceleration은 hard reject하지 않는다. 실제 전송되는 crossfade command sequence에서 재구성한 acceleration만 검사한다.

Curvature의 legacy reference 값 `0.25 /mm`는 FLEXIBLE에서 hard threshold가 아니다. 속도가 5 mm/s 이상인 부분의 p95 curvature와 normal acceleration을 soft ranking에 사용한다. cusp와 실제 command dynamics는 계속 hard이다.

## 6. T7.acquire_from_drawer_top → T3.deliver_to_floor 복구 결과

이 direct edge는 기존 STRICT에서 curvature hard filter 때문에 registry에 들어가지 못했으나 새 FLEXIBLE 검증에서는 verified가 되었다.

선택 결과:

- handoff ID: `h_25132136158b`
- source support episode: 27
- successor support episode: 7
- generator: `reference_deformation`
- selected duration: 6.0 s
- reference shortlist: 16개 중 3개 geometry 평가
- selected reference에서 duration 후보: 4/5/6 s
- 4 s: command jerk hard fail
- 5 s, 6 s: hard pass
- 최종 score가 가장 낮은 6 s 선택
- geometry search latency: 32.09 ms

선택 Bridge 지표:

| 항목 | 결과 |
|---|---:|
| max curvature | 0.671620 /mm |
| max curvature 위치의 speed | 4.511 mm/s |
| moving-region p95 curvature | 0.107151 /mm |
| max normal acceleration | 33.740 mm/s² |
| max total acceleration | 33.946 mm/s² |
| max command acceleration | 33.574 mm/s² |
| max command jerk | 499.885 mm/s³ |
| max per-tick axis step | 1.359 mm |
| max 1-step-lag ACK-span axis step | 2.717 mm |
| max ACK-span orientation step | 0.399 deg |

즉 max curvature 자체는 0.25보다 크지만, 그 위치는 거의 정지에 가까운 endpoint이고 실제 normal acceleration과 command dynamics는 hard limit 안이다. 이것이 FLEXIBLE curvature 정책이 복구하려던 대표 사례다.

Frozen T3 successor shadow 결과:

- terminal state: `RUN_B`
- takeover success: true
- fallback required: false
- selected splice index: 0
- first B XYZ delta: 13.646 mm
- bridge-prefix velocity mismatch: 15.889 mm/s
- crossfade max axis step: 1.718 mm/tick
- crossfade max acceleration: 1007.682 mm/s² (`4000` 미만)
- control p99: 1.326 ms
- control max: 24.493 ms
- deadline miss: 0

이 결과는 실제 물체를 든 실기 성공을 의미하지 않는다. 현재 확인된 것은 저장 observation과 frozen checkpoint를 사용한 command-free successor admission까지다.

## 7. Adaptive ACT-B prefix 결과

119개 verified edge 중 62개는 `B[0]`이 아닌 뒤쪽 prefix를 선택했다.

| splice index | Edge 수 |
|---:|---:|
| 0 | 57 |
| 1 | 23 |
| 3 | 3 |
| 4 | 7 |
| 6 | 8 |
| 8 | 21 |

이 결과는 `B[0]` 한 점에 고정하지 않고 bounded `B[j]` target initiation region을 사용해야 한다는 설계 근거다. 단, 현재 최대 splice index는 8이고 후보 수는 6으로 제한되어 있다.

## 8. candidate로 남은 9개 Edge

아래 edge는 Bridge geometry는 command-safe였지만 fresh ACT-B crossfade에서 hard limit를 넘었다.

| Edge | 실패 원인 | max axis step (mm) | max velocity (mm/s) | max crossfade accel (mm/s²) | velocity mismatch (mm/s) |
|---|---|---:|---:|---:|---:|
| T1.acquire → T2.deliver | crossfade step | 7.296 | 232.5 | 1445.0 | 39.0 |
| T2.deliver → T5.close | step, acceleration | 6.764 | 205.1 | 4481.8 | 25.1 |
| T6.acquire → T2.deliver | crossfade step | 7.907 | 250.0 | 1546.7 | 28.6 |
| T6.stack → T1.open | mismatch, step, velocity | 6.742 | 315.2 | 2774.1 | 102.0 |
| T6.stack → T8.acquire | mismatch, step | 7.749 | 249.4 | 1747.9 | 116.4 |
| T7.acquire → T2.deliver | crossfade step | 7.380 | 236.3 | 1468.4 | 52.5 |
| T7.deliver → T5.close | step, acceleration | 6.764 | 205.1 | 4481.2 | 25.7 |
| T8.acquire → T2.deliver | crossfade step | 8.938 | 296.4 | 2818.2 | 29.5 |
| T8.deliver → T5.close | step, acceleration | 6.764 | 205.1 | 4481.8 | 25.2 |

집계된 실패 원인은 중복 포함하여 다음과 같다.

- `crossfade_xyz_axis_step`: 9
- `crossfade_command_acceleration`: 3
- `bridge_prefix_velocity_mismatch`: 2
- `crossfade_velocity`: 1

따라서 이 9개를 curvature threshold 완화만으로 verified로 올리면 안 된다. Connector/crossfade repair 후 동일 hard validation과 fresh shadow를 다시 통과해야 한다.

## 9. Offline timing

Geometry search는 30 Hz thread가 아니라 bounded worker/episode-level 작업이다.

| Geometry search latency across 128 edges | 값 |
|---|---:|
| p50 | 31.113 ms |
| p95 | 160.560 ms |
| p99 | 163.297 ms |
| max | 225.089 ms |

Frozen shadow의 control tick 결과:

| 항목 | 값 |
|---|---:|
| 전체 deadline miss | 0 |
| edge별 control p99의 최대 | 24.938 ms |
| edge별 control max의 최대 | 28.507 ms |
| 30 Hz deadline | 33.333 ms |

이는 artificial/offline scheduling 결과다. Jetson 실기에서 카메라, CUDA contention, ROS callback, actual ACK가 포함된 p95/p99를 대신하지 않는다.

## 10. Dijkstra와 Multi-V2 연결 검증

기본 initial/goal request를 새 V3 registry로 command-free compile했다.

```text
Initial:
  drawer=closed
  blue_block=floor
  holding=none
  gripper=open

Goal:
  drawer=closed
  blue_block=drawer
  holding=none
  gripper=open
```

선택 결과:

```text
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer

policy sequence: T4 → T2 → T4
total cost: 4.500
```

두 cross-policy transition은 모두 다음으로 컴파일됐다.

```text
T4.open_drawer → T2.acquire_from_floor
  flexible_verified / flexible_level2

T2.acquire_from_floor → T4.deliver_to_drawer
  flexible_verified / flexible_level2
```

이 검증에서도 `PHYSICAL_VALIDATION_PERFORMED=false`이다.

## 11. 실행/GUI 기본값 변경

- initial/goal CLI 기본 registry를 V3 FLEXIBLE registry로 변경했다.
- Multi-V2 runner 기본 bridge mode를 `flexible_level2`로 변경했다.
- 별도 Dijkstra GUI server 기본 registry도 V3로 변경했다.
- GUI server는 plan request의 bridge mode를 runtime subprocess에 그대로 전달한다.
- `A0509_EDGE_REGISTRY` 환경변수로 다른 registry를 명시할 수 있다.
- 기존 GUI process가 떠 있다면 코드를 다시 읽도록 server를 재시작해야 한다.

기본 registry:

```text
docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30/
  edge_registry_level2_v3.json
```

## 12. 생성된 산출물

```text
docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30/
  edge_registry_level2_v3.json
  edge_refresh_summary.json
  edge_refresh_matrix.csv
  edges/<edge_id>/
    geometry_evaluation.json
    flexible_reference_manifest.json
    flexible_policy_shadow.json
    shadow_execution.json
```

재생성 도구:

```text
scripts/refresh_a0509_flexible_level2_registry.py
```

지원 stage:

```text
geometry
shadows
summary
all
```

기존 결과가 있으면 재사용하며 `--force`를 주지 않는 한 shadow/geometry artifact를 불필요하게 다시 계산하지 않는다.

## 13. 테스트 결과

관련 회귀 테스트:

```text
123 passed in 11.12s
```

패키지 전체 회귀 테스트:

```text
292 passed in 17.26s
```

포함 범위:

- FLEXIBLE bridge geometry/hard validator
- strict/flexible mode 분리
- semantic candidate 실행 차단
- V3 registry 128/119/9 계약
- adaptive B splice
- handoff/fallback/generation isolation
- Dijkstra planner와 forward-only operator
- web episode planning
- Multi-V2 compile/runtime parsing
- multi-stage retry 및 bridge retry

별도 end-to-end command-free compile도 성공했다.

## 14. 아직 검증되지 않은 항목

다음은 확인된 것으로 주장하면 안 된다.

- 실제 로봇 성공률
- 실제 object holding 유지
- drawer/container/contact 성공
- IK feasibility (`ik_checked=false`)
- environment collision (`collision_checked=false`)
- 카메라/CUDA/ROS가 동시에 동작하는 실기 p95/p99
- actual↔ACK offset이 training support에서 벗어났을 때 모든 edge의 성공
- 119개 edge 각각의 bounded physical success

현재 registry는 **LEVEL-2 command-free verified**이며 LEVEL-3 physical verified가 아니다.

## 15. 실기 전 권장 순서

1. GUI 또는 CLI에서 원하는 initial/goal plan을 만든다.
2. 생성된 모든 transition이 `flexible_verified`인지 확인한다.
3. policy shadow를 한 번 더 실행해 checkpoint/observation 경로 freshness를 확인한다.
4. command-free/dry-run trace에서 selected generator, duration, B splice index를 확인한다.
5. 단일 edge를 Live OFF shadow로 확인한다.
6. 사용자가 작업 공간을 확인한 뒤 5초 bounded physical test를 수행한다.
7. 단일 Bridge physical test를 통과한 뒤에만 full Dijkstra composition으로 확대한다.

실제 로봇 적용 전까지 candidate 9개를 강제 실행하거나 hard threshold를 단순 상향하지 않는다.
