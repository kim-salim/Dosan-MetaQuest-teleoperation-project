# A0509 상황 적응형 공통 flex bridge v12

작성일: 2026-09-04

## 목적

이 규격의 공통성은 모든 edge에 동일한 bridge 높이, 길이, phase 또는
제어점을 강제한다는 뜻이 아니다. 모든 정책 전환에 동일한 데이터 기반
선정 절차와 동일한 fail-closed 안전 게이트를 적용한다는 뜻이다.

공통 불변 조건은 다음과 같다.

- source semantic 완료가 먼저 확인되어야 한다.
- 현재 TCP가 실제 episode support 안에 있어야 한다.
- 각 정책의 semantic segment와 기록 trajectory에서 phase collar를 파생한다.
- 후보는 안전 게이트를 통과한 `A prefix + bridge + B suffix` 총길이 순으로
  선택한다.
- 속도, 축 속도, 가속도, jerk, ACK ramp, orientation ramp를 동일하게
  검사한다.
- primary commit window에서 안전한 bridge가 준비되지 않으면 semantic과
  실제 support가 계속 유지되는 동안 공통 scan deadline까지 다시 찾는다.
- Dijkstra operator cost와 symbolic effect는 바꾸지 않는다.

상황에 따라 달라져야 하는 출력은 다음과 같다.

- collar 및 commit phase window
- nominal source phase
- source/successor episode와 frame
- bridge 시작점과 끝점
- tangent, 길이, 곡률 및 실제 선택 generator

따라서 edge별 결과가 서로 다른 것은 정상이다. 숫자를 operator ID로 수동
조정해서 다른 것이 아니라, 동일한 알고리즘에 서로 다른 기록 geometry를
입력한 결과여야 한다.

## 버전 규격

- shared execution-tail profile:
  `a0509_shared_execution_tail_latched_v2_20260904`
- bridge generation profile:
  `semantic_local_execution_tail_tangent_v1`
- bridge duration horizon: `4.0 s`
- tangent minimum handle/chord regularizer: `0.04`
- degenerate low-speed tangent adjustment bound: `12 mm/s`
- ACK-span 기반 축 속도 gate: `112.5 mm/s`
- axis velocity hard limit: `225 mm/s`
- Cartesian norm velocity hard limit: `300 mm/s`
- acceleration/jerk hard limit: `4000 mm/s²`, `4000 mm/s³`

`12 mm/s`는 고속 source를 허용하기 위한 속도 여유가 아니다. 지나치게 작은
tangent handle을 올릴 때만 사용하는 저속 regularization 상한이며,
ACK-span 고속 gate는 계속 `112.5 mm/s`이다.

고정 Z 최소값은 이 규격의 일부가 아니다. 각 edge의 semantic contract가
transport floor를 요구할 때만 별도로 적용한다. 현재 T4→T7 edge에는 고정
transport floor가 없다.

## 데이터 파생 결과

| source | continuation | commit window | shared deadline | legacy v11 deadline |
|---|---|---:|---:|---:|
| T2.acquire_from_floor | same segment | 0.54–0.60 | 0.675 | 0.65 |
| T7.acquire_from_drawer_top | next segment | 0.16–0.21 | 0.35 | 0.25 |
| T4.open_drawer | post-O1 next segment S3 | 0.17–0.23 | 0.35 | 0.27 |

T1.open_white_container처럼 exact semantic exit가 필요한 전환은 execution
tail로 강제하지 않는다. T1 O1은 S2 geometry prearm 이후 stable close로 다시
무장하고, 그 뒤의 driver-confirmed open만 인정한다. 초기 approach의 reopen은
semantic history에 남지 않는다.

T4→T7도 동일하게 S2의 supported `closed_then_open` O1을 먼저 확인한 뒤 S3
execution tail을 추적한다. S3 latch는 semantic event를 대신하지 않는다.

## T4→T7 command-free 증거

- source reference: T4.S3 phase `0.17`
- successor reference: T7.S1 phase `0.94`
- 선택 objective 총길이: `394.523 mm`
- primary window replay: `26/30`
- shared deadline scan replay: `30/30`
- nominal phase 이전 통과: `24/30`
- policy shadow: `RUN_B`, takeover 성공, fallback 없음
- control deadline miss: `0`
- 선택 geometry generator: `reference_deformation`
- 최대 Cartesian 속도: 약 `106.417 mm/s`
- 최대 ACK-span 축 step: 약 `6.286 mm`
- 최대 command acceleration: 약 `151.554 mm/s²`
- 최대 command jerk: 약 `2252.338 mm/s³`
- bridge 최소 TCP Z: 약 `352.313 mm`

profile이 tangent-regularized 후보를 제공하더라도 실제 공통 검색기는 매
상황에서 가장 짧고 안전한 generator를 선택한다. 위 shadow에서
`reference_deformation`이 선택된 것은 의도된 상황 적응 동작이다.

v12 registry 전체 command-free compile audit 결과는 다음과 같다.

- flexible-verified edge: `122/122` 컴파일 성공
- execution-tail edge: `57`
- 컴파일 실패: `0`
- robot commands published: `0`

v11 rollback registry도 동일하게 `122/122`, 실패 `0`으로 확인했다.

## 실증용 registry

- v12 shared:
  `docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03/edge_registry_level2_spatial_v12.json`
- v11 rollback:
  `docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03/edge_registry_level2_spatial_v11.json`

bringup과 CPU scheduling은 기존 절차를 그대로 사용한다. v12 웹 서버는
다음과 같이 시작한다.

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui_v12_shared.sh 8766 --enable-motion
```

브라우저에서 재검증·컴파일 후 `RUNTIME PLAN READY`와 blocker 없음이 확인된
경우에만 기존 준비자세, runtime load, Live-OFF preflight 절차를 진행한다.

- bringup 승인: `I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION`
- 웹 실행 승인: `I_ACKNOWLEDGE_A0509_REAL_ROBOT_MOTION`

두 승인 문자열은 서로 바꾸어 사용할 수 없다.

## 회귀 절차

v12에서 이상 동작, 반복적인 geometry rejection, deadline 초과 또는 예상하지
못한 policy 전환이 발생하면 웹 안전 중단을 먼저 사용하고 긴급 상황에서는
물리 E-stop을 우선한다. 로봇이 안전한 상태로 복귀한 뒤 v12 서버를 종료하고
다음 launcher로 v11을 명시적으로 선택한다.

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui_v11_rollback.sh 8766 --enable-motion
```

v11은 registry만 과거 파일로 바꾸는 것이 아니다. compiler가 registry의
profile ID 부재를 legacy v11로 해석하여 T2/T7/T4의 과거 deadline과 latch
동작을 복원한다. 회귀 후에도 재검증·컴파일, 작업자 확인, 준비자세,
runtime load, Live-OFF preflight 및 웹 승인을 처음부터 다시 수행해야 한다.

## 향후 edge 적용

새 execution-tail edge는 다음 공통 도구를 사용한다.

- `scripts/rebuild_a0509_profiled_semantic_local_reference.py`
- `scripts/build_a0509_execution_tail_scan_bank.py`
- `scripts/validate_a0509_profiled_bridge_source_bank.py`
- `scripts/validate_a0509_shared_profile_registry.py`

새 edge의 operator ID, semantic segment와 기록 데이터는 달라도 되지만,
operator별 bridge 숫자를 도구 밖에서 추가해서는 안 된다. empty-gripper
free-space source의 review allowlist는 semantic eligibility 승인이지 수치
튜닝이 아니다.

## 한계

현재 상태는 command-free recorded-episode 검증 완료 상태이며 실제 성공을
의미하지 않는다. IK와 collision은 별도 검증되지 않았고 physical trial 수는
`0`이다. 첫 실증은 낮은 위험의 기존 성공 edge로 v12 회귀 동작을 확인한 뒤
T4→T7→T4 순서로 진행한다.
