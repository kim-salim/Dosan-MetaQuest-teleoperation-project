# A0509 T2/T3 공간 의미 분리 및 Dijkstra·Level-2 반영 보고서

작성일: 2026-09-02  
범위: command-free static inspection, semantic contract migration, Dijkstra/Level-2 compile, GUI/offline test  
실제 로봇 구동: 수행하지 않음

## 1. 결론

T2와 T3의 물리적 바닥 위치를 하나의 `floor`로 합치던 planner 의미를 다음처럼 분리했다.

| 정책 | 학습된 실제 동작의 planner 의미 | 안정적인 runtime ID |
|---|---|---|
| T2 | `left_floor → black_table` | `T2.acquire_from_floor`, `T2.deliver_to_black_table` |
| T3 | `black_table → right_floor` | `T3.acquire_from_black_table`, `T3.deliver_to_floor` |

사람이 읽는 planner alias는 각각 다음과 같다.

- `T2.acquire_from_left_floor`
- `T3.deliver_to_right_floor`

runtime ID는 기존 Bridge manifest, trace, registry와의 호환 때문에 바꾸지 않았다.

새 상태 계약을 적용한 결과:

- Level-1 direct cross-policy edge: 128 → 127
- context-independent edge: 36 → 35
- context-dependent edge: 92 유지
- FLEXIBLE Level-2 verified: 119 → 118
- FLEXIBLE semantic candidate: 9 유지
- 제거된 edge: `T3.deliver_to_floor → T2.acquire_from_floor` 한 개
- 유지된 핵심 direct edge: `T2.acquire_from_floor → T3.deliver_to_floor`

제거된 역방향 edge는 이제 “오른쪽 바닥에 놓은 블록을 T2가 왼쪽 바닥에서 곧바로 다시 집는다”는 공간 모순이므로 삭제되는 것이 맞다.

## 2. 원본 자료 보존 방식

원본 LeRobot dataset과 semantic-only artifact는 수정하지 않았다.

T2 raw dataset task description:

```text
bring the blue block to black table
```

T3 raw dataset task description:

```text
Move the blue block off the black table
```

원본 semantic artifact의 위치 표현도 그대로다.

- T2: `source_region`
- T3: `off_black_table`

공간 해석은 새 catalog의 명시적 `physical_setup_overrides`로만 추가했다.

- `T2_SOURCE_REGION_IS_LEFT_FLOOR`
- `T3_OFF_BLACK_TABLE_IS_RIGHT_FLOOR`

따라서 raw artifact 체크섬, dataset task contract, ACT checkpoint 입력 계약을 깨지 않는다.

## 3. 실제 데이터 근거

### T2

- episode: 31
- total frames: 27,899
- FPS: 30
- acquire operator semantic labels:
  - `approach_blue_block_in_source_region`
  - `lift_transport_and_align_blue_block_to_black_table`
- acquire interval median: frame 0 → 448
- deliver interval median: frame 448 → 685

Planner contract:

```text
left_floor
  -- T2.acquire_from_floor -->
held / gripper closed / free_transport
  -- T2.deliver_to_black_table -->
black_table / gripper open / free_space
```

### T3

- episode: 30
- total frames: 27,000
- FPS: 30
- acquire operator semantic label:
  - `approach_blue_block_on_black_table`
- deliver operator semantic label:
  - `lift_transport_and_align_blue_block_off_black_table`
- acquire interval median: frame 0 → 243
- deliver interval median: frame 243 → 602.5

Planner contract:

```text
black_table
  -- T3.acquire_from_black_table -->
held / gripper closed / free_transport
  -- T3.deliver_to_floor -->
right_floor / gripper open / free_space
```

## 4. WorldState 변경

현재 planner는 다음 값을 서로 다른 위치로 취급한다.

```text
left_floor
right_floor
black_table
drawer
white_container
drawer_top
moving_source
stacked
held
unknown
```

`floor`는 과거 catalog/artifact를 재현해 읽기 위한 legacy parser 값으로만 남겼다. 현재 operator와 GUI 선택지는 이 값을 사용하지 않는다.

이 변경으로 다음 두 요청은 완전히 다른 goal이 된다.

```text
left_floor → black_table
black_table → right_floor
```

그리고 이전에는 시작과 목표가 둘 다 `floor`라서 no-op이 되던 다음 요청이 실제 계획으로 계산된다.

```text
left_floor → right_floor
```

## 5. Dijkstra 결과

초기 상태:

```text
blue_block_location=left_floor
holding=none
gripper=open
contact_mode=free_space
```

목표 predicate:

```text
blue_block_location=right_floor
holding=none
gripper=open
```

Level-2 전용 uniform-cost search 결과:

```text
T2.acquire_from_floor
→ T3.deliver_to_floor
```

Policy sequence:

```text
T2 → T3
```

비용:

```text
T2 operator base cost       1.00
T3 operator base cost       1.00
T2→T3 policy switch cost    0.25
--------------------------------
total                       2.25
```

첫 operator의 effect가 `held`이므로 T2 전체 정책을 검은 테이블 방출까지 실행한 뒤 T3가 다시 집는 경로가 아니다. T2의 파지·들기 semantic prefix에서 V2 Bridge로 전환하고 T3의 운반·오른쪽 바닥 방출 tail로 이어지는 계획이다.

## 6. T2→T3 Level-2 근거

Registry edge:

```text
T2.acquire_from_floor
→ T3.deliver_to_floor
```

분류:

- transition type: `held_blue_block_free_transport`
- context-independent: true
- admission status: `flexible_verified`
- validation method: `reference_deformation`
- source reference: T2 S2 phase 0.5
- successor reference: T3 S2 phase 0.0
- fresh successor shadow: PASS
- physical validation: 수행하지 않음

저장된 command-free geometry/shadow evidence의 선택 결과:

| 항목 | 값 |
|---|---:|
| Bridge duration | 5.0 s |
| Bridge steps | 150 |
| Bridge length | 209.007 mm |
| max position axis step | 1.462 mm/tick |
| max orientation step | 0.191°/tick |
| max velocity | 52.772 mm/s |
| max command acceleration | 47.692 mm/s² |
| max command jerk | 709.721 mm/s³ |
| crossfade max axis step | 1.122 mm/tick |
| fresh B first XYZ delta | 9.644 mm |
| fresh B first rotation delta | 0.968° |
| B takeover | 성공, terminal `RUN_B` |
| fallback | 사용 안 함 |
| control deadline miss | 0 / 145 samples |

저속 endpoint의 raw maximum curvature는 높지만, FLEXIBLE 방식은 이를 단독 hard reject로 쓰지 않는다. 최종 command-space step, acceleration, jerk, freshness, generation, prefix/crossfade admission을 기준으로 검증된 기존 증거를 재사용했다.

## 7. Level-2 registry 마이그레이션

새 registry는 기존 물리 trajectory와 stable operator ID가 변하지 않은 edge에 대해 기존 reviewed geometry/shadow 증거를 참조한다. 동시에 새 symbolic inventory에 존재하지 않는 edge는 복사하지 않는다.

새 산출물:

- `level1_edge_inventory.json`
- `edge_registry_level2_spatial_v4.json`
- `edge_refresh_summary.json`
- `edge_refresh_matrix.csv`
- `migration_report.json`

중요한 해석:

- “verified 118개”는 기존 command-free geometry/shadow 검증 증거가 현재 공간 계약에서도 의미적으로 유효한 118개 edge에 승계되었다는 뜻이다.
- 이번 변경에서 GPU successor shadow를 118개 전부 새로 다시 실행한 것은 아니다.
- 실제 로봇 성공을 의미하지 않는다.
- 9개 semantic candidate는 여전히 실행 계획에서 fail-closed로 제외된다.

## 8. 기존 계약과 backward compatibility

보존한 항목:

- 기존 legacy catalog 파일
- 과거 128-edge registry와 분석 artifact
- stable operator ID
- ACT checkpoints
- raw datasets
- semantic graph 및 phase support
- V2 Bridge/handoff/crossfade/runtime
- MUX, Live, Safety Guard, ServoL
- STRICT/과거 회귀 재현 경로

새 기본값만 spatial catalog와 spatial Level-2 registry를 사용하도록 바꿨다.

과거 artifact 기반 회귀 테스트는 legacy catalog를 명시적으로 로드하므로 128-edge 결과도 계속 재현된다.

## 9. GUI 반영

독립 GUI에서 다음이 반영됐다.

- `left_floor (왼쪽 바닥)`
- `right_floor (오른쪽 바닥)`
- T2 planner description
- T3 planner description
- planner alias와 stable runtime ID 동시 표시
- `Level 2 · 왼쪽 바닥→오른쪽 바닥` 시나리오
- verified edge 118개 / registry record 127개
- raw dataset task description을 별도 provenance 필드로 보존

웹 control server도 같은 spatial catalog와 registry를 authoritative source로 사용한다.

## 10. 검증 결과

실행한 command-free 검사:

```text
전체 lerobot_robot_doosan_a0509 package:
332 passed

planner + Level-2 + web integration:
53 passed

legacy 128-edge/STRICT evidence regression:
10 passed

GUI/server pytest:
15 passed

GUI snapshot unittest:
10 passed

headless Chromium planner-core:
PASSED, failed=0
```

Default CLI compiler 결과:

```text
OPERATORS=T2.acquire_from_floor -> T3.deliver_to_floor
POLICIES=T2 -> T3
TOTAL_COST=2.250
EXECUTION_KIND=multi_v2
RUNTIME_COMPILABLE=true
STAGES=T2 -> T3
TRANSITIONS=1
```

## 11. 아직 검증하지 않은 것

이번 작업은 실제 로봇을 움직이지 않았다. 따라서 다음은 아직 주장할 수 없다.

- 왼쪽 바닥의 실제 물체 인식/파지 성공
- T2 actual rollout에서 source collar 진입 성공
- 실제 actual/ACK residual
- 물체 유지 상태
- 실제 T2→T3 Bridge tracking
- 실제 T3 오른쪽 바닥 방출
- collision/IK 보장
- 전체 물리 성공률

현재 판정은 다음이다.

```text
LEVEL 1: PASS
LEVEL 2 runtime contract: PASS
LEVEL 3 physical validation: NOT PERFORMED
```

## 12. 주요 파일

- `config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json`
- `config/interior_policy/a0509_web_target_left_floor_to_right_floor_request_v2.json`
- `scripts/migrate_a0509_spatial_floor_level2.py`
- `scripts/compile_a0509_initial_goal_to_multi_v2.py`
- `docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02/`
- `docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02/left_floor_to_right_floor_compile/`
- `docs/artifacts/interior_policy_symbolic_probe_spatial_floor_2026-09-02/`
- `/home/rvlab/a0509-semantic-dijkstra-gui/data/semantic_snapshot.json`

## 13. 재생성 명령

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source install/setup.bash
export PYTHONPATH=/home/rvlab/Dosan-MetaQuest-teleoperation-project/src/lerobot_robot_doosan_a0509:/home/rvlab/Dosan-MetaQuest-teleoperation-project:$PYTHONPATH

python scripts/migrate_a0509_spatial_floor_level2.py
python scripts/run_a0509_interior_policy_symbolic_probe.py \
  --output docs/artifacts/interior_policy_symbolic_probe_spatial_floor_2026-09-02/symbolic_probe_result.json

python scripts/compile_a0509_initial_goal_to_multi_v2.py \
  --request config/interior_policy/a0509_web_target_left_floor_to_right_floor_request_v2.json \
  --output-dir /tmp/a0509_left_to_right_compile
```

위 명령은 planning/compile 검증이며 자체적으로 Live ON, MUX LEROBOT, ServoL 또는 gripper Tool I/O를 실행하지 않는다.

