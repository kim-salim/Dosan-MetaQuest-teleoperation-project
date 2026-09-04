# A0509 T7/T8 Dijkstra operator catalog 확장 보고서

작성일: 2026-08-29  
검증 범위: dataset/semantic/checkpoint audit, symbolic planning, 독립 GUI  
실제 로봇 명령: 수행하지 않음

> **후속 결과:** 이 문서의 4개 T7/T8 edge 및 당시 6-edge unified registry는
> 초기 범위 검증 기록이다. T1~T8 direct Level 1 edge 128개를 전수 검사한 최신 결과는
> `docs/a0509_t1_t8_full_level2_coverage_2026-08-29_ko.md`이며, strict Level 2
> registry는 현재 80개 edge를 포함한다.

## 1. 결과

기존 T1~T6 catalog에 T7과 T8을 추가하여 현재 catalog는 다음과 같다.

```text
8 frozen ACT tasks
30 semantic segments
21 interior-policy operators
```

기존 목표의 최저비용 결과는 변하지 않았다.

```text
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer

policy sequence: T4 → T2 → T4
cost: 4.50
classification: LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE
```

T7/T8이 이 경로에 불필요하게 삽입되지 않는 것도 확인했다.

## 2. 실제 자료 확인

### T7

```text
TASK_DESCRIPTION:
Place the blue block on top of the drawer onto the black table.

dataset: /home/rvlab/lerobot_datasets/a0509_blue_block_t7
episodes: 30
frames: 27,000
fps: 30
checkpoint: act_a0509_blue_block_t7_bs32_40k_20260828/040000
semantic artifact: t7_semantic_only_graph_v3_2026-08-29
event grammar: C1 → O1
```

### T8

```text
TASK_DESCRIPTION:
Pick up the blue block on top of the other block and place it on the black table.

dataset: /home/rvlab/lerobot_datasets/a0509_blue_block_t8
episodes: 30
frames: 27,000
fps: 30
checkpoint: act_a0509_blue_block_t8_bs32_50k_20260828/050000
semantic artifact: t8_semantic_only_graph_v3_2026-08-29
event grammar: C1 → O1
```

두 task 모두 semantic checksum이 일치하고 ACT 계약은 다음과 같다.

```text
input: 13D state + front/side/zed RGB
output: 7D action
chunk_size: 100
n_action_steps: 100
```

## 3. 추가한 T7 operators

### `T7.acquire_from_drawer_top`

```text
pre:
  blue_block_location=drawer_top
  holding=none
  gripper=open
  contact_mode=free_space

effect:
  blue_block_location=held
  holding=blue_block
  gripper=closed
  contact_mode=free_transport

interval: S1 START → C1
```

### `T7.deliver_to_black_table`

```text
pre: block=held, holding=blue_block, gripper=closed
effect: block=black_table, holding=none, gripper=open
interval: S2 C1 → O1
```

artifact의 `blue_block=on_top_of_drawer`는 planner state의 `drawer_top`으로 명시적으로 매핑했다. 원본 semantic artifact는 수정하지 않았다.

## 4. 추가한 T8 operators

### `T8.acquire_top_block_from_stack`

```text
pre:
  blue_block_location=stacked
  support_blue_block_location=black_table
  stack=assembled
  holding=none
  gripper=open

effect:
  blue_block_location=held
  support_blue_block_location=black_table
  stack=unassembled
  holding=blue_block
  gripper=closed

interval: S1 START → C1
```

### `T8.deliver_unstacked_to_black_table`

```text
pre: block=held, support=black_table, stack=unassembled
effect: block=black_table, support=black_table, stack=unassembled
interval: S2 C1 → O1
```

T8 artifact의 `moving_blue_block`은 planner의 조작 대상 `blue_block_location`으로 정규화했다. artifact의 `disassembling/disassembled`는 기존 T6와 조합할 수 있도록 planner의 `unassembled`로 추상화했다. 이 추상화는 catalog의 `artifact_relaxations`에 남아 있으며 원본 artifact를 덮어쓰지 않는다.

## 5. 새로 검증한 symbolic plans

### T7 단독

```text
drawer_top
→ T7.acquire_from_drawer_top
→ held
→ T7.deliver_to_black_table
→ black_table
```

### T8 단독

```text
stacked + assembled
→ T8.acquire_top_block_from_stack
→ held + unassembled
→ T8.deliver_unstacked_to_black_table
→ black_table + unassembled
```

### T6→T8 unseen composition

"적층한 뒤 다시 분리"는 최종 state만으로는 시간 순서를 표현할 수 없으므로 다음 두 개의 ordered goal로 검증했다.

```text
Stage 1 goal: stack=assembled
T6.acquire_moving_block
→ T6.stack_on_support_block

Stage 2 goal: block=black_table, stack=unassembled
→ T8.acquire_top_block_from_stack
→ T8.deliver_unstacked_to_black_table

policy sequence: T6 → T8
cost: 4.25
goal satisfied: true
```

동일한 initial state에서 최종 goal만 직접 주면 Dijkstra는 비용이 낮은
`T6.acquire_moving_block → T8.deliver_unstacked_to_black_table`을 선택한다.
이는 이상 경로가 아니라, 최종 상태에 필요하지 않은 적층/해체를 생략한 올바른 최단경로다.

이 결과는 symbolic validity다. T6→T8의 actual Bridge manifest나 successor policy shadow를 생성했다는 뜻은 아니다.

## 6. Runtime 준비도

T7/T8 checkpoint와 semantic support artifact가 catalog에 등록됐으므로 다음 작업에 바로 사용할 수 있다.

- Dijkstra candidate operator
- forward-only cursor (`cursor_t7`, `cursor_t8`)
- phase/support index 생성
- future Multi-V2 policy visit
- GUI initial/goal 실험

2026-08-29 후속 command-free edge 검증 결과, T7/T8 관련 16개 symbolic edge 중
다음 4개는 exact manifest와 actual successor ACT shadow까지 통과하여 Level 2로
승격됐다.

```text
T5.acquire_from_drawer → T7.deliver_to_black_table
T7.deliver_to_black_table → T3.acquire_from_black_table
T8.acquire_top_block_from_stack → T2.deliver_to_black_table
T8.deliver_unstacked_to_black_table → T3.acquire_from_black_table
```

상세 근거는 `docs/a0509_t7_t8_level2_edge_validation_2026-08-29_ko.md`와
`docs/artifacts/t7_t8_level2_edge_validation_2026-08-29/`에 있다.

여전히 준비하지 않은 것:

- T7/T8 실제 로봇 검증
- 위 네 edge 외의 T7/T8 cross-policy runtime evidence

따라서 T7/T8가 catalog에 있다는 사실만으로 모든 조합이 Level 2가 되는 것은
아니다. 위 네 edge만 Level 2이며, 나머지 edge는 compiler가 fail-closed로 거부한다.

## 7. GUI

독립 GUI snapshot도 다음으로 갱신했다.

```text
tasks: 8
segments: 30
operators: 21
```

추가 시나리오:

- T7 · 서랍 위→검은 테이블
- T8 · 적층 해체→검은 테이블
- T6/T8 · 단일 최종상태에서 불필요한 적층을 생략하는 shortcut 확인

T6→T8 순차 구성은 GUI의 `T6 · 블록 적층`을 먼저 검증하고 그 final state를
`T8 · 적층 해체→검은 테이블`의 initial state로 사용하는 두 단계 실험이다.

GUI는 여전히 static JSON만 사용하며 ROS, LeRobot, checkpoint 또는 robot command를 import하지 않는다.

## 8. 검증 결과

```text
interior-policy planner: 34 passed
관련 V2/Multi-V2 회귀: 후속 전체 회귀 결과는 Level 2 보고서 참조
GUI static snapshot: 8 passed
physical robot test: not performed
```

## 9. 판정

T7/T8은 실제 dataset과 semantic artifact를 근거로 Dijkstra operator catalog에
추가됐고 네 개의 구체적인 policy-switch edge는 command-free Level 2까지
검증됐다. 그 외 조합은 계속 Level 1이며, symbolic 선택 가능성과 실제 Multi-V2
edge 실행 가능성은 edge별로 분리된다.
