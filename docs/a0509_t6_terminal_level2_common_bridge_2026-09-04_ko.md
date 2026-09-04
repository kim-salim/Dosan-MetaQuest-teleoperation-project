# A0509 T6 종착 적층 Level-2 공통 Bridge 활성화

## 결론

T6를 최종 작업으로 사용하는 합성은 별도의 T6 전용 수치 튜닝을 추가하지 않는다.
활성 v13 registry에서 이미 `flexible_verified`인 공통
`held_object_free_transport → T6.stack_on_support_block` edge를 그대로 사용한다.

2026-09-04 사후 실증에서 `T2.acquire_from_floor → T6.stack_on_support_block`과
`T7.acquire_from_drawer_top → T6.stack_on_support_block`이 모두 실제 로봇에서
작업자 확인 성공했다. 이 결과는 별도 v13 physical validation ledger에 기록하며,
command-free registry 원본은 수정하지 않는다.

GUI는 다음 두 시나리오를 제공한다.

- `Level 2 · 왼쪽 바닥→블록 적층`: 검증용 고정 조합
  `T2.acquire_from_floor → T6.stack_on_support_block`
- `Level 2 · 공통 acquire→T6 적층`: 선택한 runtime registry에서 verified인
  T6 incoming edge를 자동으로 활성화하는 공통 조합

## 활성 edge 집합

v13에서 다음 7개 cross-policy edge가 모두 `flexible_verified`이다.

- `T1.acquire_from_black_table → T6.stack_on_support_block`
- `T2.acquire_from_floor → T6.stack_on_support_block`
- `T3.acquire_from_black_table → T6.stack_on_support_block`
- `T4.acquire_from_black_table → T6.stack_on_support_block`
- `T5.acquire_from_drawer → T6.stack_on_support_block`
- `T7.acquire_from_drawer_top → T6.stack_on_support_block`
- `T8.acquire_top_block_from_stack → T6.stack_on_support_block`

이 edge들은 동일한 `execution_tail_s{source}_semantic_local_interior_dynamic_future_join`
알고리즘 family를 사용한다. source policy의 실제 held-transport tail에 맞춰 S2 또는
S4가 자동 선택되며 모든 edge에 같은 고정 phase window를 강제하지 않는다.
Dijkstra operator cost, phase gate, intermediate-release 금지 조건은 변경하지 않는다.

## 상태 계약

T6 tail 진입 시 다음 상태가 유지되어야 한다.

```text
blue_block_location=held
holding=blue_block
gripper=closed
contact_mode=free_transport
support_blue_block_location=black_table
stack=unassembled
```

T6 종료 목표는 다음과 같다.

```text
blue_block_location=stacked
support_blue_block_location=black_table
stack=assembled
holding=none
gripper=open
```

support block은 실제 검은 테이블의 T6 학습 위치에 먼저 배치해야 한다.
planner의 상태 확인은 작업자 선언이며 카메라 인식으로 자동 검증되는 값이 아니다.

## 저장된 실행 요청

대표 요청은 다음 파일에 고정한다.

`config/interior_policy/a0509_web_target_left_floor_to_stack_request_v1.json`

이 요청은 T2 acquire와 T6 stack만 활성화하므로 Dijkstra가 다른 delivery나
T6 자체 acquire로 우회할 수 없다.

재현 가능한 command-free compile 결과는 다음 경로에 저장한다.

`docs/artifacts/t2_to_t6_terminal_stack_level2_v13_2026-09-04/`

이 디렉터리는 authoritative plan result, two-stage Multi-V2 plan과
base runtime support manifest를 포함한다.

## 검증 범위

아래 항목은 최초 활성화 당시의 command-free 재계획 및 Multi-V2 컴파일 범위이다.

- 로봇 명령 발행: 0
- 물리 실증: 0 (최초 활성화 시점)
- collision/IK 물리 인증: 수행하지 않음
- T6 전용 bridge 높이·길이·속도 튜닝: 없음
- runtime profile: v13 `a0509_ramp8p5_ack_span2_v1`

사후 물리 실증 결과는 다음과 같다.

- T2→T6: 작업자 확인 성공, v13 full-transition trace 보존
- T7→T6: 작업자 확인 성공, v13 full-transition trace 보존
- 두 run 모두 successor T6 takeover 완료를 기록
- 자동 done token 부재로 작업자 Live OFF 뒤 trace는 `FAILED_HOLD`로 종료

상세 run ID와 checksum은
`docs/a0509_v13_validated_baseline_2026-09-04_ko.md`에서 관리한다.

## 회귀

8.5 mm/tick 계약에서 이상이 있으면 bringup과 GUI를 모두 종료한 뒤
기존 v12/7.5 조합으로 재시작한다.

```bash
./scripts/run_task_c_control_bringup_ramp7p5_rollback.sh
```

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui_v12_shared.sh 8766 --enable-motion
```

v12도 동일한 122개 edge admission을 유지하므로 T6 종착 의미 연결은 보존되고,
명령 ramp 계약만 7.5 mm/tick으로 회귀한다.

## Command-free 검증 결과

- semantic snapshot unittest: 12 passed
- GUI Python 전체: 19 passed
- 공통 registry/전체 verified compile/web episode 표적 회귀: 19 passed
- headless browser planner: 16 passed, 0 failed

authoritative v13 재계획은 정확히
`T2.acquire_from_floor → T6.stack_on_support_block`을 선택했고 Multi-V2
컴파일과 runtime profile 결합을 통과했다. 이후 실제 로봇에서도 T2→T6와
T7→T6가 작업자 확인 성공했으며, 이 결과는 immutable registry가 아니라 별도
physical validation ledger에 보존한다.
