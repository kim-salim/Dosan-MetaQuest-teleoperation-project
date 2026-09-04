# A0509 v13 물리 실증 완료 기준선

## 결론

2026-09-04 작업자 확인 기준으로 아래 8개 task 종류는 각각 최소 1회의 실제 로봇
성공이 확인되었다. 따라서 현재 정식 기준선은 계속 **v13**이며, 이번 정리는 v14를
만들거나 Bridge 알고리즘을 변경하지 않는다.

- baseline: `v13 validated baseline`
- runtime command profile: `a0509_ramp8p5_ack_span2_v1`
- linear ramp: `8.5 mm/tick`
- registry: `edge_registry_level2_spatial_v13.json`
- registry SHA-256:
  `b50abbf55d449eb26d820dc4a083dde105665b38d3fe25aee2482a7efd75c640`
- registry admission: 127개 중 `flexible_verified=122`, candidate 5
- 사용자 확인 성공 task 종류: 8개 모두 (각 최소 1회, 성공률 산정 아님)
- v13 profile과 전체 policy takeover trace가 함께 남은 task 종류: 5/8
- 자동 perception 기반 성공 판정: 사용하지 않음

기계 판독 가능한 환경 checkpoint와 물리 실증 원장은 다음에 고정한다.

- `docs/artifacts/a0509_v13_validated_baseline_2026-09-04/checkpoint_manifest.json`
- `docs/artifacts/a0509_v13_validated_baseline_2026-09-04/physical_validation_ledger.json`
- `docs/artifacts/a0509_v13_validated_baseline_2026-09-04/critical_files.sha256`

## Level 명칭 범위

아래 Level 1/Level 2는 이번 실증 묶음을 구분하는 사용자 실험 분류이다. runtime의
`flexible_level2` edge admission이나 GUI의 `Level 2 registry 전환만`과 같은 뜻이
아니다.

사용자가 부르는 `쓰레기통`은 현재 runtime semantic의 `white_container`와 같은
대상이며, operator ID는 기존 호환성을 위해 T1의 `white_container` 명칭을 유지한다.

## 성공 task 목록

모든 행의 결과는 `SUCCESS — operator confirmed`이다. `trace-backed`는 같은 v13
profile이 기록된 run에서 필요한 모든 cross-policy takeover까지 확인됐다는 뜻이다.
이는 물체 최종 상태를 카메라로 자동 판정했다는 뜻은 아니다.

| ID | 분류 | 실제 작업 | 정책/semantic 순서 | 보존된 증거 |
|---|---|---|---|---|
| L1-01 | Level 1 | 왼쪽 바닥 → 오른쪽 바닥 | `T2.acquire_from_floor → T3.deliver_to_floor` | v13 trace-backed, run `20260904_175424_833605_6b6fa269cbf1` |
| L1-02 | Level 1 | 서랍 위 → 오른쪽 바닥 | `T7.acquire_from_drawer_top → T3.deliver_to_floor` | v13 trace-backed, run `20260904_175722_116846_cbe7fba1e09d` |
| L1-03 | Level 1 | 왼쪽 바닥 → 블록 쌓기 | `T2.acquire_from_floor → T6.stack_on_support_block` | v13 trace-backed, run `20260904_194814_836120_035e71463cef` |
| L1-04 | Level 1 | 서랍 위 → 블록 쌓기 | `T7.acquire_from_drawer_top → T6.stack_on_support_block` | v13 trace-backed, run `20260904_195046_835748_a498e165e44b` |
| L1-05 | Level 1 | 쌓은 블록 → 오른쪽 바닥 | `T8.acquire_top_block_from_stack → T3.deliver_to_floor` | 작업자 성공 확인 + v13 offline edge evidence; 매칭되는 v13 선택 run은 미발견 |
| L2-01 | Level 2 | 서랍 위 블록 → 서랍 안 | `T4.open_drawer → T7.acquire_from_drawer_top → T4.deliver_to_drawer → T4.close_drawer` | v13 trace-backed, run `20260904_180140_340850_3fb3ace31d5d` |
| L2-02 | Level 2 | 왼쪽 바닥 블록 → 서랍 안 | `T4.open_drawer → T2.acquire_from_floor → T4.deliver_to_drawer → T4.close_drawer` | 작업자 성공 확인 + v13 plan/offline edge evidence; 보존 run `20260904_175935_749605_6efa59f345fc`는 초기 T4에서 중단되어 전체 takeover 증거가 아님 |
| L2-03 | Level 2 | 왼쪽 바닥 블록 → 쓰레기통(흰색 컨테이너) 안 | `T1.open_white_container → T2.acquire_from_floor → T1.deliver_to_white_container` | 작업자 성공 확인 + 전체 takeover가 남은 선행 run `20260903_172629_416725_28009162f7b9`; 해당 run에는 v13 profile stamp가 없음 |

정확한 run 파일 hash, evidence tier 및 누락 범위는
`physical_validation_ledger.json`을 권위 원장으로 사용한다.

## 원본 run 종료 상태 해석

현재 successor stage는 학습된 done token이 아니라 외부 작업자/서비스가 완료 권위를
가진다. 대표 실기 run에서는 작업자가 보이는 작업 성공 후 Live를 끈 경우가 있어,
원본 trace가 다음처럼 끝난다.

```text
stage_takeover_complete
→ successor policy RUN_STAGE
→ operator Live OFF
→ live_disabled_during_transition
→ FAILED_HOLD
```

따라서 원본 `execute_gate_report.json`의 `completed=false`, `KeyboardInterrupt` 또는
trace의 `FAILED_HOLD`를 사후에 성공으로 덮어쓰지 않았다. 이 checkpoint는 다음 두
사실을 분리한다.

1. policy transition/Bridge 실행 여부: 원본 runtime trace
2. 물체가 목표 상태에 도달했는지: 작업자 확인

논문에서는 이 8개를 “각 task 종류에서 최소 1회 작업자 확인 성공”으로 기술할 수
있지만, 시도 횟수가 정리되기 전에는 `8/8 success rate`로 기술하면 안 된다.

## v13 환경 checkpoint

checkpoint 시각: `2026-09-04T23:15:19+09:00`

| 항목 | 고정 값 |
|---|---|
| host | `jetson-thor-02`, aarch64 |
| OS/kernel | Ubuntu 24.04.4 LTS / Linux 6.8.12-tegra |
| ROS | Jazzy |
| Python | 3.12.3, `/home/rvlab/venvs/lerobot/bin/python` |
| Torch/CUDA | `2.11.0+cu130` / CUDA runtime 13.0 |
| local GPU | NVIDIA Thor, driver 580.00 |
| control processes | bringup OFF, GUI OFF, Multi-V2 runtime OFF |
| Git base | `fe9d75d0308df78f4656eedfda2e6b723ae732e6` on `main` |
| Git state | dirty: tracked changes 21, untracked files 2,787 before checkpoint write |

이 checkpoint는 핵심 코드와 설정의 hash를 고정하지만 clean Git tag를 대신하지
않는다. GUI도 현재 독립 Git 저장소가 아니므로, 장기 보존 단계에서는 별도 commit/tag
또는 archive가 필요하다.

## 검증 checkpoint

이번 최신화 과정에서는 실제 로봇 명령을 발행하지 않았다.

- 공통 Bridge/profile/semantic registry 핵심 회귀: `19 passed`
- GUI Python 전체: `19 passed`
- GUI semantic snapshot unittest: `12 passed`
- v13/v12/v11 launcher shell syntax: 통과
- 저장된 v13 전체 compile audit: verified edge `122/122`, 실패 0
- 저장된 headless browser planner 결과: `16 passed`, 이번 checkpoint에서는 재실행하지 않음

GUI 전체 pytest는 launcher와 동일하게 ROS overlay와 LeRobot Python을 사용해야 한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/rvlab/Dosan-MetaQuest-teleoperation-project/install/setup.bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
/home/rvlab/venvs/lerobot/bin/python -m pytest -q
```

핵심 파일 checksum 검증:

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
sha256sum -c \
  docs/artifacts/a0509_v13_validated_baseline_2026-09-04/critical_files.sha256
```

## Registry 보존 원칙

v13 registry 안의 `physical_validation_performed=false`와 각 edge의
`physical_trials=0`은 registry 승격 당시의 command-free 증거 범위를 나타낸다.
사후 물리 성공을 기록하기 위해 이 immutable registry를 다시 쓰지 않았다. 물리 결과는
registry SHA를 참조하는 별도 ledger에 추가했다.

따라서 다음이 모두 참이다.

- v13 registry 자체는 command-free/offline evidence이다.
- 별도 v13 physical ledger에는 8개 task 종류의 작업자 확인 성공이 있다.
- 이번 최신화는 알고리즘·edge 비용·profile을 바꾸지 않으므로 v14가 아니다.

## 실행 및 회귀

v13 실행:

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
source /home/rvlab/venvs/lerobot/bin/activate
MOTION_AUTHORIZATION=I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION \
CONTROL_DRY_RUN=false \
./scripts/run_task_c_control_bringup_ramp8p5_v13.sh
```

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui_v13_ramp8p5.sh 8766 --enable-motion
```

7.5 mm/tick 회귀:

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
MOTION_AUTHORIZATION=I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION \
CONTROL_DRY_RUN=false \
./scripts/run_task_c_control_bringup_ramp7p5_rollback.sh
```

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui_v12_shared.sh 8766 --enable-motion
```

T9 학습 및 향후 T9 semantic node/edge 편입은 이 v13 checkpoint 범위 밖이다. T9을
registry topology에 정식 추가하거나 공통 알고리즘/안전 계약을 변경할 때만 다음
버전을 검토한다.
