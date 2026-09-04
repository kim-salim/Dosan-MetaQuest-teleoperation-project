# A0509 Semantic Dijkstra Web Control Supervisor

작성일: 2026-08-29  
범위: T1~T8 symbolic planning → reviewed Level-2 edge → 기존 Task-C Multi-V2 실행 연결  
실기 검증 상태: **미수행**

## 1. 구현 결과

기존 `/home/rvlab/a0509-semantic-dijkstra-gui`의 정적 시각화 화면은 그대로 보존하고,
그 화면에서 선택한 초기 상태·목표 상태·operator 설정을 실제 프로젝트의 Python
planner로 다시 계산하는 loopback 전용 감독 서버를 추가했다.

```text
기존 브라우저 Dijkstra Lab
        │ 초기/목표/operator 입력
        ▼
loopback Python supervisor
        │ 브라우저 결과를 신뢰하지 않고 재계산
        ▼
T1~T8 WorldState uniform-cost search
        │ reviewed 86-edge Level-2 registry만 허용
        ▼
MultiStagePlan compile
        │ exact handoff manifest + phase supervisor
        ▼
기존 Task-C Multi-V2 runtime (Live OFF)
        │
        ├─ 준비자세 + verified gripper open
        ├─ command-free preflight
        └─ explicit authorization
                 ▼
external MUX/Live gate
                 ▼
기존 30 Hz Multi-V2 control path
```

웹 서버는 Cartesian target, gripper target 또는 ServoL 명령을 직접 publish하지 않는다.
Dijkstra도 30 Hz thread가 아니라 episode 시작 전에 한 번만 실행된다.

## 2. 현재 데이터 권위

계획 시 사용하는 source of truth는 다음과 같다.

```text
operator catalog:
config/interior_policy/a0509_t1_t6_operator_catalog_v1.json

catalog id:
a0509_t1_t8_semantic_v3_current_workspace_20260829

Level-2 edge registry:
docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29/
  edge_registry_all_level2_extended.json

operator 수: 21
reviewed Level-2 edge 수: 86
```

파일명에는 과거 호환을 위해 `t1_t6`가 남아 있지만 실제 catalog에는 T1~T8이 포함되어
있다. 브라우저의 JavaScript 계획은 시각화용이며, 실행 권위는 서버가 현재 위 파일을
다시 읽어 산출한 Python 계획에만 있다.

## 3. 대표 검증 결과

초기 상태:

```text
drawer=closed
white_container=closed
blue_block_location=floor
holding=none
gripper=open
contact_mode=free_space
```

목표:

```text
drawer=closed
blue_block_location=drawer
holding=none
gripper=open
```

T1~T8 전체 operator와 86개 Level-2 edge에서 서버가 선택한 최저비용 계획:

```text
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer
```

정책 visit:

```text
T4 → T2 → T4
```

비용:

```text
operator base cost: 4.0
policy switch: 2 × 0.25
total: 4.5
```

컴파일 결과는 다음과 같다.

```text
visit 0: T4.open_drawer
  exit authority = phase_supervisor

edge 0: T4.open_drawer → T2.acquire_from_floor
  exact reviewed V2 handoff manifest

visit 1: T2.acquire_from_floor
  exit authority = phase_supervisor

edge 1: T2.acquire_from_floor → T4.deliver_to_drawer
  exact reviewed V2 handoff manifest

visit 2: T4.deliver_to_drawer → T4.close_drawer
  terminal=true
  exit authority=policy_inference_end
  configured horizon=1800 steps
```

또한 `drawer_top → floor` 초기/목표에서는 다음의 다른 정책 조합도 생성되었다.

```text
T7.acquire_from_drawer_top
→ T7.deliver_to_black_table
→ T3.acquire_from_black_table
→ T3.deliver_to_floor

policy sequence: T7 → T3
```

따라서 웹 backend가 T4→T2→T4 한 경로만 하드코딩한 것은 아니다.

## 4. 실행 가능 판정과 차단 조건

서버는 symbolic plan이 발견됐다는 이유만으로 실기 버튼을 허용하지 않는다.

현재 웹 V1에서 Multi-V2 실행 가능 판정은 최소 다음을 만족해야 한다.

```text
1. plan이 비어 있지 않음
2. 두 개 이상의 policy visit을 포함
3. 모든 cross-policy edge가 86-edge registry에 존재
4. 시작 operator가 해당 whole-task ACT의 첫 semantic interval
5. 마지막 operator가 최종 policy의 마지막 semantic interval
6. initial holding=none
7. initial gripper=open
8. initial contact_mode=free_space
9. compiled MultiStagePlan과 모든 handoff manifest가 parser를 통과
```

대표 blocker:

```text
LEVEL2_NO_PLAN
GOAL_ALREADY_SATISFIED
SINGLE_POLICY_INTERIOR_EXECUTION_NOT_ROUTED_BY_MULTI_V2
INITIAL_INTERIOR_POLICY_REENTRY_NOT_START_CERTIFIED
FINAL_OPERATOR_IS_NOT_POLICY_TAIL_NO_LEARNED_DONE_TOKEN
INITIAL_HOLDING_STATE_NOT_SUPPORTED_BY_WEB_V1
INITIAL_GRIPPER_MUST_BE_OPEN_FOR_WEB_V1
INITIAL_CONTACT_MODE_MUST_BE_FREE_SPACE_FOR_WEB_V1
```

Single-policy symbolic plan도 화면에는 표시되지만 현재 Multi-V2 runner로 잘못 넘기지 않는다.

## 5. 안전 경계

### 웹/네트워크

- `127.0.0.1` 또는 `localhost`에만 bind한다.
- 외부 bind 요청은 서버 시작 단계에서 거부한다.
- 상태 변경 POST는 서버가 발행한 CSRF token이 필요하다.
- 브라우저 결과는 실행 권위가 아니며 서버가 Python으로 재계산한다.
- 서버 시작 시 `--enable-motion`이 없으면 runtime endpoint는 HTTP 403을 반환한다.

### 실제 제어

```text
web supervisor
→ existing Multi-V2 policy proposal
→ /control/lerobot/target_posx
→ MUX
→ /vr/target_posx
→ safety guard
→ /vr/safe_posx
→ ServoL streamer
→ /vr/commanded_posx
→ Doosan
```

- 웹은 위 경로를 우회하지 않는다.
- runtime loader 자체는 LEROBOT을 선택하거나 Live를 켜지 않는다.
- 실제 실행은 exact plan/composition 확인, fresh state, hold delta, downstream parameter
  contract, MUX DISABLED, Live OFF를 확인하는 외부 gate가 소유한다.
- execute gate에는 정확한 승인 문구와 살아 있는 web supervisor PID가 모두 필요하다.
- supervisor가 사라지거나 target/safe/actual/commanded stream이 stale해지면 gate가
  fail-closed로 종료한다.
- 중단 시 gate/runtime 종료와 Live OFF/MUX DISABLED를 반복 요청한다.
- 비상 정지는 `/vr/stop_robot`도 요청하지만, 물리 E-stop을 대체하지 않는다.

## 6. 사용 방법

### 6.1 계획·시각화·컴파일만 사용

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui.sh 8766
```

브라우저:

```text
http://127.0.0.1:8766/control.html
```

이 모드에서는 runtime 시작 요청이 차단된다.

### 6.2 실기 endpoint까지 활성화

실제 full bringup은 현재 웹이 시작하거나 재시작하지 않는다. 기존 프로젝트 방식으로
정확히 하나의 clean full bringup을 먼저 실행하고, MUX/guard/streamer/gripper service와
물리 E-stop을 확인해야 한다.

```bash
cd /home/rvlab/a0509-semantic-dijkstra-gui
./start_control_gui.sh 8766 --enable-motion
```

`--enable-motion`은 endpoint만 활성화하며 그 자체로 로봇을 움직이지 않는다.

화면 실행 순서:

1. 왼쪽에서 초기 WorldState와 목표 predicate를 설정한다.
2. 사용할 operator를 선택하고 forward-only를 유지한다.
3. `서버에서 재검증·컴파일`을 누른다.
4. operator 및 policy 순서와 blocker 유무를 검토한다.
5. 실제 workcell이 입력한 초기 상태와 일치하는지 확인한다.
6. `정책 runtime 로드`를 누른다. 이 단계에서도 Live는 OFF이다.
7. 손과 장애물을 치우고 `준비자세 + gripper open`을 누른다.
8. `Live OFF preflight`를 실행한다.
9. 화면에 안내된 실기 승인 문구를 직접 입력한다.
10. `전체 Multi-V2 실행`을 누른다.
11. 이상 시 `안전 중단` 또는 물리 E-stop을 사용한다.

## 7. Audit artifact

각 서버 계획은 변경 불가능한 새 디렉터리에 저장된다.

```text
/home/rvlab/a0509_web_runs/<timestamp>_<fingerprint>/
  authoritative_plan_result.json
  multi_stage_plan.json
  base_runtime_support_manifest.json
  runtime.log
  prepare_robot.log
  preflight_gate.log
  execute_gate.log
  preflight_gate_report.json
  execute_gate_report.json
  safe_stop.log 또는 emergency_stop.log
```

`authoritative_plan_result.json`에는 입력, 후보 계획, 선택 계획, cost, blocker,
catalog/registry fingerprint가 기록된다. `multi_stage_plan.json`에는 exact policy visit,
phase supervisor, transition manifest가 기록된다.

## 8. 구현 파일

Standalone web:

```text
/home/rvlab/a0509-semantic-dijkstra-gui/
  control.html
  server.py
  start_control_gui.sh
  assets/control-app.js
  assets/control-styles.css
  tests/test_server.py
```

Project:

```text
src/lerobot_robot_doosan_a0509/
  lerobot_robot_doosan_a0509/interior_policy/episode_control.py
  lerobot_robot_doosan_a0509/interior_policy/web_runtime_manifest.py
  test/test_episode_web_control.py
  test/test_multi_stage_web_gate.py

scripts/
  compile_a0509_initial_goal_to_multi_v2.py
  a0509_multi_stage_live_trial_gate.py
  run_a0509_multi_stage_web_gate.sh
  a0509_web_prepare_robot.sh
  a0509_web_force_safe.sh

config/interior_policy/
  a0509_web_target_drawer_floor_to_drawer_request_v1.json
```

## 9. 검증 결과

실행한 검증은 모두 실제 로봇 명령을 생성하지 않았다.

```text
신규 web/project tests:                  14 passed
standalone HTTP server tests:             3 passed
planner/Level-2/Multi-V2 regressions:     98 passed
gate/fail-closed/phase regressions:       23 passed
                                           --------
total:                                   138 passed
```

추가 smoke 결과:

```text
CLI initial/goal → MultiStagePlan compile: PASS
MultiStagePlan.load:                       PASS
RuntimeBridgePlanner.from_manifest:        PASS
RepresentativeBoundaryContract parse:     PASS
loopback POST /api/plan:                   PASS
motion-disabled POST /runtime/start:       HTTP 403 PASS
Chromium headless JavaScript render:       PASS
rendered status: 백엔드 · 86 edges / 실기 비활성 / IDLE
```

## 10. 아직 검증되지 않은 사항

다음은 본 구현에서 확인된 사실로 주장하지 않는다.

- 실제 로봇에서 임의 계획의 성공률
- T4→T2와 T2→T4의 이번 웹 경로 실기 성공
- object identity/initial scene 자동 인식
- IK 및 환경 충돌 보장
- ACT가 재진입 observation에서 올바른 semantic behavior를 선택하는지
- 실제 GPU warmup 및 모든 multi-policy latency p95/p99
- final ACT의 학습된 done 판정

현재 ACT에는 learned done token이 없으므로 마지막 stage는 설정된 inference horizon으로
종료한다. 초기 상태는 perception이 아니라 작업자가 확인한 symbolic assertion이다.

따라서 현재 판정은 다음과 같다.

```text
LEVEL 1 SYMBOLICALLY_PLANNABLE: PASS
LEVEL 2 RUNTIME_CONTRACT_COMPATIBLE: PASS (command-free artifacts/tests)
LEVEL 3 PHYSICALLY_VALIDATED: NOT PERFORMED
```

## 11. 복구

구현 전 standalone GUI 백업:

```text
/home/rvlab/project_backups/
  a0509-semantic-dijkstra-gui_pre_runtime_web_20260829.tar.gz
```

기존 `index.html`과 정적 Dijkstra Lab은 삭제하거나 대체하지 않았으므로,
`start_static_gui.sh`를 사용하면 기존 시각화-only 화면을 계속 사용할 수 있다.

