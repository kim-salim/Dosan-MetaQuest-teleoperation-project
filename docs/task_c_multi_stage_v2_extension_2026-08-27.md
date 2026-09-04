# Task-C V2 다단계 정책 재진입 확장

- 작성일: 2026-08-27
- 대상: `Dosan-MetaQuest-teleoperation-project`
- 신규 opt-in strategy: `task_c_multi_live_v2`
- 기존 `task_c_live_v2`: 변경 없이 보존
- 물리 로봇 실행: 수행하지 않음
- Live ON / MUX LEROBOT 선택: 수행하지 않음

## 1. 결과

기존 단일 edge V2를 다시 구현하지 않고, 동일한 edge coordinator를 순서대로
재사용하는 상위 stage sequencer를 추가했다.

```text
T4 visit 1 (기존 rollout ACT engine)
  ↓ external planner request
기존 V2: actual-state Bridge → async T2 → prefix admission → crossfade
  ↓
T2 visit 1 (resident AsyncPolicySession + rolling refresh)
  ↓ external planner request
기존 V2: actual-state Bridge → async T4 → prefix admission → crossfade
  ↓
T4 visit 2 (resident AsyncPolicySession + rolling refresh)
  ↓ external planner complete request
COMPLETE → external gate가 Live OFF / MUX DISABLED
```

새 runtime은 semantic/contact/held-object/segment 판단을 하지 않는다. 각 stage를
언제 떠날지는 상위 planner가 Trigger service로 승인한다. 다만 다음 기존 hard
contract는 계속 유지된다.

- actual/acknowledged state freshness 및 tracking
- Bridge workspace/dynamics/finite-value 검사
- successor generation/staleness 검사
- fresh successor prefix 및 prospective crossfade 동역학 검사
- 30 Hz command path, MUX, safety guard, ServoL streamer
- 모든 오류의 fail-closed 경로

## 2. 기존 V2와의 차이

| 항목 | 기존 `task_c_live_v2` | 신규 `task_c_multi_live_v2` |
|---|---|---|
| 정책 방문 | A→B 한 번 | ordered stage list, 동일 정책 재방문 허용 |
| V2 edge 수 | 1 | `len(stages)-1` |
| 정책 load | A context + B resident | unique policy당 한 번, setup에서 완료 |
| A exit | 기존 median/phase 또는 외부 authority | 모든 비최종 stage를 외부 planner가 요청 |
| successor takeover | fresh prefix compatibility | 동일 |
| Bridge/crossfade | 기존 V2 | 기존 V2 coordinator를 edge마다 새로 생성 |
| endpoint V1 fallback | 선택 가능 | 현재 명시적으로 비활성/fail-closed |
| completion | B successor-owned | 최종 stage의 external complete request |

다단계 모드에서 endpoint V1 fallback을 비활성화한 이유는 한 개의 legacy runtime
manifest가 T4→T2와 T2→T4의 서로 다른 endpoint를 동시에 올바르게 표현할 수 없기
때문이다. 이를 억지로 재사용하는 대신 V2 window 실패 시 fail-closed한다. 향후
edge별 V1 fallback manifest를 별도로 제공하면 이 제한을 확장할 수 있다.

## 3. 정책 및 queue 소유권

### 3.1 모델 residency

`PolicyRegistry`는 plan에 등장하는 unique policy마다 session 하나를 소유한다.

```text
T4 model weights: 1회 load (초기 rollout context 것을 재사용)
T2 model weights: 1회 load
T4 visit count: 2
T2 visit count: 1
```

T4를 두 번 사용해도 T4 모델을 두 번 load하지 않는다. 초기 T4는 기존 LeRobot
ACT engine으로 실행하고, 첫 전환 뒤 pause/reset된 같은 model/preprocessor를
`ResidentLeRobotACTBackend`로 감싸 복귀용 async session에서 사용한다.

### 3.2 재방문 queue 규칙

policy stage를 떠날 때 다음을 수행한다.

```text
current_session.deactivate_and_clear()
  → generation 증가
  → ready chunk 제거
  → active chunk 제거
  → queue index 초기화
  → 완료가 늦은 old inference 결과는 stale
```

따라서 첫 번째 T4 방문에서 남은 action을 두 번째 T4 방문에서 재사용하지 않는다.
복귀 시 최신 RGB/state observation으로 새 T4 generation을 요청하고, 기존 V2
prefix admission을 통과한 chunk만 crossfade/takeover에 사용한다.

### 3.3 rolling refresh

각 takeover 뒤에는 기존 ACT-B rolling refresh를 그대로 사용한다. 다단계 runtime은
현재 stage의 session을 기존 `_session_b` slot에 바인딩하므로 T2뿐 아니라 복귀한
T4에도 동일한 queue threshold, generation isolation, 15-step overlap이 적용된다.

## 4. Thread / CPU / GPU 계약

기존 실행 CPU pinning을 변경하지 않았다.

```text
ROS executor: CPU 6
main/camera/control: CPU 7-8
ACT inference caller: CPU 9-13
process taskset: CPU 6-13
```

각 unique policy session은 `ThreadPoolExecutor(max_workers=1)`을 갖지만, 모든
CUDA inference는 기존 process-wide ACT GPU arbiter를 공유한다. 따라서 여러
policy worker가 있어도 CUDA inference를 동시에 실행하지 않는다.

```text
control thread -- non-blocking prime/poll --> policy session worker
policy session worker -- serialized lock --> CUDA inference
```

모델 load 및 추가 policy warmup은 setup에서만 동기적으로 수행한다. Live 30 Hz
구간에서는 model load/warmup을 하지 않는다. T4 복귀는 이미 사용한 resident
weights를 재사용하므로 모델 reload 시간이 Bridge에 들어가지 않는다.

## 5. 30 Hz 불변조건

control loop는 다음만 수행한다.

- planner mailbox 상태 읽기
- actual/ack snapshot으로 한 edge Bridge commit
- precomputed Bridge/action/crossfade queue에서 target 하나 소비
- successor request submit 및 non-blocking poll
- 기존 prefix/crossfade 검사
- 기존 robot adapter에 7D proposal 전송

다음은 수행하지 않는다.

- `Future.result()` 대기
- inference worker `join()`
- CUDA 완료 대기
- model load
- full candidate search
- file write
- ROS service wait

planner Trigger callback도 locked state만 바꾸며 robot command를 publish하지 않는다.

## 6. Multi-stage plan

schema:

```text
a0509.task_c_multi_stage_plan.v1
```

필수 구조:

```yaml
policies:
  T4:
    checkpoint: /.../t4/pretrained_model
    reuse_context_policy: true
  T2:
    checkpoint: /.../t2/pretrained_model
    reuse_context_policy: false

stages:
  - {stage_id: open_drawer, policy_id: T4, exit_authority: external_planner}
  - {stage_id: pick_floor, policy_id: T2, exit_authority: external_planner}
  - {stage_id: place_drawer, policy_id: T4, exit_authority: external_complete, terminal: true}

transitions:
  - source_stage: open_drawer
    successor_stage: pick_floor
    handoff_manifest: /.../t4_to_t2.json
  - source_stage: pick_floor
    successor_stage: place_drawer
    handoff_manifest: /.../t2_to_t4.json
```

validation은 다음을 강제한다.

- first-stage policy만 `reuse_context_policy=true`
- stage 순서와 transition 순서 일치
- manifest `source.task`/`successor.task`와 stage policy ID 일치
- 모든 비최종 stage는 `external_planner`
- 최종 stage는 `external_complete`
- multi live config에서는 모든 edge manifest가 `external_planner` authority
- 첫 successor checkpoint와 inherited `checkpoint_b` 일치
- initial rollout checkpoint와 first policy 일치

템플릿:

```text
config/realtime/task_c_t4_t2_t4_plan.example.yaml
```

이 템플릿의 두 handoff manifest path는 의도적으로 placeholder다. 실제 T4→T2,
T2→T4 boundary reference를 생성·검토하지 않은 상태에서 임의 pose를 넣지 않았다.

## 7. Planner service sequence

strategy가 준비되고 external trial gate가 Live/MUX authority를 승인한 뒤:

```bash
# 현재 stage 조회
ros2 service call /control/task_c/multi_stage_status std_srvs/srv/Trigger '{}'

# 현재 비최종 stage를 떠나 다음 V2 edge 시작 요청
ros2 service call /control/task_c/request_next_stage std_srvs/srv/Trigger '{}'

# 최종 stage만 종료 승인
ros2 service call /control/task_c/complete_episode std_srvs/srv/Trigger '{}'
```

T4→T2→T4에서는 `request_next_stage`를 정확히 두 번 호출한다. 서비스 성공은
transition mailbox 승인만 의미한다. 실제 policy takeover는 각 edge에서 fresh
successor prefix admission과 crossfade가 성공한 뒤에만 발생한다.

## 8. 실행 파일

- runtime: `task_c_multi_live_v2_rollout.py`
- entrypoint: `task_c_multi_live_v2_entrypoint.py`
- pure plan/state: `task_c_handoff/multi_stage.py`
- policy registry: `task_c_handoff/policy_registry.py`
- launcher: `scripts/run_task_c_multi_stage_v2_candidate.sh`
- command-free validator: `scripts/validate_task_c_multi_stage_plan.py`
- reference config: `config/realtime/task_c_multi_stage_v2.yaml`

launcher는 시작/종료 시 safe state를 요청하지만 LEROBOT source 선택이나 Live ON은
하지 않는다.

## 9. Trace

기존 V2 trace에 다음 multi-stage event가 추가된다.

- `multi_stage_v2_ready`
- `multi_stage_started`
- `multi_transition_requested`
- `multi_transition_started`
- `multi_source_policy_invalidated`
- `multi_edge_runtime_bound`
- `multi_policy_takeover_committed`
- `multi_episode_complete`
- `multi_episode_control_complete`
- `multi_stage_fail_closed`

episode summary에는 plan, 현재 stage, 모든 stage transition event, policy별 visit
count/generation/queue 상태가 포함된다. 기존 각 edge의 B inference latency,
compatibility, crossfade, ACK, control timing trace도 그대로 유지된다.

## 10. 검증 결과

실행 명령:

```bash
source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
export PYTHONPATH=/home/rvlab/Dosan-MetaQuest-teleoperation-project/src/lerobot_robot_doosan_a0509:/home/rvlab/Dosan-MetaQuest-teleoperation-project/src/quest_a0509_teleop:/home/rvlab/Dosan-MetaQuest-teleoperation-project
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project/src/lerobot_robot_doosan_a0509
pytest -q test/test_task_c_*.py test/test_cross_task_handoff_v2.py
```

결과:

```text
106 passed
```

기존 V2 fake successor latency 50/100/200/500 ms non-blocking Bridge test도 이
회귀 집합에 포함된다. 새 다단계 테스트는 T4→T2→T4 ordered plan, 두 unique
policy/세 visit, T4 재방문 시 old queue generation invalidate, external planner
request 없이는 initial T4를 끊지 않음, legacy single-edge fallback 차단을 검증한다.

## 11. 복구 백업

작업 전 source/config/tests/docs 백업:

```text
/home/rvlab/project_backups/Dosan-MetaQuest-teleoperation-project_pre_multi_v2_20260827.tar.gz
SHA-256: 240e127c0a5d2a353014064702d53cc1391dd6dc8ec95ec3043cdd7e3817990c
```

## 12. 아직 확인되지 않은 항목

- 실제 T4→T2 handoff boundary/Bridge feasibility
- 실제 T2→T4 handoff boundary/Bridge feasibility
- 두 edge의 physical collision/IK safety
- Jetson에서 T4와 T2 동시 residency 시 실제 GPU memory 여유
- 각 복귀 inference latency p95/p99
- T4가 현재 observation에서 의도한 후반 행동을 선택하는지
- 실제 물체 파지 유지 및 최종 복합 task 성공

따라서 현재 산출물은 다단계 runtime/queue/inference 구조의 구현과 command-free
검증 완료 상태다. 실제 두 edge manifest를 만들고 난 뒤에도 검증 순서는
`plan validate → unit/offline → policy shadow → command-free ROS → bounded physical`
순서를 유지해야 한다.
