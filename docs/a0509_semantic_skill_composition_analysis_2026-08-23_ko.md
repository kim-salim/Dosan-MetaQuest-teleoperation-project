# A0509 의미 기반 스킬 분할·조합 시스템 분석 보고서

## 문서 정보

| 항목 | 내용 |
|---|---|
| 작성일 | 2026-08-23 |
| 대상 프로젝트 | Dosan-MetaQuest-teleoperation-project |
| 대상 로봇 | Doosan A0509 |
| 대상 정책 | LeRobot ACT |
| 분석 대상 | 기존 Task A/B→C 조합 알고리즘, 최근 학습한 4개 ACT 모델 및 대응 데이터셋 |
| 문서 목적 | 자연어 작업 설명, 그리퍼 이벤트, 영상, 로봇 상태를 이용해 전체 작업을 재사용 가능한 의미 스킬로 분할하고 새로운 작업으로 조합하는 구조의 현실성 및 구현 방법 제시 |

---

## 요약

현재 프로젝트는 다음과 같은 흐름으로 발전해 왔다.

```text
MetaQuest 텔레오퍼레이션
→ LeRobot 데이터셋 수집
→ 작업별 단일 ACT 정책 학습
→ 두 ACT 정책의 의미적 구간을 Cartesian Bridge로 연결
→ 자연어·영상·그리퍼 이벤트 기반 의미 스킬 라이브러리로 확장 시도
```

이 방향은 현실적이며, 기존 Task C 구현은 새로운 계층형 스킬 시스템의 중요한 기반으로 재사용할 수 있다. 특히 다음 기능은 이미 상당한 수준으로 구현·실증되어 있다.

- 두 ACT 체크포인트의 독립적인 GPU 상주 및 추론 상태 관리
- A 정책의 의미적 절단점 판정
- A 정책 정지와 큐 무효화
- Cartesian Bézier Bridge 계획
- Bridge 종점 정착 후 최신 관측으로 B 정책 재추론
- 위치·자세·속도·가속도·jerk·workspace·stream ramp 검사
- B 정책 큐의 원자적 활성화
- release 명령과 실제 그리퍼 드라이버 완료 확인
- MUX/Live 외부 Gate 및 fail-closed 처리

그러나 현재 구현은 사실상 하나의 `open→close→transport→open` 구조를 가진 두 정책을 연결하는 데 최적화되어 있다. 새로 학습한 T1과 T4처럼 한 에피소드에 여러 번의 close/open이 있으면 다음 문제가 발생한다.

- 서랍 손잡이를 잡은 상태와 파란 블록을 잡은 상태를 구분하지 못한다.
- Representative V1은 첫 번째 close→open 구간만 사용한다.
- ACT-A의 첫 close 이후 gripper latch가 유지되어 이후 open을 차단한다.
- ACT-B의 첫 open을 최종 물체 release로 오해할 수 있다.
- 실행 상태기가 고정된 `ACT_A→BRIDGE→ACT_B` 구조이다.
- 현재 ACT 체크포인트는 자연어 문장을 실제 입력으로 받지 않는다.

따라서 최종 권장 구조는 다음과 같다.

```text
자연어 작업 지시
→ LLM/VLM 의미 계획기
→ 허용된 스킬만 포함하는 검증된 스킬 그래프
→ 로컬 SkillGraphExecutor
→ 스킬별 ACT 정책
→ 성공 상태 검증
→ 필요한 경우 기존 Task C Bridge로 안전하게 다음 스킬에 연결
→ 기존 MUX/Live/Tracking 안전 계층
```

AI API는 오프라인 영상 라벨링과 상위 작업 계획에 적합하다. 반면 30 Hz 로봇 제어 루프에서 Cartesian target이나 그리퍼 명령을 직접 생성하는 용도로 사용해서는 안 된다.

---

## 1. 분석 목적

본 분석의 목적은 다음 질문에 답하는 것이다.

1. 기존 A/B→C 조합 알고리즘이 실제로 어떤 원리로 동작하는가?
2. 최근 학습한 네 모델의 데이터가 의미적 구간 분할에 적합한가?
3. `TASK_DESCRIPTION`, 그리퍼 상태, 이벤트 전후 영상으로 동작 의미를 자동 분류할 수 있는가?
4. 분류된 동작을 다른 작업에서 재사용 가능한 스킬로 만들 수 있는가?
5. AI API를 어떤 계층에 배치해야 안전하고 실용적인가?
6. 기존 프로젝트를 어떤 순서로 확장해야 하는가?

---

## 2. 분석 범위와 방법

### 2.1 분석한 코드와 문서

- `offline_tools/task_c_bridge_v0/`
  - 의미 후보 생성
  - Bézier Bridge 생성 및 최적화
  - Runtime policy session
  - A/Bridge/B coordinator
- `offline_tools/task_c_bridge_v1/`
  - 에피소드 의미 구간 정렬
  - 대표 궤적 생성
  - 40/20 mm representative boundary
- `src/lerobot_robot_doosan_a0509/.../task_c_live_rollout.py`
  - 실제 정책 실행 상태기
  - A gripper latch
  - Bridge 실행 및 acknowledgement
  - B release/완료 판정
- `scripts/run_task_c_live_candidate.sh`
- Task C 실증 기록 문서

### 2.2 분석한 모델

```text
/home/rvlab/lerobot_models/models/
├── act_a0509_blue_block_t1_throw_away_inthe_container_bs32_50k_20260820_2323
├── act_a0509_blue_block_t2_table_bs32_40k_20260821
├── act_a0509_blue_block_t3_bs32_40k_20260822
└── act_a0509_blue_block_t4_bs32_80k_20260823
```

### 2.3 분석한 데이터셋

```text
/home/rvlab/lerobot_datasets/
├── a0509_blue_block_t1_throw_away_inthe_container
├── a0509_blue_block_t2_table
├── a0509_blue_block_t3
└── a0509_blue_block_t4
```

### 2.4 분석 항목

- 저장된 실제 task 문장
- 전체 에피소드 수와 프레임 수
- action 마지막 차원의 `gripper_target`
- observation.state 마지막 차원의 `gripper_commanded_state`
- 각 에피소드의 open/close 전환 순서
- 이벤트 발생 시점 및 TCP XYZ 분포
- 시작/종료 TCP 분포
- 이벤트 전후 대표 카메라 영상
- 모델의 실제 입력/출력 feature
- 기존 Task C의 상태기와 안전 계약

원본 데이터셋과 모델 파일은 수정하지 않았다. 영상 확인을 위해 생성한 임시 프레임은 분석 후 삭제했다.

---

## 3. 현재 프로젝트 진행 방식 분석

현재 프로젝트는 단일 작업 imitation learning을 넘어서 정책 조합 단계에 진입해 있다.

### 3.1 데이터 수집 계층

수집 데이터는 다음 정보를 30 Hz로 제공한다.

- 6개 joint state
- TCP XYZ 및 Doosan orientation
- gripper commanded state
- front RGB
- side RGB
- ZED RGB
- 7차원 action
  - target XYZ
  - target orientation 3축
  - gripper target
- episode/frame/timestamp/task identity

이 데이터는 그리퍼 이벤트를 정밀하게 찾고, 해당 이벤트 전후의 카메라 장면과 로봇 궤적을 연결하기에 적합하다.

### 3.2 정책 학습 계층

네 모델은 모두 ACT 정책이며 공통적으로 다음 계약을 사용한다.

```text
n_obs_steps: 1
chunk_size: 100
n_action_steps: 100
state dimension: 13
action dimension: 7
visual inputs: front, side, zed_rgb
```

30 Hz에서 100 action은 약 3.33초에 해당한다.

### 3.3 정책 조합 계층

기존 Task C는 두 정책을 단순히 순차 실행하지 않는다. A에서 물체를 잡은 뒤 안전한 운반 구간을 찾아 A를 중단하고, 물체를 계속 든 상태로 B의 운반/release 구간에 연결한다.

```text
Task C = A retained prefix + feasible Bridge + B retained suffix
```

이 구조는 앞으로 만들 의미 스킬 시스템의 `acquire→deliver` 전환에 직접 재사용할 수 있다.

---

## 4. 기존 A/B→C 알고리즘 분석

## 4.1 오프라인 의미 후보 생성

기존 알고리즘은 다음 두 이벤트를 찾는다.

```text
close frame: gripper open → closed
open frame:  gripper closed → open
```

현재 closed-transport 후보는 일반적으로 다음 구간에서 생성된다.

```text
close frame + 30
≤ candidate frame ≤
open frame - 30
```

30프레임은 약 1초다. 이를 통해 grasp 직후 접촉 구간과 release 직전 하강/접촉 구간을 제외한다.

또한 물체 운반 중 충분한 높이를 확보하기 위해 다음 heuristic을 사용한다.

```text
transport floor
= max(grasp TCP Z, release TCP Z) + 50 mm
```

## 4.2 V0 방식

V0는 A의 모든 의미 후보, B의 모든 의미 후보, 여러 Bridge duration을 조합하여 feasibility를 평가한다.

평가 항목은 다음을 포함한다.

- 전체 Task C 길이
- Bridge 속도
- 가속도
- jerk
- curvature
- backtracking
- workspace
- payload transport floor
- semantic state compatibility

V0의 중요한 특징은 Bridge 자체의 길이가 아니라 다음 전체 경로를 최적화한다는 점이다.

```text
A prefix + Bridge + B suffix
```

## 4.3 Representative V1 방식

V1은 각 작업의 30개 에피소드에서 grasp-to-release 운반 구간을 추출하고, Cartesian arc length 기준으로 0~1 phase로 정렬한다.

각 phase에서 다음 통계를 생성한다.

- median XYZ
- median velocity
- median acceleration
- covariance
- median absolute deviation
- episode support

그 후 대표 A와 대표 B의 phase pair를 coarse search와 refined search로 탐색한다.

현재 검증된 V1 경계는:

```text
40 mm: read-only prearm
20 mm: commit 후보
접근 방향 cosine 조건
연속 프레임 debounce
```

을 사용한다.

## 4.4 런타임 전환

현재 live 흐름은 다음과 같다.

```text
1. ACT-A와 ACT-B를 GPU에 상주시킨다.
2. warmup 결과는 모두 버린다.
3. Live edge 후 ACT-A를 실행한다.
4. open→closed, close 안정, post-close delay, TCP Z 조건을 확인한다.
5. representative boundary를 사용하는 경우 위치·방향 조건도 확인한다.
6. A 정책을 pause/reset하고 action queue를 폐기한다.
7. acknowledged commanded pose에서 Bridge를 시작한다.
8. Bridge 종점까지 실행하고 actual pose/velocity 정착을 확인한다.
9. 종점의 최신 카메라·joint·TCP·gripper 관측으로 ACT-B를 재추론한다.
10. B의 첫 target과 chunk 내부 속도를 검사한다.
11. 필요하면 짧은 alignment Bridge를 추가한다.
12. 정확한 B 첫 target에서 B queue를 원자적으로 활성화한다.
13. B를 rolling inference로 실행한다.
14. release 요청을 workspace와 연속 프레임으로 검증한다.
15. gripper driver의 completed_command/open, busy=false, last_command_ok=true를 확인한다.
16. 최종 home envelope와 속도 정착 후 COMPLETE한다.
```

## 4.5 실증 상태

기존 고정 작업 셀에서 두 번 연속 전체 Task C 실행에 성공했다.

대표 V1 endpoint 방식도 한 번의 전체 물리 실행을 완료했다.

이는 다음을 의미한다.

- 정책 간 연결이라는 핵심 개념은 실제 로봇에서 작동한다.
- fresh B inference와 atomic handoff가 실제로 유효하다.
- Bridge/Guard/Streamer 파이프라인은 고정된 검증 환경에서 사용할 수 있다.

그러나 이는 임의의 장면, 물체, 경로, 접촉 동작을 자동 인증한 결과는 아니다.

---

## 5. 최근 네 데이터셋의 정량 분석

## 5.1 데이터셋별 기본 정보

| Task | 에피소드 | 프레임 | 에피소드 길이 | 저장된 자연어 설명 |
|---|---:|---:|---:|---|
| T1 | 30 | 36,000 | 40초 | `Open the White Container throw away the blue block` |
| T2 | 31 | 27,899 | 약 30초 | `bring the blue block to black table` |
| T3 | 30 | 27,000 | 30초 | `Move the blue block off the black table` |
| T4 | 30 | 54,000 | 60초 | `Open the drawer, pick up the blue block, place it inside the drawer, and then close the drawer` |

## 5.2 그리퍼 이벤트 일관성

| Task | 이벤트 순서 | 일치 에피소드 |
|---|---|---:|
| T1 | close→open→close→open | 30/30 |
| T2 | close→open | 31/31 |
| T3 | close→open | 30/30 |
| T4 | close→open→close→open | 30/30 |

그리퍼 action 값은 모든 데이터셋에서 0 또는 1이다.

action의 gripper target과 observation의 gripper commanded state 비교 결과:

- T1: 불일치 0/36,000
- T2: 불일치 1/27,899
- T3: 불일치 0/27,000
- T4: 불일치 0/54,000

T2의 한 프레임 불일치는 episode 2, frame 758의 release 명령 시점으로, action은 open이지만 observation state가 한 프레임 늦게 closed로 남아 있는 정상적인 명령-관측 지연이다.

## 5.3 이벤트 시점 및 위치

### T1

```text
CLOSE 1:  5.31 ± 0.72 s, [555.9,   1.9, 525.0] mm
OPEN  1: 15.85 ± 1.33 s, [332.7,   2.2, 506.8] mm
CLOSE 2: 23.91 ± 1.63 s, [383.5,   2.1, 349.5] mm
OPEN  2: 33.97 ± 2.00 s, [599.6,   2.7, 506.5] mm
```

대표 영상과 task 문장을 함께 보면 다음 의미에 대응한다.

```text
컨테이너 접근/잡기
→ 컨테이너 열기/손잡이 또는 뚜껑 release
→ 검은 테이블 위 블록 grasp
→ 컨테이너에 블록 release
→ 후퇴
```

### T2

```text
CLOSE: 11.56 ± 1.58 s, [378.7, 221.0, 296.5] mm
OPEN:  23.17 ± 1.93 s, [364.8,   5.4, 347.8] mm
```

대표 의미:

```text
우측의 파란 블록 grasp
→ 검은 테이블로 transport
→ black table 위에 release
→ 후퇴
```

### T3

```text
CLOSE:  8.41 ± 0.89 s, [400.4,    4.3, 353.2] mm
OPEN:  20.35 ± 1.53 s, [378.4, -210.8, 295.7] mm
```

대표 의미:

```text
검은 테이블 위 파란 블록 grasp
→ 테이블 밖 영역으로 transport
→ release
→ 후퇴
```

### T4

```text
CLOSE 1:  9.01 ± 1.43 s, [548.2, -186.1, 289.7] mm
OPEN  1: 16.69 ± 1.71 s, [397.4, -182.2, 293.2] mm
CLOSE 2: 27.35 ± 2.22 s, [385.4,    3.9, 350.0] mm
OPEN  2: 37.05 ± 2.49 s, [524.2, -186.6, 302.8] mm
```

대표 의미:

```text
서랍 손잡이 grasp
→ 서랍 열기 및 손잡이 release
→ 검은 테이블 위 블록 grasp
→ 열린 서랍 내부에 release
→ 열린 그리퍼로 서랍 닫기
→ 복귀
```

T4의 마지막 open 이후 에피소드 종료까지 평균 22.92 ± 2.49초가 남는다. 이 구간에 `close_drawer`가 포함되어 있으므로, 그리퍼 이벤트만 사용하는 분할 방식은 불충분하다.

## 5.4 공통 스킬 후보

T1, T3, T4의 블록 grasp 위치는 매우 유사하다.

```text
T1: [383.5, 2.1, 349.5] mm
T3: [400.4, 4.3, 353.2] mm
T4: [385.4, 3.9, 350.0] mm
```

대표 영상에서도 동일한 검은 받침대 위의 파란 블록을 집는 동작으로 확인됐다.

따라서 세 데이터셋의 약 90개 구간을 다음 공통 스킬의 학습 데이터로 통합할 수 있다.

```text
acquire_blue_from_black_table
```

다만 통합 전에 다음을 추가 확인해야 한다.

- Doosan orientation의 quaternion 기반 일관성
- drawer/container의 열림 상태가 정책에 미치는 영향
- camera별 조명·가림 차이
- grasp 후 안전한 free-transport boundary 분포

## 5.5 데이터 품질 주의점

T4 데이터셋은 중간 녹화 재개 과정 때문에 data Parquet 파일의 list schema가 다르다.

```text
file-000:
  action: list<float>
  observation.state: list<float>

file-001, file-002:
  action: fixed_size_list<float>[7]
  observation.state: fixed_size_list<float>[13]
```

전체 54,000프레임에는 episode/frame 중복과 불연속이 없으므로 데이터 자체가 손상된 것은 아니다. 다만 새 의미 분할 도구는 두 schema를 명시적으로 7차원/13차원 NumPy 배열로 정규화해야 한다.

---

## 6. TASK_DESCRIPTION의 현재 역할과 한계

현재 네 ACT 모델의 선언된 입력 feature는 다음과 같다.

- observation.state
- observation.images.front
- observation.images.side
- observation.images.zed_rgb

언어 또는 task text 입력은 존재하지 않는다.

따라서 현재 `TASK_DESCRIPTION`은:

- 데이터셋의 작업 메타데이터
- 사람의 데이터 해석 정보
- 외부 LLM/VLM 계획 입력
- 의미 라벨 자동 생성의 힌트

로는 사용할 수 있지만, ACT 정책이 해당 문장을 임베딩으로 입력받아 의미를 직접 학습한 것은 아니다.

또한 현재 문장은 구조적 정보가 충분하지 않은 경우가 있다.

- T1: 문법적으로 모호하며 source/target 관계가 생략됨
- T2: 블록의 source가 생략됨
- T3: `off the black table` 이후 정확한 target 영역이 생략됨
- T4: 블록의 source가 문장에는 없고 영상에서만 확인 가능

따라서 향후에는 자연어와 별도로 다음 structured task metadata를 유지하는 것이 좋다.

```json
{
  "objects": ["blue_block", "drawer"],
  "initial_state": {
    "drawer": "closed",
    "blue_block": "on_black_table",
    "gripper": "open"
  },
  "ordered_goals": [
    "drawer_open",
    "holding_blue_block",
    "blue_block_in_drawer",
    "drawer_closed"
  ]
}
```

---

## 7. 현재 Task C를 다중 동작 모델에 바로 적용할 수 없는 이유

## 7.1 held object identity가 없음

현재 `SemanticState`에는 `holding`, `contact_mode`, `object_state` 필드가 있지만 실제 closed-transport 후보 생성 시 다음처럼 처리된다.

```text
gripper closed
→ holding = true
→ contact_mode = free_transport_assumed
```

실제 held object가 파란 블록인지 서랍 손잡이인지 기록되지 않는다.

이 상태에서는 다음 잘못된 연결이 의미적으로 허용될 수 있다.

```text
서랍 손잡이를 잡고 있는 A 구간
→ Bridge
→ 파란 블록을 들고 있어야 하는 B 구간
```

## 7.2 V0와 V1의 다중 span 처리 문제

- V0는 episode의 각 close를 그 뒤 첫 open과 연결하지만 모든 span에 같은 semantic label을 부여한다.
- V1은 episode의 첫 close→open span만 사용한다.

따라서 T1/T4의 첫 span은 컨테이너/서랍 조작이 되고 두 번째 블록 운반 span은 대표 궤적에서 제외된다.

## 7.3 전역 gripper latch

현재 ACT-A는 첫 유효 close 후 gripper target을 1.0으로 고정한다.

이는 단일 grasp 정책이 close 직후 잘못 reopen하는 문제를 방지하기 위해 추가된 안전 기능이다. 하지만 T1/T4처럼 정상적으로 다시 open해야 하는 정책은 실행할 수 없다.

## 7.4 B release 의미의 고정

현재 ACT-B는 stable open과 고정된 release workspace를 최종 물체 release로 해석한다.

다중 동작 정책에서는 첫 open이 다음 중 하나일 수 있다.

- drawer handle release
- container handle release
- block release
- 단순 gripper reposition

따라서 스킬별 release 의미와 허용 workspace가 필요하다.

## 7.5 고정된 2-policy 상태기

현재 실행 상태는 다음 구조에 가깝다.

```text
WAITING_FOR_LIVE
→ ACT_A
→ BRIDGE
→ ACT_B
→ COMPLETE 또는 FAILED_HOLD
```

자연어로 생성되는 N개 스킬을 실행하려면 일반화된 skill graph state machine이 필요하다.

---

## 8. 권장 의미 스킬 분할 원칙

## 8.1 그리퍼 이벤트와 스킬 경계를 구분해야 함

그리퍼 이벤트는 강한 의미 후보이지만 항상 안전한 정책 전환점은 아니다.

예를 들어 close 명령 직후에는:

- grasp가 실제로 성공했는지 불명확함
- 물체가 아직 작업면에 접촉 중일 수 있음
- gripper driver가 아직 busy일 수 있음
- 정책을 바꾸면 물체를 떨어뜨릴 수 있음

따라서 close 이벤트 자체가 아니라 다음 조건을 만족하는 지점을 안전 경계로 사용해야 한다.

```text
gripper close confirmed
+ 일정 안정 프레임
+ grasp 이후 lift
+ 충분한 TCP Z
+ contact-free transport
+ held object 확인
```

## 8.2 권장 primitive 수준

너무 작은 `move-left`, `close-gripper`, `lift` 단위보다 다음 의미 단위가 적합하다.

### Acquire

```text
acquire(object, source)
```

포함 동작:

- object 접근
- grasp
- lift
- certified free-transport boundary 진입

종료 상태:

- gripper closed
- holding(object)
- contact-free
- transport-safe TCP envelope

### Deliver

```text
deliver(object, target)
```

포함 동작:

- held 상태에서 target으로 이동
- target 접근
- release
- 후퇴/정착

종료 상태:

- object at target
- gripper open
- not holding object

### Articulated-object interaction

다음 접촉 동작은 전체를 하나의 스킬로 유지하는 것이 좋다.

- open_drawer
- close_drawer
- open_white_container
- close_white_container

손잡이를 잡은 접촉 중간에 다른 정책으로 전환하는 것은 피해야 한다.

---

## 9. 현재 데이터로 구성 가능한 초기 스킬 카탈로그

| 스킬 ID | 데이터 출처 | 예상 샘플 수 | 시작 조건 | 종료 조건 |
|---|---|---:|---|---|
| `open_white_container` | T1 | 30 | container closed, gripper open | container open, gripper open |
| `open_drawer` | T4 | 30 | drawer closed, gripper open | drawer open, gripper open |
| `close_drawer` | T4 후반부 | 30 | drawer open, gripper open | drawer closed, gripper open |
| `acquire_blue_from_black_table` | T1+T3+T4 | 최대 90 | blue on black table, gripper open | holding blue, gripper closed, free transport |
| `acquire_blue_from_right_region` | T2 | 31 | blue at right source, gripper open | holding blue, gripper closed, free transport |
| `deliver_blue_to_white_container` | T1 | 30 | container open, holding blue | blue in container, gripper open |
| `deliver_blue_to_black_table` | T2 | 31 | holding blue | blue on black table, gripper open |
| `deliver_blue_off_black_table` | T3 | 30 | holding blue | blue at off-table target, gripper open |
| `deliver_blue_to_drawer` | T4 | 30 | drawer open, holding blue | blue in drawer, gripper open |

이 표의 샘플 수는 의미 구간 추출 전 최대 후보 수다. 실제 사용 전에는 low-confidence/outlier episode를 제외하고 train/eval을 episode 단위로 다시 나눠야 한다.

---

## 10. 권장 계층형 시스템 구조

## 10.1 전체 구조

```text
┌──────────────────────────────────────────────┐
│ 자연어 지시                                  │
│ Open the drawer and place ...               │
└──────────────────────┬───────────────────────┘
                       ▼
┌──────────────────────────────────────────────┐
│ Language/Vision Planner                      │
│ - 목표 predicate 추출                        │
│ - 기존 skill catalog 검색                    │
│ - 후보 skill graph 생성                      │
└──────────────────────┬───────────────────────┘
                       ▼
┌──────────────────────────────────────────────┐
│ Deterministic Plan Validator                 │
│ - whitelist skill 확인                       │
│ - precondition/effect 연결                   │
│ - 누락 skill 및 모순 검출                    │
└──────────────────────┬───────────────────────┘
                       ▼
┌──────────────────────────────────────────────┐
│ Local SkillGraphExecutor                     │
│ - 현재 predicate 확인                        │
│ - ACT policy 실행                            │
│ - effect 검증                                │
│ - Bridge 또는 reposition 선택                │
│ - fresh next-policy inference                │
└──────────────────────┬───────────────────────┘
                       ▼
┌──────────────────────────────────────────────┐
│ Existing Safety Layer                        │
│ - MUX/Live gate                              │
│ - workspace/orientation/ramp                 │
│ - tracking backpressure                      │
│ - gripper driver confirmation                │
│ - fail-closed                                │
└──────────────────────────────────────────────┘
```

## 10.2 스킬 manifest 예시

```yaml
skill_id: deliver_blue_to_drawer
version: 1

policy:
  type: act
  checkpoint: /home/rvlab/lerobot_models/models/...

preconditions:
  - drawer_open
  - holding_blue_block
  - gripper_closed
  - contact_mode_free_transport

effects:
  - blue_block_in_drawer
  - gripper_open
  - not_holding_blue_block

entry_contract:
  gripper: closed
  held_object: blue_block
  tcp_position_envelope_mm: ...
  tcp_orientation_envelope: ...
  visual_embedding_envelope: ...

invariants:
  - drawer_remains_open_until_release
  - blue_block_remains_held_before_release
  - workspace_guard_active

success_detector:
  - gripper_open_driver_confirmed
  - blue_block_in_drawer_visual
  - tcp_retracted

failure_actions:
  - hold_last_safe_pose
  - live_off
  - mux_disabled
```

---

## 11. 의미 구간 생성 알고리즘 제안

## 11.1 1차 이벤트 후보 생성

각 episode에서 다음을 계산한다.

1. gripper action threshold
   - close: `target >= 0.7`
   - open: `target <= 0.3`
2. observation gripper state 확인
3. 3~15프레임 debounce
4. TCP XYZ 속도와 가속도
5. Cartesian arc length
6. 방향 변화 및 정지 구간
7. gripper 이벤트 전후 ±1~2초의 카메라 clip

action은 operator/policy의 의도를 나타내고 observation state는 command가 반영된 상태를 나타내므로 두 시점을 별도로 보존해야 한다.

## 11.2 자연어 기반 예상 이벤트 순서 생성

예를 들어 T4 문장은 다음 symbolic plan으로 변환한다.

```text
drawer_closed
→ open_drawer
→ drawer_open
→ acquire(blue_block, black_table)
→ holding_blue_block
→ deliver(blue_block, drawer)
→ blue_block_in_drawer
→ close_drawer
→ drawer_closed
```

이 예상 순서와 실제 `COCO` 이벤트를 temporal alignment한다.

## 11.3 VLM 기반 구간 의미 라벨

VLM에는 각 후보마다 다음을 전달한다.

- 전체 task 문장
- 현재 예상 subgoal 순서
- event 종류와 순번
- 전후 세 카메라 대표 프레임 또는 짧은 clip
- TCP 위치/방향/속도
- gripper action/state
- 이전에 확정된 predicate

출력은 자유 문장이 아니라 JSON schema로 제한한다.

```json
{
  "skill": "open_drawer",
  "manipulated_object": "drawer_handle",
  "held_object_after": null,
  "source": null,
  "target": "drawer_open_state",
  "preconditions": ["drawer_closed", "gripper_open"],
  "effects": ["drawer_open", "gripper_open"],
  "confidence": 0.96,
  "evidence_frames": [220, 444]
}
```

## 11.4 순서 제약 및 human review

VLM 결과는 단독으로 신뢰하지 않는다.

다음 조건을 결합해 최종 라벨을 만든다.

- task 문장의 동작 순서
- gripper event signature
- TCP 영역
- multi-view 장면 변화
- 같은 ordinal event의 episode 간 일관성
- state predicate의 논리적 연결

신뢰도가 낮거나 순서가 모순되는 경우만 사람 검토 대상으로 보낸다.

## 11.5 안전 경계 생성

의미 이벤트를 찾은 뒤 실제 정책 전환 boundary는 별도로 선택한다.

예: grasp 이후 acquire 종료점

```text
close confirmed
→ 일정 시간 경과
→ object lift
→ TCP Z clearance
→ contact-free
→ 대표 궤적 support 안에 진입
→ boundary commit
```

이 방식은 기존 Task C representative boundary를 스킬 단위로 일반화한 것이다.

---

## 12. AI API의 권장 역할

## 12.1 적합한 역할

- 데이터셋 task 문장 구조화
- 이벤트 전후 영상 의미 라벨링
- object/source/target 추출
- 자연어 지시를 기존 skill graph로 변환
- 실행 전 plan 설명
- skill 종료 후 저주기 시각적 상태 확인
- 실패 원인 분류 및 재계획 후보 생성

## 12.2 부적합한 역할

- 30 Hz Cartesian target 직접 생성
- ServoL command 직접 생성
- 매 tick gripper command 결정
- 검증되지 않은 새로운 skill을 즉석 생성
- safety threshold 변경
- fail-closed 상태를 자동 우회

AI 출력은 항상 다음 절차를 거쳐야 한다.

```text
AI plan
→ schema validation
→ skill whitelist 확인
→ precondition/effect 검증
→ scene contract 확인
→ operator 또는 자동 gate 승인
→ local executor 실행
```

---

## 13. 정책 학습 전략

## 13.1 첫 단계: 스킬별 독립 ACT

초기 구현은 스킬마다 독립 정책을 학습하는 것이 적절하다.

장점:

- 오류 원인 구분이 쉬움
- 성공/실패 조건을 스킬별로 설정 가능
- gripper latch와 release 규칙을 스킬별로 정의 가능
- 필요한 스킬만 다시 수집·재학습 가능
- 기존 Task C의 dual-policy 구조를 확장하기 쉬움

## 13.2 중기: discrete skill-conditioned ACT

스킬이 늘어나면 하나의 정책에 `skill_id` embedding을 입력할 수 있다.

```text
policy input
= robot state
+ three camera views
+ discrete skill embedding
```

이는 자연어 전체를 바로 입력하는 것보다 구현과 검증이 단순하다.

## 13.3 장기: language-conditioned policy 또는 VLA

충분히 다양한 작업과 물체, 위치 데이터가 확보된 이후에만 language-conditioned policy를 고려하는 것이 좋다.

현재 4개 작업과 30개 안팎의 에피소드만으로는 자연어의 조합적 일반화를 기대하기 어렵다.

## 13.4 chunk 크기

현재 100 action chunk는 기존 runtime에서 검증된 설정이다.

첫 스킬 정책은 다음 방식이 안전하다.

- chunk size 100 유지
- receding-horizon 실행
- fresh observation rolling inference
- detector가 effect를 확인하면 남은 큐를 폐기
- 안전 경계에서 다음 policy 추론

전환 반응성이 부족하면 50 action 정책을 별도 ablation으로 비교할 수 있다.

---

## 14. SkillGraphExecutor 구현 제안

## 14.1 일반화된 상태기

```text
WAITING_FOR_START
→ LOAD_SKILL
→ VERIFY_PRECONDITIONS
→ RUN_SKILL
→ VERIFY_EFFECTS
→ SELECT_TRANSITION
→ PLAN_BRIDGE_OR_REPOSITION
→ SETTLE
→ FRESH_NEXT_POLICY_INFERENCE
→ ATOMIC_HANDOFF
→ RUN_NEXT_SKILL
→ COMPLETE
```

실패 시:

```text
FAILED_HOLD
→ last safe pose 유지
→ policy queues invalidate
→ Live OFF
→ MUX DISABLED
→ operator notification
```

## 14.2 기존 코드에서 재사용할 요소

- AsyncPolicySession
- generation isolation
- warmup discard
- stale inference rejection
- GPU arbiter
- RuntimeBridgePlanner
- endpoint settle
- full pose assessment
- orientation interpolation
- downstream ramp contract
- tracking backpressure
- exact commanded acknowledgement
- external live gate
- gripper driver confirmation

## 14.3 변경이 필요한 요소

| 현재 | 변경 후 |
|---|---|
| ACT_A/ACT_B 두 정책 | N개의 skill policy |
| 첫 close 이후 전역 latch | skill/phase별 gripper state machine |
| 첫 open을 B release로 간주 | skill-specific effect detector |
| generic holding | held object identity |
| generic contact mode | handle contact / block holding / free transport |
| 고정 release workspace | skill manifest별 workspace |
| 첫 close→open span | semantic label 및 span ID 기반 선택 |
| 하나의 Bridge manifest | skill-pair transition manifest 또는 runtime-compatible envelope |

## 14.4 Bridge를 사용할 수 있는 조건

Bridge는 모든 스킬 사이에 사용하면 안 된다.

권장 조건:

```text
current contact mode == free_transport
next contact mode == free_transport entry
held object identity 동일
gripper state 동일
object state 동일
required preconditions 충족
entry visual/pose envelope 안에 있음
```

다음 상태에서는 Bridge를 금지해야 한다.

- drawer handle 접촉 중
- container lid 접촉 중
- 물체 release 직전 작업면 접촉 중
- held object가 불명확함
- next skill이 요구하는 object identity가 다름
- visual scene이 training envelope 밖임

이 경우에는 certified reposition 또는 home/prep pose 전환 스킬을 사용해야 한다.

---

## 15. 데이터셋 생성 형식 제안

원본을 유지하고 다음 구조를 별도로 생성하는 것이 좋다.

```text
/home/rvlab/lerobot_datasets/semantic_skills_v1/
├── catalog.json
├── semantic_segments.parquet
├── review/
│   ├── pending.jsonl
│   └── approved.jsonl
├── clips/
│   └── ...
└── datasets/
    ├── open_drawer
    ├── close_drawer
    ├── acquire_blue_from_black_table
    ├── deliver_blue_to_drawer
    └── ...
```

`semantic_segments.parquet` 권장 필드:

```text
dataset_id
episode_index
segment_index
skill_id
start_frame
end_frame
intent_event_frame
observed_event_frame
entry_gripper_state
exit_gripper_state
held_object
contact_object
source_location
target_location
preconditions_json
effects_json
camera_clip_refs_json
label_confidence
label_source
human_approved
```

분할 데이터셋의 train/eval split은 반드시 원본 episode 단위로 해야 한다. 같은 원본 episode에서 나온 서로 다른 segment가 train과 eval에 동시에 들어가면 leakage가 발생한다.

---

## 16. 성공 상태 검출기

## 16.1 결정론적 신호

- gripper commanded state
- completed_command
- driver_busy
- last_command_ok
- TCP pose
- TCP velocity
- workspace envelope
- policy event order

## 16.2 시각 predicate

- drawer_open
- drawer_closed
- white_container_open
- blue_on_black_table
- blue_off_black_table
- blue_in_drawer
- blue_in_white_container
- blue_between_gripper_jaws

## 16.3 추가 센서 권장

현재 `gripper closed = holding`은 heuristic이다. 가능하면 다음 중 하나를 추가해야 한다.

- gripper motor current
- gripper jaw width/encoder
- force/contact
- ZED depth
- 블록 검출 및 jaw-region occupancy

Bridge 중 물체가 빠지면 commanded state만으로는 감지할 수 없다.

---

## 17. 구현 로드맵

## 단계 1: 의미 인덱서

목표:

- 네 데이터셋의 모든 episode에서 event index 생성
- T4 schema 정규화
- action intent와 observed state 경계 분리
- 이벤트 전후 clip reference 생성
- 원본 데이터 무변경

산출물:

- `semantic_segments.parquet`
- 자동 분석 summary
- review manifest

## 단계 2: 초기 ontology 및 라벨 검토

목표:

- T1~T4 expected symbolic plan 작성
- gripper ordinal 기반 1차 라벨
- 대표 clip VLM 분석
- outlier만 사람 검토

산출물:

- 승인된 skill labels
- skill catalog V1

## 단계 3: T4 분해 데이터셋

T4를 다음 네 스킬로 분해한다.

```text
open_drawer
→ acquire_blue_from_black_table
→ deliver_blue_to_drawer
→ close_drawer
```

이 단계에서는 기존 T4 전체 정책을 정답 기준으로 사용할 수 있다.

## 단계 4: 스킬별 정책 학습

우선 학습 후보:

1. open_drawer
2. acquire_blue_from_black_table
3. deliver_blue_to_drawer
4. close_drawer

`acquire_blue_from_black_table`은 T1/T3/T4의 최대 90개 segment를 활용한다.

## 단계 5: T4 의미 재구성

목표:

```text
4개의 독립 스킬을 순차 실행해 기존 T4 전체 동작을 재현
```

검증 항목:

- 각 skill precondition/effect
- gripper event order
- drawer 상태
- block 위치
- policy handoff 시 pose/orientation
- Bridge intervention 0
- Live/MUX cleanup

## 단계 6: 교차 데이터셋 조합

예시:

```text
open_drawer(T4)
→ acquire_blue_from_right_region(T2)
→ deliver_blue_to_drawer(T4)
→ close_drawer(T4)
```

이 단계가 실제 novel composition 검증이다.

## 단계 7: 자연어 계획기 연결

자연어를 다음 DSL로 변환한다.

```json
{
  "goal": ["blue_block_in_drawer", "drawer_closed"],
  "skills": [
    {"id": "open_drawer"},
    {"id": "acquire_blue_from_black_table"},
    {"id": "deliver_blue_to_drawer"},
    {"id": "close_drawer"}
  ]
}
```

Validator가 catalog에 없는 skill이나 precondition 모순을 거부한다.

---

## 18. 검증 전략

## 18.1 오프라인 검증

- event sequence 100% 재현
- frame/timestamp 연속성
- segment boundary 시각 검토
- train/eval episode leakage 없음
- task description과 symbolic plan 일치
- held object/contact identity 일치

## 18.2 command-free replay

- 각 skill entry/exit envelope
- predicted action jump
- orientation continuity
- workspace
- speed/acceleration/jerk
- Bridge feasibility
- visual OOD score

## 18.3 shadow 실행

- 실제 카메라와 state 사용
- 로봇 명령 publish 없음
- skill transition timing 측정
- AI API latency가 제어 루프에 영향을 주지 않는지 확인
- stale policy result가 실행되지 않는지 확인

## 18.4 물리 실행 단계

권장 순서:

1. 스킬별 독립 실행 10회
2. 두 스킬 조합 10회
3. 원래 T4를 분해한 네 스킬 조합 10회
4. 교차 데이터셋 novel 조합
5. 자연어 계획 포함 전체 실행

각 단계에서 다음을 요구한다.

- terminal success predicate 충족
- guard/stream intervention 0
- driver-confirmed gripper events
- payload loss 0
- collision/contact 이상 0
- 종료 후 Live OFF
- 종료 후 MUX DISABLED

---

## 19. 예시 작업의 가능 여부

예시:

```text
Open the drawer,
take out the blue block,
and place it on the black table.
```

현재 보유한 데이터에는 다음 스킬이 있다.

- `open_drawer`: T4
- `deliver_blue_to_black_table`: T2

현재 없는 스킬:

- `acquire_blue_from_drawer`

T4의 `deliver_blue_to_drawer`를 시간 역순으로 실행하는 것은 `acquire_blue_from_drawer` 정책이 되지 않는다. ACT와 실제 접촉 동작은 일반적으로 시간 가역적이지 않다.

따라서 이 작업을 수행하려면 `acquire_blue_from_drawer` primitive를 추가로 수집해야 한다.

스킬 라이브러리 방식의 장점은 전체 작업을 처음부터 다시 30회 수집할 필요 없이 부족한 primitive만 추가할 수 있다는 점이다.

---

## 20. 현실성 평가

| 목표 | 판단 | 근거 |
|---|---|---|
| 그리퍼 이벤트 기반 1차 분할 | 높음 | 모든 episode의 이벤트 순서가 매우 일관됨 |
| 문장+영상으로 이벤트 의미 구분 | 높음 | 세 카메라와 명확한 장면 변화가 존재함 |
| T4의 close_drawer 검출 | 중간~높음 | 그리퍼 이벤트는 없지만 영상과 긴 후반 궤적이 존재함 |
| 공통 black-table grasp 통합 | 높음 | T1/T3/T4 위치와 영상이 유사함 |
| 기존 whole-task checkpoint의 임의 중간 진입 | 중간 이하 | phase conditioning이 없고 전체 장면 분포에 의존함 |
| 분할 데이터 재학습 후 고정 작업 셀 조합 | 중간~높음 | 기존 Task C가 실제 정책 전환을 실증함 |
| 현재 네 작업만으로 임의 문장 zero-shot 수행 | 낮음 | 없는 primitive를 언어만으로 생성할 수 없음 |
| AI API를 상위 계획/라벨링에 사용 | 높음 | 저주기·구조화 출력으로 안전 계층과 분리 가능 |
| AI API를 30 Hz 제어에 직접 사용 | 부적합 | 지연, 비결정성, 네트워크 의존성 |

---

## 21. 주요 위험과 대응

### 위험 1: gripper closed를 holding으로 오인

대응:

- held object visual detector
- current/width/force 신호 추가
- Bridge 중 loss detector

### 위험 2: drawer handle과 blue block의 의미 혼동

대응:

- held_object와 contact_object를 별도 필드로 저장
- VLM label + 위치 + 순서 제약
- object identity가 다르면 Bridge 금지

### 위험 3: VLM hallucination

대응:

- JSON schema
- expected event order
- confidence threshold
- human approval
- AI label은 명령 권한을 갖지 않음

### 위험 4: whole-task ACT의 phase 불확실성

대응:

- segmented dataset 재학습
- skill별 checkpoint
- fresh observation inference
- entry distribution/OOD 검사

### 위험 5: transition geometry는 맞지만 장면 의미가 다름

대응:

- Cartesian envelope와 visual envelope를 함께 검사
- symbolic precondition/effect 검증
- contact mode 검사

### 위험 6: 새 조합에서 collision/IK 문제가 발생

대응:

- 기존 workspace/ramp 검사 유지
- sampled pose IK 및 joint limit 검사 추가
- 물체를 포함한 swept-volume collision 검사
- 검증되지 않은 scene에서는 fail-closed

---

## 22. 최우선 권장 실험

가장 먼저 수행할 실험은 T4 의미 재구성이다.

```text
open_drawer
→ acquire_blue_from_black_table
→ deliver_blue_to_drawer
→ close_drawer
```

이 실험은 다음을 한 번에 검증한다.

- 다중 close/open 의미 분리
- gripper 이벤트가 없는 close_drawer 분리
- 물체 identity 유지
- closed-holding Bridge
- N개 스킬 상태기
- 각 스킬 effect 검증
- 기존 전체 T4 정책과의 직접 비교

T4 재구성이 안정화된 이후에 T2의 source와 T4의 destination을 결합하는 교차 조합을 수행하는 것이 가장 합리적이다.

---

## 23. 최종 결론

현재 구상은 실현 가능하다. 단, 성공 조건은 다음과 같다.

1. 자연어를 직접 로봇 action으로 바꾸지 않는다.
2. 자연어는 검증된 symbolic skill graph로 변환한다.
3. 그리퍼 이벤트는 의미 후보로 사용하되 안전 경계와 구분한다.
4. 영상으로 held object, target, drawer/container 상태를 판정한다.
5. whole-task checkpoint를 그대로 잘라 쓰는 것보다 segmented dataset으로 primitive를 재학습한다.
6. 기존 Task C의 fresh inference, Bridge, atomic handoff, tracking guard, fail-closed 구조를 재사용한다.
7. 없는 primitive는 언어로 만들어내지 않고 해당 구간의 데이터를 추가 수집한다.

현재 프로젝트의 가장 큰 강점은 이미 실제 로봇에서 A→Bridge→B 전환을 성공시켰다는 점이다. 다음 단계에서는 이 전환기를 고정된 두 정책 전용 코드에서 `N개의 의미 스킬을 실행하는 SkillGraphExecutor`로 일반화하고, 네 데이터셋에서 의미 segment와 스킬 manifest를 생성해야 한다.

최우선 구현 항목은 다음 두 가지다.

```text
1. semantic_segments 생성기
2. T4를 네 개 스킬로 재구성하는 command-free/offline 검증
```

이 두 단계가 완료되면 자연어 계획과 새로운 작업 조합을 안전하게 확장할 수 있는 기반이 마련된다.

---

## 부록 A. 주요 근거 파일

- [Task C V0 설계](../offline_tools/task_c_bridge_v0/README.md)
- [V0 의미 후보 생성](../offline_tools/task_c_bridge_v0/semantic_candidates.py)
- [Task C 의미 상태 구조](../offline_tools/task_c_bridge_v0/trajectory_states.py)
- [Representative V1 궤적 생성](../offline_tools/task_c_bridge_v1/representative_trajectory.py)
- [Representative boundary 추적기](../offline_tools/task_c_bridge_v1/runtime_boundary.py)
- [Task C live rollout](../src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/task_c_live_rollout.py)
- [Task C live 실행 스크립트](../scripts/run_task_c_live_candidate.sh)
- [Task C 전체 실증 기록](task_c_full_live_validation_2026-08-14.md)
- [Task C Representative V1 실증 기록](task_c_representative_bridge_v1_2026-08-18.md)

## 부록 B. 분석 대상 로컬 모델

```text
/home/rvlab/lerobot_models/models/
act_a0509_blue_block_t1_throw_away_inthe_container_bs32_50k_20260820_2323/050000/pretrained_model

/home/rvlab/lerobot_models/models/
act_a0509_blue_block_t2_table_bs32_40k_20260821/040000/pretrained_model

/home/rvlab/lerobot_models/models/
act_a0509_blue_block_t3_bs32_40k_20260822/040000/pretrained_model

/home/rvlab/lerobot_models/models/
act_a0509_blue_block_t4_bs32_80k_20260823/080000/pretrained_model
```

## 부록 C. 분석 대상 로컬 데이터셋

```text
/home/rvlab/lerobot_datasets/a0509_blue_block_t1_throw_away_inthe_container
/home/rvlab/lerobot_datasets/a0509_blue_block_t2_table
/home/rvlab/lerobot_datasets/a0509_blue_block_t3
/home/rvlab/lerobot_datasets/a0509_blue_block_t4
```
