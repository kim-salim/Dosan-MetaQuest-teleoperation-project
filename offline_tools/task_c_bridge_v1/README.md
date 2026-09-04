# Task-C representative Bridge V1

이 디렉터리는 기존 전수 후보 방식인
`offline_tools/task_c_bridge_v0`를 수정하지 않고 추가한 V1 알고리즘이다.
기존 V0 manifest와 live 실행 스크립트의 기본 동작은 그대로이며, V1은 전용
manifest와 명시적인 live 옵션을 함께 제공했을 때만 활성화된다.

## 고정한 알고리즘

### 1. A/B 각각 하나의 대표 경로

A와 B의 30개 episode에서 첫 close부터 첫 open까지의 운반 구간을 찾는다.
close 이후 30 frame, open 이전 30 frame을 제외하고, grasp/release 높이보다
50 mm 높은 연속 운반 구간만 남긴다. 각 경로는 Cartesian arc length를
0..1 phase 101점으로 resample한다.

여기서 standardization은 좌표 z-score가 아니다. Doosan base frame의 mm 값을
그대로 유지한 채 semantic 구간과 진행률만 정렬한다. 같은 phase의 XYZ,
causal-regression 방식 속도, 가속도는 episode component median으로 대표화하고,
공분산과 MAD 및 모든 source episode residual도 함께 보존한다.

### 2. 전체 C 경로 길이를 우선하는 coarse-to-fine 탐색

목적함수는 다음과 같다.

    A 시작부터 cut까지 길이
    + velocity-matched cubic Bridge 길이
    + B entry부터 B 종료까지 길이

1차 탐색은 A phase 21개 x B phase 21개 x duration 13개를 30 Hz로
검사한다. 우수 phase pair 주변은 0.01 phase 간격으로 재탐색한다. 최종 후보는
60 Hz로 다시 검사하고 30개 A residual 및 30개 B residual에 대한 robust pass
fraction도 계산한다.

속도, 축별 속도, 가속도, 경계 가속도 jump, curvature, jerk, backtracking,
workspace, payload transport floor는 hard gate다. 또한 선택한 A phase는
30개 episode 중 최소 80%가 40 mm 안에 있고 최소 50%가 20 mm 안에 있어야 한다.
feasible 후보 중 전체 C 길이의 최솟값에서 0.5% 이내인 집합만 남긴 뒤 경계
가속도 jump, 가속도, curvature, jerk 순으로 자연스러운 후보를 고른다.

Offline 대표 Bridge는 A floor의 최댓값과 B floor p95 중 큰 값을 사용한다.
Live는 이미 관측된 A 상태에서 시작하므로 기존 V0 runtime과 동일하게 B entry
floor만 적용한다. 단일 B 대표값에서는 한 outlier의 max 대신 30개 episode의
p95를 사용하며 그 수치와 출처를 manifest에 기록한다.

30 Hz는 탐색 비용 절감용이고 안전 판정의 최종 해상도는 60 Hz다. 실제 downstream
command 주기는 기존과 같은 30 Hz다.

### 3. live 40/20 mm 계약

기존 semantic cut 조건인 open-before-close, stable closed, close 후 1초,
TCP Z 400 mm 이상은 모두 유지한다.

- 대표 A boundary 40 mm 진입: 실제 TCP의 접근 방향을 확인하고 3 frame 연속
  접근하면 pre-arm한다. 이때는 pure planner만 호출하며 ACT queue, publisher,
  gripper 및 command counter를 변경하지 않는다.
- 대표 A boundary 20 mm 진입: 방향 조건과 closed 상태가 3 frame 연속이고
  기존 semantic cut도 ready일 때 최종 Bridge를 다시 검증한다.
- 최종 검증이 끝난 다음에만 ACT-A producer/queue를 정지한다. 검증 전후 planner
  결과가 달라지면 fail-closed한다.
- boundary 판정 위치는 actual TCP다. Bridge 시작점은 마지막으로 확인된
  `/vr/commanded_posx`, 시작 속도는 최근 15 frame actual TCP의 causal
  regression이다. ACT-A 예측 속도는 진단값일 뿐 경계조건으로 쓰지 않는다.
- initial Bridge는 고정 endpoint에서 zero velocity로 정지한다. 그 후 fresh
  ACT-B inference를 수행하고 postprocessed B chunk에 맞춘 tail을 검증한 뒤
  atomic handoff한다. 기존 rolling ACT-B 실행과 semantic completion은 유지한다.

## 오프라인 생성

    cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
    ./scripts/run_task_c_representative_v1_offline.sh

다른 조합은 `DATASET_A`, `DATASET_B`, `CONFIG`, `OUTPUT_DIR` 환경변수로
지정한다. 출력 폴더에는 다음이 생성된다.

- `representative_trajectories.npz/csv`: 단일 대표 A/B와 분산
- `all_boundary_candidates.jsonl`: 모든 coarse/refined/final 평가
- `selected_candidate.json`, `top_k_candidates.json`
- `runtime_transition_manifest.json`: V1 live 입력
- `representative_summary.json`, `config.snapshot.json`
- `checksums.sha256`

핵심 출력 파일이 이미 있으면 덮어쓰지 않고 종료한다.

## 명시적인 live 후보 실행

V1 manifest를 검토한 후에만 다음 전용 wrapper를 사용한다.

    export RUNTIME_MANIFEST=/absolute/path/runtime_transition_manifest.json
    ./scripts/run_task_c_representative_v1_live_candidate.sh

wrapper는 `REPRESENTATIVE_BOUNDARY_ENABLED=true`를 명시하고 기존 bounded live
runner를 호출한다. 기존 `scripts/run_task_c_live_candidate.sh`를 그대로
실행하면 이 값의 기본은 false이므로 V0 방식이 유지된다.

## 안전 범위

오프라인 생성과 테스트는 ROS를 시작하지 않고 로봇 명령을 발행하지 않는다.
생성 manifest는 계속 `robot_executable=false`, `dry_run_only=true`이며,
orientation, IK, collision-with-payload 검증을 완료했다는 의미가 아니다.
실물 실행에는 기존 external gate, 작업셀 점검, operator 승인과 별도의 manifest
검토가 필요하다.

현재 30+30 episode 결과와 과거 성공 run의 무동작 replay 수치는
[2026-08-18 V1 검증 기록](../../docs/task_c_representative_bridge_v1_2026-08-18.md)에
정리되어 있다.
