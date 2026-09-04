# A0509 semantic segmentation

이 디렉터리는 LeRobot 원본 데이터셋을 수정하지 않고, 여러 demonstration을
semantic event로 정렬하여 하나의 대표 그래프로 만드는 오프라인 분석 도구를
보관한다.

T1 분석은 다음 명령으로 재생성한다.

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
PYTHONPATH="/usr/lib/python3/dist-packages:$PWD" \
  /home/rvlab/venvs/lerobot/bin/python \
  -m offline_tools.semantic_segmentation.analyze_t1_standardized_graph \
  --dataset-root /home/rvlab/lerobot_datasets/a0509_blue_block_t1_throw_away_inthe_container \
  --output-dir docs/artifacts/t1_semantic_graph_2026-08-23
```

T2~T8 semantic-only V3 분석은 task를 지정하여 생성한다. 현재 Jetson의
시스템 `matplotlib`과 LeRobot 환경의 `pyarrow`를 함께 쓰는 재현 명령은 다음과
같다.

```bash
cd /home/rvlab/Dosan-MetaQuest-teleoperation-project
/usr/bin/python3 -c '
import runpy
import sys

sys.path.append("/home/rvlab/venvs/lerobot/lib/python3.12/site-packages")
sys.argv = [
    "offline_tools/semantic_segmentation/analyze_a0509_semantic_only_v3.py",
    "--tasks", "t7", "t8",
    "--date", "2026-08-29",
]
runpy.run_path(sys.argv[0], run_name="__main__")
'
```

분석기는 다음 원칙을 사용한다.

- `observation.state`의 `gripper_commanded_state`에서 task별로 선언된
  `C1/O1/C2/O2` event grammar를 검출하고 모든 episode가 일치하는지 검사한다.
- 좌표는 Doosan base frame의 mm 단위를 유지한다.
- 각 semantic 구간만 Cartesian arc length `0..1`로 정렬한다.
- component median과 함께 mean, covariance, MAD 및 phase residual을 저장하되,
  특정 episode·frame·이미지를 평균 대표로 선택하지 않는다.
- 구형 runtime 경계, 전환 적합도, Bridge 비용은 semantic-only 산출물에 포함하지 않는다.
- 결과는 오프라인 annotation/분석 자료이며 실물 로봇 실행 manifest가 아니다.
