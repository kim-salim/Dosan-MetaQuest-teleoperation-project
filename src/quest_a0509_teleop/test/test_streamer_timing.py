import csv

from quest_a0509_teleop.servol_rt_streamer_node import (
    DR_COND_NONE,
    _TIMING_TRACE_HEADER,
    _servol_motion_conditions,
    _write_timing_trace_csv,
)


def test_servol_motion_conditions_use_doosan_auto_sentinel():
    velocity, acceleration = _servol_motion_conditions(True)

    assert velocity == [DR_COND_NONE] * 6
    assert acceleration == [DR_COND_NONE] * 6
    assert velocity is not acceleration


def test_servol_motion_conditions_can_restore_explicit_zero_mode():
    velocity, acceleration = _servol_motion_conditions(False)

    assert velocity == [0.0] * 6
    assert acceleration == [0.0] * 6


def test_timing_trace_is_atomically_written_with_header(tmp_path):
    output_path = tmp_path / "trace.csv"
    row = tuple(range(len(_TIMING_TRACE_HEADER)))

    _write_timing_trace_csv(output_path, [row])

    assert output_path.exists()
    assert not output_path.with_suffix(".csv.tmp").exists()
    with output_path.open(newline="", encoding="utf-8") as stream:
        contents = list(csv.reader(stream))
    assert contents[0] == list(_TIMING_TRACE_HEADER)
    assert contents[1] == [str(value) for value in row]
