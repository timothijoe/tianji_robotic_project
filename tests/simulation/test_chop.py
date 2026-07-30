import csv
from statistics import median

import pytest

from twin_sim.tasks.chop import ChopConfig, run_chop


def test_one_chop_completes_and_logs_all_phases(tmp_path):
    path = tmp_path / "chop.csv"

    with pytest.warns(RuntimeWarning):
        result = run_chop(ChopConfig(control_dt_s=0.01), log_path=path)

    assert result.completed
    phases = {row["phase"] for row in csv.DictReader(path.open())}
    assert phases >= {"APPROACH", "DESCEND", "HOLD", "RETRACT", "COMPLETE"}
    assert result.final_tip_z_m >= result.safe_tip_z_m - 0.005


def test_force_warning_does_not_cancel_task(tmp_path):
    with pytest.warns(RuntimeWarning):
        result = run_chop(
            ChopConfig(control_dt_s=0.01, force_warning_threshold_n=0.0),
            log_path=tmp_path / "warning.csv",
        )

    assert result.completed
    assert result.warning_count > 0


def test_chop_force_has_contact_and_lift_trend(tmp_path):
    with pytest.warns(RuntimeWarning):
        result = run_chop(ChopConfig(), log_path=tmp_path / "trend.csv")

    approach = [sample.filtered_force_n for sample in result.samples if sample.phase == "APPROACH"]
    descent = [sample.filtered_force_n for sample in result.samples if sample.phase == "DESCEND"]
    retract = [sample.filtered_force_n for sample in result.samples if sample.phase == "RETRACT"]
    baseline = median(approach)
    contact = median(descent[len(descent) // 2 :])
    lifted = median(retract[len(retract) // 2 :])
    assert contact > baseline
    assert lifted < contact


def test_chop_validates_actual_log_destination_before_config_or_motion(tmp_path):
    with pytest.raises(IsADirectoryError):
        run_chop(ChopConfig(control_dt_s=0.0), log_path=tmp_path)
