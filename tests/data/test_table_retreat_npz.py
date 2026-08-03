import numpy as np

from tianji_robotics.data.table_retreat import load_corrected_trajectory_npz, save_corrected_trajectory_npz, write_correction_report_json
from tianji_robotics.workflows.wuji_table_retreat import CorrectedHandTrajectory, CorrectionReport


def test_corrected_trajectory_round_trips_without_pickle(tmp_path):
    value=CorrectedHandTrajectory(np.array([0,2_000_000]),np.zeros((2,20)),np.zeros((2,3)),np.tile([1.,0,0,0],(2,1)),("PLACE","HOLD"),7)
    path=save_corrected_trajectory_npz(value,tmp_path/"nested/result.npz")
    loaded=load_corrected_trajectory_npz(path)
    np.testing.assert_array_equal(loaded.positions_rad,value.positions_rad)
    assert loaded.phases==value.phases and loaded.source_frame==7


def test_report_writes_plain_json(tmp_path):
    report=CorrectionReport(7,123,.5,(.03,0,0),.03,.011,.001,.002,.0004,.006)
    path=write_correction_report_json(report,tmp_path/"report.json")
    assert '"source_frame":7' in path.read_text()
