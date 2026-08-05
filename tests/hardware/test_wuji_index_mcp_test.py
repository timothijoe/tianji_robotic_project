import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from tianji_robotics.hardware.wuji_hand.index_mcp_test import (
    IndexMcpTestPlan,
    build_index_mcp_test_plan,
    run_index_mcp_test,
)


_CLI_SPEC = importlib.util.spec_from_file_location(
    "test_wuji_right_index_mcp_cli",
    Path(__file__).parents[2] / "scripts" / "test_wuji_right_index_mcp.py",
)
cli = importlib.util.module_from_spec(_CLI_SPEC)
assert _CLI_SPEC.loader is not None
_CLI_SPEC.loader.exec_module(cli)


class FakeHand:
    def __init__(
        self,
        *,
        side=0,
        errors=None,
        temperatures=None,
        position=0.4,
        lower=None,
        upper=None,
        fail_on_target=None,
    ):
        self.side = side
        self.errors = np.zeros((5, 4), dtype=np.uint32) if errors is None else errors
        self.temperatures = np.full((5, 4), 25.0) if temperatures is None else temperatures
        self.positions = np.zeros((5, 4))
        self.positions[1, 0] = position
        self.lower = np.full((5, 4), -1.0) if lower is None else lower
        self.upper = np.full((5, 4), 1.0) if upper is None else upper
        self.calls = []
        self.fail_on_target = fail_on_target
        self._target_writes = 0

    def read_handedness(self):
        return self.side

    def read_joint_error_code(self):
        return self.errors

    def read_joint_temperature(self):
        return self.temperatures

    def read_joint_actual_position(self):
        return self.positions

    def read_joint_lower_limit(self):
        return self.lower

    def read_joint_upper_limit(self):
        return self.upper

    def write_joint_enabled(self, enabled):
        self.calls.append(("enabled", enabled))

    def finger(self, finger_index):
        assert finger_index == 1
        return self

    def joint(self, joint_index):
        assert joint_index == 0
        return self

    def write_joint_target_position(self, target):
        self._target_writes += 1
        if self._target_writes == self.fail_on_target:
            raise RuntimeError("write failed")
        self.calls.append(("target", target))


def test_cli_defaults_to_dry_run(monkeypatch, capsys):
    captured = {}
    monkeypatch.setattr(cli, "_create_hand", lambda serial: FakeHand())
    monkeypatch.setattr(
        cli,
        "run_index_mcp_test",
        lambda hand, plan, **kw: captured.update(kw) or False,
    )

    assert cli.main(["--serial-number", "365939643134"]) == 0

    assert captured == {"execute": False, "dwell_s": 0.5}
    assert "dry-run" in capsys.readouterr().out


def test_cli_prints_plan_before_execution(monkeypatch, capsys):
    hand = FakeHand(position=0.4)
    monkeypatch.setattr(cli, "_create_hand", lambda serial: hand)
    monkeypatch.setattr(cli, "run_index_mcp_test", lambda *args, **kwargs: True)

    assert cli.main(["--serial-number", "365939643134", "--execute"]) == 0

    output = capsys.readouterr().out
    assert "current angle: 0.400000 rad" in output
    assert "targets: 0.350000, 0.450000, 0.400000 rad" in output
    assert "max temperature: 25.0°C" in output


def test_cli_reports_preflight_or_sdk_errors(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_create_hand", lambda serial: (_ for _ in ()).throw(RuntimeError("offline")))

    assert cli.main(["--serial-number", "365939643134"]) == 1

    assert capsys.readouterr().out.strip() == "preflight failed: offline"


def test_cli_imports_when_executed_directly_from_repo_root():
    result = subprocess.run(
        [
            sys.executable,
            "scripts/test_wuji_right_index_mcp.py",
            "--help",
        ],
        cwd=Path(__file__).parents[2],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--serial-number" in result.stdout


def test_builds_one_small_index_mcp_return_motion():
    current = 0.4
    plan = build_index_mcp_test_plan(FakeHand(position=current))

    assert plan == IndexMcpTestPlan(
        current_rad=current,
        targets_rad=(current - 0.05, current + 0.05, current),
        max_temperature_c=25.0,
    )


def test_preserves_exact_non_round_number_targets():
    current = 0.123456789012345

    plan = build_index_mcp_test_plan(FakeHand(position=current))

    assert plan.targets_rad == (current - 0.05, current + 0.05, current)


@pytest.mark.parametrize(
    ("hand", "message"),
    [
        (FakeHand(side=1), "right hand"),
        (FakeHand(errors=np.ones((5, 4), dtype=np.uint32)), "error codes"),
        (FakeHand(temperatures=np.full((5, 4), 40.1)), "temperature"),
    ],
)
def test_rejects_unsafe_preflight(hand, message):
    with pytest.raises(ValueError, match=message):
        build_index_mcp_test_plan(hand)


def test_rejects_targets_without_hardware_limit_margin():
    lower = np.full((5, 4), -1.0)
    upper = np.full((5, 4), 1.0)
    lower[1, 0] = 0.35

    with pytest.raises(ValueError, match="hardware-limit margin"):
        build_index_mcp_test_plan(FakeHand(lower=lower))


def test_dry_run_never_enables_or_writes():
    hand, plan = FakeHand(), build_index_mcp_test_plan(FakeHand())

    assert run_index_mcp_test(hand, plan, execute=False, dwell_s=0) is False

    assert hand.calls == []


def test_execution_writes_three_targets_then_disables():
    hand = FakeHand()
    plan = build_index_mcp_test_plan(hand)

    assert run_index_mcp_test(hand, plan, execute=True, dwell_s=0)

    assert hand.calls == [
        ("enabled", True),
        *[("target", target) for target in plan.targets_rad],
        ("enabled", False),
    ]


def test_write_error_still_disables():
    hand = FakeHand(fail_on_target=2)

    with pytest.raises(RuntimeError, match="write failed"):
        run_index_mcp_test(
            hand, build_index_mcp_test_plan(hand), execute=True, dwell_s=0
        )

    assert hand.calls[-1] == ("enabled", False)


@pytest.mark.parametrize("dwell_s", [-0.1, np.inf, np.nan])
def test_execution_rejects_non_finite_or_negative_dwell_before_enabling(dwell_s):
    hand = FakeHand()

    with pytest.raises(ValueError, match="finite and non-negative"):
        run_index_mcp_test(
            hand, build_index_mcp_test_plan(hand), execute=True, dwell_s=dwell_s
        )

    assert hand.calls == []
