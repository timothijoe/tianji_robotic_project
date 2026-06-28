"""Tests for twin_control.kinematics — FK, IK, Jacobian, unit modes.

Requires MuJoCo for FK (the kinematics module uses MuJoCo site poses as
ground truth).  Tests use the right chopping scene model.
"""

import math

import numpy as np
import pytest

from twin_control.kinematics import (
    IkResult,
    MarvinKinematics,
    matrix_to_xyzabc,
    xyzabc_to_matrix,
)
from twin_description.paths import right_chopping_scene_path
from twin_mujoco.runtime import TwinMujocoRuntime


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def runtime() -> TwinMujocoRuntime:
    return TwinMujocoRuntime.load(str(right_chopping_scene_path()))


@pytest.fixture
def left_kin(runtime: TwinMujocoRuntime) -> MarvinKinematics:
    k = MarvinKinematics("left", unit_mode="si")
    k.set_runtime(runtime)
    return k


@pytest.fixture
def right_kin(runtime: TwinMujocoRuntime) -> MarvinKinematics:
    k = MarvinKinematics("right", unit_mode="si")
    k.set_runtime(runtime)
    return k


@pytest.fixture
def left_kin_sdk(runtime: TwinMujocoRuntime) -> MarvinKinematics:
    k = MarvinKinematics("left", unit_mode="sdk")
    k.set_runtime(runtime)
    return k


# Joint configurations (radians) within limits
HOME_Q = np.array((0.4, -1.3, 0.0, -1.606525, 0.057176, 0.79256, 1.5))
ZERO_Q = np.zeros(7)
RANDOM_QS = [
    np.array((0.1, -0.5, 0.3, -1.0, 0.2, 0.5, 0.8)),
    np.array((-0.3, -1.0, 0.8, -1.5, -0.5, 0.3, -0.6)),
    np.array((0.8, -0.2, -0.4, -0.8, 0.6, -0.3, 1.0)),
    np.array((-1.0, -1.5, 1.0, -2.0, 1.0, 0.7, -0.2)),
    np.array((1.5, -0.8, -1.0, -0.5, -0.3, -0.5, 0.5)),
]


# ---------------------------------------------------------------------------
# FK tests
# ---------------------------------------------------------------------------

class TestForwardKinematics:
    def test_fk_returns_4x4_and_6d(self, left_kin: MarvinKinematics):
        matrix, xyzabc = left_kin.fk(ZERO_Q)
        assert matrix.shape == (4, 4)
        assert xyzabc.shape == (6,)

    def test_fk_matrix_is_homogeneous(self, left_kin: MarvinKinematics):
        for q in RANDOM_QS:
            matrix, _ = left_kin.fk(q)
            assert np.allclose(matrix[3, :], (0, 0, 0, 1), atol=1e-12)

    def test_fk_matrix_rotation_is_orthonormal(self, left_kin: MarvinKinematics):
        for q in RANDOM_QS:
            matrix, _ = left_kin.fk(q)
            R = matrix[:3, :3]
            assert np.allclose(R.T @ R, np.eye(3), atol=1e-10)

    def test_fk_matches_mujoco_site(self, right_kin: MarvinKinematics, runtime: TwinMujocoRuntime):
        """Our FK should match MuJoCo's sensor site pose exactly."""
        runtime.set_arm_positions("right", HOME_Q)
        import mujoco
        site_id = mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_SITE, "right_force_sensor_site")
        mujoco_pos = runtime.data.site_xpos[site_id].copy()
        mujoco_rot = runtime.data.site_xmat[site_id].reshape(3, 3).copy()

        matrix, _ = right_kin.fk(HOME_Q)
        assert np.allclose(matrix[:3, 3], mujoco_pos, atol=1e-8)
        assert np.allclose(matrix[:3, :3], mujoco_rot, atol=1e-8)

    def test_fk_xyzabc_roundtrip(self, left_kin: MarvinKinematics):
        for q in RANDOM_QS:
            matrix, xyzabc = left_kin.fk(q)
            recovered = xyzabc_to_matrix(xyzabc)
            assert np.allclose(matrix, recovered, atol=1e-10)

    def test_fk_can_target_tool_tip_site(self, runtime: TwinMujocoRuntime):
        kin = MarvinKinematics("right", unit_mode="si", tcp_site_name="right_tool_tip_site")
        kin.set_runtime(runtime)
        runtime.set_arm_positions("right", HOME_Q)
        import mujoco
        site_id = mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_SITE, "right_tool_tip_site")
        mujoco_pos = runtime.data.site_xpos[site_id].copy()
        mujoco_rot = runtime.data.site_xmat[site_id].reshape(3, 3).copy()

        matrix, _ = kin.fk(HOME_Q)

        assert np.allclose(matrix[:3, 3], mujoco_pos, atol=1e-8)
        assert np.allclose(matrix[:3, :3], mujoco_rot, atol=1e-8)

    def test_ik_roundtrip_tool_tip_site(self, runtime: TwinMujocoRuntime):
        kin = MarvinKinematics("right", unit_mode="si", tcp_site_name="right_tool_tip_site")
        kin.set_runtime(runtime)
        target_q = RANDOM_QS[0]
        target_matrix, _ = kin.fk(target_q)

        result = kin.ik(target_matrix, HOME_Q)

        assert result.success, f"IK failed: residual={result.residual}"
        recovered_matrix, _ = kin.fk(result.joints_rad)
        assert np.allclose(target_matrix, recovered_matrix, atol=1e-4)


class TestForwardKinematicsSDKMode:
    def test_sdk_mode_degrees_output(self, left_kin_sdk: MarvinKinematics):
        q_deg = np.array((10.0, -20.0, 30.0, -40.0, 50.0, 10.0, 10.0))
        _, xyzabc = left_kin_sdk.fk(q_deg)
        assert abs(xyzabc[3]) > 0.1 or abs(xyzabc[5]) > 0.1

    def test_sdk_vs_si_consistency(self, runtime: TwinMujocoRuntime):
        left_si = MarvinKinematics("left", unit_mode="si")
        left_si.set_runtime(runtime)
        left_sdk = MarvinKinematics("left", unit_mode="sdk")
        left_sdk.set_runtime(runtime)
        q_rad = RANDOM_QS[0]
        q_deg = np.rad2deg(q_rad)
        _, xyzabc_si = left_si.fk(q_rad)
        _, xyzabc_sdk = left_sdk.fk(q_deg)
        xyzabc_sdk_si = xyzabc_sdk.copy()
        xyzabc_sdk_si[:3] *= 0.001
        xyzabc_sdk_si[3:] = np.deg2rad(xyzabc_sdk_si[3:])
        assert np.allclose(xyzabc_si, xyzabc_sdk_si, atol=1e-8)


# ---------------------------------------------------------------------------
# IK tests
# ---------------------------------------------------------------------------

class TestInverseKinematics:
    def test_ik_recovers_fk_result(self, left_kin: MarvinKinematics):
        """FK(q) → IK → should recover q."""
        for q in RANDOM_QS[:3]:
            matrix, _ = left_kin.fk(q)
            result = left_kin.ik(matrix, q)
            assert result.success, f"IK failed for q={q}, residual={result.residual}"
            assert np.linalg.norm(result.joints_rad - q) < 0.1

    def test_ik_fk_roundtrip(self, left_kin: MarvinKinematics):
        """IK(target) → FK should match target."""
        for q_ref in RANDOM_QS[:3]:
            target_q = RANDOM_QS[3]
            target_matrix, _ = left_kin.fk(target_q)
            result = left_kin.ik(target_matrix, q_ref)
            assert result.success, f"IK failed: residual={result.residual}"
            recovered_matrix, _ = left_kin.fk(result.joints_rad)
            assert np.allclose(target_matrix, recovered_matrix, atol=1e-4)

    def test_ik_returns_ikresult(self, left_kin: MarvinKinematics):
        matrix, _ = left_kin.fk(HOME_Q)
        result = left_kin.ik(matrix, HOME_Q)
        assert isinstance(result, IkResult)
        assert result.joints_rad.shape == (7,)
        assert result.iterations > 0

    def test_ik_respects_joint_limits(self, left_kin: MarvinKinematics):
        limits = left_kin.joint_limits_rad
        matrix, _ = left_kin.fk(HOME_Q)
        result = left_kin.ik(matrix, HOME_Q)
        for i in range(7):
            assert limits[i, 0] - 1e-6 <= result.joints_rad[i] <= limits[i, 1] + 1e-6

    def test_ik_right_arm(self, right_kin: MarvinKinematics):
        matrix, _ = right_kin.fk(HOME_Q)
        result = right_kin.ik(matrix, HOME_Q)
        assert result.success


# ---------------------------------------------------------------------------
# Jacobian tests
# ---------------------------------------------------------------------------

class TestJacobian:
    def test_jacobian_shape(self, right_kin: MarvinKinematics):
        J = right_kin.jacobian(ZERO_Q)
        assert J.shape == (6, 7)

    def test_jacobian_finite_difference(self, right_kin: MarvinKinematics):
        """Analytical Jacobian should match finite differences."""
        for q in RANDOM_QS[:3]:
            J_analytical = right_kin.jacobian(q)
            J_fd = np.zeros((6, 7))
            eps = 1e-6
            matrix_base, _ = right_kin.fk(q)
            p_base = matrix_base[:3, 3].copy()
            for i in range(7):
                dq = np.zeros(7)
                dq[i] = eps
                matrix_pert, _ = right_kin.fk(q + dq)
                p_pert = matrix_pert[:3, 3]
                J_fd[:3, i] = (p_pert - p_base) / eps
                R_base = matrix_base[:3, :3]
                R_pert = matrix_pert[:3, :3]
                R_diff = R_pert @ R_base.T
                theta = math.acos(max(-1.0, min(1.0, (np.trace(R_diff) - 1.0) / 2.0)))
                if abs(theta) < 1e-12:
                    omega = np.zeros(3)
                else:
                    omega = theta / (2.0 * math.sin(theta)) * np.array((
                        R_diff[2, 1] - R_diff[1, 2],
                        R_diff[0, 2] - R_diff[2, 0],
                        R_diff[1, 0] - R_diff[0, 1],
                    ))
                J_fd[3:, i] = omega / eps
            assert np.allclose(J_analytical[:3, :], J_fd[:3, :], atol=0.1), (
                f"Linear mismatch at q={q}"
            )


# ---------------------------------------------------------------------------
# Tool offset tests
# ---------------------------------------------------------------------------

class TestToolOffset:
    def test_set_tool_changes_fk(self, right_kin: MarvinKinematics):
        matrix_before, _ = right_kin.fk(ZERO_Q)
        right_kin.set_tool((0.0, 0.0, 0.05, 0.0, 0.0, 0.0))
        matrix_after, _ = right_kin.fk(ZERO_Q)
        assert not np.allclose(matrix_before, matrix_after)
        assert matrix_after[2, 3] > matrix_before[2, 3]

    def test_remove_tool_restores_fk(self, right_kin: MarvinKinematics):
        matrix_original, _ = right_kin.fk(ZERO_Q)
        right_kin.set_tool((0.0, 0.0, 0.05, 0.0, 0.0, 0.0))
        right_kin.remove_tool()
        matrix_restored, _ = right_kin.fk(ZERO_Q)
        assert np.allclose(matrix_original, matrix_restored, atol=1e-12)

    def test_tool_with_ik(self, right_kin: MarvinKinematics):
        right_kin.set_tool((0.0, 0.0, 0.02, 0.0, 0.0, 0.0))
        matrix, _ = right_kin.fk(HOME_Q)
        result = right_kin.ik(matrix, HOME_Q)
        assert result.success


# ---------------------------------------------------------------------------
# Matrix ↔ XYZABC
# ---------------------------------------------------------------------------

class TestMatrixXYZABC:
    def test_identity(self):
        I = np.eye(4)
        xyzabc = matrix_to_xyzabc(I)
        assert np.allclose(xyzabc, (0, 0, 0, 0, 0, 0), atol=1e-12)

    def test_roundtrip(self, left_kin: MarvinKinematics):
        for q in RANDOM_QS[:3]:
            matrix, _ = left_kin.fk(q)
            xyzabc = matrix_to_xyzabc(matrix)
            recovered = xyzabc_to_matrix(xyzabc)
            assert np.allclose(matrix, recovered, atol=1e-10)


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_fk_requires_7_joints(self, left_kin: MarvinKinematics):
        with pytest.raises(Exception):
            left_kin.fk(np.zeros(6))

    def test_ik_requires_7_ref_joints(self, left_kin: MarvinKinematics):
        matrix, _ = left_kin.fk(HOME_Q)
        with pytest.raises(Exception):
            left_kin.ik(matrix, np.zeros(6))

    def test_invalid_arm_raises(self):
        with pytest.raises(ValueError, match="arm"):
            MarvinKinematics("up")

    def test_invalid_unit_mode_raises(self):
        with pytest.raises(ValueError, match="unit_mode"):
            MarvinKinematics("left", unit_mode="cgs")
