"""Tests for twin_control.controller — UnifiedController, all three modes."""

import math

import numpy as np
import pytest

from twin_control.controller import (
    CartesianImpedanceParams,
    ControlMode,
    ForceControlParams,
    JointImpedanceParams,
    UnifiedController,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def joint_limits() -> np.ndarray:
    return np.array((
        (-3.11, 3.11),
        (-2.093, 2.093),
        (-3.11, 3.11),
        (-2.531, 1.047),
        (-3.11, 3.11),
        (-1.0467, 1.0467),
        (-1.57, 1.57),
    ))


@pytest.fixture
def torque_limits() -> np.ndarray:
    return np.array((108.0, 108.0, 66.0, 66.0, 18.0, 18.0, 18.0))


@pytest.fixture
def controller(joint_limits: np.ndarray, torque_limits: np.ndarray) -> UnifiedController:
    return UnifiedController(joint_limits, torque_limits, dt_s=0.002)


@pytest.fixture
def zero_state() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """q=0, qd=0, identity Jacobian (approximate)."""
    q = np.zeros(7)
    qd = np.zeros(7)
    J = np.zeros((6, 7))
    J[0, 0] = 1.0
    J[1, 1] = 1.0
    J[2, 2] = 1.0
    J[3, 3] = 1.0
    J[4, 4] = 1.0
    J[5, 5] = 1.0
    return q, qd, J


# ---------------------------------------------------------------------------
# Joint Impedance
# ---------------------------------------------------------------------------

class TestJointImpedance:
    def test_stiffness_produces_restoring_torque(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        controller.set_joint_impedance_params(
            JointImpedanceParams(stiffness=(10.0,) * 7, damping=(1.0,) * 7),
        )
        # Target is 0.1 rad on joint 0
        controller.set_joint_cmd(np.array((0.1, 0, 0, 0, 0, 0, 0)))
        q, qd, J = zero_state
        tau = controller.compute(q, qd, J)
        # Torque on joint 0 should be positive (pulling toward target)
        assert tau[0] > 0.0

    def test_damping_opposes_velocity(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        controller.set_joint_impedance_params(
            JointImpedanceParams(stiffness=(0.0,) * 7, damping=(5.0,) * 7),
        )
        controller.set_joint_cmd(np.zeros(7))
        q = np.zeros(7)
        qd = np.array((1.0, 0, 0, 0, 0, 0, 0))  # positive velocity
        J = np.zeros((6, 7))
        tau = controller.compute(q, qd, J)
        # Damping should produce negative torque (opposing velocity)
        assert tau[0] < 0.0

    def test_bias_torque_added(self, controller: UnifiedController, zero_state):
        controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        controller.set_joint_cmd(np.zeros(7))
        q, qd, J = zero_state
        bias = np.array((1.0, 2.0, 3.0, 0, 0, 0, 0))
        tau = controller.compute(q, qd, J, bias_torque=bias)
        assert np.allclose(tau[:3], bias[:3], atol=1e-10)


# ---------------------------------------------------------------------------
# Cartesian Impedance
# ---------------------------------------------------------------------------

class TestCartesianImpedance:
    def test_position_error_produces_wrench(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.CARTESIAN_IMPEDANCE)
        controller.set_cartesian_impedance_params(
            CartesianImpedanceParams(
                translational_stiffness=(1000.0, 1000.0, 1000.0),
                translational_damping=(0.0, 0.0, 0.0),
                rotational_stiffness=(0.0, 0.0, 0.0),
                rotational_damping=(0.0, 0.0, 0.0),
                nullspace_stiffness=0.0,
                nullspace_damping=0.0,
            ),
        )
        # Target is 0.1 m in X
        T_target = np.eye(4)
        T_target[0, 3] = 0.1
        controller.set_cart_cmd(T_target)
        q, qd, J = zero_state
        T_cur = np.eye(4)
        # Call compute multiple times to let rate limiter ramp up
        tau = np.zeros(7)
        for _ in range(50):
            tau = controller.compute(q, qd, J, current_pose_matrix=T_cur)
        # After ramping past rate limiting, torque on joint 0 should settle
        # near Kp * error = 1000 * 0.1 = 100, clamped by torque limit (108)
        assert abs(tau[0]) > 50.0, f"tau[0]={tau[0]} expected > 50 after ramp"

    def test_nullspace_projection(
        self, controller: UnifiedController,
    ):
        controller.set_mode(ControlMode.CARTESIAN_IMPEDANCE)
        controller.set_cartesian_impedance_params(
            CartesianImpedanceParams(
                translational_stiffness=(0.0, 0.0, 0.0),
                translational_damping=(0.0, 0.0, 0.0),
                rotational_stiffness=(0.0, 0.0, 0.0),
                rotational_damping=(0.0, 0.0, 0.0),
                nullspace_stiffness=10.0,
                nullspace_damping=0.0,
            ),
        )
        # Use a non-trivial Jacobian so the nullspace projector is non-zero
        controller.set_joint_cmd(np.array((0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)))
        q = np.zeros(7)
        qd = np.zeros(7)
        # A full-rank Jacobian (approximate, 7-DOF random-like)
        J = np.array((
            (1.0, 0.5, 0.3, 0.0, 0.0, 0.0, 0.0),
            (0.2, 1.0, 0.4, 0.0, 0.0, 0.0, 0.0),
            (0.1, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        ))
        T_cur = np.eye(4)
        controller.set_cart_cmd(T_cur)
        tau = controller.compute(q, qd, J, current_pose_matrix=T_cur)
        # Nullspace should produce non-zero torque
        assert np.any(np.abs(tau) > 0.0)


# ---------------------------------------------------------------------------
# Force Control
# ---------------------------------------------------------------------------

class TestForceControl:
    def test_force_error_drives_admittance(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.FORCE)
        controller.set_cartesian_impedance_params(
            CartesianImpedanceParams(
                translational_stiffness=(1000.0, 1000.0, 1000.0),
                translational_damping=(0.0, 0.0, 0.0),
                rotational_stiffness=(0.0, 0.0, 0.0),
                rotational_damping=(0.0, 0.0, 0.0),
                nullspace_stiffness=0.0,
                nullspace_damping=0.0,
            ),
        )
        controller.set_force_params(
            ForceControlParams(
                target_force_n=10.0,
                direction=(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
                admittance_gain_m_per_ns=0.01,
                max_position_offset_m=0.05,
                feedback_alpha=1.0,
            ),
        )
        controller.set_force_cmd(10.0)
        T_target = np.eye(4)
        T_target[2, 3] = 0.3  # target Z position
        controller.set_cart_cmd(T_target)

        q, qd, J = zero_state
        T_cur = np.eye(4)
        # No contact force → admittance should push downward
        wrench = np.zeros(6)
        tau = controller.compute(q, qd, J, current_pose_matrix=T_cur, wrench=wrench)
        # Force error = 10 N → admittance offset should increase → Z target moves down
        # → position error in Z → torque via J^T
        assert np.any(np.abs(tau) > 0.0)

    def test_admittance_offset_direction(self, controller: UnifiedController):
        """Admittance offset should match force error sign."""
        controller.set_mode(ControlMode.FORCE)
        controller.set_force_params(
            ForceControlParams(
                target_force_n=10.0,
                direction=(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
                admittance_gain_m_per_ns=0.01,
                max_position_offset_m=0.05,
                feedback_alpha=1.0,
            ),
        )
        controller.set_force_cmd(10.0)
        # Run a few steps with zero wrench → offset should accumulate
        T_target = np.eye(4)
        T_target[2, 3] = 0.3
        controller.set_cart_cmd(T_target)
        q = np.zeros(7)
        qd = np.zeros(7)
        J = np.zeros((6, 7))
        J[2, 2] = 1.0
        T_cur = np.eye(4)
        for _ in range(10):
            tau = controller.compute(q, qd, J, current_pose_matrix=T_cur, wrench=np.zeros(6))
        # After several steps with no contact, offset should be positive
        # (pushing down to find contact)
        assert controller._force_offset_m > 0.0


# ---------------------------------------------------------------------------
# Torque limiting
# ---------------------------------------------------------------------------

class TestTorqueLimiting:
    def test_torque_magnitude_clamped(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        # Very high stiffness to produce large torque
        controller.set_joint_impedance_params(
            JointImpedanceParams(stiffness=(1e6,) * 7, damping=(0.0,) * 7),
        )
        controller.set_joint_cmd(np.array((1.0, 0, 0, 0, 0, 0, 0)))
        q, qd, J = zero_state
        tau = controller.compute(q, qd, J)
        # Joint 0 torque limit is 108 N·m
        assert abs(tau[0]) <= 108.0 + 1e-6

    def test_torque_rate_limited(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        controller.set_joint_impedance_params(
            JointImpedanceParams(stiffness=(1000.0,) * 7, damping=(0.0,) * 7),
        )
        controller.set_joint_cmd(np.array((0.5, 0, 0, 0, 0, 0, 0)))
        q, qd, J = zero_state
        tau1 = controller.compute(q, qd, J)
        tau2 = controller.compute(q, qd, J)
        # Rate limit = 1500 N·m/s, dt = 0.002 → max delta = 3.0 N·m
        max_delta = 1500.0 * 0.002
        assert np.all(np.abs(tau2 - tau1) <= max_delta + 1e-6)


# ---------------------------------------------------------------------------
# Parameter validation
# ---------------------------------------------------------------------------

class TestParameterValidation:
    def test_joint_impedance_params_construct(self):
        # Construction with correct length should work
        p = JointImpedanceParams(stiffness=(1.0,) * 7, damping=(0.5,) * 7)
        assert len(p.stiffness) == 7
        assert len(p.damping) == 7

    def test_cartesian_params_defaults(self):
        p = CartesianImpedanceParams()
        assert len(p.translational_stiffness) == 3
        assert all(v >= 0 for v in p.translational_stiffness)

    def test_force_params_defaults(self):
        p = ForceControlParams()
        assert p.target_force_n == 10.0
        assert len(p.direction) == 6


# ---------------------------------------------------------------------------
# Mode switching
# ---------------------------------------------------------------------------

class TestModeSwitching:
    def test_switch_from_force_resets_offset(
        self, controller: UnifiedController, zero_state,
    ):
        controller.set_mode(ControlMode.FORCE)
        controller.set_force_params(ForceControlParams())
        controller.set_force_cmd(10.0)
        controller.set_cart_cmd(np.eye(4))
        q, qd, J = zero_state
        # Run a few force steps
        for _ in range(5):
            controller.compute(q, qd, J, current_pose_matrix=np.eye(4), wrench=np.zeros(6))
        assert controller._force_offset_m != 0.0
        # Switch away
        controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        assert controller._force_offset_m == 0.0
        assert controller._filtered_force_n == 0.0
