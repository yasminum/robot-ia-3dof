import numpy as np
from pathlib import Path
from robot_ia.kinematics import (
    fk, fk_homogeneous, jacobian, cartesian_velocity,
    jacobian_det, manipulability, dls_pseudoinverse,
    inverse_differential_kinematics, singularity_analysis
)
from robot_ia.controllers import KinematicController
from robot_ia.mpc import MPC
from robot_ia.ctc_evolutivo import cargar_ganancias, GANANCIAS_REFERENCIA


def test_dh_homogeneous_matches_fk():
    for q in ([0.2, 0.3, -0.5], [-1.1, 0.85, -0.75], [0.0, 0.0, 0.0]):
        _, _, T03 = fk_homogeneous(q)
        assert np.allclose(T03[:3, 3], fk(q), atol=1e-12)


def test_analytical_determinant_matches_numpy_det():
    for q in ([0.2, 0.4, -0.7], [-0.8, 0.1, 1.2], [0.5, -0.3, 0.0]):
        J = jacobian(q)
        assert np.isclose(jacobian_det(q), np.linalg.det(J), atol=1e-12)
        assert np.isclose(manipulability(q), abs(np.linalg.det(J)), atol=1e-12)


def test_singularity_detection_elbow_and_regular():
    # Codo extendido (q3 = 0) -> singularidad de codo
    s_elbow = singularity_analysis([0.3, 0.2, 0.0])
    assert s_elbow["is_singular"] is True
    assert "CODO" in s_elbow["tipo"]
    assert abs(s_elbow["det_J"]) < 1e-12

    # Postura regular
    s_reg = singularity_analysis([0.3, 0.35, -0.85])
    assert s_reg["is_singular"] is False
    assert s_reg["manipulability"] > 1e-3


def test_dls_pseudoinverse_bounded_at_singularity():
    q_sing = np.array([0.0, 0.1, 0.0])
    v_des = np.array([0.05, -0.02, 0.03])
    dq = inverse_differential_kinematics(q_sing, v_des, damping=0.015)
    assert np.all(np.isfinite(dq))
    assert np.linalg.norm(dq) < 10.0

    # Lejos de singularidad, J * J^+_DLS(q) * v ≈ v cuando damping -> 0
    q_reg = np.array([0.2, 0.3, -0.8])
    J_pinv = dls_pseudoinverse(q_reg, damping=1e-6)
    assert np.allclose(jacobian(q_reg) @ (J_pinv @ v_des), v_des, atol=1e-5)


def test_cartesian_velocity():
    q = np.array([0.2, 0.3, -0.6])
    dq = np.array([0.5, -0.4, 0.8])
    v = cartesian_velocity(q, dq)
    assert np.allclose(v, jacobian(q) @ dq)


def test_kinematic_controller_and_hybrid_ctc_mpc():
    q = np.array([0.1, 0.2, -0.6])
    dq = np.zeros(3)
    qref = np.array([0.2, 0.3, -0.5])
    dqref = np.zeros(3)
    ddqref = np.zeros(3)
    M = np.diag([0.015, 0.02, 0.01])
    bias = np.array([0.0, 0.12, 0.05])
    limits = np.array([0.7, 0.7, 0.45])

    kin = KinematicController()
    tau_kin = kin.step(q, dq, qref, dqref, ddqref, M, bias, limits)
    assert tau_kin.shape == (3,)
    assert np.all(np.abs(tau_kin) <= limits + 1e-9)

    mpc = MPC(horizon=6)
    tau_hyb = mpc.command(q, dq, qref, M, bias, limits, dqref, ddqref, theta_ctc=GANANCIAS_REFERENCIA)
    assert tau_hyb.shape == (3,)
    assert np.all(np.abs(tau_hyb) <= limits + 1e-9)


def test_nsga2_json_loaded():
    path = Path(__file__).resolve().parents[1] / "results" / "evolutivo_v040" / "ganancias_seleccionadas.json"
    assert path.exists()
    theta = cargar_ganancias(path)
    assert theta.shape == (6,)
