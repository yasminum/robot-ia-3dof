"""Geometría basada en Tabla 3.2 y ecuación 3.16 de Conejo Benítez (2021).
Incluye:
- Cinemática directa (vectorial y por matrices homogéneas Denavit-Hartenberg T01, T02, T03)
- Cinemática inversa (ambas ramas: codo abajo / codo arriba)
- Cinemática diferencial directa (p_dot = J(q) q_dot) e inversa amortiguada (DLS)
- Análisis de singularidades (determinante analítico, índice de Yoshikawa, número de condición)
"""
import numpy as np

BASE, L1, L2 = 0.079, 0.113, 0.140


def dh_table(q):
    """Tabla de parámetros Denavit-Hartenberg estándar [theta_i, d_i, a_i, alpha_i] para el brazo 3GDL RRR."""
    q1, q2, q3 = np.asarray(q, dtype=float)
    return [
        (q1, BASE, 0.0, np.pi / 2.0),
        (q2, 0.0, L1, 0.0),
        (q3, 0.0, L2, 0.0),
    ]


def dh_matrix(theta, d, a, alpha):
    """Matriz de transformación homogénea 4x4 según convención Denavit-Hartenberg estándar."""
    ct, st = np.cos(float(theta)), np.sin(float(theta))
    ca, sa = np.cos(float(alpha)), np.sin(float(alpha))
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.,       sa,       ca,      d],
        [0.,       0.,       0.,     1.],
    ], dtype=float)


def fk_homogeneous(q):
    """Retorna las matrices homogéneas acumuladas (T01, T02, T03) de la cadena cinemática."""
    params = dh_table(q)
    A1 = dh_matrix(*params[0])
    A2 = dh_matrix(*params[1])
    A3 = dh_matrix(*params[2])
    T01 = A1
    T02 = T01 @ A2
    T03 = T02 @ A3
    return T01, T02, T03


def fk(q):
    a, b, c = np.asarray(q, dtype=float)
    r = L1*np.cos(b) + L2*np.cos(b+c)
    return np.array([r*np.cos(a), r*np.sin(a), BASE + L1*np.sin(b) + L2*np.sin(b+c)])


def jacobian(q):
    a, b, c = np.asarray(q, dtype=float)
    r = L1*np.cos(b) + L2*np.cos(b+c)
    drb = -L1*np.sin(b) - L2*np.sin(b+c)
    drc = -L2*np.sin(b+c)
    return np.array([[-r*np.sin(a), drb*np.cos(a), drc*np.cos(a)],
                     [ r*np.cos(a), drb*np.sin(a), drc*np.sin(a)],
                     [0., L1*np.cos(b)+L2*np.cos(b+c), L2*np.cos(b+c)]])


def cartesian_velocity(q, dq):
    """Cinemática diferencial directa: p_dot = J(q) * q_dot [m/s]."""
    J = jacobian(q)
    dq = np.asarray(dq, dtype=float)
    return J @ dq


def jacobian_det(q):
    """Determinante analítico exacto: det(J) = -L1 * L2 * sin(q3) * (L1*cos(q2) + L2*cos(q2+q3))."""
    _, q2, q3 = np.asarray(q, dtype=float)
    r = L1 * np.cos(q2) + L2 * np.cos(q2 + q3)
    return float(-L1 * L2 * np.sin(q3) * r)


def manipulability(q):
    """Índice de manipulabilidad de Yoshikawa w(q) = sqrt(det(J * J^T)) = |det(J)|."""
    J = jacobian(q)
    jjt = J @ J.T
    return float(np.sqrt(max(0.0, float(np.linalg.det(jjt)))))


def dls_pseudoinverse(q, damping=0.015):
    """Pseudoinversa amortiguada de Levenberg-Marquardt (Damped Least-Squares): J^T (J J^T + lambda^2 I)^-1."""
    J = jacobian(q)
    lam2 = float(damping) ** 2
    return J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(3))


def inverse_differential_kinematics(q, v_cart, damping=0.015):
    """Cinemática diferencial inversa regularizada: q_dot = J^+_DLS(q) * p_dot."""
    J_pinv = dls_pseudoinverse(q, damping=damping)
    return J_pinv @ np.asarray(v_cart, dtype=float)


def singularity_analysis(q, threshold=4e-4):
    """Analiza proximidad a singularidades geométricas (codo extendido/plegado y eje vertical del hombro)."""
    _, q2, q3 = np.asarray(q, dtype=float)
    J = jacobian(q)
    det_val = jacobian_det(q)
    w_val = manipulability(q)
    r_val = float(L1 * np.cos(q2) + L2 * np.cos(q2 + q3))
    s3_val = float(np.sin(q3))
    sv = np.linalg.svd(J, compute_uv=False)
    cond_val = float(sv[0] / max(sv[-1], 1e-12))

    elbow_near = abs(s3_val) < 0.08
    shoulder_near = abs(r_val) < 0.015
    is_singular = (w_val < threshold) or (cond_val > 80.0)

    if elbow_near and shoulder_near:
        tipo = "SINGULARIDAD DOBLE (CODO + HOMBRO)"
    elif elbow_near:
        tipo = "SINGULARIDAD DE CODO (sin(q3)≈0)"
    elif shoulder_near:
        tipo = "SINGULARIDAD AXIAL HOMBRO (r≈0)"
    elif is_singular:
        tipo = "CERCA DE SINGULARIDAD"
    else:
        tipo = "OPERACIÓN REGULAR"

    return {
        "det_J": det_val,
        "manipulability": w_val,
        "cond_J": cond_val,
        "radial_reach_m": r_val,
        "elbow_sin_q3": s3_val,
        "singular_values": sv,
        "is_singular": bool(is_singular),
        "tipo": tipo,
    }


def ik(point, elbow=1):
    """Ambas ramas: elbow=+1/-1; raises ValueError if geometrically unreachable."""
    x, y, z = np.asarray(point, dtype=float)
    rho = np.hypot(x, y); zz = z-BASE
    D = (rho*rho + zz*zz - L1*L1 - L2*L2)/(2*L1*L2)
    if abs(D) > 1+1e-9:
        raise ValueError('Objetivo fuera del alcance geométrico')
    D = np.clip(D, -1, 1)
    q3 = np.arctan2(np.sign(elbow)*np.sqrt(max(0.,1-D*D)), D)
    q2 = np.arctan2(zz, rho)-np.arctan2(L2*np.sin(q3), L1+L2*np.cos(q3))
    return np.array([np.arctan2(y,x), q2, q3])


def quintic(t, duration):
    s = np.clip(t/duration, 0., 1.)
    return 10*s**3-15*s**4+6*s**5
