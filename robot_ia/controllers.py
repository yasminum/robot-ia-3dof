"""Controladores del brazo antropomórfico 3GDL:
1. SmoothReference: interpolador polinomial quíntico C² (posición, velocidad, aceleración).
2. PID (PD por torque calculado / CTC base): tau = M(q)*(qdd_ref + Kp*e + Kd*edot) + bias(q,dq).
3. KinematicController (Control Cinemático CLIK con pseudoinversa amortiguada DLS J^+_DLS):
   qdot_cmd = J^+_DLS(q) * (pdot_ref + K_cin * (p_ref - p(q))), acoplado a compensación dinámica.
"""
import numpy as np
from robot_ia.kinematics import fk, jacobian, dls_pseudoinverse


class SmoothReference:
    def __init__(self, initial, target, time0=0., duration=3.0):
        self.begin=np.asarray(initial,dtype=float).copy()
        self.end=np.asarray(target,dtype=float).copy()
        self.t0=float(time0); self.duration=float(duration)
    def retarget(self, current, target, now, duration=3.0):
        self.__init__(current,target,now,duration)
    def evaluate(self, t):
        u=np.clip((float(t)-self.t0)/self.duration,0.,1.)
        s=10*u**3-15*u**4+6*u**5
        ds=(30*u*u-60*u**3+30*u**4)/self.duration if 0.<u<1. else 0.
        dds=(60*u-180*u*u+120*u**3)/(self.duration**2) if 0.<u<1. else 0.
        delta=self.end-self.begin
        return self.begin+s*delta,ds*delta,dds*delta


class PID:
    """PD de torque calculado; se conserva el nombre PID como referencia histórica del panel."""
    def __init__(self, wn=13., zeta=1.1):
        self.kp=np.full(3,float(wn)**2)
        self.kd=np.full(3,2*float(zeta)*float(wn))
    def step(self,q,dq,qref,dqref,ddqref,M,bias,limits):
        q=np.asarray(q);dq=np.asarray(dq)
        accel=np.asarray(ddqref)+self.kp*(qref-q)+self.kd*(dqref-dq)
        tau=np.asarray(M)@accel+np.asarray(bias)
        return np.clip(tau,-limits,limits)


class KinematicController:
    """Controlador Cinemático en lazo cerrado (CLIK - Closed-Loop Inverse Kinematics)
    con pseudoinversa amortiguada (Damped Least-Squares) para robustez ante singularidades:
        p_ref = fk(q_ref),  e_cart = p_ref - fk(q)
        qdot_cmd = dq_ref + J^+_DLS(q) * (K_cart * e_cart)
        tau = M(q) * (ddq_ref + Kv * (qdot_cmd - dq) + Kp_joint * (q_ref - q)) + bias
    """
    def __init__(self, k_cart=4.5, kv_joint=28.6, kp_joint=169.0, damping=0.025):
        self.k_cart = float(k_cart)
        self.kv_joint = float(kv_joint)
        self.kp_joint = float(kp_joint)
        self.damping = float(damping)
        self.last_qdot_cmd = np.zeros(3)

    def step(self, q, dq, qref, dqref, ddqref, M, bias, limits):
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)
        qref = np.asarray(qref, dtype=float)
        dqref = np.asarray(dqref, dtype=float)
        ddqref = np.asarray(ddqref, dtype=float)
        M = np.asarray(M, dtype=float)
        bias = np.asarray(bias, dtype=float)
        limits = np.asarray(limits, dtype=float)

        p_now = fk(q)
        p_des = fk(qref)
        e_cart = p_des - p_now

        J_pinv = dls_pseudoinverse(q, damping=self.damping)
        # Cuando el codo cruza de rama (sign(q[2]) != sign(qref[2])), ponderar la corrección cartesiana
        branch_scale = 1.0 if (q[2] * qref[2] >= 0.0 or np.linalg.norm(qref - q) < 0.35) else 0.15
        qdot_cmd = dqref + branch_scale * (J_pinv @ (self.k_cart * e_cart))
        self.last_qdot_cmd = qdot_cmd.copy()

        accel = ddqref + self.kv_joint * (qdot_cmd - dq) + self.kp_joint * (qref - q)
        tau = M @ accel + bias
        return np.clip(tau, -limits, limits)
