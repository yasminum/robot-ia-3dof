import numpy as np
from robot_ia.mpc import MPC
from main import reachable_target
from robot_ia.kinematics import fk


def test_mpc_returns_bounded_torques():
    m=MPC(horizon=7)
    u=m.command(np.array([0.,.2,.75]),np.zeros(3),np.array([.4,.1,1.]),np.eye(3)*.01,
                np.array([0.,.12,.05]),np.array([.7,.7,.45]))
    assert np.all(np.isfinite(u))
    assert np.all(np.abs(u)<=np.array([.7,.7,.45])+1e-9)
    assert m.status=='OK'


def test_mpc_steady_state():
    m=MPC(horizon=8)
    q=np.array([.2,.1,.7]);bias=np.array([0.,.13,.06]);limits=np.array([.7,.7,.45])
    u=m.command(q,np.zeros(3),q,np.eye(3)*.01,bias,limits)
    assert np.linalg.norm(u-bias)<.005


def test_reachable_target():
    q=np.array([.4,.15,.8]);target=fk(q)
    resolved=reachable_target(target,np.array([[-2.8,2.8],[-1.25,1.35],[-2.4,2.4]]))
    assert np.linalg.norm(fk(resolved)-target)<1e-8
