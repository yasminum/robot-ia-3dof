import numpy as np
from robot_ia.controllers import SmoothReference,PID

def test_smooth_reference():
    a=np.array([0.,.2,.75]);b=np.array([.4,-.3,1.])
    r=SmoothReference(a,b,0.,3.)
    assert np.allclose(r.evaluate(0)[0],a)
    assert np.allclose(r.evaluate(3)[0],b)
    for time in (0.,3.):
        _,v,acc=r.evaluate(time)
        assert np.allclose(v,0)
        assert np.allclose(acc,0)

def test_pid_holds_reference_with_bias():
    c=PID();q=np.array([.2,.1,.7]); M=np.diag([.01,.02,.03]);bias=np.array([0.,.13,.06]);lim=np.array([.7,.7,.45])
    u=c.step(q,np.zeros(3),q,np.zeros(3),np.zeros(3),M,bias,lim)
    assert np.allclose(u,bias)

def test_pid_limits():
    c=PID();lim=np.array([.7,.7,.45])
    u=c.step(np.zeros(3),np.zeros(3),np.ones(3),np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),lim)
    assert np.all(np.abs(u)<=lim)
