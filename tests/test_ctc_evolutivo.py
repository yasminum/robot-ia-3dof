import numpy as np
import pytest
from robot_ia.ctc_evolutivo import validar_ganancias,torque_ctc,GANANCIAS_REFERENCIA
from robot_ia.controllers import PID

def test_seis_ganancias():
    assert validar_ganancias(GANANCIAS_REFERENCIA).shape == (6,)

def test_rechazo_fuera_rango():
    with pytest.raises(ValueError): validar_ganancias([-1]*6)

def test_saturacion():
    t=torque_ctc(np.zeros(3),np.zeros(3),np.ones(3),np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.ones(3)*0.4,GANANCIAS_REFERENCIA)
    assert np.all(np.abs(t)<=0.4+1e-12)

def test_equivalencia_ctc_base():
    p=PID();q=np.array([.1,.2,-.3]);dq=np.array([-.1,.4,.01]);qr=np.array([.3,.5,-.2]);dr=np.ones(3)*.01;ar=np.ones(3)*.1;M=np.diag([.2,.3,.25]);bias=np.array([.01,.08,.2]);limit=np.ones(3)*.7
    np.testing.assert_allclose(torque_ctc(q,dq,qr,dr,ar,M,bias,limit,GANANCIAS_REFERENCIA),p.step(q,dq,qr,dr,ar,M,bias,limit))
