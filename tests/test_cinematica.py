import numpy as np
import pytest
from robot_ia.kinematics import fk,ik,jacobian,BASE,L1,L2

def test_extension_horizontal():
    assert np.allclose(fk([0,0,0]),[L1+L2,0,BASE])

@pytest.mark.parametrize('q', [[0.1,0.1,0.6],[0.5,0.4,-0.8],[-1.1,-0.2,1.5]])
def test_ik_fk(q):
    p=fk(q)
    actual=ik(p,elbow=1 if q[2]>=0 else -1)
    assert np.linalg.norm(fk(actual)-p)<1e-9

def test_jacobiano_diferencias_finitas():
    q=np.array([.3,.2,.9]); eps=1e-6
    estimated=np.column_stack([(fk(q+eps*np.eye(3)[i])-fk(q-eps*np.eye(3)[i]))/(2*eps) for i in range(3)])
    assert np.allclose(jacobian(q),estimated,atol=1e-9)

def test_punto_inaccesible():
    with pytest.raises(ValueError):ik([2.,0.,0.])
