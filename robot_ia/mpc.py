"""MPC lineal local de torque y arquitectura Híbrida CTC–MPC, con referencia de posición/velocidad/aceleración.

Modelo aproximado de doble integrador con M y bias constantes en el horizonte.
En modo Híbrido CTC–MPC (con ganancias NSGA-II), el bloque CTC evolutivo proporciona la ley nominal
de linealización por realimentación y el optimizador predictivo calcula la corrección óptima bajo
restricciones de torque sin duplicar la compensación dinámica.
"""
import numpy as np
from scipy.optimize import minimize


class MPC:
    def __init__(self, horizon=8, step_s=0.04):
        self.N=int(horizon)
        self.h=float(step_s)
        self.last=np.zeros(3)
        self.status='sin resolver'
        self.success_count=0
        self.failure_count=0
        self.approx_count=0
        self.last_reason=''
        self._kp=np.full(3, 13.0**2)
        self._kd=np.full(3, 2.0*1.1*13.0)

    def command(self,q,dq,qref,M,bias,limits,dqref=None,ddqref=None,theta_ctc=None):
        vals=[np.asarray(z,dtype=float) for z in (q,dq,qref,M,bias,limits)]
        q,dq,qref,M,bias,limits=vals
        dqref=np.zeros(3) if dqref is None else np.asarray(dqref,float)
        ddqref=np.zeros(3) if ddqref is None else np.asarray(ddqref,float)
        if theta_ctc is not None:
            th=np.asarray(theta_ctc,dtype=float)
            kp_use,kd_use=th[:3],th[3:]
        else:
            kp_use,kd_use=self._kp,self._kd
        pd_tau=np.clip(M@(ddqref+kp_use*(qref-q)+kd_use*(dqref-dq))+bias,-limits,limits)
        if not all(np.all(np.isfinite(x)) for x in (q,dq,qref,M,bias,dqref,ddqref)):
            self.status='FALLBACK';self.failure_count+=1;self.last_reason='entrada no finita'
            return np.clip(np.nan_to_num(pd_tau),-limits,limits)
        try:
            Minv=np.linalg.solve(M,np.eye(3))
            h,n=self.h,self.N
            A=np.eye(6);A[:3,3:]=np.eye(3)*h
            B=np.vstack((.5*h*h*Minv,h*Minv))
            powers=[np.eye(6)]
            for _ in range(n):powers.append(powers[-1]@A)
            S=np.zeros((6*n,3*n));baseline=np.zeros(6*n)
            # Control decision is physical torque (not gravity-shifted variable).
            x0=np.r_[q,dq]
            for k in range(n):
                seg=slice(6*k,6*k+6)
                baseline[seg]=powers[k+1]@x0
                for j in range(k+1):
                    S[seg,3*j:3*j+3]=powers[k-j]@B
                baseline[seg]-=sum((powers[k-j]@B@bias for j in range(k+1)),np.zeros(6))
            ref=np.concatenate([np.r_[qref+(k+1)*h*dqref+.5*((k+1)*h)**2*ddqref,
                                      dqref+(k+1)*h*ddqref] for k in range(n)])
            Q=np.diag([350.,350.,350.,6.,6.,6.]);Qbig=np.kron(np.eye(n),Q)
            # Follow safe PD/CTC baseline as a regularizer; exact optimization still adjusts torque.
            Rbig=np.eye(3*n)*1.5
            pd_seq=np.tile(pd_tau,n)
            H=S.T@Qbig@S+Rbig
            g=S.T@Qbig@(baseline-ref)-Rbig@pd_seq
            lo=np.tile(-limits,n);hi=np.tile(limits,n)
            initial=np.tile(np.clip(self.last,-limits,limits),n)
            def objective(u):return float(u@H@u+2*g@u),2*(H@u+g)
            res=minimize(objective,initial,jac=True,bounds=list(zip(lo,hi)),method='L-BFGS-B',
                         options={'maxiter':110,'ftol':1e-10,'gtol':1e-7})
            if np.all(np.isfinite(res.x)) and res.success:
                torque=res.x[:3];self.status='OK';self.success_count+=1
            elif np.all(np.isfinite(res.x)) and np.linalg.norm(res.jac,np.inf)<1e-3:
                torque=res.x[:3];self.status='APROX';self.approx_count+=1
            else:
                torque=pd_tau;self.status='FALLBACK';self.failure_count+=1
            self.last_reason=str(res.message)
        except (ValueError,TypeError,np.linalg.LinAlgError,FloatingPointError) as exc:
            torque=pd_tau;self.status='FALLBACK';self.failure_count+=1;self.last_reason=str(exc)
        self.last=np.clip(torque,-limits,limits)
        return self.last.copy()
