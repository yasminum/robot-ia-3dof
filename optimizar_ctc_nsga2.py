"""Sintonización evolutiva multiobjetivo CTC + MuJoCo; NO altera archivos existentes.

Ejemplos:
 python optimizar_ctc_nsga2.py --modo rapido
 python optimizar_ctc_nsga2.py --modo tesis --semilla 10
 python optimizar_ctc_nsga2.py --solo-base

Requiere mujoco, pymoo, numpy. Se guarda cada resultado de simulación real.
"""
import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
from main import ROOT, INITIAL, DEFAULT_TARGET, LIMITS, reachable_target
from robot_ia.controllers import SmoothReference
from robot_ia.ctc_evolutivo import torque_ctc, GANANCIAS_REFERENCIA, LIMITES_GANANCIAS
from robot_ia.kinematics import fk

FOLDER = ROOT / "results" / "evolutivo_v040"


def ejecutar_ctc(theta, escenario='nominal', duration=9., incertidumbre=False):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'models' / 'brazo_3d.xml'))
    data = mujoco.MjData(model)
    # Modelo nominal independiente, sólo para cálculo de M y bias.
    nominal = mujoco.MjModel.from_xml_path(str(ROOT / 'models' / 'brazo_3d.xml'))
    dn = mujoco.MjData(nominal)
    if incertidumbre:
        # Cambio exclusivamente en planta: evita conocimiento exacto de incertidumbre.
        model.body_mass[model.body('eslabon_1').id] *= 1.25
        model.body_mass[model.body('eslabon_2').id] *= 1.15
        model.dof_damping[:3] *= 1.30
        mujoco.mj_setConst(model, data)
    data.qpos[:3] = INITIAL
    data.qvel[:] = 0.
    data.mocap_pos[0] = DEFAULT_TARGET
    mujoco.mj_forward(model, data)
    qr_final = reachable_target(DEFAULT_TARGET, np.asarray(model.jnt_range[:3]))
    reference = SmoothReference(INITIAL, qr_final, 0., 4.5)
    dt = float(model.opt.timestep)
    cada = max(1, round(0.01/dt))
    registro = max(1, round(0.02/dt))
    M = np.empty((nominal.nv, nominal.nv))
    tau = np.zeros(3)
    t_log=[]; e_track=[]; e_final=[]; esfuerzos=[]; sat=[]
    times=[]
    site = model.site('punta').id
    for k in range(int(duration/dt)):
        t=float(data.time)
        if k % cada == 0:
            dn.qpos[:3] = data.qpos[:3]
            dn.qvel[:3] = data.qvel[:3]
            mujoco.mj_forward(nominal,dn)
            mujoco.mj_fullM(nominal, dn, M)
            qr,dqr,ddqr=reference.evaluate(t)
            t0=time.perf_counter_ns()
            tau=torque_ctc(data.qpos[:3], data.qvel[:3],qr,dqr,ddqr,M[:3,:3],dn.qfrc_bias[:3],LIMITS,theta)
            times.append((time.perf_counter_ns()-t0)/1e6)
        perturb = np.array([.035,-.04,.025]) if escenario=='perturbacion' and 6 <= t < 6.6 else np.zeros(3)
        data.ctrl[:3] = tau
        data.qfrc_applied[:3] = perturb
        mujoco.mj_step(model,data)
        if k % registro == 0:
            xyz=data.site_xpos[site]
            qref,_,_=reference.evaluate(data.time)
            t_log.append(data.time)
            e_track.append(float(np.linalg.norm(xyz-fk(qref))*1000))
            e_final.append(float(np.linalg.norm(xyz-DEFAULT_TARGET)*1000))
            esfuerzos.append(float(np.dot(tau,tau)))
            sat.append(bool(np.any(np.abs(tau)>=LIMITS-1e-5)))
    t_log=np.asarray(t_log)
    steady=t_log>= max(4.5,duration-2.)
    return dict(rmse_seguimiento_mm=float(np.sqrt(np.mean(np.square(e_track)))),
                rmse_final_2s_mm=float(np.sqrt(np.mean(np.square(np.asarray(e_final)[steady])))),
                error_final_mm=e_final[-1],
                esfuerzo_torque2=float(np.trapezoid(esfuerzos,t_log)),
                saturacion_pct=100*float(np.mean(sat)),
                tiempo_ctc_medio_ms=float(np.mean(times)),
                tiempo_ctc_p95_ms=float(np.percentile(times,95)),
                error_max_seguimiento_mm=float(np.max(e_track)))


def evaluar_robusto(theta, duration, escenarios):
    results=[]
    for escenario,incertidumbre in escenarios:
        results.append(ejecutar_ctc(theta, escenario, duration, incertidumbre))
    # Dos objetivos de igual definición; se optimiza con valores físicos crudos.
    return [float(np.mean([r['rmse_seguimiento_mm'] for r in results])),
            float(np.mean([r['esfuerzo_torque2'] for r in results]))], results


def escribir_csv(ruta, registros):
    ruta.parent.mkdir(parents=True,exist_ok=True)
    if not registros: return
    keys=list(registros[0].keys())
    with ruta.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(registros)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--modo',choices=('rapido','tesis'),default='rapido')
    p.add_argument('--semilla',type=int,default=10)
    p.add_argument('--solo-base',action='store_true')
    p.add_argument('--duration',type=float,default=9.)
    args=p.parse_args()
    if args.duration <= 7.:p.error('La simulación debe durar más de 7 segundos')
    # Las perturbaciones se evalúan en todos los candidatos; no solo al finalizar.
    casos=[('nominal',False),('perturbacion',False),('nominal',True),('perturbacion',True)]
    FOLDER.mkdir(parents=True,exist_ok=True)
    base_obj,base_det=evaluar_robusto(GANANCIAS_REFERENCIA,args.duration,casos)
    records=[dict(tipo='BASE',Kp1=169.,Kp2=169.,Kp3=169.,Kd1=28.6,Kd2=28.6,Kd3=28.6,
                  rmse_mm=base_obj[0],esfuerzo_Nm2s=base_obj[1])]
    print('BASE CTC',base_obj,flush=True)
    escribir_csv(FOLDER/'comparacion_candidatos.csv',records)
    if args.solo_base:return
    try:
        from pymoo.algorithms.moo.nsga2 import NSGA2
        from pymoo.core.problem import ElementwiseProblem
        from pymoo.optimize import minimize
        from pymoo.termination import get_termination
    except ImportError as exc:
        raise SystemExit('Falta pymoo. Instala con: python -m pip install pymoo') from exc
    tam,gens=(8,4) if args.modo=='rapido' else (24,20)
    evaluations=[]
    class Problem(ElementwiseProblem):
        def __init__(self):super().__init__(n_var=6,n_obj=2,xl=LIMITES_GANANCIAS[:,0],xu=LIMITES_GANANCIAS[:,1])
        def _evaluate(self,x,out,*a,**kw):
            i=len(evaluations)+1
            try:
                values,_=evaluar_robusto(x,args.duration,casos)
                if not np.all(np.isfinite(values)):raise ValueError('objetivos no finitos')
            except (ValueError, FloatingPointError, RuntimeError) as ex:
                print('Candidato penalizado:',i,str(ex),flush=True)
                values=[1e8,1e8]
            evaluations.append(dict(numero=i,**{f'g{j+1}':float(v) for j,v in enumerate(x)},
                                    rmse_mm=values[0],esfuerzo_Nm2s=values[1]))
            if i%4==0:
                escribir_csv(FOLDER/'historial_evaluaciones.csv',evaluations)
                print(f'Evaluaciones: {i} | RMSE={values[0]:.3f} mm | esfuerzo={values[1]:.4f}',flush=True)
            out['F']=values
    start=time.perf_counter()
    result=minimize(Problem(),NSGA2(pop_size=tam,eliminate_duplicates=True),
                    get_termination('n_gen',gens),seed=args.semilla,verbose=True)
    elapsed=time.perf_counter()-start
    escribir_csv(FOLDER/'historial_evaluaciones.csv',evaluations)
    xx=np.atleast_2d(result.X);ff=np.atleast_2d(result.F)
    front=[]
    for x, f in zip(xx,ff):
        if f[0]>=1e8:continue
        front.append(dict(**{n:float(v) for n,v in zip(['Kp1','Kp2','Kp3','Kd1','Kd2','Kd3'],x)},
                          rmse_mm=float(f[0]),esfuerzo_Nm2s=float(f[1])))
    front.sort(key=lambda a:a['rmse_mm'])
    escribir_csv(FOLDER/'frente_pareto.csv',front)
    # Selección reproducible: prioriza candidatos del frente de Pareto que mejoran el RMSE base, y dentro de ellos menor distancia normalizada al ideal.
    if front:
        improving=[r for r in front if r['rmse_mm']<=base_obj[0]]
        pool=improving if improving else front
        A=np.array([[r['rmse_mm'],r['esfuerzo_Nm2s']] for r in pool]);lo=A.min(axis=0);span=np.maximum(A.max(axis=0)-lo,1e-12)
        idx=int(np.argmin(np.linalg.norm((A-lo)/span,axis=1)))
        chosen=pool[idx];theta=[chosen[k] for k in ('Kp1','Kp2','Kp3','Kd1','Kd2','Kd3')]
        records.append(dict(tipo='EVOLUTIVO',**{k:chosen[k] for k in ('Kp1','Kp2','Kp3','Kd1','Kd2','Kd3')},
                            rmse_mm=chosen['rmse_mm'],esfuerzo_Nm2s=chosen['esfuerzo_Nm2s']))
        escribir_csv(FOLDER/'comparacion_candidatos.csv',records)
        (FOLDER/'ganancias_seleccionadas.json').write_text(json.dumps(dict(ganancias=theta,criterio='distancia normalizada al ideal, igual peso a ambos objetivos',
            semilla=args.semilla,modo=args.modo,duracion=args.duration),indent=2),encoding='utf-8')
        print('PARETO:',len(front),'soluciones. Seleccionada:',theta,flush=True)
    print(f'Tiempo de optimización: {elapsed:.2f} segundos. Archivos: {FOLDER}',flush=True)

if __name__=='__main__': main()
