"""Comparación reproducible de todos los controladores (PD/CTC base, CTC NSGA-II, MPC, Híbrido CTC-MPC y Cinemático CLIK)
sobre exactamente el mismo modelo MuJoCo.

Ejecutar: .venv\Scripts\python.exe evaluar_controladores.py --duration 12
No inventa resultados; guarda datos de simulaciones realmente ejecutadas.
"""
import argparse
import csv
from pathlib import Path
import numpy as np

from main import ROOT, INITIAL, DEFAULT_TARGET, LIMITS, reachable_target
from robot_ia.controllers import PID, SmoothReference, KinematicController
from robot_ia.mpc import MPC
from robot_ia.ctc_evolutivo import torque_ctc, GANANCIAS_REFERENCIA, cargar_ganancias
from robot_ia.kinematics import fk, cartesian_velocity, manipulability


def run_case(controller, scenario, duration, dt_control=0.01, dt_log=0.02):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'models' / 'brazo_3d.xml'))
    data = mujoco.MjData(model)
    data.qpos[:3] = INITIAL
    data.qvel[:] = 0
    data.mocap_pos[0] = DEFAULT_TARGET
    mujoco.mj_forward(model, data)
    joint_limits = np.asarray(model.jnt_range[:3])
    q_goal = reachable_target(DEFAULT_TARGET, joint_limits)
    reference = SmoothReference(INITIAL, q_goal, 0, 4.5)
    pd, mpc, kin = PID(), MPC(), KinematicController()
    evo_path = ROOT / 'results' / 'evolutivo_v040' / 'ganancias_seleccionadas.json'
    evo_theta = cargar_ganancias(evo_path) if evo_path.exists() else GANANCIAS_REFERENCIA
    dt = float(model.opt.timestep)
    every = max(1, round(dt_control / dt))
    log_every = max(1, round(dt_log / dt))
    applied = np.zeros(3)
    rows = []
    M = np.zeros((model.nv, model.nv))
    basis = np.zeros(model.nv)
    result = np.zeros(model.nv)
    target_site = model.site('punta').id
    solver_status = controller
    for k in range(int(duration / dt)):
        t = float(data.time)
        if k % every == 0:
            q, dq = data.qpos[:3].copy(), data.qvel[:3].copy()
            for j in range(model.nv):
                basis[:] = 0
                basis[j] = 1
                mujoco.mj_mulM(model, data, result, basis)
                M[:, j] = result
            bias = data.qfrc_bias[:3].copy()
            qr, dqr, ddqr = reference.evaluate(t)
            if controller == 'PD':
                applied = pd.step(q, dq, qr, dqr, ddqr, M[:3, :3], bias, LIMITS)
                solver_status = 'PD'
            elif controller == 'EVO':
                applied = torque_ctc(q, dq, qr, dqr, ddqr, M[:3, :3], bias, LIMITS, evo_theta)
                solver_status = 'CTC-NSGA2'
            elif controller == 'HYB':
                applied = mpc.command(q, dq, qr, M[:3, :3], bias, LIMITS, dqr, ddqr, theta_ctc=evo_theta)
                solver_status = f'HYB-{mpc.status}'
            elif controller == 'KIN':
                applied = kin.step(q, dq, qr, dqr, ddqr, M[:3, :3], bias, LIMITS)
                solver_status = 'CLIK-DLS'
            else:
                applied = mpc.command(q, dq, qr, M[:3, :3], bias, LIMITS, dqr, ddqr)
                solver_status = mpc.status
        perturb = np.array([.035, -.04, .025]) if scenario == 'perturbacion' and 6 <= t < 6.6 else np.zeros(3)
        data.ctrl[:3] = applied
        data.qfrc_applied[:3] = perturb
        mujoco.mj_step(model, data)
        if k % log_every == 0:
            xyz = data.site_xpos[target_site].copy()
            q = data.qpos[:3].copy()
            dq = data.qvel[:3].copy()
            qr, _, _ = reference.evaluate(data.time)
            xyz_ref = np.asarray(fk(qr), dtype=float)
            err_final = np.linalg.norm(xyz - DEFAULT_TARGET) * 1000
            err_track = np.linalg.norm(xyz - xyz_ref) * 1000
            v_mms = float(np.linalg.norm(cartesian_velocity(q, dq)) * 1000)
            w_manip = float(manipulability(q))
            rows.append([controller, scenario, float(data.time), *q, *xyz, *xyz_ref,
                         float(err_final), float(err_track), *applied,
                         float(np.linalg.norm(perturb)), solver_status, v_mms, w_manip])
    return rows


def summarize(rows, duration):
    # Columnas 12 y 13: errores final y trayectoria; 14 a 16: torques; 19: v_mms; 20: w_manip.
    a = np.asarray([r[2] for r in rows], dtype=float)
    ef = np.asarray([r[12] for r in rows], dtype=float)
    et = np.asarray([r[13] for r in rows], dtype=float)
    torques = np.asarray([r[14:17] for r in rows], dtype=float)
    speeds = np.asarray([r[19] for r in rows], dtype=float)
    manips = np.asarray([r[20] for r in rows], dtype=float)
    steady = a >= max(4.5, duration - 2.)
    if not np.any(steady): steady = np.ones_like(a, dtype=bool)
    # Tiempo de captura (< 2 mm)
    capture_idx = np.where((a >= 3.0) & (ef < 2.0))[0]
    t_capture = float(a[capture_idx[0]]) if len(capture_idx) > 0 else float('nan')
    out = dict(controlador=rows[0][0], escenario=rows[0][1],
               rmse_seguimiento_mm=float(np.sqrt(np.mean(et**2))),
               rmse_final_ultimos_2s_mm=float(np.sqrt(np.mean(ef[steady]**2))),
               error_final_mm=float(ef[-1]),
               max_error_final_mm=float(ef.max()),
               esfuerzo_integral_torque2_Nm2s=float(np.trapezoid(np.sum(torques**2, axis=1), a)),
               velocidad_pico_mms=float(speeds.max()),
               tiempo_captura_2mm_s=t_capture,
               manipulabilidad_min=float(manips.min()),
               saturacion_pct=float(100*np.mean(np.any(np.abs(torques) >= LIMITS-1e-5, axis=1))),
               mpc_fallbacks=sum('FALLBACK' in str(r[18]) for r in rows))
    if rows[0][1] == 'perturbacion':
        # Tiempo tras finalizar la fuerza para permanecer <=5mm durante 0.5 segundos.
        end = 6.6
        recovery = None
        for i, ti in enumerate(a):
            if ti < end: continue
            j = np.searchsorted(a, ti + 0.5)
            if j < len(a) and np.all(ef[i:j+1] <= 5.):
                recovery = float(ti-end)
                break
        out['recuperacion_5mm_s'] = recovery if recovery is not None else 'no alcanzada'
    else:
        out['recuperacion_5mm_s'] = 'no aplica'
    return out


def save_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerows(rows)


def main():
    p = argparse.ArgumentParser(description='Experimentos integrales controlando el mismo brazo 3GDL')
    p.add_argument('--duration', type=float, default=12.)
    p.add_argument('--controllers', nargs='+', choices=['PD', 'EVO', 'MPC', 'HYB', 'KIN'], default=['PD', 'EVO', 'MPC', 'HYB', 'KIN'])
    p.add_argument('--scenarios', nargs='+', choices=['nominal', 'perturbacion'], default=['nominal', 'perturbacion'])
    args = p.parse_args()
    if args.duration <= 7.: p.error('Se requiere --duration superior a 7 segundos para evaluar la perturbación')
    folder = ROOT / 'results' / 'validacion_v027'
    summaries = []
    for controller in args.controllers:
        for scenario in args.scenarios:
            print(f'Ejecutando: {controller} / {scenario}', flush=True)
            rows = run_case(controller, scenario, args.duration)
            save_csv(folder / f'muestras_{controller}_{scenario}.csv',
                     [['controlador','escenario','tiempo_s','q1','q2','q3','x','y','z','xref','yref','zref',
                       'error_objetivo_mm','error_seguimiento_mm','tau1','tau2','tau3','perturbacion_Nm','solver','velocidad_mms','manipulabilidad_w'], *rows])
            result = summarize(rows, args.duration)
            summaries.append(result)
            print(f"  error final={result['error_final_mm']:.3f} mm, RMSE seg={result['rmse_seguimiento_mm']:.3f} mm, RMSE final={result['rmse_final_ultimos_2s_mm']:.3f} mm, v_max={result['velocidad_pico_mms']:.1f} mm/s")
    cols = list(summaries[0])
    save_csv(folder / 'comparacion_resumen.csv', [cols, *[[r[c] for c in cols] for r in summaries]])
    print('Resultados guardados en:', folder)

if __name__ == '__main__':
    main()
