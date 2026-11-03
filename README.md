# ROBOT-IA 3DOF v0.5.0 — CTC + MPC + NSGA-II + CLIK-DLS + Modo Juego Encestar 5 Aros

**Asignatura:** Robótica y Sistemas Autónomos (Ciclo 2026-2 — Posgrado FIIS UNI)  
**Docente:** Ing. Wilson Dennis Reyes Vásquez  
**Modelo Físico:** Manipulador Antropomórfico RRR de 3 GDL basado en Conejo Benítez (2021) (`d1 = 0.079 m`, `L1 = 0.113 m`, `L2 = 0.140 m`).

---

## Novedades de la versión v0.5.0 (Trabajo Parcial Completo)

1. **Cinemática Directa, Inversa, Diferencial y Singularidades (`robot_ia/kinematics.py`)**:
   - Matrices de Transformación Homogénea Denavit-Hartenberg (`dh_table`, `dh_matrix`, `fk_homogeneous`).
   - Cinemática inversa analítica en ambas ramas (`elbow = +1` codo abajo, `elbow = -1` codo arriba).
   - Cinemática diferencial directa ($\dot{p} = J(q)\dot{q}$, rapidez instantánea $\|\dot{p}\|$ y velocidad pico $v_{\max}$ en mm/s) e inversa amortiguada Levenberg-Marquardt (`dls_pseudoinverse`, `inverse_differential_kinematics`).
   - Análisis automático de singularidades (`jacobian_det`, índice de manipulabilidad de Yoshikawa $w(q) = \sqrt{\det(J J^T)}$, número de condición $\kappa(J)$ y diagnóstico de singularidad de codo $\sin(q_3)=0$ y axial de hombro $r=0$).
2. **Cinco Controladores en Tiempo Real (`main.py`, `robot_ia/controllers.py`, `robot_ia/mpc.py`, `robot_ia/ctc_evolutivo.py`)**:
   - `CTC base`: Control por torque computado nominal ($K_p = 169$, $K_d = 28.6$).
   - `CTC NSGA-II`: Ganancias evolutivas Pareto óptimas cargadas automáticamente desde `results/evolutivo_v040/ganancias_seleccionadas.json` ($\theta^* = [239.88, 191.92, 215.83, 50.22, 29.67, 9.00]$).
   - `MPC`: Control predictivo local con restricciones de actuador $[-\tau_{\max}, +\tau_{\max}]$.
   - `Híbrido CTC-MPC`: Arquitectura coordinada donde el bloque CTC evolutivo fija la ley nominal y el MPC optimiza las correcciones predictivas bajo restricciones.
   - `Cinemático CLIK`: Control cinemático en lazo cerrado por pseudoinversa amortiguada $J^\dagger_{\text{DLS}}(q)$ acoplado a compensación dinámica.
3. **Máquina de Estados Visual 3D y Modo Juego "Circuito de 5 Aros (Encestar Bola)"**:
   - Esfera visual sin masa entre las mordazas de la pinza (`bola_efector`) y aro objetivo (`objetivo_aro_base`) que cambian de color en 3D según el estado del robot:
     - **Azul Cian**: `ESPERANDO ORDEN`
     - **Naranja Brillante**: `EN MOVIMIENTO` (en tránsito hacia el aro rojo)
     - **Magenta**: `PERTURBACIÓN ACTIVA ⚡`
     - **Verde Esmeralda**: `¡ENCESTADO / OBJETIVO ALCANZADO! 🎯` (cuando $\|p - p_{\text{obj}}\| < 2.0\text{ mm}$)
   - Selector de rapidez (`Estándar 4.5s`, `Rápido 2.5s`, `Turbo 1.5s`) y botón **`🏀 JUEGO: ENCESTAR 5 AROS`** con cronómetro de captura, contador de encestes y velocímetro pico $v_{\max}$ (mm/s).
4. **Simulación en MATLAB (`matlab/simulacion_3gdl_examen_parcial.m`)**:
   - Script autocontenido en MATLAB con DH, IK de 2 ramas, mapa 2D de manipulabilidad y singularidades, y simulación 3D del Circuito de 5 Aros.

---

## Ejecución Rápida (Windows PowerShell / CMD)

```powershell
cd "EXAMEN PARCIAL\ROBOT_IA_3DOF_v0\ROBOT_IA_3DOF_v0"

# 1. Ejecutar las 28 pruebas unitarias automatizadas
.\.venv\Scripts\python.exe -m pytest -q

# 2. Abrir el simulador interactivo 3D (Laboratorio + Modo Juego Encestar 5 Aros)
.\.venv\Scripts\python.exe main.py

# 3. Ejecutar el benchmark comparativo de los 5 controladores (nominal y con perturbación)
.\.venv\Scripts\python.exe evaluar_controladores.py --duration 10

# 4. Regenerar figuras 3D de estados, telemetría del circuito de 5 aros e Informe Word (.docx)
.\.venv\Scripts\python.exe -X utf8 generar_figuras_y_reporte.py
```
