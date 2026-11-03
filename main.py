"""ROBOT-IA 3DOF v0.5.0: Simulador MuJoCo embebido en ventana Tkinter.
Incluye:
- Cinemática Directa (DH y geométrica), Inversa (2 ramas), Diferencial (p_dot = J*q_dot, ||v|| y v_max)
- Análisis de Singularidades en vivo (det(J), manipulabilidad de Yoshikawa w(q), número de condición k(J))
- 5 Controladores: CTC base, CTC NSGA-II, MPC, Híbrido CTC-MPC, Cinemático CLIK (J^+_DLS)
- Máquina de Estados Visual 3D (cambio de color de efector/aro al alcanzar objetivo o encestar)
- Modo Lente de Precisión y Modo Juego "Circuito Encestar 5 Aros" con cronómetro y velocímetro pico.
"""
import argparse
import csv
import time
from pathlib import Path
import numpy as np
from robot_ia.kinematics import (
    ik, fk, jacobian, cartesian_velocity, singularity_analysis
)
from robot_ia.controllers import PID, SmoothReference, KinematicController
from robot_ia.mpc import MPC
from robot_ia.ctc_evolutivo import torque_ctc, GANANCIAS_REFERENCIA, cargar_ganancias

ROOT=Path(__file__).resolve().parent
LIMITS=np.array([.7,.7,.45])
# Base más separada del objetivo. Posición inicial deliberadamente visible.
INITIAL=np.array([-1.10,.85,-.75])
DEFAULT_TARGET=np.array([.165,.075,.165])

def reachable_target(xyz,joint_range):
    point=np.asarray(xyz,float)
    for elbow in (1,-1):
        try:q=ik(point,elbow=elbow)
        except ValueError:continue
        if np.all(q>=joint_range[:,0]) and np.all(q<=joint_range[:,1]):return q
    raise ValueError('Objetivo fuera del alcance o de los límites articulares')

class Laboratory:
    def __init__(self,controller='PID'):
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image,ImageTk
        import mujoco
        self.tk=tk;self.Image=Image;self.ImageTk=ImageTk;self.mujoco=mujoco
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/'models'/'brazo_3d.xml'))
        self.data=mujoco.MjData(self.model)
        self.data.qpos[:3]=INITIAL
        self.target=DEFAULT_TARGET.copy()
        self.data.mocap_pos[0]=self.target
        mujoco.mj_forward(self.model,self.data)
        self.joint_range=np.asarray(self.model.jnt_range[:3]).copy()
        self.move_duration=4.5
        self.reference=SmoothReference(INITIAL,INITIAL,0,self.move_duration)
        self.pid=PID();self.mpc=MPC();self.kin_ctrl=KinematicController();self.u=np.zeros(3)
        self.evo_theta=None; self.evo_source="Sin cargar"; self.load_evo()
        self.comparison=[]; self.cycle_ms=0.0; self.torque_energy=0.0; self.prev_energy_t=0.0
        self.state='ESPERANDO ORDEN';self.paused=False;self.perturb_end=0.
        self.records=[];self.step_number=0;self.last_status='—';self.closing=False
        self.last_mode=controller
        self.goal_index=0
        self.goal_candidates=[
            np.array([.15,-.08,.11]),
            np.array([.105,.14,.205]),
            np.array([.18,.06,.095]),
            np.array([-.12,.11,.15]),
            DEFAULT_TARGET.copy()
        ]
        # Métricas de velocidad, estado y Modo Juego (Encestar / Lente de Precisión)
        self.speed_mms=0.0
        self.peak_speed_mms=0.0
        self.move_start_t=0.0
        self.last_capture_time=0.0
        self.score_count=0
        self.game_active=False
        self.game_target_idx=0
        self.game_hold_until=0.0
        self.game_start_t=0.0

        self.camera=mujoco.MjvCamera()
        self.camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:]=[.065,0,.100]
        self.camera.distance=.53;self.camera.azimuth=118;self.camera.elevation=-12
        self.render_width=1050;self.render_height=580
        self.renderer=mujoco.Renderer(self.model,height=self.render_height,width=self.render_width)
        self.win=tk.Tk();self.win.title('ROBOT-IA 3DOF · CTC / MPC / NSGA-II / CLIK · v0.5.0 · Laboratorio + Juego Encestar')
        mode_map={'PID':'CTC base','CTC':'CTC base','EVO':'CTC NSGA-II','HYB':'Híbrido CTC-MPC','KIN':'Cinemático CLIK'}
        self.mode=tk.StringVar(master=self.win,value=mode_map.get(controller,controller))
        self.speed_preset=tk.StringVar(master=self.win,value='Estándar (4.5s)')
        self.win.geometry('1480x880+15+15');self.win.minsize(1120,730)
        self.win.configure(bg='#e3e9ef')
        s=ttk.Style(self.win);s.theme_use('clam')
        s.configure('TFrame',background='#e3e9ef');s.configure('TLabel',background='#e3e9ef',foreground='#23384b',font=('Segoe UI',10))
        s.configure('TButton',font=('Segoe UI',9,'bold'),padding=5)
        s.configure('TCombobox',font=('Segoe UI',10))
        top=ttk.Frame(self.win,padding=(14,8));top.pack(fill='x')
        ttk.Label(top,text='ROBOT-IA   /   LABORATORIO DE CONTROL 3DOF & MODO ENCESTAR',font=('Segoe UI',15,'bold')).pack(side='left')
        self.badge_label=tk.Label(top,text=' ESTADO: ESPERANDO ORDEN ',font=('Segoe UI',10,'bold'),bg='#1e5b94',fg='#ffffff',padx=10,pady=3)
        self.badge_label.pack(side='right',padx=(10,0))
        ttk.Label(top,text='MuJoCo · Digital Twin · Cinemática, Singularidades y Control',font=('Segoe UI',10)).pack(side='right')
        stage=ttk.Frame(self.win,padding=(14,0,14,8));stage.pack(fill='both',expand=True)
        # Panel matemático visible: ecuaciones teóricas y valores calculados sobre el estado físico.
        math_panel=tk.Frame(stage,bg='#f3f6fa',width=355,highlightbackground='#becbd5',highlightthickness=1)
        math_panel.pack(side='left',fill='y',padx=(0,10))
        math_panel.pack_propagate(False)
        tk.Label(math_panel,text='MODELO MATEMÁTICO · EN VIVO',font=('Segoe UI',11,'bold'),bg='#f3f6fa',fg='#17334b').pack(anchor='w',padx=12,pady=(10,4))
        eq=tk.Label(math_panel,text='p(q) = [r cos(q1), r sin(q1), z]ᵀ\nr = L1 cos(q2) + L2 cos(q2+q3)\nz = h + L1 sin(q2) + L2 sin(q2+q3)\nṗ = J(q)q̇  |  det(J) = -L1·L2·sin(q3)·r\nM(q)q̈ + h(q,q̇) = τ + d',font=('Consolas',8),justify='left',bg='#f3f6fa',fg='#36556c')
        eq.pack(anchor='w',padx=12,pady=(0,4))
        self.math_text=tk.Text(math_panel,width=43,height=30,font=('Consolas',8),bg='#ffffff',fg='#153549',relief='flat',borderwidth=0,wrap='none',padx=8,pady=6,takefocus=0)
        self.math_text.pack(fill='both',expand=True,padx=10,pady=(0,10))
        self.math_text.configure(state='disabled')
        self.view=tk.Label(stage,bg='#d3dee7',bd=0,highlightthickness=0)
        self.view.pack(side='left',fill='both',expand=True)
        controls=ttk.Frame(self.win,padding=(14,3));controls.pack(fill='x')
        ttk.Label(controls,text='Controlador').grid(row=0,column=0,padx=(0,6),sticky='w')
        ttk.Combobox(controls,state='readonly',values=['CTC base','CTC NSGA-II','MPC','Híbrido CTC-MPC','Cinemático CLIK'],textvariable=self.mode,width=16).grid(row=0,column=1,padx=(0,12))
        self.entries=[]
        for i,axis in enumerate('XYZ'):
            ttk.Label(controls,text=axis+' (m)').grid(row=0,column=2+i*2,padx=2)
            v=tk.StringVar(value=f'{self.target[i]:.3f}')
            ttk.Entry(controls,textvariable=v,width=8).grid(row=0,column=3+i*2,padx=(0,8))
            self.entries.append(v)
        ttk.Button(controls,text='MOVER AL OBJETIVO',command=self.move).grid(row=0,column=8,padx=3)
        ttk.Button(controls,text='NUEVO OBJETIVO ↗',command=self.new_goal).grid(row=0,column=9,padx=3)
        ttk.Button(controls,text='REINICIAR',command=self.reset).grid(row=0,column=10,padx=3)
        ttk.Button(controls,text='PERTURBAR ⚡',command=self.disturb).grid(row=0,column=11,padx=3)
        self.pause_btn=ttk.Button(controls,text='PAUSAR',command=self.pause)
        self.pause_btn.grid(row=0,column=12,padx=3)

        # Segunda fila de controles: Velocidad de trayectoria, Modo Juego (Encestar 5 Aros) y Recarga NSGA-II
        ttk.Label(controls,text='Rapidez:').grid(row=1,column=0,padx=(0,6),pady=4,sticky='w')
        speed_cb=ttk.Combobox(controls,state='readonly',values=['Estándar (4.5s)','Rápido (2.5s)','Turbo (1.5s)'],textvariable=self.speed_preset,width=16)
        speed_cb.grid(row=1,column=1,padx=(0,12),pady=4)
        speed_cb.bind('<<ComboboxSelected>>',self.on_speed_change)
        self.game_btn=ttk.Button(controls,text='🏀 JUEGO: ENCESTAR 5 AROS',command=self.start_game)
        self.game_btn.grid(row=1,column=8,columnspan=2,padx=3,pady=4,sticky='ew')
        ttk.Button(controls,text='RECARGAR NSGA-II',command=self.reload_evo).grid(row=1,column=10,columnspan=2,padx=3,pady=4,sticky='ew')

        self.status=tk.StringVar(value='Esperando orden de movimiento')
        self.metrics=tk.StringVar(value='')
        self.game_metrics=tk.StringVar(value='')
        ttk.Label(self.win,textvariable=self.status,font=('Segoe UI',10,'bold')).pack(anchor='w',padx=17,pady=(4,0))
        ttk.Label(self.win,textvariable=self.metrics,font=('Consolas',9)).pack(anchor='w',padx=17,pady=(0,2))
        ttk.Label(self.win,textvariable=self.game_metrics,font=('Consolas',9,'bold'),foreground='#0d5c3a').pack(anchor='w',padx=17,pady=(0,2))
        self.evo_status=tk.StringVar(value='')
        ttk.Label(self.win,textvariable=self.evo_status,font=('Consolas',9)).pack(anchor='w',padx=17,pady=(0,2))
        ttk.Label(self.win,text='Cambio de estado 3D: Azul=Espera · Naranja=En tránsito · Magenta=Perturbado · Verde Esmeralda=¡Objetivo Encestado/Enfocado!',font=('Segoe UI',9)).pack(anchor='w',padx=17,pady=(0,6))
        self.update_evo_status()
        self.update_visual_status()
        self.win.protocol('WM_DELETE_WINDOW',self.close)
        self.last_wall=time.perf_counter()
        self.update_math()
        self.render_frame()
        self.win.after(33,self.tick)

    def _get_duration(self):
        val=self.speed_preset.get()
        if '1.5' in val:return 1.5
        if '2.5' in val:return 2.5
        return 4.5

    def on_speed_change(self,_event=None):
        self.move_duration=self._get_duration()
        self.status.set(f'Duración de trayectoria ajustada a {self.move_duration:.1f} s')

    def load_evo(self):
        path=ROOT/'results'/'evolutivo_v040'/'ganancias_seleccionadas.json'
        try:
            self.evo_theta=cargar_ganancias(path)
            self.evo_source=str(path)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.evo_theta=None
            self.evo_source='No disponible: '+str(exc)

    def update_evo_status(self):
        if self.evo_theta is None:
            self.evo_status.set('NSGA-II sin importar · Copia results/evolutivo_v040/ganancias_seleccionadas.json; selector usa CTC base como respaldo')
        else:
            self.evo_status.set('NSGA-II activo · Kp='+np.array2string(self.evo_theta[:3],precision=2)+' · Kd='+np.array2string(self.evo_theta[3:],precision=2))

    def reload_evo(self):
        self.load_evo();self.update_evo_status()
        self.status.set('Ganancias NSGA-II recargadas' if self.evo_theta is not None else 'No se encontró JSON válido; CTC base sigue disponible')

    def update_visual_status(self):
        """Actualiza dinámicamente los colores 3D en MuJoCo y el indicador visual según el estado del robot."""
        try:
            id_centro = self.model.geom('objetivo_centro').id
            ring_names = ('objetivo_norte','objetivo_sur','objetivo_este','objetivo_oeste','objetivo_frente','objetivo_atras')
            ring_ids = [self.model.geom(n).id for n in ring_names]
            id_bola = self.model.geom('bola_efector').id
            id_aro = self.model.geom('objetivo_aro_base').id
        except Exception:
            return

        if self.data.time < self.perturb_end:
            # Estado Perturbado: Magenta alerta
            self.model.geom_rgba[id_bola] = [0.95, 0.12, 0.60, 0.90]
            self.model.geom_rgba[id_centro] = [0.98, 0.13, 0.19, 0.32]
            self.badge_label.configure(text=' ESTADO: PERTURBACIÓN ACTIVA ⚡ ', bg='#b8145a')
        elif 'ALCANZADO' in self.state or 'ENCESTADO' in self.state or 'COMPLETADO' in self.state:
            # Estado Objetivo Alcanzado / Encestado: Verde Esmeralda brillante en bola y aro
            self.model.geom_rgba[id_bola] = [0.03, 0.97, 0.35, 0.95]
            self.model.geom_rgba[id_centro] = [0.04, 0.92, 0.38, 0.48]
            self.model.geom_rgba[id_aro] = [0.04, 0.92, 0.38, 0.85]
            for rid in ring_ids:
                self.model.geom_rgba[rid] = [0.04, 0.95, 0.40, 1.0]
            self.badge_label.configure(text=f' ESTADO: {self.state} 🎯 ', bg='#0d8a47')
        elif self.state == 'EN MOVIMIENTO':
            # En tránsito: Bola naranja energética, aro rojo objetivo
            self.model.geom_rgba[id_bola] = [1.00, 0.56, 0.04, 0.88]
            self.model.geom_rgba[id_centro] = [0.98, 0.13, 0.19, 0.25]
            self.model.geom_rgba[id_aro] = [0.98, 0.35, 0.10, 0.55]
            for rid in ring_ids:
                self.model.geom_rgba[rid] = [1.0, 0.12, 0.15, 1.0]
            self.badge_label.configure(text=' ESTADO: EN MOVIMIENTO ↗ ', bg='#c96a08')
        else:
            # Esperando orden: Azul cian
            self.model.geom_rgba[id_bola] = [0.10, 0.65, 0.98, 0.78]
            self.model.geom_rgba[id_centro] = [0.98, 0.13, 0.19, 0.25]
            self.model.geom_rgba[id_aro] = [0.98, 0.35, 0.10, 0.55]
            for rid in ring_ids:
                self.model.geom_rgba[rid] = [1.0, 0.12, 0.15, 1.0]
            self.badge_label.configure(text=f' ESTADO: {self.state} ', bg='#1e5b94')

    def move(self):
        self.move_duration=self._get_duration()
        try:
            xyz=np.array([float(x.get().replace(',','.')) for x in self.entries])
            desired=reachable_target(xyz,self.joint_range)
        except Exception as exc:
            self.status.set('Objetivo rechazado: '+str(exc));return
        self.target=xyz;self.data.mocap_pos[0]=xyz
        self.move_start_t=float(self.data.time)
        self.reference.retarget(self.data.qpos[:3].copy(),desired,self.data.time,self.move_duration)
        self.state='EN MOVIMIENTO'
        self.update_visual_status()
        self.status.set(f'Trayectoria en ejecución ({self.move_duration:.1f} s simulados)')

    def new_goal(self):
        self.game_active=False
        # Posiciones deliberadamente distintas; se validan con ambas ramas de IK.
        for _ in range(len(self.goal_candidates)):
            candidate=self.goal_candidates[self.goal_index % len(self.goal_candidates)].copy()
            self.goal_index+=1
            try:
                reachable_target(candidate,self.joint_range)
            except ValueError:
                continue
            for box,value in zip(self.entries,candidate):box.set(f'{value:.3f}')
            self.move()
            self.status.set('Nuevo objetivo XYZ generado: el robot inicia el seguimiento')
            return
        self.status.set('Sin posiciones válidas en la colección de objetivos')

    def start_game(self):
        """Inicia el Modo Juego: Circuito de 5 Aros donde el brazo debe encestar la esfera en los 5 objetivos."""
        self.game_active=True
        self.game_target_idx=0
        self.score_count=0
        self.peak_speed_mms=0.0
        self.game_start_t=float(self.data.time)
        self.game_hold_until=0.0
        candidate=self.goal_candidates[0].copy()
        for box,value in zip(self.entries,candidate):box.set(f'{value:.3f}')
        self.move()
        self.status.set('🏀 MODO JUEGO INICIADO: Circuito de 5 Aros — ¡Encestando aro 1/5!')

    def update_math(self):
        q=self.data.qpos[:3].copy();dq=self.data.qvel[:3].copy()
        p=fk(q);J=jacobian(q);M=self.mass_matrix();h=self.data.qfrc_bias[:3].copy()
        v_cart=cartesian_velocity(q,dq)
        sing=singularity_analysis(q)
        def vec(v):return ' '.join(f'{a: .4f}' for a in v)
        def mat(v):return '\n'.join('  '+vec(row) for row in v)
        lines=(f'ARTICULACIONES (rad)\nq    = [{vec(q)}]\n'
               f'qdot = [{vec(dq)}]\n'
               f'Ángulos (grados)\n[{vec(np.degrees(q))}]\n\n'
               f'CINEMÁTICA DIRECTA (m)\np = [{vec(p)}]\n'
               f'p objetivo = [{vec(self.target)}]\n'
               f'Error XYZ = {np.linalg.norm(p-self.target)*1000:.3f} mm\n\n'
               f'CINEMÁTICA DIFERENCIAL (ṗ = J q̇)\n'
               f'v_xyz = [{vec(v_cart)}] m/s\n'
               f'Rapidez ||v|| = {np.linalg.norm(v_cart)*1000:.2f} mm/s\n'
               f'Pico v_max    = {self.peak_speed_mms:.2f} mm/s\n\n'
               f'SINGULARIDADES Y MANIPULABILIDAD\n'
               f'det(J) = {sing["det_J"]: .6f}\n'
               f'w(q)   = {sing["manipulability"]: .6f}\n'
               f'cond(J)= {sing["cond_J"]: .2f}\n'
               f'Diag   = {sing["tipo"]}\n\n'
               f'JACOBIANO J(q) · m/rad\n{mat(J)}\n\n'
               f'MATRIZ DE INERCIA M(q)\n{mat(M)}\n\n'
               f'h(q,qdot) = Cqdot + G\n[{vec(h)}] Nm\n\n'
               f'TORQUES τ (Nm)\n[{vec(self.u)}]')
        self.math_text.configure(state='normal')
        self.math_text.delete('1.0','end');self.math_text.insert('1.0',lines)
        self.math_text.configure(state='disabled')

    def reset(self):
        m=self.mujoco
        m.mj_resetData(self.model,self.data)
        self.data.qpos[:3]=INITIAL;self.data.mocap_pos[0]=self.target
        m.mj_forward(self.model,self.data)
        self.pid=PID();self.mpc=MPC();self.kin_ctrl=KinematicController();self.u[:]=0
        self.torque_energy=0.0;self.prev_energy_t=0.0;self.comparison=[]
        self.speed_mms=0.0;self.peak_speed_mms=0.0;self.last_capture_time=0.0
        self.game_active=False;self.game_hold_until=0.0
        self.move_duration=self._get_duration()
        self.reference=SmoothReference(INITIAL,INITIAL,self.data.time,self.move_duration)
        self.perturb_end=0.;self.step_number=0
        self.state='ESPERANDO ORDEN';self.paused=False
        self.update_visual_status()
        self.pause_btn.configure(text='PAUSAR');self.status.set('Reiniciado; brazo esperando una nueva orden')
    def disturb(self):
        self.perturb_end=self.data.time+.6
        self.update_visual_status()
        self.status.set('Perturbación externa aplicada durante 0,6 s simulados')
    def pause(self):
        self.paused=not self.paused
        self.pause_btn.configure(text='REANUDAR' if self.paused else 'PAUSAR')
    def mass_matrix(self):
        m=self.mujoco;nv=self.model.nv
        out=np.zeros((nv,nv));e=np.zeros(nv);col=np.zeros(nv)
        for j in range(nv):
            e[:]=0;e[j]=1
            m.mj_mulM(self.model,self.data,col,e)
            out[:,j]=col
        return out[:3,:3]
    def physics_step(self):
        m=self.mujoco;d=self.data
        mode=self.mode.get()
        if mode!=self.last_mode:
            self.pid=PID();self.mpc=MPC();self.kin_ctrl=KinematicController();self.last_mode=mode
        if self.step_number%5==0:
            q=d.qpos[:3].copy();dq=d.qvel[:3].copy()
            M=self.mass_matrix();bias=d.qfrc_bias[:3].copy()
            qr,dqr,ddqr=self.reference.evaluate(d.time)
            t_start=time.perf_counter()
            if mode=='CTC base':
                self.u=torque_ctc(q,dq,qr,dqr,ddqr,M,bias,LIMITS,GANANCIAS_REFERENCIA)
                self.last_status='CTC original'
            elif mode=='CTC NSGA-II':
                if self.evo_theta is not None:
                    self.u=torque_ctc(q,dq,qr,dqr,ddqr,M,bias,LIMITS,self.evo_theta)
                    self.last_status='CTC evolutivo'
                else:
                    self.u=torque_ctc(q,dq,qr,dqr,ddqr,M,bias,LIMITS,GANANCIAS_REFERENCIA)
                    self.last_status='SIN GANANCIAS: CTC BASE'
                    self.status.set('Falta ganancias_seleccionadas.json; utilizando CTC base')
            elif mode=='Híbrido CTC-MPC':
                th=self.evo_theta if self.evo_theta is not None else GANANCIAS_REFERENCIA
                self.u=self.mpc.command(q,dq,qr,M,bias,LIMITS,dqr,ddqr,theta_ctc=th)
                self.last_status=f'HYB-{self.mpc.status}'
            elif mode=='Cinemático CLIK':
                self.u=self.kin_ctrl.step(q,dq,qr,dqr,ddqr,M,bias,LIMITS)
                self.last_status='CLIK-DLS'
            else:
                self.u=self.mpc.command(q,dq,qr,M,bias,LIMITS,dqr,ddqr)
                self.last_status=str(self.mpc.status)
            self.cycle_ms=(time.perf_counter()-t_start)*1000
        d.ctrl[:3]=self.u
        d.qfrc_applied[:3]=[.035,-.04,.025] if d.time<self.perturb_end else [0,0,0]
        d.mocap_pos[0]=self.target
        m.mj_step(self.model,d)
        self.torque_energy += float(np.dot(self.u,self.u))*float(self.model.opt.timestep)
        self.step_number+=1
        if self.step_number%20==0:
            q_now=d.qpos[:3].copy();dq_now=d.qvel[:3].copy()
            v_now=cartesian_velocity(q_now,dq_now)
            self.speed_mms=float(np.linalg.norm(v_now)*1000.0)
            if self.speed_mms>self.peak_speed_mms:
                self.peak_speed_mms=self.speed_mms
            xyz=d.site_xpos[self.model.site('punta').id].copy()
            err=float(np.linalg.norm(xyz-self.target)*1000)
            self.records.append([d.time,mode,*q_now,*xyz,*self.target,err,*self.u, float(d.time<self.perturb_end),self.last_status,self.state])
            if self.state=='EN MOVIMIENTO' and err<2.0 and (d.time>=self.reference.t0+0.65*self.reference.duration):
                self.last_capture_time=float(d.time-self.move_start_t)
                self.score_count+=1
                if self.game_active:
                    self.state=f'¡ENCESTADO {self.game_target_idx+1}/5!'
                    self.game_hold_until=float(d.time)+0.45
                    self.status.set(f'🏀 ¡Aro {self.game_target_idx+1}/5 encestado en {self.last_capture_time:.2f} s! (Error={err:.3f} mm, v_max={self.peak_speed_mms:.1f} mm/s)')
                else:
                    self.state='OBJETIVO ALCANZADO'
                    self.status.set(f'🎯 Objetivo alcanzado y bloqueado en {self.last_capture_time:.2f} s (Error={err:.3f} mm)')
                self.update_visual_status()
            elif self.game_active and self.game_hold_until>0.0 and d.time>=self.game_hold_until:
                self.game_hold_until=0.0
                self.game_target_idx+=1
                if self.game_target_idx<len(self.goal_candidates):
                    nxt=self.goal_candidates[self.game_target_idx].copy()
                    for box,val in zip(self.entries,nxt):box.set(f'{val:.3f}')
                    self.move()
                    self.status.set(f'🏀 Modo Juego: Buscando aro {self.game_target_idx+1}/5...')
                else:
                    total_game_s=float(d.time-self.game_start_t)
                    self.game_active=False
                    self.state='CIRCUITO 5/5 COMPLETADO'
                    self.update_visual_status()
                    self.status.set(f'🏆 ¡CIRCUITO DE 5 AROS COMPLETADO en {total_game_s:.2f} s! Velocidad pico: {self.peak_speed_mms:.1f} mm/s')
            else:
                self.update_visual_status()

            self.metrics.set(f'Tiempo {d.time:6.2f}s | {self.state:22} | Error {err:7.3f} mm | ||v|| {self.speed_mms:6.1f} mm/s | Cálculo {self.cycle_ms:.3f} ms | ∫τ²dt {self.torque_energy:.4f}')
            self.game_metrics.set(f'🏀 Marcador Encestes/Objetivos: {self.score_count}  |  Último tiempo de captura: {self.last_capture_time:.2f} s  |  Velocidad Pico v_max: {self.peak_speed_mms:.1f} mm/s')
            self.comparison.append([d.time,mode,err,float(np.linalg.norm(self.u)),self.torque_energy,self.cycle_ms,*self.u,self.last_status])
    def render_frame(self):
        try:
            self.renderer.update_scene(self.data,camera=self.camera)
            frame=self.renderer.render()
            # PhotoImage conserva la imagen a través de una referencia de instancia
            image=self.Image.fromarray(frame)
            self.photo=self.ImageTk.PhotoImage(image)
            self.view.configure(image=self.photo)
        except Exception as exc:
            self.status.set('Error de renderizado: '+str(exc))
            raise
    def tick(self):
        if self.closing:return
        if not self.paused:
            # 16 pasos de 2 ms = 32 ms de simulación por cuadro.
            for _ in range(16):self.physics_step()
        try:
            self.render_frame()
            self.update_math()
        except Exception as exc:
            self.status.set('Error: '+str(exc))
            return
        self.win.after(33,self.tick)
    def save(self):
        if not self.records:return
        out=ROOT/'results';out.mkdir(exist_ok=True)
        name=out/'estudio_3d_v025.csv'
        head=['tiempo_s','modo','q1','q2','q3','x_m','y_m','z_m','objetivo_x','objetivo_y','objetivo_z','error_mm','tau1','tau2','tau3','perturbacion_activa','solver','estado']
        with name.open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.writer(f);w.writerow(head);w.writerows(self.records)
        print('Registro guardado en',name)
        if self.comparison:
            comparison=ROOT/'results'/'comparacion_visual_v041.csv'
            with comparison.open('w',newline='',encoding='utf-8-sig') as f:
                w=csv.writer(f); w.writerow(['tiempo_s','controlador','error_xyz_mm','torque_norm_Nm','integral_torque2','calculo_control_ms','tau1_Nm','tau2_Nm','tau3_Nm','estado_control']);w.writerows(self.comparison)
            print('Comparación visual guardada en',comparison)
    def close(self):
        self.closing=True
        self.save()
        try:self.renderer.close()
        finally:self.win.destroy()
    def run(self):self.win.mainloop()

def headless(control='PID',duration=15.,disturbance=False):
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(ROOT/'models'/'brazo_3d.xml'))
    data=mujoco.MjData(model)
    data.qpos[:3]=INITIAL;mujoco.mj_forward(model,data)
    goal=reachable_target(DEFAULT_TARGET,np.asarray(model.jnt_range[:3]))
    ref=SmoothReference(INITIAL,goal,0,4.5)
    pid=PID();mpc=MPC();kin_ctrl=KinematicController();u=np.zeros(3);records=[]
    evo_path=ROOT/'results'/'evolutivo_v040'/'ganancias_seleccionadas.json'
    evo_th=cargar_ganancias(evo_path) if evo_path.exists() else GANANCIAS_REFERENCIA
    steps=int(duration/model.opt.timestep)
    for i in range(steps):
        if i%5==0:
            q=data.qpos[:3].copy();dq=data.qvel[:3].copy()
            M=np.zeros((model.nv,model.nv));b=np.zeros(model.nv);c=np.zeros(model.nv)
            for j in range(model.nv):
                b[:]=0;b[j]=1;mujoco.mj_mulM(model,data,c,b);M[:,j]=c
            r,rd,rdd=ref.evaluate(data.time)
            if control=='PID':u=pid.step(q,dq,r,rd,rdd,M[:3,:3],data.qfrc_bias[:3],LIMITS)
            elif control=='EVO':u=torque_ctc(q,dq,r,rd,rdd,M[:3,:3],data.qfrc_bias[:3],LIMITS,evo_th)
            elif control=='HYB':u=mpc.command(q,dq,r,M[:3,:3],data.qfrc_bias[:3],LIMITS,rd,rdd,theta_ctc=evo_th)
            elif control=='KIN':u=kin_ctrl.step(q,dq,r,rd,rdd,M[:3,:3],data.qfrc_bias[:3],LIMITS)
            else:u=mpc.command(q,dq,r,M[:3,:3],data.qfrc_bias[:3],LIMITS,rd,rdd)
        data.ctrl[:3]=u
        data.qfrc_applied[:3]=[.035,-.04,.025] if disturbance and 6<=data.time<6.6 else [0,0,0]
        mujoco.mj_step(model,data)
        if i%20==0:
            xyz=data.site_xpos[model.site('punta').id].copy();err=float(np.linalg.norm(xyz-DEFAULT_TARGET)*1000)
            v_mms=float(np.linalg.norm(cartesian_velocity(data.qpos[:3],data.qvel[:3]))*1000)
            records.append([float(data.time),err,v_mms])
    steady=[r[1] for r in records if r[0]>max(4.,duration-3.)]
    vmax=max(r[2] for r in records)
    print(f'{control}: error final={records[-1][1]:.3f} mm, RMSE últimos 3s={np.sqrt(np.mean(np.square(steady))):.3f} mm, v_max={vmax:.2f} mm/s')

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--viewer',action='store_true',help='Abrir ventana única de laboratorio')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--controller',choices=['PID','MPC','EVO','HYB','KIN'],default='PID')
    p.add_argument('--duration',type=float,default=15.)
    p.add_argument('--disturbance',action='store_true')
    args=p.parse_args()
    if args.headless:headless(args.controller,args.duration,args.disturbance)
    else:Laboratory(args.controller).run()
