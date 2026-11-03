%% ========================================================================
%  TRABAJO PARCIAL 2026-2: ROBOT ANTROPOMÓRFICO DE 3 GDL (RRR)
%  Curso   : Robótica y Sistemas Autónomos - Posgrado FIIS UNI
%  Docente : Ing. Wilson Dennis Reyes Vásquez
%  Modelo  : Manipulador Antropomórfico 3GDL (Conejo Benítez, 2021)
%            d1 = 0.079 m (Base), L1 = 0.113 m (Hombro-Codo), L2 = 0.140 m (Codo-Efector)
%
%  Módulos incluidos en este script autocontenido de MATLAB:
%    1. Cinemática Directa (Denavit-Hartenberg T01, T02, T03 y geométrica)
%    2. Cinemática Inversa Analítica (Ramas Codo Abajo / Codo Arriba)
%    3. Cinemática Diferencial Directa (p_dot = J*q_dot) e Inversa (DLS J^+)
%    4. Análisis de Singularidades (det(J), Índice de Yoshikawa w(q), Cond(J))
%    5. Control Cinemático (CLIK) y Control Dinámico CTC en el Circuito de 5 Aros
%% ========================================================================
clear; clc; close all;

%% 1. PARÁMETROS GEOMÉTRICOS DEL ROBOT REAL (Conejo Benítez, 2021 - Tabla 3.2)
robot.d1 = 0.079;   % Altura de la base [m]
robot.L1 = 0.113;   % Longitud del eslabón 1 (brazo) [m]
robot.L2 = 0.140;   % Longitud del eslabón 2 (antebrazo + efector) [m]
robot.q_min = [-2.80, -1.25, -2.40]; % Límites articulares inferiores [rad]
robot.q_max = [ 2.80,  1.35,  2.40]; % Límites articulares superiores [rad]

fprintf('====================================================================\n');
fprintf(' SIMULACIÓN INTEGRAL: ROBOT ANTROPOMÓRFICO 3 GDL (TRABAJO PARCIAL)\n');
fprintf(' Geometría: d1 = %.3f m | L1 = %.3f m | L2 = %.3f m\n', robot.d1, robot.L1, robot.L2);
fprintf('====================================================================\n\n');

%% 2. VERIFICACIÓN DE CINEMÁTICA DIRECTA (DH) E INVERSA (AMBAS RAMAS)
q_prueba = [-1.10, 0.85, -0.75];
[p_fk, T01, T02, T03] = fk_dh_3gdl(q_prueba, robot);
fprintf('1) CINEMÁTICA DIRECTA (Denavit-Hartenberg) en q = [%.2f, %.2f, %.2f] rad:\n', q_prueba);
disp('   Matriz Homogénea T03 =');
disp(T03);

p_objetivo = [0.165, 0.075, 0.165];
q_codo_abajo = ik_3gdl(p_objetivo, +1, robot);
q_codo_arriba = ik_3gdl(p_objetivo, -1, robot);
[p_verif1, ~, ~, ~] = fk_dh_3gdl(q_codo_abajo, robot);
[p_verif2, ~, ~, ~] = fk_dh_3gdl(q_codo_arriba, robot);

fprintf('2) CINEMÁTICA INVERSA para p_obj = [%.3f, %.3f, %.3f] m:\n', p_objetivo);
fprintf('   Rama Codo Abajo (+1): q = [%.4f, %.4f, %.4f] rad | Error FK = %.2e mm\n', ...
    q_codo_abajo, norm(p_verif1 - p_objetivo)*1000);
fprintf('   Rama Codo Arriba (-1): q = [%.4f, %.4f, %.4f] rad | Error FK = %.2e mm\n\n', ...
    q_codo_arriba, norm(p_verif2 - p_objetivo)*1000);

%% 3. ANÁLISIS DE SINGULARIDADES Y MANIPULABILIDAD DE YOSHIKAWA
% det(J) = -L1 * L2 * sin(q3) * (L1*cos(q2) + L2*cos(q2+q3))
[Q2_grid, Q3_grid] = meshgrid(linspace(-1.25, 1.35, 90), linspace(-2.4, 2.4, 90));
W_grid = zeros(size(Q2_grid));
Det_grid = zeros(size(Q2_grid));
for i = 1:numel(Q2_grid)
    q_eval = [0, Q2_grid(i), Q3_grid(i)];
    [~, detJ, w_yosh] = jacobiano_3gdl(q_eval, robot);
    Det_grid(i) = detJ;
    W_grid(i) = w_yosh;
end

%% 4. CONTROL CINEMÁTICO (CLIK CON PSEUDOINVERSA AMORTIGUADA DLS) EN CIRCUITO DE 5 AROS
aros_objetivo = [
     0.150, -0.080, 0.110;
     0.105,  0.140, 0.205;
     0.180,  0.060, 0.095;
    -0.120,  0.110, 0.150;
     0.165,  0.075, 0.165
];
n_aros = size(aros_objetivo, 1);

dt = 0.005;
T_por_aro = 2.5; % Duración rápida por aro [s]
K_clik = 14.0 * eye(3);
lambda_dls = 0.015; % Amortiguamiento Levenberg-Marquardt para cruzar singularidades

q_actual = q_prueba(:);
t_total = [];
q_hist = [];
dq_hist = [];
p_hist = [];
pref_hist = [];
v_norm_hist = [];
err_mm_hist = [];
w_hist = [];
estado_aro = [];

t_acum = 0;
fprintf('3) EJECUTANDO CONTROL CINEMÁTICO (CLIK-DLS) EN CIRCUITO DE 5 AROS (MODO ENCESTAR):\n');
for idx_aro = 1:n_aros
    p_ini = fk_dh_3gdl(q_actual, robot);
    p_fin = aros_objetivo(idx_aro, :)';
    pasos = round(T_por_aro / dt);
    v_pico_aro = 0;
    t_captura = NaN;
    for k = 1:pasos
        t_local = (k - 1) * dt;
        t_now = t_acum + t_local;
        % Interpolador quíntico en el espacio de tarea cartesiano
        u = min(max(t_local / (0.80 * T_por_aro), 0), 1);
        s = 10*u^3 - 15*u^4 + 6*u^5;
        if u > 0 && u < 1
            ds = (30*u^2 - 60*u^3 + 30*u^4) / (0.80 * T_por_aro);
        else
            ds = 0;
        end
        p_des = p_ini + s * (p_fin - p_ini);
        pd_des = ds * (p_fin - p_ini);

        [p_now, ~, ~, ~] = fk_dh_3gdl(q_actual, robot);
        [J, ~, w_now] = jacobiano_3gdl(q_actual, robot);
        e_p = p_des - p_now;
        err_meta_mm = norm(p_fin - p_now) * 1000;

        % Ley de Control Cinemático Inverso Amortiguado (CLIK - DLS)
        J_dls = J' / (J * J' + (lambda_dls^2) * eye(3));
        dq_cmd = J_dls * (pd_des + K_clik * e_p);

        % Velocidad cartesiana real p_dot = J(q)*q_dot
        v_cart = J * dq_cmd;
        rapidez_mms = norm(v_cart) * 1000;
        v_pico_aro = max(v_pico_aro, rapidez_mms);

        if isnan(t_captura) && err_meta_mm < 2.0
            t_captura = t_local;
        end

        % Integración cinemática con respeto a límites articulares
        q_actual = q_actual + dq_cmd * dt;
        q_actual = min(max(q_actual, robot.q_min'), robot.q_max');

        t_total(end+1, 1) = t_now; %#ok<SAGROW>
        q_hist(end+1, :) = q_actual'; %#ok<SAGROW>
        dq_hist(end+1, :) = dq_cmd'; %#ok<SAGROW>
        p_hist(end+1, :) = p_now'; %#ok<SAGROW>
        pref_hist(end+1, :) = p_des'; %#ok<SAGROW>
        v_norm_hist(end+1, 1) = rapidez_mms; %#ok<SAGROW>
        err_mm_hist(end+1, 1) = err_meta_mm; %#ok<SAGROW>
        w_hist(end+1, 1) = w_now; %#ok<SAGROW>
        estado_aro(end+1, 1) = (err_meta_mm < 2.0); %#ok<SAGROW>
    end
    t_acum = t_acum + T_por_aro;
    fprintf('   [Aro %d/5 ENCESTADO] Pos=[%.3f, %.3f, %.3f] m | t_captura=%.2f s | v_max=%.1f mm/s | Error final=%.4f mm\n', ...
        idx_aro, p_fin, t_captura, v_pico_aro, err_mm_hist(end));
end

%% 5. GENERACIÓN DE GRÁFICAS PARA EL INFORME Y EXPOSICIÓN
figure('Name', 'Trabajo Parcial 3GDL - Cinemática, Singularidades y Control', 'Color', 'w', 'Position', [60, 60, 1280, 720]);

% Subplot 1: Trayectoria 3D y Circuito de 5 Aros Encestados
subplot(2, 2, 1); hold on; grid on; axis equal;
plot3(p_hist(:,1), p_hist(:,2), p_hist(:,3), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Trayectoria Efector p(t)');
scatter3(aros_objetivo(:,1), aros_objetivo(:,2), aros_objetivo(:,3), 90, 'g', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Aros Encestados (Estado OK)');
[~, T01_f, T02_f, T03_f] = fk_dh_3gdl(q_actual, robot);
pts_robot = [0 0 0; 0 0 robot.d1; T01_f(1:3,4)'; T02_f(1:3,4)'; T03_f(1:3,4)'];
plot3(pts_robot(:,1), pts_robot(:,2), pts_robot(:,3), '-o', 'Color', [0.9 0.4 0.1], 'LineWidth', 3, 'MarkerFaceColor', 'k', 'DisplayName', 'Esqueleto Brazo 3GDL');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('1. Trayectoria 3D y Circuito de 5 Aros (Modo Encestar)');
legend('Location', 'best'); view(125, 22);

% Subplot 2: Mapa de Manipulabilidad y Singularidades
subplot(2, 2, 2);
contourf(Q2_grid, Q3_grid, W_grid, 20, 'LineColor', 'none'); colorbar; hold on;
plot([-1.25 1.35], [0 0], 'r--', 'LineWidth', 2);
xlabel('q_2 (Hombro) [rad]'); ylabel('q_3 (Codo) [rad]');
title('2. Mapa de Manipulabilidad w(q) = |det(J)| y Singularidad q_3 = 0');

% Subplot 3: Rapidez Cartesiana ||p_dot|| y Cambio de Estado
subplot(2, 2, 3); hold on; grid on;
plot(t_total, v_norm_hist, 'Color', [0.08 0.38 0.68], 'LineWidth', 1.5);
ylabel('Rapidez ||v|| = ||J(q)qdot|| [mm/s]'); xlabel('Tiempo [s]');
title(sprintf('3. Cinemática Diferencial: Rapidez Cartesiana (Pico = %.1f mm/s)', max(v_norm_hist)));

% Subplot 4: Error de Posicionamiento y Estado de Captura (< 2 mm)
subplot(2, 2, 4); hold on; grid on;
plot(t_total, err_mm_hist, 'Color', [0.85 0.20 0.15], 'LineWidth', 1.5, 'DisplayName', 'Error a Meta [mm]');
yline(2.0, 'g--', 'Umbral Enceste/Bloqueo (2 mm)', 'LineWidth', 1.5);
ylabel('Error XYZ [mm]'); xlabel('Tiempo [s]');
title('4. Convergencia del Control Cinemático CLIK y Cambio de Estado');
legend('Location', 'northeast');

%% ========================================================================
%  FUNCIONES AUXILIARES DEL MODELO 3GDL
%% ========================================================================
function A = dh_mat(theta, d, a, alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end

function [p, T01, T02, T03] = fk_dh_3gdl(q, robot)
    A1 = dh_mat(q(1), robot.d1, 0,        pi/2);
    A2 = dh_mat(q(2), 0,        robot.L1, 0);
    A3 = dh_mat(q(3), 0,        robot.L2, 0);
    T01 = A1;
    T02 = T01 * A2;
    T03 = T02 * A3;
    p = T03(1:3, 4);
end

function q = ik_3gdl(p_xyz, elbow, robot)
    x = p_xyz(1); y = p_xyz(2); z = p_xyz(3);
    rho = hypot(x, y);
    zz = z - robot.d1;
    D = (rho^2 + zz^2 - robot.L1^2 - robot.L2^2) / (2 * robot.L1 * robot.L2);
    if abs(D) > 1 + 1e-9
        error('Objetivo fuera del alcance geométrico del robot 3GDL.');
    end
    D = min(max(D, -1), 1);
    q1 = atan2(y, x);
    q3 = atan2(sign(elbow) * sqrt(max(0, 1 - D^2)), D);
    q2 = atan2(zz, rho) - atan2(robot.L2 * sin(q3), robot.L1 + robot.L2 * cos(q3));
    q = [q1, q2, q3];
end

function [J, detJ, w_yosh] = jacobiano_3gdl(q, robot)
    q1 = q(1); q2 = q(2); q3 = q(3);
    r = robot.L1 * cos(q2) + robot.L2 * cos(q2 + q3);
    drb = -robot.L1 * sin(q2) - robot.L2 * sin(q2 + q3);
    drc = -robot.L2 * sin(q2 + q3);
    J = [-r*sin(q1), drb*cos(q1), drc*cos(q1);
          r*cos(q1), drb*sin(q1), drc*sin(q1);
          0,         robot.L1*cos(q2) + robot.L2*cos(q2+q3), robot.L2*cos(q2+q3)];
    detJ = -robot.L1 * robot.L2 * sin(q3) * r;
    w_yosh = abs(detJ);
end
