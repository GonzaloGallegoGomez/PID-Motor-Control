%% ═══════════════════════════════════════════════════════
%  PID Motor Control — MATLAB Simulation
%  Gonzalo Gallego Gomez — EEBE UPC
%  Modelo derivado desde ecuaciones físicas del motor DC
%% ═══════════════════════════════════════════════════════

clear; clc; close all;

%% ── 1. PARÁMETROS FÍSICOS DEL MOTOR ─────────────────────
% Parte eléctrica
R = 2.0;        % Resistencia del bobinado (Ohm)
L = 0.0005;     % Inductancia del bobinado (H)

% Parte mecánica
J = 0.00002;    % Inercia del rotor (kg·m²)
b = 0.0001;     % Fricción viscosa (N·m·s/rad)

% Constante del motor (Ke = Kt = K para motor DC imán permanente)
K = 0.02;       % Constante de par y fcem (V·s/rad)

%% ── 2. FUNCIÓN DE TRANSFERENCIA DEL MOTOR ───────────────
% G(s) = K / [(Ls + R)(Js + b) + K²]
%
% Expandiendo el denominador:
% (Ls + R)(Js + b) = LJ·s² + (Lb + RJ)·s + Rb
% + K² en el término independiente

% Coeficientes del numerador: [K]
num = [K];

% Coeficientes del denominador: [LJ, Lb+RJ, Rb+K²]
den = [L*J, (L*b + R*J), (R*b + K^2)];

% Creamos la función de transferencia en laplace
motor = tf(num, den);

disp('═══════════════════════════════════════')
disp('Función de transferencia del motor:')
disp('═══════════════════════════════════════')
display(motor)

%% ── 3. DISEÑO DEL CONTROLADOR PID ───────────────────────
% Empezamos con valores iniciales y los iremos ajustando
% Método: prueba y error guiado por la respuesta al escalón

Kp = 100;   % Ganancia proporcional
Ki = 200;   % Ganancia integral
Kd = 10;    % Ganancia derivativa

% Creamos el controlador PID
PID = pid(Kp, Ki, Kd);

%% ── 4. SISTEMA EN BUCLE CERRADO ──────────────────────────
% Conectamos el PID con el motor en bucle cerrado
% feedback() cierra el bucle con retroalimentación unitaria
system_P   = feedback(pid(Kp, 0, 0) * motor, 1);
system_PI  = feedback(pid(Kp, Ki, 0) * motor, 1);
system_PID = feedback(PID * motor, 1);

%% ── 5. RESPUESTA AL ESCALÓN ──────────────────────────────
% El escalón unitario simula dar una orden de velocidad
% al motor de golpe (de 0 a setpoint instantáneamente)

t = 0:0.001:2;  % Vector de tiempo: 0 a 2 segundos, paso 1ms

figure('Name', 'PID Motor Control — Step Response', ...
       'Position', [100 100 1000 600]);

% ── Controlador P ──
subplot(1,3,1);
step(system_P, t);
title('Control P');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (rad/s)');
grid on;

% ── Controlador PI ──
subplot(1,3,2);
step(system_PI, t);
title('Control PI');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (rad/s)');
grid on;

% ── Controlador PID ──
subplot(1,3,3);
step(system_PID, t);
title('Control PID');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (rad/s)');
grid on;

sgtitle('Comparación P vs PI vs PID — Motor DC JGA25-370', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── 6. MÉTRICAS DE RENDIMIENTO ───────────────────────────
% Extraemos automáticamente los indicadores clave de cada controlador

info_P   = stepinfo(system_P);
info_PI  = stepinfo(system_PI);
info_PID = stepinfo(system_PID);

disp(' ')
disp('═══════════════════════════════════════════════════════')
disp('              MÉTRICAS DE RENDIMIENTO')
disp('═══════════════════════════════════════════════════════')
fprintf('%-25s %10s %10s %10s\n', 'Métrica', 'P', 'PI', 'PID');
disp('───────────────────────────────────────────────────────')
fprintf('%-25s %10.4f %10.4f %10.4f\n', 'Tiempo de subida (s)',   info_P.RiseTime,      info_PI.RiseTime,      info_PID.RiseTime);
fprintf('%-25s %10.4f %10.4f %10.4f\n', 'Tiempo de establecim.(s)',info_P.SettlingTime,  info_PI.SettlingTime,  info_PID.SettlingTime);
fprintf('%-25s %10.2f %10.2f %10.2f\n', 'Sobreoscilación (%%)',    info_P.Overshoot,     info_PI.Overshoot,     info_PID.Overshoot);
disp('═══════════════════════════════════════════════════════')
