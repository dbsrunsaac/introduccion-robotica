%% Control P de un Uniciclo Cinematico tomando punto por fuera
close all; clear; clc;

% sim('robot_uniciclo');

simOut = sim('robot_uniciclo');

%% Extraer variables desde Simulink
t  = simOut.tout;   % Tiempo
P  = simOut.P;      % Posición real
Pd = simOut.Pd;     % Posición deseada
v  = simOut.v;      % Velocidad lineal
w  = simOut.w;      % Velocidad angular

%% Figura 1: Trayectoria en el plano
figure(1)
plot(P(:,1), P(:,2), '--r', Pd(:,1), Pd(:,2), 'b');
grid on; hold on;
plot(P(1,1), P(1,2), '*r');
xlabel('X [m]');
ylabel('Y [m]');
title('Posicion en el plano');
legend('P', 'Pd');

%% Figura 2: Errores de seguimiento
figure(2)
plot(t, Pd(:,1) - P(:,1), 'r', ...
     t, Pd(:,2) - P(:,2), 'b');
grid on;
xlabel('t [s]');
ylabel('e [m]');
title('Errores de seguimiento');
legend('e_x', 'e_y');

%% Figura 3: Entradas de control
figure(3)
plot(t, v, 'r', t, w, 'b');
grid on;
xlabel('t [s]');
ylabel('U [m/s, rad/s]');
title('Entradas de control');
legend('v', 'w');
