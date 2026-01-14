%% SIMULACION DE ROBOT DIFERENCIAL DE DOS RUEDAS
clc; clear; close all;

%% 1. PARAMETROS DEL ROBOT
r = 0.05;      % Radio de las ruedas [m]
L = 0.30;      % Distancia entre ruedas [m]
m = 5;         % Masa del robot [kg]
I = 0.25;      % Momento de inercia [kg·m^2]

%% 2. TIEMPO DE SIMULACION
dt = 0.01;     % Paso de integracion [s]
T  = 10;       % Tiempo total [s]
t  = 0:dt:T;   % Vector de tiempo
N  = length(t);

%% 3. VARIABLES DE ESTADO
x     = zeros(1,N);
y     = zeros(1,N);
theta = zeros(1,N);
v     = zeros(1,N);
omega = zeros(1,N);

%% 4. CONDICIONES INICIALES
x(1)     = 0;
y(1)     = 0;
theta(1)= 0;
v(1)     = 0;
omega(1)= 0;

%% 5. ENTRADAS (TORQUES DE LOS MOTORES)
tau_R = 0.2 * ones(1,N);    % Torque rueda derecha [Nm]
tau_L = 0.15 * ones(1,N);   % Torque rueda izquierda [Nm]

%% 6. BUCLE DE SIMULACION
for k = 1:N-1
    
    % --- MODELO DINAMICO ---
    dv     = (tau_R(k) + tau_L(k)) / (m * r);
    domega = (L * (tau_R(k) - tau_L(k))) / (2 * I * r);
    
    % Integracion de velocidades
    v(k+1)     = v(k)     + dv * dt;
    omega(k+1) = omega(k) + domega * dt;
    
    % --- MODELO CINEMATICO ---
    x(k+1)     = x(k) + v(k) * cos(theta(k)) * dt;
    y(k+1)     = y(k) + v(k) * sin(theta(k)) * dt;
    theta(k+1)= theta(k) + omega(k) * dt;
end

%% 7. GRAFICA DE TRAYECTORIA
figure;
plot(x,y,'LineWidth',2);
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Trayectoria del robot diferencial');

%% 8. GRAFICAS DE ESTADO
figure;
subplot(3,1,1)
plot(t,x,'LineWidth',1.5)
ylabel('x [m]')
grid on

subplot(3,1,2)
plot(t,y,'LineWidth',1.5)
ylabel('y [m]')
grid on

subplot(3,1,3)
plot(t,theta,'LineWidth',1.5)
ylabel('\theta [rad]')
xlabel('Tiempo [s]')
grid on

%% 9. VELOCIDADES
figure;
subplot(2,1,1)
plot(t,v,'LineWidth',1.5)
ylabel('v [m/s]')
grid on

subplot(2,1,2)
plot(t,omega,'LineWidth',1.5)
ylabel('\omega [rad/s]')
xlabel('Tiempo [s]')
grid on

%% GRAFICO 3D: TIEMPO - POSICION - ORIENTACION
figure;
plot3(t, x, theta, 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('x [m]');
zlabel('\theta [rad]');
title('Evolucion temporal del robot en 3D');
view(50,25);


%% ANIMACION 3D DEL ROBOT DIFERENCIAL

figure('Color','w');
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Animacion 3D del Robot Diferencial');

view(45,30);
hold on;

% Limites del espacio
margin = 0.5;
xlim([min(x)-margin, max(x)+margin]);
ylim([min(y)-margin, max(y)+margin]);
zlim([0 0.3]);

%% PARAMETROS GEOMETRICOS DEL ROBOT
robot_radius = 0.15;   % Radio del cuerpo
robot_height = 0.05;   % Altura del cuerpo
wheel_radius = 0.05;
wheel_width  = 0.02;

%% CREACION DEL CUERPO DEL ROBOT
[XC, YC, ZC] = cylinder(robot_radius, 30);
ZC = ZC * robot_height;

body = surf(XC, YC, ZC, ...
    'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');

%% CREACION DE LAS RUEDAS
[XR, YR, ZR] = cylinder(wheel_radius, 20);
ZR = ZR * wheel_width;

wheel_R = surf(XR, YR, ZR, ...
    'FaceColor', 'k', 'EdgeColor', 'none');

wheel_L = surf(XR, YR, ZR, ...
    'FaceColor', 'k', 'EdgeColor', 'none');

%% BUCLE DE ANIMACION
for k = 1:10:length(x)
    
    % Transformacion del cuerpo
    R = [cos(theta(k)) -sin(theta(k)) 0;
         sin(theta(k))  cos(theta(k)) 0;
         0              0             1];
    
    % Cuerpo del robot
    pts = R * [XC(:)'; YC(:)'; ZC(:)'];
    body.XData = reshape(pts(1,:) + x(k), size(XC));
    body.YData = reshape(pts(2,:) + y(k), size(YC));
    body.ZData = reshape(pts(3,:), size(ZC));
    
    % Rueda derecha
    offset_R = R * [0;  robot_radius; 0];
    wheel_R.XData = XR + x(k) + offset_R(1);
    wheel_R.YData = YR + y(k) + offset_R(2);
    wheel_R.ZData = ZR;
    
    % Rueda izquierda
    offset_L = R * [0; -robot_radius; 0];
    wheel_L.XData = XR + x(k) + offset_L(1);
    wheel_L.YData = YR + y(k) + offset_L(2);
    wheel_L.ZData = ZR;
    
    drawnow;
    pause(0.05);   % <<< CONTROLA LA VELOCIDAD DE LA ANIMACION
end



%% ================== ANIMACION 3D AVANZADA ==================
figure('Color','w','Position',[100 100 1200 500]);

%% ---------- SUBPLOT 1: VISTA SUPERIOR ----------
subplot(1,2,1)
hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Vista Superior (Trayectoria en tiempo real)');
view(2)

traj = plot(NaN,NaN,'b','LineWidth',2); % Trayectoria
robotTop = plot(0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');
dirArrow = quiver(0,0,0,0,'r','LineWidth',2,'MaxHeadSize',2);

margin = 0.5;
xlim([min(x)-margin max(x)+margin])
ylim([min(y)-margin max(y)+margin])

%% ---------- SUBPLOT 2: VISTA 3D ----------
subplot(1,2,2)
hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Animacion 3D del Robot Diferencial');
view(45,30)

xlim([min(x)-margin max(x)+margin])
ylim([min(y)-margin max(y)+margin])
zlim([0 0.3])

%% ---------- PARAMETROS GEOMETRICOS ----------
robot_radius = 0.15;
robot_height = 0.05;
wheel_radius = r;
wheel_width  = 0.02;

%% ---------- CUERPO DEL ROBOT ----------
[XC, YC, ZC] = cylinder(robot_radius, 30);
ZC = ZC * robot_height;
body = surf(XC, YC, ZC, 'FaceColor',[0.7 0.7 0.7],'EdgeColor','none');

%% ---------- RUEDAS ----------
[XR, YR, ZR] = cylinder(wheel_radius, 20);
ZR = ZR * wheel_width;

wheelR = surf(XR, YR, ZR, 'FaceColor','k','EdgeColor','none');
wheelL = surf(XR, YR, ZR, 'FaceColor','k','EdgeColor','none');

%% ---------- TEXTO VELOCIDADES ----------
velText = text(0,0,0.25,'','FontSize',10,'FontWeight','bold');

%% ================== BUCLE DE ANIMACION ==================
for k = 1:10:length(x)

    %% ----- CINEMATICA DE RUEDAS -----
    omegaR = (2*v(k) + omega(k)*L) / (2*r);
    omegaL = (2*v(k) - omega(k)*L) / (2*r);

    %% ----- MATRIZ DE ROTACION -----
    R = [cos(theta(k)) -sin(theta(k)) 0;
         sin(theta(k))  cos(theta(k)) 0;
         0              0             1];

    %% ----- ACTUALIZAR CUERPO -----
    pts = R * [XC(:)'; YC(:)'; ZC(:)'];
    body.XData = reshape(pts(1,:) + x(k), size(XC));
    body.YData = reshape(pts(2,:) + y(k), size(YC));
    body.ZData = reshape(pts(3,:), size(ZC));

    %% ----- ACTUALIZAR RUEDAS -----
    offsetR = R*[0;  robot_radius; 0];
    offsetL = R*[0; -robot_radius; 0];

    wheelR.XData = XR + x(k) + offsetR(1);
    wheelR.YData = YR + y(k) + offsetR(2);
    wheelR.ZData = ZR;

    wheelL.XData = XR + x(k) + offsetL(1);
    wheelL.YData = YR + y(k) + offsetL(2);
    wheelL.ZData = ZR;

    %% ----- VISTA SUPERIOR -----
    traj.XData = x(1:k);
    traj.YData = y(1:k);

    robotTop.XData = x(k);
    robotTop.YData = y(k);

    dirArrow.XData = x(k);
    dirArrow.YData = y(k);
    dirArrow.UData = 0.2*cos(theta(k));
    dirArrow.VData = 0.2*sin(theta(k));

    %% ----- TEXTO DE VELOCIDADES -----
    velText.Position = [x(k) y(k) 0.25];
    velText.String = sprintf(['v = %.2f m/s\n', ...
                              '\\omega = %.2f rad/s\n', ...
                              '\\omega_R = %.2f rad/s\n', ...
                              '\\omega_L = %.2f rad/s'], ...
                              v(k), omega(k), omegaR, omegaL);

    drawnow;
    pause(0.05);   % <<< CONTROLA LA VELOCIDAD DE LA ANIMACION
end
