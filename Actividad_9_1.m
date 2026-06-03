%Limpieza de pantalla
clear all
close all
clc

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=80;             % Tiempo de simulación en segundos (s)
ts=0.1;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras

%2 INTERPOLACIÓN DE TRAYECTORIA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Puntos = [
    -3.14,  1.74; -2.50,  2.00; -1.71,  1.79; -1.50,  1.22; -1.52,  0.89; 
    -1.48,  0.57; -1.75,  0.56; -1.93,  0.80; -2.42,  0.89; -2.95,  1.29; 
    -1.39,  0.58; -1.18,  0.11; -1.45, -0.58; -1.68, -1.12; -1.63, -1.62; 
    -1.15, -2.42; -0.98, -2.17; -0.52, -2.27; -0.01, -2.35;  0.50, -2.29; 
     1.02, -2.18;  1.27, -2.43;  1.68, -1.64;  1.75, -1.05;  1.50, -0.50; 
     1.32,  0.12;  1.56,  0.66;  1.77,  1.16;  1.86,  1.58;  2.30,  1.95; 
     2.93,  2.03;  3.65,  1.79;  3.25,  1.27;  2.69,  0.94;  2.16,  0.89; 
     2.13,  0.59;  1.78,  0.60;  1.14, -2.48;  0.80, -2.60;  0.54, -2.84; 
     0.42, -3.14;  0.32, -3.52;  0.14, -3.69; -0.08, -3.65; -0.22, -3.43; 
    -0.29, -3.11; -0.45, -2.76; -0.69, -2.59; -0.99, -2.55;  0.10, -3.97; 
     0.19, -4.39;  0.59, -4.53;  1.09, -4.74;  1.60, -4.80; -0.07, -4.39; 
    -0.59, -4.63; -1.09, -4.85; -1.69, -4.99; -2.18, -4.71; -2.48, -3.92; 
    -2.71, -3.31; -2.85, -2.42; -3.68, -2.00; -4.35, -1.39; -4.70, -0.68; 
    -4.93,  0.19; -1.44, -5.30; -0.53, -5.75;  0.52, -5.71;  1.55, -5.26; 
     2.31, -4.57;  2.74, -3.74;  3.00, -3.00;  2.95, -2.15;  3.85, -1.76; 
     4.68, -1.47;  4.51, -1.76;  5.17, -0.84;  5.46, -0.03; 
     5.61,  1.01;  5.61,  1.99;  5.81,  2.77;  6.28,  3.49;  5.84,  4.26; 
     6.55,  4.49;  6.88,  5.30;  7.01,  6.33;  6.60,  7.11;  5.87,  7.40; 
     4.97,  7.18;  4.06,  6.66;  3.32,  5.99;  2.87,  6.20;  1.77,  6.73; 
     0.79,  6.90; -0.41,  6.88; -1.48,  6.63; -2.32,  6.17; -3.17,  5.96; 
    -3.73,  6.43; -4.35,  6.85; -4.98,  7.22; -5.82,  7.18; -6.35,  6.62; 
    -6.52,  5.79; -6.29,  5.02; -5.97,  4.33; -5.35,  4.15; -5.74,  3.35; 
    -5.23,  2.70; -5.12,  1.87; -4.97,  0.73
];

s = linspace(0, 1, size(Puntos, 1));
ss = linspace(0, 1, N);
hx_ref = interp1(s, Puntos(:,1), ss, 'linear');
hy_ref = interp1(s, Puntos(:,2), ss, 'linear');

%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros(1, N+1); 
y1 = zeros(1, N+1); 
phi = zeros(1, N+1);
u = zeros(1, N);
w = zeros(1, N);

x1(1) = Puntos(1,1);    % Iniciamos en el punto A
y1(1) = Puntos(1,2);
phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1)); % Orientación inicial hacia el punto B

%4 BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:N-1
    % Derivadas para obtener velocidades deseadas
    dx = (hx_ref(k+1) - hx_ref(k))/ts;
    dy = (hy_ref(k+1) - hy_ref(k))/ts;
    
    u(k) = sqrt(dx^2 + dy^2); % Velocidad lineal
    
    % Calculamos la orientación deseada
    phi_d = atan2(dy, dx);
    
    % Velocidad angular 
    w(k) = atan2(sin(phi_d - phi(k)), cos(phi_d - phi(k)))/ts;
    
    % Actualizamos la orientación para el siguiente paso del cálculo
    phi(k+1) = phi(k) + w(k)*ts;
end

phi = zeros(1, N+1);
phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1));

for k=1:N 
    phi(k+1) = phi(k) + w(k)*ts; 
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    x1(k+1) = x1(k) + xp1*ts;
    y1(k+1) = y1(k) + yp1*ts;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Configurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([-0.1 35]); % Orientacion de la figura
axis([-10 10 -8 10 0 1]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 2;
MobileRobot_5;

% c) Graficar Trayectorias
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;
H3 = plot3(hx_ref, hy_ref, zeros(1, length(hx_ref)), 'g', 'lineWidth', 1.5);
H2 = plot3(x1(1), y1(1), 0, 'r', 'lineWidth', 2);

% d) Bucle de simulacion de movimiento del robot
step=10; % pasos para simulacion
for k=1:step:N
    
    delete(H1);

    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    set(H2, 'XData', x1(1:k), 'YData', y1(1:k), 'ZData', zeros(1,k));

    drawnow limitrate;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(311)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
dist_error = sqrt((hx_ref - x1(1:N)).^2 + (hy_ref - y1(1:N)).^2);
plot(t, dist_error, 'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');
