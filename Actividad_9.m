%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:360;         % Time array

initPose = [-3.14;1.74;-pi/4];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [
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

%[4, 9.0; 6, 11.0; 8, 11.0; 10, 9.0; 4.05, 8.95; 9.95, 8.95; 
% 7, 5.0; 6, 6.0; 7.05, 5.05; 8, 5.0; 7.05, 4.95; 9, 5.0; 
% 9, 3.0; 7, 1.0; 5, 1.0; 3, 3.0; 3, 5.0; 5, 7.0; 7, 7.0; 8.95, 5.05];

%0, 5.0; 2, 7.0; 0, 9.0; 3, 9.0; 4, 11.0; 5, 9.0; 8, 9.0; 6, 7.0; 8, 5.0; 
%5, 5.0; 5, 3.0; 8, 3.0; 6, 1.0; 2, 1.0; 0, 3.0; 3, 3.0; 3, 5.0; 0.05, 5.0; 
%3.05, 4.95; 3.05, 3.05; 4, 4.95; 4.95, 3.05; 4, 1.05; 3.1, 3.1; 4.05, 1.05;
%4.05, 4.95; 4.95, 4.95; 5, 6.0; 3, 6.0; 3, 8.0; 5, 8.0; 4.95, 5.95 FIGURA FLOR

%6, 0; 5, 2; 5, 4; 8, 5; 10, 6; 7, 4; 5, 4; 4, 6; 2, 6; 1, 8; 1, 9; 
%2, 9; 3, 10; 5, 11; 5, 12; 6, 11; 7, 11; 7, 12; 8, 9; 9, 7; 10, 6;
%9, 7; 8, 9; 7, 8; 7, 11; 6, 11; 5, 11; 3, 10; 4, 10; 4, 9; 3, 9; 4, 10; 
%3, 10; 2, 9; 1, 9; 1, 8;  2, 6; 4, 6; 5, 4; 5, 2; 6, 0 FIGURA PERRO


%2,5; 5,2; 4,-3; 2,-5; -4,-4; -5,-2; -3,2; -5,3 SEGUNDO SEGUIMIENTO
%1,4; 3,3; 2,-1; 1,-3; -3,-4; -3,4; -1,-1 TERCER SEGUIMIENTO
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.4;
controller.MaxAngularVelocity = 14.5;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
 
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints);
    %waitfor(r);
    
end