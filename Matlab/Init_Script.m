%% Phase 1: Initialization and Matrix Computation for LKA System
% M&S-321 Project - Module 3
% Reference Vehicle: 2026 Toyota Corolla Altis X 1.6 Manual

clear; clc;

% 1. Vehicle Parameters (Dynamic Bicycle Model)
m = 1275;       % Vehicle mass (kg) 
Iz = 2400;      % Yaw inertia (kg-m^2)
lf = 1.1;       % Distance from CG to front axle (m) 
lr = 1.6;       % Distance from CG to rear axle (m)
L = 2.7;        % Total Wheelbase (m) 
Cf = 88000;     % Front cornering stiffness (N/rad) 
Cr = 94000;     % Rear cornering stiffness (N/rad) 
vx = 20;        % Longitudinal velocity (m/s) 

% 2. System Matrices Formulation (A, B, C, D)
% State vector: x = [y_dot; psi_dot; e_y; e_psi]

% State Matrix A 
A = [ -(Cf + Cr)/(m*vx),            ((lr*Cr - lf*Cf)/(m*vx)) - vx,   0,   0;
      (lr*Cr - lf*Cf)/(Iz*vx),      -(lf^2*Cf + lr^2*Cr)/(Iz*vx),    0,   0;
      1,                            0,                               0,   vx;
      0,                            1,                               0,   0 ];

% Input Matrix B 
B = [ Cf/m;
      (lf*Cf)/Iz;
      0;
      0 ];

% Output Matrix C (Extracting lateral error and heading error) 
C = [ 0, 0, 1, 0;
      0, 0, 0, 1 ];

% Feedforward Matrix D 
D = [ 0;
      0 ];

% Create Linear Time-Invariant (LTI) continuous system object
sys = ss(A, B, C, D);

% 3. LQR Controller Design
% The Q matrix penalizes states: [lateral velocity, yaw rate, lateral error, heading error]
Q = diag([1, 1, 100, 10]); % Heavy penalty on lateral error (100) 
R = 1;                     % Penalty on steering effort

% Compute optimal gain matrix K and Riccati matrix P 
[K, P, E] = lqr(sys, Q, R);

% 4. Disturbance and Noise Matrices (For robust simulation)
% Disturbance Matrix (e.g., crosswind force acting on mass)
Bd = [1/m; 0; 0; 0]; 
Bc = [0; 0; 0; -1];

% Camera Measurement Noise standard deviations
noise_variance = [0.0004, 0.000025]; 

% Initial Conditions
x0 = [0; 0; 0.3; 0.05]; 

% Display computed results in the Command Window to verify against your report
disp('System Matrix A:'); disp(A);
disp('Input Matrix B:'); disp(B);
disp('Computed Feedback Gain Matrix (K):'); disp(K);