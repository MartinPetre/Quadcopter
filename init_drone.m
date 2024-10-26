% Author: Ludwig Horvath, Martin Petré

% Date 12/19/2023

clear, clc, close all

working_directory = pwd;

addpath(append(pwd, '/CLASSES'));

addpath(append(pwd, '/Functions'));

addpath(append(pwd, '/Simulation'));

%% Initialize Drone

h = 0.1; % Microcontroller sampling time
drone = DRONE(0.923, diag([0.006, 0.006, 0.0011]), 0.28, 1.e-05, ...
    3.5e-8, 2300, 1300, 14.8, h, 9.82);

world = WORLD(9.82, 'WGS84', [59.2816353; 18.1076279; 48], ...
    pi/4, 273, 0, 0.0289644, 8.3144598, 6.3/1000, 1010);

%% Initializing linear System
[A, B, A_d, B_d] = linearization(drone, world);

n = size(A,2); m = size(B,2);
% C = [[eye(3), zeros(3, 9)]; [zeros(1,11), 1]]; % Makes the ARE solvable for (A_dBar, B_dBar) when
% adding the integrator to the LQR controller.
C = [zeros(1,2), 1, zeros(1,9); zeros(3,9), eye(3)];
p = size(C,1);

%Might not have unique solution, look out for nonsquare or singular Rosenbrock matrix
Rosenbrock = [[A_d-eye(n), B_d]; [C, zeros(p,m)]];
MxMu = Rosenbrock\[zeros(n,p); eye(p)]; 
Mx = MxMu(1:12,:);
Mu = MxMu(13:end,:);

%% Initializing LQR Controller 
x_max = [pi/180, pi/180, 0.1*pi/180, 0.1*pi/180, 0.1*pi/180, pi/180, 10, 10, 10, 0.05, 0.05, 0.05];
[Q,R] = generate_QR_brysons_rule(x_max, drone);

[P,~,Eig,info] = idare(A_d,B_d,Q,R,[],[]);
L = inv(R+B_d'*P*B_d)*B_d'*P*A_d; % Feedback law u = -Lx

%% Initializing LQR + Integrator Controller 
% [QI, RI] = generate_QR_brysons_rule([x_max, x_max(1:3), x_max(12)], drone);
QI = eye(n+p);
RI = eye(m);

A_dI = [A_d, zeros(n,p); -C, eye(p)];
B_dI = [B_d; zeros(p,m)];

Rosenbrock = [A_d-eye(n), B_d; -C, zeros(p,m)];
rank(Rosenbrock)
[PI,~,EigI,infoI] = idare(A_dI,B_dI,QI,RI,[],[]);
LI = inv(RI+B_dI'*PI*B_dI)*B_dI'*PI*A_dI;

%% Initializing PID Controller 

%% Initializing MPC Controller 

%% System Errors
model_error_exists = false;