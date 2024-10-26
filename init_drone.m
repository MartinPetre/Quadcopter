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
C = eye(12,12);

P = [[A_d-eye(12), B_d];[C, zeros(size(C,1),size(B,2))]];
MxMu = P\[zeros(12,12);eye(12)];
Mx = MxMu(1:12,:);
Mu = MxMu(13:end,:);

%% Initializing LQR Controller 
x_max = [pi/180, pi/180, 0.1*pi/180, 0.1*pi/180, 0.1*pi/180, pi/180, 10, 10, 10, 0.05, 0.05, 0.05];
[Q,R] = generate_QR_brysons_rule(x_max, drone);

[P,~,eig,info] = idare(A_d,B_d,Q,R,[],[]);
L = inv(R+B_d'*P*B_d)*B_d'*P*A_d; % Feedback law u = -Lx

%% Initializing LQR + Integrator Controller 

%% Initializing PID Controller 

%% Initializing MPC Controller 

%% System Errors
model_error_exists = false;