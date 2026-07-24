% run_kinematics_check.m
clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

% 1. Setup trajectory time-series arrays
t = linspace(0, 2, 1000);

% Compute base 5th-order polynomial trajectory profile vectors
s6 = (3*t.^5)/16 - (15*t.^4)/16 + (5*t.^3)/4 + (3*t)/1125899906842624;
ds6 = (15*t.^4)/16 - (15*t.^3)/4 + (15*t.^2)/4 + 3/1125899906842624;
dds6 = (15*t.^3)/4 - (45*t.^2)/4 + (15*t)/2;

% Actuate all joints
s1 = s6; ds1 = ds6; dds1 = dds6;
s2 = s6; ds2 = ds6; dds2 = dds6;
s3 = s6; ds3 = ds6; dds3 = dds6;
s4 = s6; ds4 = ds6; dds4 = dds6;
s5 = s6; ds5 = ds6; dds5 = dds6;

% Assign variables to base workspace for Simulink
assignin('base', 't', t);
assignin('base', 's1', s1); assignin('base', 'ds1', ds1); assignin('base', 'dds1', dds1);
assignin('base', 's2', s2); assignin('base', 'ds2', ds2); assignin('base', 'dds2', dds2);
assignin('base', 's3', s3); assignin('base', 'ds3', ds3); assignin('base', 'dds3', dds3);
assignin('base', 's4', s4); assignin('base', 'ds4', ds4); assignin('base', 'dds4', dds4);
assignin('base', 's5', s5); assignin('base', 'ds5', ds5); assignin('base', 'dds5', dds5);
assignin('base', 's6', s6); assignin('base', 'ds6', ds6); assignin('base', 'dds6', dds6);

% Run Simscape simulation
model_name = 'simscape_robot_model';
load_system([model_name '.slx']);
out = sim(model_name, 'StopTime', '2.0');

% Get simulated EE position and time
sim_ee = out.out1;
t_sim = sim_ee(:, 4);
x_sim = sim_ee(:, 1);
y_sim = sim_ee(:, 2);
z_sim = sim_ee(:, 3);

% Compute DH kinematics at simulated time steps
a2=0.425; a3 = 0.39225; a5=0; a6=0;
d1=0.089159; d4=0.10915; d5=0.09465; d6=0.0823;
syms s1_ s2_ s3_ s4_ s5_ s6_ 'real'
DHpram=[0,pi/2,s1_,d1,1;
    a2,0,s2_,0,1;
    a3,0,s3_,0,1;
    0,pi/2,s4_,d4,1;
    0,-pi/2,s5_,d5,1;
    0,0,s6_,d6,1];

Ai_fn = @(Dh) [cos(Dh(3)) -sin(Dh(3))*cos(Dh(2)) sin(Dh(3))*sin(Dh(2)) Dh(1)*cos(Dh(3));
               sin(Dh(3)) cos(Dh(3))*cos(Dh(2)) -cos(Dh(3))*sin(Dh(2)) Dh(1)*sin(Dh(3));
               0          sin(Dh(2))             cos(Dh(2))            Dh(4);
               0          0                      0                     1];

T = eye(4);
for idx = 1:6
    T = T * Ai_fn(DHpram(idx, :));
end
pos_expr = simplify(T(1:3, 4))';

n_sim = length(t_sim);
s_sim = (3*t_sim.^5)/16 - (15*t_sim.^4)/16 + (5*t_sim.^3)/4 + (3*t_sim)/1125899906842624;

pos_dh = zeros(n_sim, 3);
for idx = 1:n_sim
    q_val = s_sim(idx);
    pos_dh(idx, :) = double(subs(pos_expr, [s1_ s2_ s3_ s4_ s5_ s6_], [q_val q_val q_val q_val q_val q_val]));
end

err = [x_sim, y_sim, z_sim] - pos_dh;
disp('=== KINEMATICS ERROR (MAX ABSOLUTE DIFFERENCE IN M) ===');
fprintf('X error: %f m\n', max(abs(err(:, 1))));
fprintf('Y error: %f m\n', max(abs(err(:, 2))));
fprintf('Z error: %f m\n', max(abs(err(:, 3))));
disp('=== END-EFFECTOR POSITIONS COMPARISON AT t=0 ===');
fprintf('Simscape: X=%f, Y=%f, Z=%f\n', x_sim(1), y_sim(1), z_sim(1));
fprintf('DH:       X=%f, Y=%f, Z=%f\n', pos_dh(1, 1), pos_dh(1, 2), pos_dh(1, 3));
disp('=== END-EFFECTOR POSITIONS COMPARISON AT t=2 ===');
fprintf('Simscape: X=%f, Y=%f, Z=%f\n', x_sim(end), y_sim(end), z_sim(end));
fprintf('DH:       X=%f, Y=%f, Z=%f\n', pos_dh(end, 1), pos_dh(end, 2), pos_dh(end, 3));
