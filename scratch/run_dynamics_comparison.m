% scratch/run_dynamics_comparison.m
clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

% 1. Setup trajectory time-series arrays
t = linspace(0, 2, 10000);

% Compute base 5th-order polynomial trajectory profile vectors
s6 = (3*t.^5)/16 - (15*t.^4)/16 + (5*t.^3)/4 + (3*t)/1125899906842624;
ds6 = (15*t.^4)/16 - (15*t.^3)/4 + (15*t.^2)/4 + 3/1125899906842624;
dds6 = (15*t.^3)/4 - (45*t.^2)/4 + (15*t)/2;

% Actuate only joint 5 and 6, others fixed to 0
s1 = zeros(size(t)); ds1 = zeros(size(t)); dds1 = zeros(size(t));
s2 = zeros(size(t)); ds2 = zeros(size(t)); dds2 = zeros(size(t));
s3 = zeros(size(t)); ds3 = zeros(size(t)); dds3 = zeros(size(t));
s4 = zeros(size(t)); ds4 = zeros(size(t)); dds4 = zeros(size(t));
s5 = s6;             ds5 = ds6;             dds5 = dds6;

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
fprintf('Running Simscape simulation...\n');
out = sim(model_name, 'StopTime', '2.0');

% Get simulated torques
% Columns: T1, T2, T3, T4, T5, T6, time
sim_data = out.out;
t_sim = sim_data(:, 7);
T_sim = sim_data(:, 1:6);

% We need to evaluate analytical torques at simulated time steps
% Interpolate s, ds, dds to t_sim
s_at_t = @(time) (3*time.^5)/16 - (15*time.^4)/16 + (5*time.^3)/4 + (3*time)/1125899906842624;
ds_at_t = @(time) (15*time.^4)/16 - (15*time.^3)/4 + (15*time.^2)/4 + 3/1125899906842624;
dds_at_t = @(time) (15*time.^3)/4 - (45*time.^2)/4 + (15*time)/2;

n_sim = length(t_sim);
s6_val = s_at_t(t_sim);   ds6_val = ds_at_t(t_sim);   dds6_val = dds_at_t(t_sim);
s5_val = s6_val;          ds5_val = ds6_val;          dds5_val = dds6_val;

s1_val = zeros(n_sim, 1); ds1_val = zeros(n_sim, 1); dds1_val = zeros(n_sim, 1);
s2_val = zeros(n_sim, 1); ds2_val = zeros(n_sim, 1); dds2_val = zeros(n_sim, 1);
s3_val = zeros(n_sim, 1); ds3_val = zeros(n_sim, 1); dds3_val = zeros(n_sim, 1);
s4_val = zeros(n_sim, 1); ds4_val = zeros(n_sim, 1); dds4_val = zeros(n_sim, 1);

% Evaluate analytical torques with Original manual parameters
a2=0.425; a3 = 0.39225; a5=0; a6=0;
d1=0.089159; d4=0.10915; d5=0.09465; d6=0.0823;
m1=3.7000; m2=8.3930; m3=2.2750; m4=1.2190; m5=1.219; m6=0.1879; 

% Original parameters from Test_Dynamic_with_SimScape.m
Pc1_orig = [0.000, d1, 0.28];
Pc2_orig = [a2-0.302, 0.000, 0.160];
Pc3_orig = [a3-0.25, 0, 0];
Pc4_orig = [0.000, 0.000, 0.093];
Pc5_orig = [0, d5, 0];
Pc6_orig = [0, 0, d6-0.001159];

I1_orig = [0.0103 0.0103 0.0067];
I2_orig = [0.1210 0.2269 0.1210];
I3_orig = [0.0268 0.0494 0.0268];
I4_orig = [0.1112 0.2194 0.1112];
I5_orig = [0.111172755531, 0.21942, 0.111172755531];
I6_orig = [0.033822, 0.0171364731454, 0.0171364731454];

[T_orig, err_orig] = evaluate_torques(n_sim, s1_val, s2_val, s3_val, s4_val, s5_val, s6_val, ...
    ds1_val, ds2_val, ds3_val, ds4_val, ds5_val, ds6_val, ...
    dds1_val, dds2_val, dds3_val, dds4_val, dds5_val, dds6_val, ...
    d1, d4, d5, d6, a2, a3, m1, m2, m3, m4, m5, m6, ...
    Pc1_orig, Pc2_orig, Pc3_orig, Pc4_orig, Pc5_orig, Pc6_orig, ...
    I1_orig, I2_orig, I3_orig, I4_orig, I5_orig, I6_orig, T_sim);

% Evaluate analytical torques with CORRECT DH parameters from URDF
Pc1_corr = [0.000000, 0.089159, 0.000000];
Pc2_corr = [0.280000, 0.000000, -0.135850];
Pc3_corr = [0.250000, 0.000000, -0.016150];
Pc4_corr = [0.000000, -0.016150, -0.000000];
Pc5_corr = [0.000000, 0.000000, -0.218300];
Pc6_corr = [0.000000, 0.000000, -0.218300];

I1_corr = [1.026750e-02, 6.660000e-03, 1.026750e-02];
I2_corr = [1.510740e-02, 2.268907e-01, 2.268907e-01];
I3_corr = [4.095000e-03, 4.944331e-02, 4.944331e-02];
I4_corr = [1.111728e-01, 1.111728e-01, 2.194200e-01];
I5_corr = [1.111728e-01, 2.194200e-01, 1.111728e-01];
I6_corr = [1.713647e-02, 3.382200e-02, 1.713647e-02];

[T_corr, err_corr] = evaluate_torques(n_sim, s1_val, s2_val, s3_val, s4_val, s5_val, s6_val, ...
    ds1_val, ds2_val, ds3_val, ds4_val, ds5_val, ds6_val, ...
    dds1_val, dds2_val, dds3_val, dds4_val, dds5_val, dds6_val, ...
    d1, d4, d5, d6, a2, a3, m1, m2, m3, m4, m5, m6, ...
    Pc1_corr, Pc2_corr, Pc3_corr, Pc4_corr, Pc5_corr, Pc6_corr, ...
    I1_corr, I2_corr, I3_corr, I4_corr, I5_corr, I6_corr, T_sim);

disp('=== TORQUE ERROR COMPARISON (MAX ABSOLUTE ERROR IN NM) ===');
fprintf('Joint\tOriginal Params Error\tCorrected Params Error\n');
for j = 1:6
    fprintf('Joint %d\t%f N.m\t\t%f N.m\n', j, max(abs(err_orig(:, j))), max(abs(err_corr(:, j))));
end

function [T_eval, err] = evaluate_torques(n_sim, s1, s2, s3, s4, s5, s6, ...
    ds1, ds2, ds3, ds4, ds5, ds6, ...
    dds1, dds2, dds3, dds4, dds5, dds6, ...
    d1, d4, d5, d6, a2, a3, m1, m2, m3, m4, m5, m6, ...
    Pc1, Pc2, Pc3, Pc4, Pc5, Pc6, ...
    I1, I2, I3, I4, I5, I6, T_sim)

    % Unpack com variables
    pc1x = Pc1(1); pc1y = Pc1(2); pc1z = Pc1(3);
    pc2x = Pc2(1); pc2y = Pc2(2); pc2z = Pc2(3);
    pc3x = Pc3(1); pc3y = Pc3(2); pc3z = Pc3(3);
    pc4x = Pc4(1); pc4y = Pc4(2); pc4z = Pc4(3);
    pc5x = Pc5(1); pc5y = Pc5(2); pc5z = Pc5(3);
    pc6x = Pc6(1); pc6y = Pc6(2); pc6z = Pc6(3);

    % Unpack inertia variables
    I1xx = I1(1); I1yy = I1(2); I1zz = I1(3);
    I2xx = I2(1); I2yy = I2(2); I2zz = I2(3);
    I3xx = I3(1); I3yy = I3(2); I3zz = I3(3);
    I4xx = I4(1); I4yy = I4(2); I4zz = I4(3);
    I5xx = I5(1); I5yy = I5(2); I5zz = I5(3);
    I6xx = I6(1); I6yy = I6(2); I6zz = I6(3);

    T_eval = zeros(n_sim, 6);
    
    % Evaluate Torques.m script row-by-row
    for i = 1:n_sim
        % Set scalars for current time-step
        s1_temp = s1(i); s2_temp = s2(i); s3_temp = s3(i); s4_temp = s4(i); s5_temp = s5(i); s6_temp = s6(i);
        ds1_temp = ds1(i); ds2_temp = ds2(i); ds3_temp = ds3(i); ds4_temp = ds4(i); ds5_temp = ds5(i); ds6_temp = ds6(i);
        dds1_temp = dds1(i); dds2_temp = dds2(i); dds3_temp = dds3(i); dds4_temp = dds4(i); dds5_temp = dds5(i); dds6_temp = dds6(i);
        
        % We map the local variables s1..s6 for Torques.m
        s1 = s1_temp; s2 = s2_temp; s3 = s3_temp; s4 = s4_temp; s5 = s5_temp; s6 = s6_temp;
        ds1 = ds1_temp; ds2 = ds2_temp; ds3 = ds3_temp; ds4 = ds4_temp; ds5 = ds5_temp; ds6 = ds6_temp;
        dds1 = dds1_temp; dds2 = dds2_temp; dds3 = dds3_temp; dds4 = dds4_temp; dds5 = dds5_temp; dds6 = dds6_temp;
        
        % Run Torques.m to get T1..T6
        Torques;
        
        T_eval(i, :) = [T1, T2, T3, T4, T5, T6];
    end
    
    err = T_sim - T_eval;
end
