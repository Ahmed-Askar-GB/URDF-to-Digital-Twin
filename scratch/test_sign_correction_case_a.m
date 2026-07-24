% test_sign_correction_case_a.m
clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

% Load simulation data
load('data_compare.mat');
sim_data = out.out;
t_sim = sim_data(:, 7);
T_sim = sim_data(:, 1:6);

n_sim = length(t_sim);
s_at_t = @(time) (3*time.^5)/16 - (15*time.^4)/16 + (5*time.^3)/4 + (3*time)/1125899906842624;
ds_at_t = @(time) (15*time.^4)/16 - (15*time.^3)/4 + (15*time.^2)/4 + 3/1125899906842624;
dds_at_t = @(time) (15*time.^3)/4 - (45*time.^2)/4 + (15*time)/2;

y = s_at_t(t_sim);
dy = ds_at_t(t_sim);
ddy = dds_at_t(t_sim);

% Corrected parameters
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

a2=0.425; a3 = 0.39225; a5=0; a6=0;
d1=0.089159; d4=0.10915; d5=0.09465; d6=0.0823;
m1=3.7000; m2=8.3930; m3=2.2750; m4=1.2190; m5=1.219; m6=0.1879; 

% Unpack variables for the symbolic script
pc1x = Pc1_corr(1); pc1y = Pc1_corr(2); pc1z = Pc1_corr(3);
pc2x = Pc2_corr(1); pc2y = Pc2_corr(2); pc2z = Pc2_corr(3);
pc3x = Pc3_corr(1); pc3y = Pc3_corr(2); pc3z = Pc3_corr(3);
pc4x = Pc4_corr(1); pc4y = Pc4_corr(2); pc4z = Pc4_corr(3);
pc5x = Pc5_corr(1); pc5y = Pc5_corr(2); pc5z = Pc5_corr(3);
pc6x = Pc6_corr(1); pc6y = Pc6_corr(2); pc6z = Pc6_corr(3);

I1xx = I1_corr(1); I1yy = I1_corr(2); I1zz = I1_corr(3);
I2xx = I2_corr(1); I2yy = I2_corr(2); I2zz = I2_corr(3);
I3xx = I3_corr(1); I3yy = I3_corr(2); I3zz = I3_corr(3);
I4xx = I4_corr(1); I4yy = I4_corr(2); I4zz = I4_corr(3);
I5xx = I5_corr(1); I5yy = I5_corr(2); I5zz = I5_corr(3);
I6xx = I6_corr(1); I6yy = I6_corr(2); I6zz = I6_corr(3);
g = 0;

T_eval = zeros(n_sim, 6);
for i = 1:n_sim
    % Fixed joints 1-4, actuate 5-6
    s1 = 0;
    s2 = 0;
    s3 = 0;
    s4 = 0;
    s5 = y(i);
    s6 = -y(i);
    
    ds1 = 0;
    ds2 = 0;
    ds3 = 0;
    ds4 = 0;
    ds5 = dy(i);
    ds6 = -dy(i);
    
    dds1 = 0;
    dds2 = 0;
    dds3 = 0;
    dds4 = 0;
    dds5 = ddy(i);
    dds6 = -ddy(i);
    
    Torques;
    
    % Apply sign correction to computed analytical torques to match Simscape joint direction
    T_eval(i, :) = [T1, -T2, -T3, -T4, T5, -T6];
end

err = T_sim - T_eval;
disp('=== DYNAMICS COMPARISON WITH SIGN CORRECTIONS (CASE A: ONLY 5 & 6 MOVING) ===');
fprintf('Joint\tMax Sim Torque\tMax Analytical Torque\tMax Error\n');
for j = 1:6
    fprintf('Joint %d\t%f N.m\t%f N.m\t\t%f N.m\n', ...
        j, max(abs(T_sim(:,j))), max(abs(T_eval(:,j))), max(abs(err(:,j))));
end
