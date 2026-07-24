% compare_casadi_vs_analytical.m
clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

% Load simulation data containing the trajectory
if exist('data_compare.mat', 'file')
    load('data_compare.mat');
else
    error('data_compare.mat not found. Run a simulation first.');
end

sim_data = out.out;
t_sim = sim_data(:, 7);
T_sim = sim_data(:, 1:6);
n_sim = length(t_sim);

% Extract actual logged joint trajectories for ALL 6 joints (converting degrees to radians)
q_sim_all = zeros(n_sim, 6);
dq_sim_all = zeros(n_sim, 6);
ddq_sim_all = zeros(n_sim, 6);

joint_nodes = {
    'Revolute_Joint'
    'shoulder_lift_joint'
    'elbow_joint'
    'wrist_1_joint'
    'wrist_2_joint'
    'wrist_3_joint'
};

for j = 1:6
    q_sim_all(:, j) = out.simlog.(joint_nodes{j}).Rz.q.series.values * pi/180;
    dq_sim_all(:, j) = out.simlog.(joint_nodes{j}).Rz.w.series.values * pi/180;
    ddq_sim_all(:, j) = gradient(dq_sim_all(:, j), t_sim);
end

% 1. Instantiate the CasADi dynamic model
fprintf('Instantiating CasADi dynamics model...\n');
robot = ur5_dynamics_casadi();

% Corrected DH parameters for analytical model
a2=0.425; a3 = 0.39225; a5=0; a6=0;
d1=0.089159; d5=0.09465; d6=0.0823;
m1=3.7000; m2=8.3930; m3=2.2750; m4=1.2190; m5=1.219; m6=0.1879; 

% Mathematically exact DH parameters from URDF alignment
d4 = -0.10915; % Negative d4 places frame 4 origin at positive Y, matching URDF Joint 5

Pc1_corr = [0.000000, 0.089159, 0.000000];
Pc2_corr = [0.280000, 0.000000, -0.135850];
Pc3_corr = [0.250000, 0.000000, -0.016150];
Pc4_corr = [0.000000, -0.016150, -0.000000];
Pc5_corr = [0.000000, 0.000000, 0.000000]; % Becomes zero when d4 aligns frames
Pc6_corr = [0.000000, 0.000000, 0.000000]; % Becomes zero when d4 aligns frames

I1_corr = [1.026750e-02, 6.660000e-03, 1.026750e-02];
I2_corr = [1.510740e-02, 2.268907e-01, 2.268907e-01];
I3_corr = [4.095000e-03, 4.944331e-02, 4.944331e-02];
I4_corr = [1.111728e-01, 1.111728e-01, 2.194200e-01];
I5_corr = [1.111728e-01, 2.194200e-01, 1.111728e-01];
I6_corr = [1.713647e-02, 3.382200e-02, 1.713647e-02];

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
g = -9.81; % Downward gravity in DH coordinates

tau_cas = zeros(n_sim, 6);
tau_ana = zeros(n_sim, 6);

fprintf('Evaluating dynamics along the trajectory (with gravity enabled)...\n');
for i = 1:n_sim
    % A. Evaluate CasADi model (expects URDF coordinates directly, gravity vector [0; 0; -9.81])
    q_i = q_sim_all(i, :)';
    dq_i = dq_sim_all(i, :)';
    ddq_i = ddq_sim_all(i, :)';
    tau_cas(i, :) = full(robot.torque_fun(q_i, dq_i, ddq_i, [0; 0; -9.81]))';
    
    % B. Evaluate analytical model (expects DH coordinates)
    s1 = q_sim_all(i, 1);
    s2 = -q_sim_all(i, 2);
    s3 = -q_sim_all(i, 3);
    s4 = -q_sim_all(i, 4);
    s5 = q_sim_all(i, 5);
    s6 = -q_sim_all(i, 6);
    
    ds1 = dq_sim_all(i, 1);
    ds2 = -dq_sim_all(i, 2);
    ds3 = -dq_sim_all(i, 3);
    ds4 = -dq_sim_all(i, 4);
    ds5 = dq_sim_all(i, 5);
    ds6 = -dq_sim_all(i, 6);
    
    dds1 = ddq_sim_all(i, 1);
    dds2 = -ddq_sim_all(i, 2);
    dds3 = -ddq_sim_all(i, 3);
    dds4 = -ddq_sim_all(i, 4);
    dds5 = ddq_sim_all(i, 5);
    dds6 = -ddq_sim_all(i, 6);
    
    Torques;
    
    % Map analytical DH torques to Simscape joint direction
    tau_ana(i, :) = [T1, -T2, -T3, -T4, T5, -T6];
end

% Align Simscape sensor sign conventions
T_sim_aligned = T_sim;
T_sim_aligned(:, 2) = -T_sim(:, 2); % Sensor sign flipped in Simscape due to Base/Follower port wiring
T_sim_aligned(:, 3) = -T_sim(:, 3); % Sensor sign flipped in Simscape due to Base/Follower port wiring

% Compute Errors
err_cas_sim = T_sim_aligned - tau_cas;
err_ana_cas = tau_cas - tau_ana;

fprintf('\n===================================================\n');
fprintf('VERIFICATION RESULTS: CasADi vs Simscape (Sensor Aligned)\n');
fprintf('===================================================\n');
for j = 1:6
    fprintf('Joint %d Max Error: %.4f N.m\n', j, max(abs(err_cas_sim(:, j))));
end

fprintf('\n===================================================\n');
fprintf('VERIFICATION RESULTS: Analytical vs CasADi\n');
fprintf('===================================================\n');
for j = 1:6
    fprintf('Joint %d Max Error: %.4f N.m\n', j, max(abs(err_ana_cas(:, j))));
end

% Plotting results
figure('Position', [100, 100, 1200, 800]);
for j = 1:6
    subplot(3, 2, j);
    plot(t_sim, T_sim_aligned(:, j), 'r-', 'LineWidth', 2); hold on;
    plot(t_sim, tau_cas(:, j), 'g--', 'LineWidth', 1.5);
    plot(t_sim, tau_ana(:, j), 'b:', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('Torque (N.m)');
    title(sprintf('Joint %d Torque Comparison', j));
    legend('Simscape (Aligned)', 'CasADi (URDF)', 'Analytical (DH)');
end
saveas(gcf, 'compare_casadi_vs_analytical.png');
fprintf('Saved comparison plot as compare_casadi_vs_analytical.png\n');
