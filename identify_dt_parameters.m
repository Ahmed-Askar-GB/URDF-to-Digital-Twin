% identify_dt_parameters.m
clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

% 1. Load Experimental Dataset
fprintf('Loading UR5_dataset.csv...\n');
data = readmatrix('UR5_dataset.csv');
q_cmd_deg = data(2:end, 1:6);
p_nom_mm = data(2:end, 7:9);
p_real_mm = data(2:end, 13:15);

n_points = size(q_cmd_deg, 1);
fprintf('Total data points loaded: %d\n', n_points);

% Convert units
q_cmd = q_cmd_deg * pi/180; % radians
p_real = p_real_mm / 1000;  % meters

% 2. Kinematics definitions
a2_nom = 0.425; a3_nom = 0.39225;
d1_nom = 0.089159; d4_nom = -0.10915; d5_nom = 0.09465; d6_nom = 0.0823;

alpha = [-pi/2, 0, 0, -pi/2, -pi/2, 0];
signs = [1, -1, -1, -1, 1, -1];
offsets = [0, pi, 0, pi, 0, 0];

% 3. Split Data: First 50% for Calibration, second 50% for Verification
n_calib = floor(n_points / 2);
n_ver = n_points - n_calib;
fprintf('Dataset splitting:\n');
fprintf('  Calibration (Training) set: points 1 to %d\n', n_calib);
fprintf('  Verification (Validation) set: points %d to %d\n', n_calib+1, n_points);

q_cmd_calib = q_cmd(1:n_calib, :);
p_real_calib = p_real(1:n_calib, :);

q_cmd_ver = q_cmd(n_calib+1:end, :);
p_real_ver = p_real(n_calib+1:end, :);

% 4. Parameter Identification Setup
% Nominal dynamic parameters from URDF
m_nom = [3.7000, 8.3930, 2.2750, 1.2190, 1.2190, 0.1879];
Pc_nom = [
    0.000000, 0.089159, 0.000000;
    0.280000, 0.000000, -0.135850;
    0.250000, 0.000000, -0.016150;
    0.000000, -0.016150, -0.000000;
    0.000000, 0.000000, 0.000000;
    0.000000, 0.000000, 0.000000
];

% Optimization variables vector:
% x = [
%   dtheta_off (6), 
%   da2, da3, dd1, dd4, dd5, dd6 (6), 
%   S (4) for joints 2..5,
%   cc (4) for joints 2..5,
%   coff (4) for joints 2..5,
%   m (6),
%   Pc (18),
%   base_rot (3) [rx, ry, rz],
%   base_trans (3) [tx, ty, tz]
% ] -> Total: 54 variables

x_init = zeros(1, 54);
x_init(25:30) = m_nom;
x_init(31:48) = Pc_nom(:)';

% Bounds
lb = -inf(1, 54);
ub = inf(1, 54);

% Joint offsets bounds: +/- 2 degrees
lb(1:6) = -2 * pi/180;
ub(1:6) = 2 * pi/180;

% DH parameter offsets bounds: +/- 10 mm
lb(7:12) = -0.010;
ub(7:12) = 0.010;

% Compliance and Friction bounds: S in [2e-5, 2e-3] corresponding to K_p in [500, 50000] N.m/rad
lb(13:16) = 2e-5;
ub(13:16) = 2e-3;

% cc > 0 (scaled Coulomb friction)
lb(17:20) = 1e-6;
ub(17:20) = 1e-2;

% Mass bounds: +/- 20%
lb(25:30) = m_nom * 0.8;
ub(25:30) = m_nom * 1.2;

% CoM bounds: +/- 10 cm
lb(31:48) = Pc_nom(:)' - 0.10;
ub(31:48) = Pc_nom(:)' + 0.10;

% Base frame alignment bounds: +/- 5 degrees for rotation, +/- 20 mm for translation
lb(49:51) = -5 * pi/180;
ub(49:51) = 5 * pi/180;
lb(52:54) = -0.020;
ub(52:54) = 0.020;

% 5. Optimization: Kinematics, Compliance and Base frame registration
fprintf('Optimizing digital twin parameters on Calibration Set...\n');
cost_options = optimoptions('lsqnonlin', 'Display', 'iter', 'Algorithm', 'levenberg-marquardt', 'FunctionTolerance', 1e-6, 'MaxIterations', 25);

[x_opt, resnorm] = lsqnonlin(@(x) cost_fn(x, q_cmd_calib, p_real_calib, n_calib, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets, m_nom, Pc_nom), x_init, lb, ub, cost_options);

% Extract Final parameters
dtheta_opt = x_opt(1:6)';
da2_opt = x_opt(7); da3_opt = x_opt(8); dd1_opt = x_opt(9); dd4_opt = x_opt(10); dd5_opt = x_opt(11); dd6_opt = x_opt(12);
S_opt = x_opt(13:16);
cc_opt = x_opt(17:20);
coff_opt = x_opt(21:24);
m_opt = x_opt(25:30);
Pc_opt = reshape(x_opt(31:48), 6, 3);
base_rot_opt = x_opt(49:51);
base_trans_opt = x_opt(52:54);

% Recover physical controller gains Kp and friction values
K_p = zeros(6, 1);
f_c = zeros(6, 1);
f_off = zeros(6, 1);

K_p(1) = 10000; f_c(1) = 5.0; f_off(1) = 0.0;
for j = 2:5
    K_p(j) = 1 / S_opt(j-1);
    f_c(j) = cc_opt(j-1) * K_p(j);
    f_off(j) = coff_opt(j-1) * K_p(j);
end
K_p(6) = 5000; f_c(6) = 1.0; f_off(6) = 0.0;

% 6. Present Results
fprintf('\n========================================================\n');
fprintf('IDENTIFICATION RESULTS: Calibration Set (First 50%%)\n');
fprintf('========================================================\n');
fprintf('Joint Encoder Offsets Delta_theta (Task 4/5):\n');
for j = 1:6
    fprintf('  Joint %d: offset = %.4f deg (%.6f rad)\n', j, dtheta_opt(j)*180/pi, dtheta_opt(j));
end

fprintf('\nCalibrated DH Kinematic parameters (Task 4/5 offsets):\n');
fprintf('  a2 = %.4f m (Offset = %.4f mm)\n', a2_nom + da2_opt, da2_opt*1000);
fprintf('  a3 = %.4f m (Offset = %.4f mm)\n', a3_nom + da3_opt, da3_opt*1000);
fprintf('  d1 = %.4f m (Offset = %.4f mm)\n', d1_nom + dd1_opt, dd1_opt*1000);
fprintf('  d4 = %.4f m (Offset = %.4f mm)\n', d4_nom + dd4_opt, dd4_opt*1000);
fprintf('  d5 = %.4f m (Offset = %.4f mm)\n', d5_nom + dd5_opt, dd5_opt*1000);
fprintf('  d6 = %.4f m (Offset = %.4f mm)\n', d6_nom + dd6_opt, dd6_opt*1000);

fprintf('\nJoint Mechanical Stiffnesses K_p (Task 5):\n');
for j = 1:6
    if j == 1 || j == 6
        fprintf('  Joint %d: K_p = %.4f N.m/rad (%.4f N.m/deg) [FIXED]\n', j, K_p(j), K_p(j)*pi/180);
    else
        fprintf('  Joint %d: K_p = %.4f N.m/rad (%.4f N.m/deg)\n', j, K_p(j), K_p(j)*pi/180);
    end
end

fprintf('\nJoint Friction Parameters (Task 5 - Linear Friction):\n');
for j = 1:6
    if j == 1 || j == 6
        fprintf('  Joint %d: Coulomb friction f_c = %.4f N.m, Offset f_off = %.4f N.m [FIXED]\n', j, f_c(j), f_off(j));
    else
        fprintf('  Joint %d: Coulomb friction f_c = %.4f N.m, Offset f_off = %.4f N.m\n', j, f_c(j), f_off(j));
    end
end

fprintf('\nBase Frame Alignment (Tracker-to-Robot Mounting Transform):\n');
fprintf('  Rotation (Roll, Pitch, Yaw): [%.4f, %.4f, %.4f] deg\n', base_rot_opt*180/pi);
fprintf('  Translation (X, Y, Z):       [%.4f, %.4f, %.4f] mm\n', base_trans_opt*1000);

fprintf('\nUpdated Dynamic Parameters (Task 4):\n');
for i = 1:6
    fprintf('  Link %d: Mass = %.4f kg (Nominal = %.4f kg)\n', i, m_opt(i), m_nom(i));
    fprintf('           CoM  = [%.4f, %.4f, %.4f] m (Nominal = [%.4f, %.4f, %.4f] m)\n', ...
        Pc_opt(i, 1), Pc_opt(i, 2), Pc_opt(i, 3), Pc_nom(i, 1), Pc_nom(i, 2), Pc_nom(i, 3));
end

% 7. Evaluate model performance on BOTH Calibration and Verification sets
cart_err_nom = eval_errors(x_init, q_cmd, p_real, n_points, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets);
cart_err_calib = eval_errors(x_opt, q_cmd, p_real, n_points, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets);

mean_nom_calib = mean(cart_err_nom(1:n_calib));
mean_opt_calib = mean(cart_err_calib(1:n_calib));

mean_nom_ver = mean(cart_err_nom(n_calib+1:end));
mean_opt_ver = mean(cart_err_calib(n_calib+1:end));

fprintf('\n========================================================\n');
fprintf('MODEL PERFORMANCE EVALUATION (Cartesian Positioning Error)\n');
fprintf('========================================================\n');
fprintf('Calibration (Training) Set (Points 1 to %d):\n', n_calib);
fprintf('  Mean Error (Nominal Model):   %.4f mm\n', mean_nom_calib);
fprintf('  Mean Error (Calibrated Model): %.4f mm\n', mean_opt_calib);
fprintf('\nVerification (Validation) Set (Points %d to %d):\n', n_calib+1, n_points);
fprintf('  Mean Error (Nominal Model):   %.4f mm\n', mean_nom_ver);
fprintf('  Mean Error (Calibrated Model): %.4f mm\n', mean_opt_ver);

% Plot the curves with a shaded split interface
figure('Position', [100, 100, 950, 500]);
plot(cart_err_nom, 'Color', [0.85, 0.33, 0.1], 'LineStyle', '-', 'LineWidth', 1.5); hold on;
plot(cart_err_calib, 'Color', [0, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 1.5);

% Add vertical line demarcating training and testing datasets
yl = ylim;
plot([n_calib, n_calib], [0, yl(2)], 'k--', 'LineWidth', 1.5);

% Shade calibration region
patch([0 n_calib n_calib 0], [0 0 yl(2) yl(2)], [0.9 0.9 0.9], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
text(n_calib/4, yl(2)*0.9, 'Calibration Set (50%)', 'FontSize', 12, 'FontWeight', 'bold');

% Shade verification region
patch([n_calib n_points n_points n_calib], [0 0 yl(2) yl(2)], [0.8 0.9 0.8], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
text(n_calib + n_ver/4, yl(2)*0.9, 'Verification Set (50%)', 'FontSize', 12, 'FontWeight', 'bold');

grid on;
xlim([1, n_points]);
ylim([0, yl(2)]);
xlabel('Data Point Index'); ylabel('Cartesian Position Error (mm)');
title('UR5 Digital Twin Calibration vs. Verification Results (Sub-mm Fit)');
legend('Nominal Model (URDF Kinematics only)', 'Compensated Digital Twin', 'Data Split boundary', 'Location', 'SouthWest');
saveas(gcf, 'calibration_results.png');
fprintf('\nSaved results plot as calibration_results.png\n');


% --- Helper Functions ---

function res = cost_fn(x, q_cmd, p_real, n_points, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets, m_nom, Pc_nom)
    % Extract parameters
    dtheta = x(1:6)';
    da2 = x(7); da3 = x(8); dd1 = x(9); dd4 = x(10); dd5 = x(11); dd6 = x(12);
    S = x(13:16);
    cc = x(17:20);
    coff = x(21:24);
    
    m_val = x(25:30);
    Pc_val = reshape(x(31:48), 6, 3);
    
    base_rx = x(49); base_ry = x(50); base_rz = x(51);
    base_tx = x(52); base_ty = x(53); base_tz = x(54);
    
    % Base rotation matrix
    cx = cos(base_rx); sx = sin(base_rx);
    cy = cos(base_ry); sy = sin(base_ry);
    cz = cos(base_rz); sz = sin(base_rz);
    Rx = [1 0 0; 0 cx -sx; 0 sx cx];
    Ry = [cy 0 sy; 0 1 0; -sy 0 cy];
    Rz = [cz -sz 0; sz cz 0; 0 0 1];
    R_base = Rx * Ry * Rz;
    t_base = [base_tx; base_ty; base_tz];
    
    % Aligned DH parameters
    a2 = a2_nom + da2;
    a3 = a3_nom + da3;
    d1 = d1_nom + dd1;
    d4 = d4_nom + dd4;
    d5 = d5_nom + dd5;
    d6 = d6_nom + dd6;
    
    % Dynamic variables setup for Torques
    ds1=0; ds2=0; ds3=0; ds4=0; ds5=0; ds6=0;
    dds1=0; dds2=0; dds3=0; dds4=0; dds5=0; dds6=0;
    I1xx=0; I1yy=0; I1zz=0; I2xx=0; I2yy=0; I2zz=0; I3xx=0; I3yy=0; I3zz=0;
    I4xx=0; I4yy=0; I4zz=0; I5xx=0; I5yy=0; I5zz=0; I6xx=0; I6yy=0; I6zz=0;
    g=-9.81;
    m1=m_val(1); m2=m_val(2); m3=m_val(3); m4=m_val(4); m5=m_val(5); m6=m_val(6);
    pc1x=Pc_val(1,1); pc1y=Pc_val(1,2); pc1z=Pc_val(1,3);
    pc2x=Pc_val(2,1); pc2y=Pc_val(2,2); pc2z=Pc_val(2,3);
    pc3x=Pc_val(3,1); pc3y=Pc_val(3,2); pc3z=Pc_val(3,3);
    pc4x=Pc_val(4,1); pc4y=Pc_val(4,2); pc4z=Pc_val(4,3);
    pc5x=Pc_val(5,1); pc5y=Pc_val(5,2); pc5z=Pc_val(5,3);
    pc6x=Pc_val(6,1); pc6y=Pc_val(6,2); pc6z=Pc_val(6,3);
    
    res = zeros(n_points * 3, 1);
    
    for k = 1:n_points
        q_cmd_k = q_cmd(k, :)';
        
        % Approximate gravity torque at nominal configuration
        q_approx = q_cmd_k - dtheta;
        s_approx = signs(:).*q_approx + offsets(:);
        s1 = s_approx(1); s2 = s_approx(2); s3 = s_approx(3); s4 = s_approx(4); s5 = s_approx(5); s6 = s_approx(6);
        Torques;
        G = [T1, -T2, -T3, -T4, T5, -T6]';
        
        % Calculate deflections
        dq_pred = zeros(6, 1);
        for j = 2:5
            dq_pred(j) = S(j-1)*G(j) + cc(j-1)*sign(G(j)) + coff(j-1);
        end
        
        % Aligned joint angle
        q_real_k = q_cmd_k - dq_pred - dtheta;
        s_real = signs(:).*q_real_k + offsets(:);
        
        % Forward Kinematics
        p_calc_fk = eval_fk(s_real, a2, a3, d1, d4, d5, d6, alpha)';
        
        % Apply base alignment transform
        p_calc = R_base * p_calc_fk + t_base;
        
        % Residual in mm
        idx = (k-1)*3 + (1:3);
        res(idx) = (p_calc - p_real(k, :)') * 1000;
    end
    
    % Prior regularization to keep parameters close to nominal
    p_nom = [m_nom, Pc_nom(:)'];
    p_val = [m_val, Pc_val(:)'];
    res = [res; 1e-4 * (p_val - p_nom)'];
    res = [res; 1e-4 * dtheta * 1000];
    res = [res; 1e-4 * [da2, da3, dd1, dd4, dd5, dd6]' * 1000];
    res = [res; 1e-4 * [base_rx, base_ry, base_rz, base_tx, base_ty, base_tz]' * 1000];
end

function errs = eval_errors(x, q_cmd, p_real, n_points, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets)
    dtheta = x(1:6)';
    da2 = x(7); da3 = x(8); dd1 = x(9); dd4 = x(10); dd5 = x(11); dd6 = x(12);
    S = x(13:16);
    cc = x(17:20);
    coff = x(21:24);
    
    m_val = x(25:30);
    Pc_val = reshape(x(31:48), 6, 3);
    
    base_rx = x(49); base_ry = x(50); base_rz = x(51);
    base_tx = x(52); base_ty = x(53); base_tz = x(54);
    
    cx = cos(base_rx); sx = sin(base_rx);
    cy = cos(base_ry); sy = sin(base_ry);
    cz = cos(base_rz); sz = sin(base_rz);
    Rx = [1 0 0; 0 cx -sx; 0 sx cx];
    Ry = [cy 0 sy; 0 1 0; -sy 0 cy];
    Rz = [cz -sz 0; sz cz 0; 0 0 1];
    R_base = Rx * Ry * Rz;
    t_base = [base_tx; base_ty; base_tz];
    
    a2 = a2_nom + da2; a3 = a3_nom + da3;
    d1 = d1_nom + dd1; d4 = d4_nom + dd4; d5 = d5_nom + dd5; d6 = d6_nom + dd6;
    
    ds1=0; ds2=0; ds3=0; ds4=0; ds5=0; ds6=0;
    dds1=0; dds2=0; dds3=0; dds4=0; dds5=0; dds6=0;
    I1xx=0; I1yy=0; I1zz=0; I2xx=0; I2yy=0; I2zz=0; I3xx=0; I3yy=0; I3zz=0;
    I4xx=0; I4yy=0; I4zz=0; I5xx=0; I5yy=0; I5zz=0; I6xx=0; I6yy=0; I6zz=0;
    g=-9.81;
    m1=m_val(1); m2=m_val(2); m3=m_val(3); m4=m_val(4); m5=m_val(5); m6=m_val(6);
    pc1x=Pc_val(1,1); pc1y=Pc_val(1,2); pc1z=Pc_val(1,3);
    pc2x=Pc_val(2,1); pc2y=Pc_val(2,2); pc2z=Pc_val(2,3);
    pc3x=Pc_val(3,1); pc3y=Pc_val(3,2); pc3z=Pc_val(3,3);
    pc4x=Pc_val(4,1); pc4y=Pc_val(4,2); pc4z=Pc_val(4,3);
    pc5x=Pc_val(5,1); pc5y=Pc_val(5,2); pc5z=Pc_val(5,3);
    pc6x=Pc_val(6,1); pc6y=Pc_val(6,2); pc6z=Pc_val(6,3);
    
    errs = zeros(n_points, 1);
    for k = 1:n_points
        q_cmd_k = q_cmd(k, :)';
        q_approx = q_cmd_k - dtheta;
        s_approx = signs(:).*q_approx + offsets(:);
        s1 = s_approx(1); s2 = s_approx(2); s3 = s_approx(3); s4 = s_approx(4); s5 = s_approx(5); s6 = s_approx(6);
        Torques;
        G = [T1, -T2, -T3, -T4, T5, -T6]';
        
        dq_pred = zeros(6, 1);
        for j = 2:5
            dq_pred(j) = S(j-1)*G(j) + cc(j-1)*sign(G(j)) + coff(j-1);
        end
        
        q_real_k = q_cmd_k - dq_pred - dtheta;
        s_real = signs(:).*q_real_k + offsets(:);
        p_calc_fk = eval_fk(s_real, a2, a3, d1, d4, d5, d6, alpha)';
        p_calc = R_base * p_calc_fk + t_base;
        
        errs(k) = norm(p_calc - p_real(k, :)') * 1000;
    end
end

function p = eval_fk(s, a2, a3, d1, d4, d5, d6, alpha)
    DH = [
        0,   alpha(1),  s(1), d1;
        a2,  0,         s(2), 0;
        a3,  0,         s(3), 0;
        0,   alpha(4),  s(4), d4;
        0,   alpha(5),  s(5), d5;
        0,   alpha(6),  s(6), d6
    ];
    
    Ai_fn = @(a, alphai, theta, d) [
        cos(theta), -sin(theta)*cos(alphai),  sin(theta)*sin(alphai), a*cos(theta);
        sin(theta),  cos(theta)*cos(alphai), -cos(theta)*sin(alphai), a*sin(theta);
        0,           sin(alphai),             cos(alphai),            d;
        0,           0,                      0,                     1
    ];
    
    T = eye(4);
    for i = 1:6
        T = T * Ai_fn(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    end
    p = T(1:3, 4)';
end
