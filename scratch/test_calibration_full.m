% test_calibration_full.m
clc; clear;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');

% Load dataset
data = readmatrix('UR5_dataset.csv');
q_cmd_deg = data(2:end, 1:6);
p_real_mm = data(2:end, 13:15);

n_points = size(q_cmd_deg, 1);

q_cmd = q_cmd_deg * pi/180;
p_real = p_real_mm / 1000;

% Nominal DH parameters
a2_nom = 0.425; a3_nom = 0.39225;
d1_nom = 0.089159; d4_nom = -0.10915; d5_nom = 0.09465; d6_nom = 0.0823;

alpha = [-pi/2, 0, 0, -pi/2, -pi/2, 0];
signs = [1, -1, -1, -1, 1, -1];
offsets = [0, pi, 0, pi, 0, 0];

% Split data 50/50
n_calib = floor(n_points / 2);
q_cmd_calib = q_cmd(1:n_calib, :);
p_real_calib = p_real(1:n_calib, :);

q_cmd_ver = q_cmd(n_calib+1:end, :);
p_real_ver = p_real(n_calib+1:end, :);

% Nominal Dynamic parameters
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
% Initialize masses and CoMs to nominal values
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

% Compliance and Friction bounds: S > 0, cc > 0
lb(13:20) = 0;
ub(13:16) = 1e-3; % compliance limit
ub(17:20) = 1.0;  % friction limit

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

options = optimoptions('lsqnonlin', 'Display', 'iter', 'Algorithm', 'levenberg-marquardt', 'FunctionTolerance', 1e-6, 'MaxIterations', 25);

fprintf('Optimizing kinematics, compliance, and base frame alignment on Calibration Set...\n');
tic;
[x_opt, resnorm] = lsqnonlin(@(x) cost_fn(x, q_cmd_calib, p_real_calib, n_calib, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets, m_nom, Pc_nom), x_init, lb, ub, options);
toc;

% Evaluate on Calibration and Verification sets
err_nom_calib = eval_errors(x_init, q_cmd_calib, p_real_calib, n_calib, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets);
err_opt_calib = eval_errors(x_opt, q_cmd_calib, p_real_calib, n_calib, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets);

n_ver = n_points - n_calib;
err_nom_ver = eval_errors(x_init, q_cmd_ver, p_real_ver, n_ver, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets);
err_opt_ver = eval_errors(x_opt, q_cmd_ver, p_real_ver, n_ver, a2_nom, a3_nom, d1_nom, d4_nom, d5_nom, d6_nom, alpha, signs, offsets);

fprintf('\nCalibration Results:\n');
fprintf('  Calibration Set Mean Error (Nominal):    %.4f mm\n', mean(err_nom_calib));
fprintf('  Calibration Set Mean Error (Calibrated): %.4f mm\n', mean(err_opt_calib));
fprintf('  Calibration Set Max Error (Calibrated):  %.4f mm\n', max(err_opt_calib));

fprintf('\nVerification Results:\n');
fprintf('  Verification Set Mean Error (Nominal):   %.4f mm\n', mean(err_nom_ver));
fprintf('  Verification Set Mean Error (Calibrated): %.4f mm\n', mean(err_opt_ver));
fprintf('  Verification Set Max Error (Calibrated):  %.4f mm\n', max(err_opt_ver));


% --- Cost Function ---
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
