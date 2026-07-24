% test_ik.m
clc; clear;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');

% Load dataset
data = readmatrix('UR5_dataset.csv');
q_cmd_deg = data(2:end, 1:6);
p_nom_mm = data(2:end, 7:9);
p_real_mm = data(2:end, 13:15);

n_points = size(q_cmd_deg, 1);
fprintf('Total data points: %d\n', n_points);

% Fixed DH parameters from URDF
a2 = 0.425; a3 = 0.39225; a5 = 0; a6 = 0;
d1 = 0.089159; d5 = 0.09465; d6 = 0.0823;
d4 = -0.10915; % negative d4 aligns standard DH with URDF Joint 5

alpha = [-pi/2, 0, 0, -pi/2, -pi/2, 0];
signs = [1, -1, -1, -1, 1, -1];
offsets = [0, pi, 0, pi, 0, 0];

% Forward kinematics function
fk_fn = @(s) eval_fk(s, a2, a3, d1, d4, d5, d6, alpha);

q_real_deg = zeros(n_points, 6);
cart_err_before = zeros(n_points, 1);
cart_err_after = zeros(n_points, 1);

options = optimoptions('lsqnonlin', 'Display', 'none', 'FunctionTolerance', 1e-12, 'StepTolerance', 1e-12);

fprintf('Running Numerical IK for all points...\n');
tic;
for i = 1:n_points
    % Nominal and target positions in meters
    p_nom = p_nom_mm(i, :)' / 1000;
    p_real = p_real_mm(i, :)' / 1000;
    
    q_cmd_i = q_cmd_deg(i, :)'*pi/180;
    s_init = signs(:).*q_cmd_i + offsets(:);
    
    % Check FK before optimization
    p_calc_init = fk_fn(s_init)';
    cart_err_before(i) = norm(p_calc_init - p_real)*1000; % in mm
    
    % Define optimization function: match target position, regularize to stay near nominal joints
    % We use lambda = 1e-6 to resolve joint redundancy
    lambda = 1e-6;
    opt_fn = @(s) [ (fk_fn(s)' - p_real)*1000; lambda*(s - s_init) ];
    
    [s_opt, ~] = lsqnonlin(opt_fn, s_init, [], [], options);
    
    p_calc_opt = fk_fn(s_opt)';
    cart_err_after(i) = norm(p_calc_opt - p_real)*1000; % in mm
    
    % Map back to joint coordinates (q = (s - offsets) ./ signs)
    q_real_deg(i, :) = ((s_opt - offsets(:)) ./ signs(:))' * 180/pi;
end
toc;

dq_deg = q_cmd_deg - q_real_deg;

fprintf('\nIK Results:\n');
fprintf('  Mean Cartesian error before IK: %.4f mm\n', mean(cart_err_before));
fprintf('  Mean Cartesian error after IK:  %.4e mm\n', mean(cart_err_after));
fprintf('  Max Cartesian error after IK:   %.4e mm\n', max(cart_err_after));

fprintf('\nTracking Error (q_cmd - q_real) statistics (deg):\n');
for j = 1:6
    fprintf('  Joint %d: Mean = %.4f, Std = %.4f, Min = %.4f, Max = %.4f\n', ...
        j, mean(dq_deg(:, j)), std(dq_deg(:, j)), min(dq_deg(:, j)), max(dq_deg(:, j)));
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
