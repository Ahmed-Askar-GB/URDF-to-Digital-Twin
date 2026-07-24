% check_dataset_fk.m
clc; clear;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');

% Load dataset
data = readmatrix('UR5_dataset.csv');

% Extract second row (first actual data row)
row = data(2, :);
q_deg = row(1:6);
p_nom_data = row(7:9); % in mm
p_real_data = row(13:15); % in mm

fprintf('Dataset row 2 Joint Angles (deg): %s\n', num2str(q_deg));
fprintf('Dataset row 2 Nominal Position (mm): %s\n', num2str(p_nom_data));
fprintf('Dataset row 2 Real Position (mm): %s\n', num2str(p_real_data));

% Nominal DH parameters of UR5
a2 = 0.425; a3 = 0.39225; a5 = 0; a6 = 0;
d1 = 0.089159; d5 = 0.09465; d6 = 0.0823;

% Forward kinematics function using standard DH
fk_fn = @(s1, s2, s3, s4, s5, s6, d4_val) ...
    eval_fk(s1, s2, s3, s4, s5, s6, a2, a3, d1, d4_val, d5, d6);

% Let's test different sign mappings of the joint angles:
q_rad_raw = q_deg * pi/180;

% Let's test combinations of coordinate signs to find the one matching the nominal dataset [x, y, z]
% We will check if one of the standard mappings matches:
% Mapping 1: q_rad_raw
p_1_pos = fk_fn(q_rad_raw(1), q_rad_raw(2), q_rad_raw(3), q_rad_raw(4), q_rad_raw(5), q_rad_raw(6), 0.10915) * 1000;
p_1_neg = fk_fn(q_rad_raw(1), q_rad_raw(2), q_rad_raw(3), q_rad_raw(4), q_rad_raw(5), q_rad_raw(6), -0.10915) * 1000;

% Mapping 2: standard DH mapped (s2, s3, s4, s6 negated)
q_rad_map = [q_rad_raw(1), -q_rad_raw(2), -q_rad_raw(3), -q_rad_raw(4), q_rad_raw(5), -q_rad_raw(6)];
p_2_pos = fk_fn(q_rad_map(1), q_rad_map(2), q_rad_map(3), q_rad_map(4), q_rad_map(5), q_rad_map(6), 0.10915) * 1000;
p_2_neg = fk_fn(q_rad_map(1), q_rad_map(2), q_rad_map(3), q_rad_map(4), q_rad_map(5), q_rad_map(6), -0.10915) * 1000;

% Mapping 3: Another mapping (e.g. s2, s3, s4 negated, but s6 not negated)
q_rad_map3 = [q_rad_raw(1), -q_rad_raw(2), -q_rad_raw(3), -q_rad_raw(4), q_rad_raw(5), q_rad_raw(6)];
p_3_pos = fk_fn(q_rad_map3(1), q_rad_map3(2), q_rad_map3(3), q_rad_map3(4), q_rad_map3(5), q_rad_map3(6), 0.10915) * 1000;
p_3_neg = fk_fn(q_rad_map3(1), q_rad_map3(2), q_rad_map3(3), q_rad_map3(4), q_rad_map3(5), q_rad_map3(6), -0.10915) * 1000;

% Mapping 4: standard DH with offset mapping
% Sometimes standard DH includes offset theta_2 = theta_2 - pi/2, etc.
% Let's search for a combination of sign flips and offsets that yields error < 1 mm
best_err = 1e9;
best_p = [];
best_map = [];
best_d4 = 0;
signs = [1, -1];
for s1_sign = signs
    for s2_sign = signs
        for s3_sign = signs
            for s4_sign = signs
                for s5_sign = signs
                    for s6_sign = signs
                        for theta2_off = [0, -pi/2, pi/2]
                            for theta3_off = [0]
                                for theta4_off = [0, -pi/2, pi/2]
                                    for d4_test = [0.10915, -0.10915]
                                        s_test = [
                                            s1_sign*q_rad_raw(1);
                                            s2_sign*q_rad_raw(2) + theta2_off;
                                            s3_sign*q_rad_raw(3) + theta3_off;
                                            s4_sign*q_rad_raw(4) + theta4_off;
                                            s5_sign*q_rad_raw(5);
                                            s6_sign*q_rad_raw(6)
                                        ];
                                        p_test = fk_fn(s_test(1), s_test(2), s_test(3), s_test(4), s_test(5), s_test(6), d4_test) * 1000;
                                        err = norm(p_test - p_nom_data);
                                        if err < best_err
                                            best_err = err;
                                            best_p = p_test;
                                            best_map = [s1_sign, s2_sign, s3_sign, s4_sign, s5_sign, s6_sign, theta2_off, theta4_off];
                                            best_d4 = d4_test;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

fprintf('\nBest match found:\n');
fprintf('  Calculated position: %s\n', num2str(best_p));
fprintf('  Error: %.4f mm\n', best_err);
fprintf('  d4: %.5f\n', best_d4);
fprintf('  Signs (s1..s6): %s\n', num2str(best_map(1:6)));
fprintf('  Offsets (theta2, theta4): [%.4f, %.4f]\n', best_map(7), best_map(8));

function p = eval_fk(s1, s2, s3, s4, s5, s6, a2, a3, d1, d4, d5, d6)
    DH = [
        0,   pi/2,  s1, d1;
        a2,  0,     s2, 0;
        a3,  0,     s3, 0;
        0,   pi/2,  s4, d4;
        0,  -pi/2,  s5, d5;
        0,   0,     s6, d6
    ];
    
    Ai_fn = @(a, alpha, theta, d) [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),            d;
        0,           0,                      0,                     1
    ];
    
    T = eye(4);
    for i = 1:6
        T = T * Ai_fn(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    end
    p = T(1:3, 4)';
end
