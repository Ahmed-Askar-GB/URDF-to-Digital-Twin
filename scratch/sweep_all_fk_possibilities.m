% sweep_all_fk_possibilities.m
clc; clear;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');

% Load dataset
data = readmatrix('UR5_dataset.csv');
row = data(2, :);
q_deg = row(1:6);
p_nom_data = row(7:9); % nominal position in mm

fprintf('Target nominal position (mm): %s\n', num2str(p_nom_data));

% Nominal DH lengths
a2 = 0.425; a3 = 0.39225;
d1 = 0.089159; d5 = 0.09465; d6 = 0.0823;

% Let us try different values of d4: [0.10915, -0.10915]
d4_candidates = [0.10915, -0.10915];

% Let us run a randomized or systematic sweep on the signs and offsets
% to see if we can find a perfect match (error < 0.1 mm)
q_rad = q_deg * pi/180;

best_err = 1e9;
best_p = [];
best_params = struct();

% Loop over:
% - signs for each joint s_i = +/- 1
% - offsets for each joint theta_off_i = [0, pi/2, -pi/2, pi]
% - alpha twist angles
% To make it computationally feasible, let us sweep:
% signs for all joints: 2^6 = 64 combinations
% offsets for joints 2, 3, 4: 3^3 = 27 combinations (since joints 1, 5, 6 rarely have offsets other than 0 or pi)
% alpha twist signs: 2^3 = 8 combinations

s_comb = dec2bin(0:63) - '0';
s_comb(s_comb == 0) = -1; % 64 x 6 matrix

alpha_signs = [
    pi/2,  0, 0,  pi/2, -pi/2, 0;
    pi/2,  0, 0,  pi/2,  pi/2, 0;
   -pi/2,  0, 0, -pi/2, -pi/2, 0;
    pi/2,  0, 0, -pi/2, -pi/2, 0;
];

offsets_pool = [0, -pi/2, pi/2, pi];

for d4_val = d4_candidates
    for a_idx = 1:size(alpha_signs, 1)
        alpha = alpha_signs(a_idx, :);
        for s_idx = 1:size(s_comb, 1)
            signs = s_comb(s_idx, :);
            
            % Sweep offsets for joint 2, 3, 4
            for o2 = offsets_pool
                for o3 = offsets_pool
                    for o4 = offsets_pool
                        for o1 = [0, pi]
                            s_test = [
                                signs(1)*q_rad(1) + o1;
                                signs(2)*q_rad(2) + o2;
                                signs(3)*q_rad(3) + o3;
                                signs(4)*q_rad(4) + o4;
                                signs(5)*q_rad(5);
                                signs(6)*q_rad(6)
                            ];
                            
                            p_test = eval_fk(s_test, a2, a3, d1, d4_val, d5, d6, alpha) * 1000;
                            err = norm(p_test - p_nom_data);
                            if err < best_err
                                best_err = err;
                                best_p = p_test;
                                best_params.signs = signs;
                                best_params.offsets = [o1, o2, o3, o4];
                                best_params.alpha = alpha;
                                best_params.d4 = d4_val;
                                
                                if best_err < 0.1
                                    break;
                                end
                            end
                        end
                    end
                end
            end
            if best_err < 0.1, break; end
        end
        if best_err < 0.1, break; end
    end
    if best_err < 0.1, break; end
end

fprintf('Sweep completed.\n');
fprintf('Min Error: %.4f mm\n', best_err);
if best_err < 10.0
    fprintf('Calculated position: %s\n', num2str(best_p));
    fprintf('d4: %.5f\n', best_params.d4);
    fprintf('Signs: %s\n', num2str(best_params.signs));
    fprintf('Offsets (o1, o2, o3, o4): [%.4f, %.4f, %.4f, %.4f]\n', best_params.offsets);
    fprintf('Alpha: %s\n', num2str(best_params.alpha));
else
    disp('No good match found. We need to check if the parameters used are different.');
end

function p = eval_fk(s, a2, a3, d1, d4, d5, d6, alpha)
    DH = [
        0,   alpha(1),  s(1), d1;
        a2,  alpha(2),  s(2), 0;
        a3,  alpha(3),  s(3), 0;
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
