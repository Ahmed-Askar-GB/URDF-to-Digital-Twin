%% run_track2_casadi.m
% =========================================================================
% TRACK 2: MATLAB / CasADi + Simscape Multibody Digital Twin Framework
% =========================================================================
% Orchestrates algorithmic differentiation (CasADi SX) and Simscape Multibody
% model generation for industrial Model-Based Design (MBD).
%
% Evaluates across four industrial manipulators:
%   1. Universal Robots UR5 (6-DOF)
%   2. Universal Robots UR16e (6-DOF Heavy Payload)
%   3. ABB IRB 120 (6-DOF Spherical Wrist)
%   4. Unimation Puma 560 (6-DOF Classic Benchmark)
% =========================================================================

function summary = run_track2_casadi(robot_select)
    if nargin < 1 || isempty(robot_select)
        robot_select = 'all'; % 'ur5', 'ur16e', 'abb', 'puma', or 'all'
    end

    this_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(fileparts(this_dir));
    casadi_impl_dir = fullfile(root_dir, 'End-to-End MATLAB-CasADi Implementation');

    % Set up paths
    addpath(casadi_impl_dir);
    addpath(fullfile(casadi_impl_dir, 'parsing'));
    addpath(fullfile(casadi_impl_dir, 'kinematics'));
    addpath(fullfile(casadi_impl_dir, 'dynamics'));
    addpath(fullfile(casadi_impl_dir, 'compilation'));
    addpath(fullfile(casadi_impl_dir, 'digital_twin'));
    addpath(fullfile(casadi_impl_dir, 'control'));
    addpath(fullfile(casadi_impl_dir, 'verification'));
    addpath(fullfile(casadi_impl_dir, 'utils'));

    robots = {
        'UR5',         fullfile(root_dir, 'universalUR5.urdf');
        'UR16e',       fullfile(casadi_impl_dir, 'test_UR16e', 'universalUR16e.urdf');
        'ABB_IRB120',  fullfile(root_dir, 'abbIrb120', 'abbIrb120.urdf');
        'Puma560',     fullfile(casadi_impl_dir, 'puma560_test', 'puma560_robot_dynamic.urdf')
    };

    fprintf('\n====================================================================\n');
    fprintf('  TRACK 2: MATLAB-CASADI + SIMSCAPE MULTIBODY DIGITAL TWIN PIPELINE \n');
    fprintf('====================================================================\n');

    summary = struct();

    for i = 1:size(robots, 1)
        name = robots{i, 1};
        urdf_path = robots{i, 2};

        if ~strcmpi(robot_select, 'all') && ~strcmpi(robot_select, name)
            continue;
        end

        fprintf('\n>>> Processing Robot [%s] <<<\n', name);
        fprintf('    URDF Path: %s\n', urdf_path);

        if ~exist(urdf_path, 'file')
            warning('URDF file not found for %s at %s. Skipping.', name, urdf_path);
            continue;
        end

        try
            t_start = tic;
            robot   = urdf_parse(urdf_path);
            chain   = urdf_build_chain(robot, false);
            dh      = urdf_extract_dh(chain, false);
            inertia = urdf_extract_inertia(robot, chain, dh, false);

            % CasADi Forward Kinematics & Jacobians
            fk  = fk_casadi(dh, chain, struct('verbose', false));
            jac = jacobian_casadi(dh, inertia, chain, struct('verbose', false));

            % CasADi Dynamics
            dyn = dynamics_casadi(dh, inertia, chain, struct('gen_c', false, 'verbose', false));
            t_elapsed = toc(t_start);

            % Latency benchmark
            q_test   = zeros(chain.n, 1);
            dq_test  = zeros(chain.n, 1);
            ddq_test = zeros(chain.n, 1);

            t_bench = tic;
            for k = 1:1000
                M_val = full(dyn.M_fun(q_test));
                C_val = full(dyn.C_fun(q_test, dq_test));
                G_val = full(dyn.G_fun(q_test));
                tau_val = full(dyn.tau_fun(q_test, dq_test, ddq_test));
            end
            t_eval = toc(t_bench);
            latency_us = (t_eval / 1000) * 1e6;

            summary.(name).n = chain.n;
            summary.(name).t_generation_s = t_elapsed;
            summary.(name).latency_us = latency_us;
            summary.(name).status = 'SUCCESS';

            fprintf('    [OK] %d-DOF model generated in %.3f s | Latency: %.2f us\n', ...
                chain.n, t_elapsed, latency_us);

        catch ME
            fprintf('    [ERROR] Failed for %s: %s\n', name, ME.message);
            summary.(name).status = 'FAILED';
            summary.(name).error = ME.message;
        end
    end
    fprintf('\n====================================================================\n\n');
end
