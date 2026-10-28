%% run_track1_sym.m
% =========================================================================
% TRACK 1: Pure MATLAB Symbolic Robot Dynamics & Glass-Box Inspection
% =========================================================================
% Demonstrates symbolic forward kinematics, geometric Jacobians, generalized
% mass matrix M(q), Christoffel symbols c_{ijk}(q), and gravity vector G(q)
% using the native MATLAB Symbolic Math Toolbox.
%
% This track prioritizes maximum algebraic inspectability and pedagogical
% clarity, allowing direct analytical verification of textbook derivations.
% =========================================================================

function results = run_track1_sym(urdf_path)
    if nargin < 1 || isempty(urdf_path)
        this_dir = fileparts(mfilename('fullpath'));
        root_dir = fileparts(fileparts(this_dir));
        urdf_path = fullfile(root_dir, 'universalUR5.urdf');
    end

    this_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(fileparts(this_dir));

    % Add paths to parsing utilities from MATLAB implementation
    matlab_impl_dir = fullfile(root_dir, 'End-to-End MATLAB Implementation');
    addpath(this_dir);
    addpath(fullfile(matlab_impl_dir, 'parsing'));
    addpath(fullfile(matlab_impl_dir, 'utils'));

    fprintf('\n====================================================================\n');
    fprintf('  TRACK 1: PURE MATLAB SYMBOLIC DYNAMICS (GLASS-BOX INSPECTABILITY) \n');
    fprintf('====================================================================\n');
    fprintf('  Robot URDF : %s\n', urdf_path);

    % 1. Parse URDF and extract DH chain
    t0 = tic;
    robot   = urdf_parse(urdf_path);
    chain   = urdf_build_chain(robot, false);
    dh      = urdf_extract_dh(chain, false);
    inertia = urdf_extract_inertia(robot, chain, dh, false);
    t_parse = toc(t0);
    fprintf('  [OK] Parsed %d-DOF chain in %.3f s\n', chain.n, t_parse);

    % 2. Derive Symbolic Forward Kinematics
    t1 = tic;
    fk = fk_sym(dh, chain, struct('export_matlabFunction', true, 'verbose', false));
    t_fk = toc(t1);
    fprintf('  [OK] Symbolic Forward Kinematics derived in %.3f s\n', t_fk);

    % 3. Derive Symbolic Jacobians
    t2 = tic;
    jac = jacobian_sym(dh, inertia, chain, struct('export_matlabFunction', true, 'verbose', false));
    t_jac = toc(t2);
    fprintf('  [OK] Symbolic CoM & Geometric Jacobians derived in %.3f s\n', t_jac);

    % 4. Derive Symbolic Euler-Lagrange Dynamics
    t3 = tic;
    dyn = dynamics_sym(dh, inertia, chain, struct('export_matlabFunction', true, 'verbose', false));
    t_dyn = toc(t3);
    fprintf('  [OK] Symbolic Mass Matrix & Christoffel Symbols derived in %.3f s\n', t_dyn);

    % 5. Inspect Algebraic Expressions (Glass-Box Transparency)
    fprintf('\n--- Glass-Box Algebraic Inspection ---\n');
    fprintf('M(1,1)(q) expression length : %d characters\n', length(char(dyn.M_sym(1, 1))));
    fprintf('G(2)(q) expression length   : %d characters\n', length(char(dyn.G_sym(2))));
    fprintf('Sample M(1,1):\n%s\n\n', char(vpa(dyn.M_sym(1, 1), 4)));
    fprintf('Sample G(2):\n%s\n\n', char(vpa(dyn.G_sym(2), 4)));

    % 6. Numerical Benchmark at home configuration
    q_test = zeros(chain.n, 1);
    dq_test = zeros(chain.n, 1);
    ddq_test = zeros(chain.n, 1);

    t_eval = tic;
    for k = 1:1000
        M_val = dyn.M_fun(q_test);
        C_val = dyn.C_fun(q_test, dq_test);
        G_val = dyn.G_fun(q_test);
        tau_val = dyn.tau_fun(q_test, dq_test, ddq_test);
    end
    t_1000 = toc(t_eval);
    latency_us = (t_1000 / 1000) * 1e6;
    fprintf('  [BENCHMARK] Average single-step evaluation latency: %.2f microseconds\n', latency_us);

    results = struct();
    results.fk = fk;
    results.jac = jac;
    results.dyn = dyn;
    results.latency_us = latency_us;
    results.t_total_derivation = t_fk + t_jac + t_dyn;
    fprintf('====================================================================\n\n');
end
