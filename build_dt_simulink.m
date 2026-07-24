% build_dt_simulink.m
% Programmatic Simscape Multibody Model Generator for Calibrated Robot
%
% This script builds a complete 6-DOF Simscape Multibody model in Simulink
% relying ONLY on the numerical kinematics (DH) and identified dynamic parameters,
% without any reference or dependency on the original URDF file.

clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');

model_name = 'dt_simscape_robot';
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Load the required Simscape and Simulink libraries into memory
fprintf('Loading Simscape Multibody libraries...\n');
load_system('sm_lib');
load_system('nesl_utility');

% 1. DEFINE NUMERICAL PARAMETERS (Relying only on identified digital twin values)
% Aligned Standard DH Kinematic parameters
a = [0, 0.425, 0.39225, 0, 0, 0];
alpha = [-pi/2, 0, 0, -pi/2, -pi/2, 0];
d = [0.089159, 0, 0, -0.10915, 0.09465, 0.0823];

% Identified Dynamic parameters (Mass and Center of Mass coordinates)
m = [3.7000, 8.3951, 2.4542, 1.0536, 1.0434, 0.1503];
Pc = [
   -0.0000, -0.0108, -0.0002;
    0.2800, -0.1000, -0.0619;
    0.2500, -0.0000,  0.0013;
   -0.1000, -0.0023,  0.0021;
   -0.1000,  0.0000, -0.0014;
   -0.0000, -0.0742,  0.0046
];

% Approximate Diagonal Inertia Tensors (unobservable in static data but needed for simulation)
I_tensors = {
    diag([0.0084, 0.0084, 0.0044]); % Link 1
    diag([0.0031, 0.2375, 0.2375]); % Link 2
    diag([0.0023, 0.0955, 0.0955]); % Link 3
    diag([0.0016, 0.0016, 0.0008]); % Link 4
    diag([0.0016, 0.0016, 0.0008]); % Link 5
    diag([0.0001, 0.0001, 0.0001])  % Link 6
};

% 2. CREATE NEW SIMULINK SYSTEM
fprintf('Creating new Simulink system: %s.slx...\n', model_name);
new_system(model_name);
open_system(model_name);

% 3. ADD SIMSCAPE SYSTEM SOLVER AND WORLD COORD SYSTEM
world_path = [model_name, '/World Frame'];
solver_path = [model_name, '/Solver Configuration'];
mech_path = [model_name, '/Mechanism Configuration'];

h_wf = add_block('sm_lib/Frames and Transforms/World Frame', world_path, 'Position', [100, 100, 140, 140]);
h_sc = add_block('nesl_utility/Solver Configuration', solver_path, 'Position', [100, 200, 140, 240]);
h_mc = add_block('sm_lib/Utilities/Mechanism Configuration', mech_path, 'Position', [100, 300, 140, 340]);

% Get Port Handles for precise connection routing
ports_wf = get_param(h_wf, 'PortHandles');
ports_sc = get_param(h_sc, 'PortHandles');
ports_mc = get_param(h_mc, 'PortHandles');

% Connect solver and world configuration blocks
add_line(model_name, ports_wf.RConn(1), ports_sc.RConn(1), 'autorouting', 'on');
add_line(model_name, ports_wf.RConn(1), ports_mc.RConn(1), 'autorouting', 'on');

last_port = ports_wf.RConn(1);
x_pos = 250;

% 4. PROGRAMMATICALLY BUILD KINEMATIC AND DYNAMIC CHAIN
for i = 1:6
    fprintf('Building Link %d and Joint %d...\n', i, i);
    
    % A. Add Joint-to-Joint Transform based on DH Parameters (Rotation & Translation)
    tf_joint_name = sprintf('DH_Transform_Joint_%d', i);
    tf_joint_path = [model_name, '/', tf_joint_name];
    h_tf_joint = add_block('sm_lib/Frames and Transforms/Rigid Transform', tf_joint_path, 'Position', [x_pos, 100, x_pos+60, 160]);
    ports_tf_joint = get_param(h_tf_joint, 'PortHandles');
    
    % Configure DH translation (a_i along X, d_i along Z)
    set_param(tf_joint_path, 'TranslationMethod', 'Cartesian');
    set_param(tf_joint_path, 'TranslationCartesianOffset', sprintf('[%f, 0, %f]', a(i), d(i)));
    set_param(tf_joint_path, 'TranslationCartesianOffsetUnits', 'm');
    
    % Configure DH rotation (alpha_i along X)
    set_param(tf_joint_path, 'RotationMethod', 'StandardAxis');
    set_param(tf_joint_path, 'RotationStandardAxis', '+X');
    set_param(tf_joint_path, 'RotationAngle', sprintf('%f', alpha(i)));
    set_param(tf_joint_path, 'RotationAngleUnits', 'rad');
    
    % Connect previous frame port to B port (LConn(1)) of Rigid Transform
    add_line(model_name, last_port, ports_tf_joint.LConn(1), 'autorouting', 'on');
    last_port = ports_tf_joint.RConn(1);
    x_pos = x_pos + 120;
    
    % B. Add Revolute Joint
    joint_name = sprintf('Revolute_Joint_%d', i);
    joint_path = [model_name, '/', joint_name];
    h_joint = add_block('sm_lib/Joints/Revolute Joint', joint_path, 'Position', [x_pos, 100, x_pos+50, 150]);
    ports_joint = get_param(h_joint, 'PortHandles');
    
    % Connect Rigid Transform to Revolute Joint Base port (B - LConn(1))
    add_line(model_name, last_port, ports_joint.LConn(1), 'autorouting', 'on');
    last_port = ports_joint.RConn(1);
    x_pos = x_pos + 100;
    
    % C. Add Link Inertia Block (Point Mass + Inertia Tensor at Center of Mass)
    tf_com_name = sprintf('Transform_CoM_Link_%d', i);
    tf_com_path = [model_name, '/', tf_com_name];
    h_tf_com = add_block('sm_lib/Frames and Transforms/Rigid Transform', tf_com_path, 'Position', [x_pos, 100, x_pos+60, 160]);
    ports_tf_com = get_param(h_tf_com, 'PortHandles');
    
    % Configure Translation to identified Center of Mass CoM
    set_param(tf_com_path, 'TranslationMethod', 'Cartesian');
    set_param(tf_com_path, 'TranslationCartesianOffset', sprintf('[%f, %f, %f]', Pc(i,1), Pc(i,2), Pc(i,3)));
    set_param(tf_com_path, 'TranslationCartesianOffsetUnits', 'm');
    
    % Connect Joint follower to CoM Transform Base (B - LConn(1))
    add_line(model_name, last_port, ports_tf_com.LConn(1), 'autorouting', 'on');
    
    x_pos = x_pos + 100;
    
    % Add Inertia Block
    inertia_name = sprintf('Inertia_Link_%d', i);
    inertia_path = [model_name, '/', inertia_name];
    h_inertia = add_block('sm_lib/Body Elements/Inertia', inertia_path, 'Position', [x_pos, 30, x_pos+50, 80]);
    ports_inertia = get_param(h_inertia, 'PortHandles');
    
    % Configure mass, center of mass, and custom inertia tensor values
    set_param(inertia_path, 'Mass', sprintf('%f', m(i)));
    set_param(inertia_path, 'MassUnits', 'kg');
    set_param(inertia_path, 'CenterOfMass', '[0, 0, 0]');
    set_param(inertia_path, 'CenterOfMassUnits', 'm');
    I_val = I_tensors{i};
    set_param(inertia_path, 'MomentsOfInertia', sprintf('[%f, %f, %f]', I_val(1,1), I_val(2,2), I_val(3,3)));
    set_param(inertia_path, 'MomentsOfInertiaUnits', 'kg*m^2');
    set_param(inertia_path, 'ProductsOfInertia', '[0, 0, 0]');
    set_param(inertia_path, 'ProductsOfInertiaUnits', 'kg*m^2');
    
    % Connect CoM Transform follower (F - RConn(1)) to Inertia block Reference port (RConn(1))
    add_line(model_name, ports_tf_com.RConn(1), ports_inertia.RConn(1), 'autorouting', 'on');
    
    x_pos = x_pos + 80;
end

% 5. SAVE MODEL
fprintf('Saving programmatically generated Simscape model...\n');
save_system(model_name);
fprintf('Successfully generated and saved %s.slx!\n', model_name);
