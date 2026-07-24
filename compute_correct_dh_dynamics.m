% compute_correct_dh_dynamics.m
clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

% 1. Extract URDF parameters using the existing parser
urdf_file = 'universalUR5.urdf';
[DH_extracted, P_Ci_extracted, I_extracted, m] = urdf_to_dh_matlab(urdf_file);

% 2. Define standard DH parameters for UR5
d1 = 0.089159;
a2 = 0.425;
a3 = 0.39225;
d4 = 0.10915; % Corrected value
d5 = 0.09465;
d6 = 0.0823;

% Standard DH parameters at q = 0
% Columns: a, alpha, theta, d
DH_standard = [
    0,   pi/2,  0, d1;
    a2,  0,     0, 0;
    a3,  0,     0, 0;
    0,   pi/2,  0, d4;
    0,  -pi/2,  0, d5;
    0,   0,     0, d6
];

% Homogeneous transformation helper function (Standard DH)
Ai_fn = @(a, alpha, theta, d) [
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1
];

% Compute global standard DH frames at q = 0
T_dh = cell(6, 1);
T_accum = eye(4);
for i = 1:6
    T_accum = T_accum * Ai_fn(DH_standard(i,1), DH_standard(i,2), DH_standard(i,3), DH_standard(i,4));
    T_dh{i} = T_accum;
end

% We also need the global URDF frames at q = 0.
% Let's load the URDF structure directly using the parser's logic
% to get exact global COM and global Inertia.
xmlData = xmlread(urdf_file);

% Extract links mass and com in URDF frame
links = struct();
linkNodes = xmlData.getElementsByTagName('link');
for i = 0:linkNodes.getLength()-1
    node = linkNodes.item(i);
    name = char(node.getAttribute('name'));
    mass = 0; com = [0;0;0]; inertia = zeros(3,3);
    
    ineNode = node.getElementsByTagName('inertial');
    if ineNode.getLength() > 0
        ine = ineNode.item(0);
        mNode = ine.getElementsByTagName('mass');
        if mNode.getLength() > 0
            mass = str2double(char(mNode.item(0).getAttribute('value')));
        end
        oNode = ine.getElementsByTagName('origin');
        if oNode.getLength() > 0
            xyzStr = char(oNode.item(0).getAttribute('xyz'));
            if ~isempty(xyzStr), com = str2num(xyzStr)'; end
            rpyStr = char(oNode.item(0).getAttribute('rpy'));
            if ~isempty(rpyStr)
                rpy_vals = str2num(rpyStr);
                R_ine = rpy_to_R(rpy_vals);
            else
                R_ine = eye(3);
            end
        else
            R_ine = eye(3);
        end
        iNode = ine.getElementsByTagName('inertia');
        if iNode.getLength() > 0
            ixx = str2double(char(iNode.item(0).getAttribute('ixx')));
            ixy = str2double(char(iNode.item(0).getAttribute('ixy')));
            ixz = str2double(char(iNode.item(0).getAttribute('ixz')));
            iyy = str2double(char(iNode.item(0).getAttribute('iyy')));
            iyz = str2double(char(iNode.item(0).getAttribute('iyz')));
            izz = str2double(char(iNode.item(0).getAttribute('izz')));
            raw_I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
            inertia = R_ine * raw_I * R_ine';
        end
    end
    links.(name) = struct('mass', mass, 'com', com, 'inertia', inertia);
end

% We define the joint chain
joint_names = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
parent_links = {'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link'};
child_links = {'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'};

% Compute global URDF frames at q = 0
T_urdf = cell(6, 1);
T_urdf_accum = eye(4);

% URDF joint definitions
joint_xyz = [
    0, 0, 0.089159;
    0, 0.13585, 0;
    0, -0.1197, 0.425;
    0, 0, 0.39225;
    0, 0.093, 0;
    0, 0, 0.09465
];

joint_rpy = [
    0, 0, 0;
    0, 1.57079632679, 0;
    0, 0, 0;
    0, 1.57079632679, 0;
    0, 0, 0;
    0, 0, 0
];

for i = 1:6
    T_j = eye(4);
    T_j(1:3, 1:3) = rpy_to_R(joint_rpy(i, :));
    T_j(1:3, 4) = joint_xyz(i, :)';
    T_urdf_accum = T_urdf_accum * T_j;
    T_urdf{i} = T_urdf_accum;
end

% Compute standard DH parameters for COM and Inertia
P_Ci_standard = zeros(6, 3);
I_standard = zeros(3, 3, 6);

for i = 1:6
    child_name = child_links{i};
    com_urdf = links.(child_name).com;
    
    % Global COM of link i
    p_com_global = T_urdf{i} * [com_urdf; 1];
    p_com_global = p_com_global(1:3);
    
    % Origin of standard DH frame i-1
    if i == 1
        o_prev_dh = [0; 0; 0];
    else
        o_prev_dh = T_dh{i-1}(1:3, 4);
    end
    
    % R of standard DH frame i
    R_curr_dh = T_dh{i}(1:3, 1:3);
    
    % P_Ci: relative to frame i-1 expressed in frame i
    P_Ci_standard(i, :) = (R_curr_dh' * (p_com_global - o_prev_dh))';
    
    % Transform inertia tensor to match standard DH frame i
    R_urdf_to_dh = R_curr_dh' * T_urdf{i}(1:3, 1:3);
    I_urdf = links.(child_name).inertia;
    I_standard(:, :, i) = R_urdf_to_dh * I_urdf * R_urdf_to_dh';
end

disp('=== COMPUTED STANDARD DH COM VECTORS (P_Ci) ===');
for i = 1:6
    fprintf('Pc%d = [%.6f, %.6f, %.6f];\n', i, P_Ci_standard(i, 1), P_Ci_standard(i, 2), P_Ci_standard(i, 3));
end

disp('=== COMPUTED STANDARD DH DIAGONAL INERTIAS ===');
for i = 1:6
    I_diag = diag(I_standard(:, :, i));
    fprintf('I%d = [%.6e, %.6e, %.6e];\n', i, I_diag(1), I_diag(2), I_diag(3));
end

% Helper function
function R = rpy_to_R(rpy)
    roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
end
