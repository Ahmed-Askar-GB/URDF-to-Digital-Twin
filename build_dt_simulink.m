% build_dt_simulink.m
% Programmatic Simscape Multibody Model Generator for Calibrated UR5 Robot
%
% Uses URDF joint origin transforms (xyz + rpy) so each STL mesh is correctly
% positioned/oriented in the kinematic chain.
% Rotation is set via ArbitraryAxis (axis-angle extracted from the RPY matrix),
% which is compatible with all MATLAB/Simscape Multibody versions.

clc; clear; close all;
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');

model_name = 'dt_simscape_robot';
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

fprintf('Loading Simscape Multibody libraries...\n');
load_system('sm_lib');
load_system('nesl_utility');

%% 1. URDF JOINT ORIGIN PARAMETERS
% Taken directly from universalUR5.urdf <joint><origin xyz rpy>
% Order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3

joint_xyz = [
    0.0,     0.0,     0.089159;   % shoulder_pan_joint
    0.0,     0.13585, 0.0;        % shoulder_lift_joint
    0.0,    -0.1197,  0.425;      % elbow_joint
    0.0,     0.0,     0.39225;    % wrist_1_joint
    0.0,     0.093,   0.0;        % wrist_2_joint
    0.0,     0.0,     0.09465;    % wrist_3_joint
];

joint_rpy = [
    0,   0,     0;      % shoulder_pan   — Z axis
    0,   pi/2,  0;      % shoulder_lift  — Y axis (URDF), rpy rotates frame
    0,   0,     0;      % elbow          — Y axis
    0,   pi/2,  0;      % wrist_1        — Y axis, rpy rotates frame
    0,   0,     0;      % wrist_2        — Z axis
    0,   0,     0;      % wrist_3        — Y axis
];

% URDF joint rotation axes (in joint origin frame)
% 1=Z, 2=Y, 3=Y, 4=Y, 5=Z, 6=Y
% Simscape Revolute Joint always rotates about local Z.
% For Y-axis joints we insert Rx(-90°) before and Rx(+90°) after the joint.
% pre_rpy / post_rpy: roll-pitch-yaw for the alignment rigid transforms
pre_rpy = [
    0,     0,  0;       % joint 1: Z-axis — no alignment
   -pi/2,  0,  0;       % joint 2: Y-axis — Rx(-90°)
   -pi/2,  0,  0;       % joint 3: Y-axis
   -pi/2,  0,  0;       % joint 4: Y-axis
    0,     0,  0;       % joint 5: Z-axis — no alignment
   -pi/2,  0,  0;       % joint 6: Y-axis
];

post_rpy = [
    0,    0,  0;        % joint 1
    pi/2, 0,  0;        % joint 2: Rx(+90°) restores frame
    pi/2, 0,  0;        % joint 3
    pi/2, 0,  0;        % joint 4
    0,    0,  0;        % joint 5
    pi/2, 0,  0;        % joint 6
];

%% 2. IDENTIFIED DYNAMIC PARAMETERS
m = [3.7000, 8.3951, 2.4542, 1.0536, 1.0434, 0.1503];

% CoM in each link's URDF frame (m) — from URDF inertial origins
Pc = [
    0.0,  0.0,   0.0;    % shoulder
    0.0,  0.0,   0.28;   % upper_arm
    0.0,  0.0,   0.25;   % forearm
    0.0,  0.0,   0.0;    % wrist_1
    0.0,  0.0,   0.0;    % wrist_2
    0.0,  0.0,   0.0;    % wrist_3
];

I_data = [
    0.010267, 0.010267, 0.006660;   % shoulder
    0.226891, 0.226891, 0.015107;   % upper_arm
    0.049443, 0.049443, 0.004095;   % forearm
    0.111173, 0.111173, 0.219420;   % wrist_1
    0.111173, 0.111173, 0.219420;   % wrist_2
    0.017136, 0.017136, 0.033822;   % wrist_3
];

%% 3. STL GEOMETRY FILENAMES
stl_files = {
    fullfile(pwd,'STL_Files','shoulder.stl');
    fullfile(pwd,'STL_Files','upperarm.stl');
    fullfile(pwd,'STL_Files','forearm.stl');
    fullfile(pwd,'STL_Files','wrist1.stl');
    fullfile(pwd,'STL_Files','wrist2.stl');
    fullfile(pwd,'STL_Files','wrist3.stl');
};
link_names = {'Shoulder','UpperArm','Forearm','Wrist1','Wrist2','Wrist3'};

%% 4. CREATE SIMULINK SYSTEM
fprintf('Creating Simulink system: %s.slx...\n', model_name);
new_system(model_name);
open_system(model_name);
set_param(model_name, 'StopTime', '5');

%% 5. WORLD FRAME / SOLVER / MECHANISM CONFIGURATION
h_wf = add_block('sm_lib/Frames and Transforms/World Frame', ...
    [model_name,'/World Frame'], 'Position',[50,200,90,240]);
h_sc = add_block('nesl_utility/Solver Configuration', ...
    [model_name,'/Solver Configuration'], 'Position',[50,60,90,100]);
h_mc = add_block('sm_lib/Utilities/Mechanism Configuration', ...
    [model_name,'/Mechanism Configuration'], 'Position',[50,320,90,360]);

p_wf = get_param(h_wf,'PortHandles');
p_sc = get_param(h_sc,'PortHandles');
p_mc = get_param(h_mc,'PortHandles');
add_line(model_name, p_wf.RConn(1), p_sc.RConn(1), 'autorouting','on');
add_line(model_name, p_wf.RConn(1), p_mc.RConn(1), 'autorouting','on');

%% 6. BASE LINK GEOMETRY (fixed to world)
h_base = add_block('sm_lib/Body Elements/File Solid', ...
    [model_name,'/Base_Geometry'], 'Position',[150,190,230,250]);
base_path = [model_name,'/Base_Geometry'];
set_param(base_path,'ExtGeomFileName', fullfile(pwd,'STL_Files','base.stl'));
set_param(base_path,'UnitType','Custom');
set_param(base_path,'ExtGeomFileUnits','m');
set_param(base_path,'InertiaType','Custom');
set_param(base_path,'Mass','4.0');
set_param(base_path,'MassUnits','kg');
set_param(base_path,'CenterOfMass','[0, 0, 0]');
set_param(base_path,'CenterOfMassUnits','m');
set_param(base_path,'MomentsOfInertia','[0.00443, 0.00443, 0.0072]');
set_param(base_path,'MomentsOfInertiaUnits','kg*m^2');
set_param(base_path,'ProductsOfInertia','[0, 0, 0]');
set_param(base_path,'ProductsOfInertiaUnits','kg*m^2');
p_base = get_param(h_base,'PortHandles');
add_line(model_name, p_wf.RConn(1), p_base.RConn(1), 'autorouting','on');

%% 7. BUILD KINEMATIC CHAIN
last_port = p_wf.RConn(1);
x_col = 280;
y_main = 200;

for i = 1:6
    fprintf('Building joint %d (%s)...\n', i, link_names{i});

    %-----------------------------------------------------------------
    % A. JOINT ORIGIN RIGID TRANSFORM  (URDF xyz + rpy)
    %-----------------------------------------------------------------
    tf_pre_path = [model_name, sprintf('/JointOrigin_%d',i)];
    h_tf_pre = add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
        tf_pre_path, 'Position',[x_col, y_main-30, x_col+70, y_main+30]);

    % Translation
    set_param(tf_pre_path,'TranslationMethod','Cartesian');
    set_param(tf_pre_path,'TranslationCartesianOffset', ...
        sprintf('[%f, %f, %f]', joint_xyz(i,1), joint_xyz(i,2), joint_xyz(i,3)));
    set_param(tf_pre_path,'TranslationCartesianOffsetUnits','m');

    % Rotation from URDF rpy
    set_rotation_rpy(tf_pre_path, joint_rpy(i,:));

    p_tf_pre = get_param(h_tf_pre,'PortHandles');
    add_line(model_name, last_port, p_tf_pre.LConn(1), 'autorouting','on');
    x_col = x_col + 100;

    %-----------------------------------------------------------------
    % B. PRE-JOINT AXIS ALIGNMENT  (maps URDF joint axis → Simscape Z)
    %-----------------------------------------------------------------
    needs_align = any(pre_rpy(i,:) ~= 0);
    if needs_align
        tf_aln_path = [model_name, sprintf('/AxisPre_%d',i)];
        h_tf_aln = add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
            tf_aln_path, 'Position',[x_col, y_main-30, x_col+70, y_main+30]);
        set_param(tf_aln_path,'TranslationMethod','None');
        set_rotation_rpy(tf_aln_path, pre_rpy(i,:));
        p_tf_aln = get_param(h_tf_aln,'PortHandles');
        add_line(model_name, p_tf_pre.RConn(1), p_tf_aln.LConn(1), 'autorouting','on');
        joint_base_port = p_tf_aln.RConn(1);
        x_col = x_col + 100;
    else
        joint_base_port = p_tf_pre.RConn(1);
    end

    %-----------------------------------------------------------------
    % C. REVOLUTE JOINT (Simscape always rotates about local Z)
    %-----------------------------------------------------------------
    jt_path = [model_name, sprintf('/Revolute_%d_%s',i,link_names{i})];
    h_jt = add_block('sm_lib/Joints/Revolute Joint', ...
        jt_path, 'Position',[x_col, y_main-25, x_col+60, y_main+25]);
    p_jt = get_param(h_jt,'PortHandles');
    add_line(model_name, joint_base_port, p_jt.LConn(1), 'autorouting','on');
    x_col = x_col + 100;

    %-----------------------------------------------------------------
    % D. POST-JOINT AXIS RESTORATION (restores URDF link frame)
    %-----------------------------------------------------------------
    if needs_align
        tf_post_path = [model_name, sprintf('/AxisPost_%d',i)];
        h_tf_post = add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
            tf_post_path, 'Position',[x_col, y_main-30, x_col+70, y_main+30]);
        set_param(tf_post_path,'TranslationMethod','None');
        set_rotation_rpy(tf_post_path, post_rpy(i,:));
        p_tf_post = get_param(h_tf_post,'PortHandles');
        add_line(model_name, p_jt.RConn(1), p_tf_post.LConn(1), 'autorouting','on');
        link_origin_port = p_tf_post.RConn(1);
        x_col = x_col + 100;
    else
        link_origin_port = p_jt.RConn(1);
    end

    %-----------------------------------------------------------------
    % E. LINK VISUAL GEOMETRY — File Solid (STL + identified inertia)
    %    Connected to the link origin (= URDF link frame at zero angle)
    %-----------------------------------------------------------------
    geom_path = [model_name, sprintf('/Geometry_%s',link_names{i})];
    h_geom = add_block('sm_lib/Body Elements/File Solid', ...
        geom_path, 'Position',[x_col, y_main-85, x_col+70, y_main-15]);
    set_param(geom_path,'ExtGeomFileName',    stl_files{i});
    set_param(geom_path,'UnitType',           'Custom');
    set_param(geom_path,'ExtGeomFileUnits',   'm');
    set_param(geom_path,'InertiaType',        'Custom');
    set_param(geom_path,'Mass',               sprintf('%f', m(i)));
    set_param(geom_path,'MassUnits',          'kg');
    set_param(geom_path,'CenterOfMass',       sprintf('[%f, %f, %f]', Pc(i,1),Pc(i,2),Pc(i,3)));
    set_param(geom_path,'CenterOfMassUnits',  'm');
    set_param(geom_path,'MomentsOfInertia',   sprintf('[%f, %f, %f]', I_data(i,1),I_data(i,2),I_data(i,3)));
    set_param(geom_path,'MomentsOfInertiaUnits','kg*m^2');
    set_param(geom_path,'ProductsOfInertia',  '[0, 0, 0]');
    set_param(geom_path,'ProductsOfInertiaUnits','kg*m^2');
    p_geom = get_param(h_geom,'PortHandles');
    % R port = reference/base port of the body in its current frame
    add_line(model_name, link_origin_port, p_geom.RConn(1), 'autorouting','on');

    last_port = link_origin_port;
    x_col = x_col + 120;
end

%% 8. SAVE
fprintf('Saving model...\n');
save_system(model_name);
fprintf('Done: %s.slx saved.\n', model_name);
fprintf('Open Mechanics Explorer via Simulation > Update Diagram (Ctrl+T).\n');

%% ================================================================
%  LOCAL HELPER FUNCTION
%  Sets the rotation of a Rigid Transform block from RPY angles.
%  URDF convention: R = Rz(yaw)*Ry(pitch)*Rx(roll).
%  Uses ArbitraryAxis (axis + angle) — compatible with all MATLAB versions.
% ================================================================
function set_rotation_rpy(blk, rpy)
    r = rpy(1);  p = rpy(2);  y = rpy(3);

    cr = cos(r); sr = sin(r);
    cp = cos(p); sp = sin(p);
    cy = cos(y); sy = sin(y);

    % Rotation matrix: R = Rz*Ry*Rx  (URDF extrinsic XYZ = intrinsic ZYX)
    R = [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr ;
          sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr ;
         -sp,     cp*sr,             cp*cr            ];

    % Axis-angle extraction
    cosT = (trace(R) - 1) / 2;
    th   = acos(max(-1, min(1, cosT)));   % angle in [0, pi]

    if th < 1e-9
        % Identity rotation — no rotation block needed
        set_param(blk, 'RotationMethod', 'None');
        return;
    end

    if abs(th - pi) < 1e-6
        % 180° case: axis from (R + I) columns
        M = R + eye(3);
        [~, col] = max(sum(M.^2));
        ax = M(:, col);
    else
        ax = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)] / (2*sin(th));
    end
    ax = ax / norm(ax);

    set_param(blk, 'RotationMethod',        'ArbitraryAxis');
    set_param(blk, 'RotationArbitraryAxis', sprintf('[%.10f, %.10f, %.10f]', ax(1), ax(2), ax(3)));
    set_param(blk, 'RotationAngle',         sprintf('%.10f', th));
    set_param(blk, 'RotationAngleUnits',    'rad');
end
