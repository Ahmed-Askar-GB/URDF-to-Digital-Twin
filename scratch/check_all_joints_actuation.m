% check_all_joints_actuation.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');

joints = {'Revolute Joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

for idx = 1:length(joints)
    block_name = ['simscape_robot_model/' joints{idx}];
    disp(['=== ' joints{idx} ' ===']);
    try
        motion_mode = get_param(block_name, 'MotionActuationMode');
        disp(['  MotionActuationMode: ' motion_mode]);
    catch ME
        disp(['  Error Motion: ' ME.message]);
    end
    try
        torque_mode = get_param(block_name, 'TorqueActuationMode');
        disp(['  TorqueActuationMode: ' torque_mode]);
    catch ME
        disp(['  Error Torque: ' ME.message]);
    end
end
