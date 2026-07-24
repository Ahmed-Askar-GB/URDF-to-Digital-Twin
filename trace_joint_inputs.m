% trace_joint_inputs.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');

joints = {'Revolute Joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

for idx = 1:length(joints)
    block_name = ['simscape_robot_model/' joints{idx}];
    disp(['=== Joint: ' joints{idx} ' ===']);
    try
        actuation_mode = get_param(block_name, 'ActuationMotion');
        disp(['  ActuationMotion: ' actuation_mode]);
    catch
        disp('  ActuationMotion parameter not found.');
    end
    
    conn = get_param(block_name, 'PortConnectivity');
    for p = 1:length(conn)
        % Look for physical signal inputs or connections
        if strcmp(conn(p).Type, 'RConn1') || strcmp(conn(p).Type, 'LConn1')
            continue;
        end
        disp(['  Port ' num2str(p) ' Type=' conn(p).Type]);
        if ~isempty(conn(p).SrcBlock)
            try
                disp(['    SrcBlock Name: ' get_param(conn(p).SrcBlock, 'Name')]);
            catch
                disp('    SrcBlock: [Multiple/Invalid]');
            end
        end
    end
end
