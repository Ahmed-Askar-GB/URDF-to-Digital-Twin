% check_joint_bf.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
joints = {'Revolute Joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
for idx = 1:length(joints)
    disp(['=== ' joints{idx} ' ===']);
    conn = get_param(['simscape_robot_model/' joints{idx}], 'PortConnectivity');
    for p = 1:length(conn)
        type = conn(p).Type;
        if strcmp(type, 'LConn1') || strcmp(type, 'RConn1') || strcmp(type, 'LConn2') || strcmp(type, 'RConn2')
            disp(['  Port ' num2str(p) ' (' type '):']);
            if ~isempty(conn(p).SrcBlock)
                for s = 1:length(conn(p).SrcBlock)
                    h = conn(p).SrcBlock(s);
                    try
                        disp(['    SrcBlock ' num2str(s) ' Name: ' get_param(h, 'Name')]);
                    catch
                    end
                end
            end
            if ~isempty(conn(p).DstBlock)
                for d = 1:length(conn(p).DstBlock)
                    h = conn(p).DstBlock(d);
                    try
                        disp(['    DstBlock ' num2str(d) ' Name: ' get_param(h, 'Name')]);
                    catch
                    end
                end
            end
        end
    end
end
