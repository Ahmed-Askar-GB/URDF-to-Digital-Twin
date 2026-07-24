% trace_sensor.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
conn = get_param('simscape_robot_model/Transform Sensor', 'PortConnectivity');
disp('=== Transform Sensor Port Connectivity ===');
for i = 1:length(conn)
    disp(['Port ' num2str(i) ' details:']);
    disp(conn(i));
    if ~isempty(conn(i).SrcBlock)
        for j = 1:length(conn(i).SrcBlock)
            h = conn(i).SrcBlock(j);
            try
                disp(['  SrcBlock ' num2str(j) ' Name: ' get_param(h, 'Name')]);
            catch
            end
        end
    end
    if ~isempty(conn(i).DstBlock)
        for j = 1:length(conn(i).DstBlock)
            h = conn(i).DstBlock(j);
            try
                disp(['  DstBlock ' num2str(j) ' Name: ' get_param(h, 'Name')]);
            catch
            end
        end
    end
end
