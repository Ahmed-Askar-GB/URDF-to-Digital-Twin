% trace_dst.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
conn = get_param('simscape_robot_model/Simulink-PS Converter1', 'PortConnectivity');
dst_blocks = conn(2).DstBlock;
disp('=== Simulink-PS Converter1 Destination Ports/Blocks ===');
for idx = 1:length(dst_blocks)
    h = dst_blocks(idx);
    try
        disp(['Handle ' num2str(h) ': Name=' get_param(h, 'Name') ', Type=' get_param(h, 'Type')]);
    catch
        try
            p = get_param(h, 'Parent');
            disp(['Handle ' num2str(h) ' Parent Block: ' p]);
        catch ME
            disp(['Handle ' num2str(h) ' failed: ' ME.message]);
        end
    end
end
