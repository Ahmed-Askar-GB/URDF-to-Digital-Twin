% trace_converters.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
converters = {'PS-Simulink Converter', 'PS-Simulink Converter1', 'PS-Simulink Converter2', 'PS-Simulink Converter3', 'PS-Simulink Converter4', 'PS-Simulink Converter5'};
for idx = 1:length(converters)
    block_name = ['simscape_robot_model/' converters{idx}];
    conn = get_param(block_name, 'PortConnectivity');
    disp(['=== ' converters{idx} ' ===']);
    for p = 1:length(conn)
        disp(['  Port ' num2str(p) ': Type=' conn(p).Type]);
        if ~isempty(conn(p).SrcBlock)
            try
                disp(['    SrcBlock Name: ' get_param(conn(p).SrcBlock, 'Name')]);
            catch
                disp('    SrcBlock: [Invalid/Multiple]');
            end
        end
        if ~isempty(conn(p).DstBlock)
            for d = 1:length(conn(p).DstBlock)
                try
                    disp(['    DstBlock ' num2str(d) ' Name: ' get_param(conn(p).DstBlock(d), 'Name')]);
                catch
                    disp(['    DstBlock ' num2str(d) ': [Invalid/Multiple]']);
                end
            end
        end
    end
end
