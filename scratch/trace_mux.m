% scratch/trace_mux.m
load_system('simscape_robot_model');
conn = get_param('simscape_robot_model/Mux1', 'PortConnectivity');
disp('=== Mux1 Port Connectivity ===');
for i = 1:length(conn)
    disp(['Port ', num2str(i), ' details:']);
    disp(conn(i));
    if ~isempty(conn(i).SrcBlock)
        for j = 1:length(conn(i).SrcBlock)
            h = conn(i).SrcBlock(j);
            try
                disp(['  SrcBlock ', num2str(j), ' Name: ', get_param(h, 'Name')]);
            catch ME
                disp(['  Error for SrcBlock ', num2str(j), ': ', ME.message]);
            end
        end
    end
end
