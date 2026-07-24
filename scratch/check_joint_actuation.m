% check_joint_actuation.m
load_system('simscape_robot_model');
params = get_param('simscape_robot_model/Revolute Joint', 'ObjectParameters');
fields = fieldnames(params);
disp('=== Revolute Joint Parameters ===');
for i = 1:length(fields)
    try
        val = get_param('simscape_robot_model/Revolute Joint', fields{i});
        if ischar(val) || isnumeric(val)
            disp([fields{i} ': ' num2str(val)]);
        end
    catch
    end
end
