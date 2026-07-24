% check_rz_vars.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
out = sim('simscape_robot_model', 'StopTime', '2.0');

disp('=== Rz Child Nodes ===');
disp(out.simlog.Revolute_Joint.Rz);
for node = out.simlog.Revolute_Joint.Rz.childIds
    disp(node{1});
end
