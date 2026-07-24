% check_logged_derivatives.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
out = sim('simscape_robot_model', 'StopTime', '2.0');

disp('=== Revolute_Joint Logged Variables ===');
disp(out.simlog.Revolute_Joint);
for node = out.simlog.Revolute_Joint.childIds
    disp(node{1});
end
