% print_traj_raw.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
out = sim('simscape_robot_model', 'StopTime', '2.0');

t_sim = out.simlog.Revolute_Joint.Rz.q.series.points;
q_act = out.simlog.Revolute_Joint.Rz.q.series.values;

disp('=== Raw Log Data ===');
disp('Size of t_sim:'); disp(size(t_sim));
disp('Size of q_act:'); disp(size(q_act));
disp('t_sim:'); disp(t_sim);
disp('q_act:'); disp(q_act);
