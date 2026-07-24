% print_traj_sample.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
out = sim('simscape_robot_model', 'StopTime', '2.0');

t_sim = out.simlog.Revolute_Joint.Rz.q.series.points;
q_act = out.simlog.Revolute_Joint.Rz.q.series.values;
w_act = out.simlog.Revolute_Joint.Rz.w.series.values;

disp('=== Sample Data ===');
disp('t_sim (first 5):'); disp(t_sim(1:5));
disp('q_act (first 5):'); disp(q_act(1:5));
disp('w_act (first 5):'); disp(w_act(1:5));
disp('Size of t_sim:'); disp(size(t_sim));
disp('Size of q_act:'); disp(size(q_act));
