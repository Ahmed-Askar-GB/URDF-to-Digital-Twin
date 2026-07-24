% print_extracted_simlog.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
out = sim('simscape_robot_model', 'StopTime', '2.0');

q5 = out.simlog.wrist_2_joint.Rz.q.series.values * pi/180;
w5 = out.simlog.wrist_2_joint.Rz.w.series.values * pi/180;
q6 = out.simlog.wrist_3_joint.Rz.q.series.values * pi/180;
w6 = out.simlog.wrist_3_joint.Rz.w.series.values * pi/180;

disp('Max q5:'); disp(max(abs(q5)));
disp('Max w5:'); disp(max(abs(w5)));
disp('Max q6:'); disp(max(abs(q6)));
disp('Max w6:'); disp(max(abs(w6)));
