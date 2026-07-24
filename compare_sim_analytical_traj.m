% compare_sim_analytical_traj.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');
out = sim('simscape_robot_model', 'StopTime', '2.0');

% Get simulated time steps from simlog
t_sim = out.simlog.Revolute_Joint.Rz.q.series.points;
s_at_t = @(time) (3*time.^5)/16 - (15*time.^4)/16 + (5*time.^3)/4 + (3*time)/1125899906842624;
ds_at_t = @(time) (15*time.^4)/16 - (15*time.^3)/4 + (15*time.^2)/4 + 3/1125899906842624;

q_sim = s_at_t(t_sim);
dq_sim = ds_at_t(t_sim);

joints = {'Revolute_Joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

for idx = 1:length(joints)
    q_act = out.simlog.(joints{idx}).Rz.q.series.values;
    w_act = out.simlog.(joints{idx}).Rz.w.series.values;
    
    max_q_diff = max(abs(q_act - q_sim));
    max_w_diff = max(abs(w_act - dq_sim));
    
    disp(['=== ' joints{idx} ' ===']);
    disp(['  Max Position Difference: ', num2str(max_q_diff), ' rad']);
    disp(['  Max Velocity Difference: ', num2str(max_w_diff), ' rad/s']);
end
