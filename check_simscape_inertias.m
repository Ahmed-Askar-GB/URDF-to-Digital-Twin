% check_simscape_inertias.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);
load_system('simscape_robot_model');

links = {'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'};

for idx = 1:length(links)
    block_name = ['simscape_robot_model/' links{idx} '/Inertia'];
    disp(['=== ' links{idx} ' ===']);
    try
        mass = get_param(block_name, 'Mass');
        com = get_param(block_name, 'CenterOfMass');
        moments = get_param(block_name, 'MomentsOfInertia');
        products = get_param(block_name, 'ProductsOfInertia');
        disp(['  Mass: ', mass]);
        disp(['  CenterOfMass: ', com]);
        disp(['  MomentsOfInertia: ', moments]);
        disp(['  ProductsOfInertia: ', products]);
    catch ME
        disp(['  Error: ', ME.message]);
    end
end
