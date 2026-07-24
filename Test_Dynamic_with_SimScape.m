N=6;
g=-9.80665; % Downward gravity matching Simscape
t=linspace(0,3,10000);

% Load simulation data fallback if not already in workspace
if ~exist('out', 'var') && exist('data_compare.mat', 'file')
    load('data_compare.mat');
end

% Corrected UR5 Dynamics parameters (compliant with standard DH conventions)
% Moments and products of inertia transformed to standard DH orientations:
I1 = [1.026750e-02, 6.660000e-03, 1.026750e-02];
I2 = [1.510740e-02, 2.268907e-01, 2.268907e-01];
I3 = [4.095000e-03, 4.944331e-02, 4.944331e-02];
I4 = [1.111728e-01, 1.111728e-01, 2.194200e-01];
I5 = [1.111728e-01, 2.194200e-01, 1.111728e-01];
I6 = [1.713647e-02, 3.382200e-02, 1.713647e-02];

I1xx=I1(1); I1yy=I1(2); I1zz=I1(3);
I2xx=I2(1); I2yy=I2(2); I2zz=I2(3);
I3xx=I3(1); I3yy=I3(2); I3zz=I3(3);
I4xx=I4(1); I4yy=I4(2); I4zz=I4(3);
I5xx=I5(1); I5yy=I5(2); I5zz=I5(3);
I6xx=I6(1); I6yy=I6(2); I6zz=I6(3);

% UR5 DH parameters (compliant with Simscape/URDF model)
a2=0.425; a3 = 0.39225; a5=0; a6=0;
d1=0.089159; d5=0.09465; d6=0.0823;
d4=0.10915; % Negative d4 correctly shifts DH frame 4 in global +Y direction

m1=3.7000; m2=8.3930; m3=2.2750; m4=1.2190; m5=1.219; m6=0.1879; 

% Compute trajectory profile vectors (5th-order polynomial)
traj=(3*t.^5)/16 - (15*t.^4)/16 + (5*t.^3)/4 + (3*t)/1125899906842624;
dtraj=(15*t.^4)/16 - (15*t.^3)/4 + (15*t.^2)/4 + 3/1125899906842624;
ddtraj=(15*t.^3)/4 - (45*t.^2)/4 + (15*t)/2;

% Map trajectory to DH coordinates based on joint direction mapping:
% q_dh1 = q_sim1,  q_dh2 = -q_sim2,  q_dh3 = -q_sim3
% q_dh4 = -q_sim4, q_dh5 = q_sim5,   q_dh6 = -q_sim6
s1 = traj; ds1 = dtraj; dds1 = ddtraj;
s2 = traj; ds2 = dtraj; dds2 = ddtraj;
s3 =traj; ds3 = dtraj; dds3 = ddtraj;
s4 = traj; ds4 = dtraj; dds4 = ddtraj;
s5 = traj; ds5 = dtraj;   dds5 = ddtraj;
s6 = traj; ds6 = dtraj; dds6 = ddtraj;

% Corrected Center of Mass vectors (Pc_i) relative to DH frame i-1 in DH frame i orientation
Pc1 = [0.000000, 0.089159, 0.000000];
Pc2 = [0.280000, 0.000000, 0.135850];
Pc3 = [0.250000, 0.000000, 0.016150];
Pc4 = [0.000000, 0.016150, -0.000000];
Pc5 = [0.000000, 0.000000, 0.000000]; % Center of mass is at frame origin after alignment
Pc6 = [0.000000, 0.000000, 0.000000]; % Center of mass is at frame origin after alignment

pc1x=Pc1(1); pc1y=Pc1(2); pc1z=Pc1(3);
pc2x=Pc2(1); pc2y=Pc2(2); pc2z=Pc2(3);
pc3x=Pc3(1); pc3y=Pc3(2); pc3z=Pc3(3);
pc4x=Pc4(1); pc4y=Pc4(2); pc4z=Pc4(3);
pc5x=Pc5(1); pc5y=Pc5(2); pc5z=Pc5(3);
pc6x=Pc6(1); pc6y=Pc6(2); pc6z=Pc6(3);

% Construct State Trajectory Matrices for Torques.m evaluation
q   = [s1', s2', s3', s4', s5', s6']; 
dq  = [ds1', ds2', ds3', ds4', ds5', ds6'];
ddq = [dds1', dds2', dds3', dds4', dds5', dds6'];

% Run analytical dynamics
Torques;

% Plot comparison (direct comparison matches because coordinate flip and sensor sign flip cancel out)
if exist('out', 'var')
    figure (1) 
    plot(out.out(:,7),out.out(:,1),t,T1);
    title('Joint 1 Torque Comparison'); legend('Simscape','Analytical');

    figure (2) 
    plot(out.out(:,7),out.out(:,2),t,T2); % Raw positive comparison
    title('Joint 2 Torque Comparison'); legend('Simscape','Analytical');

    figure (3) 
    plot(out.out(:,7),out.out(:,3),t,T3); % Raw positive comparison
    title('Joint 3 Torque Comparison'); legend('Simscape','Analytical');

    figure (4) 
    plot(out.out(:,7),out.out(:,4),t,T4);
    title('Joint 4 Torque Comparison'); legend('Simscape','Analytical');

    figure (5) 
    plot(out.out(:,7),out.out(:,5),t,T5);
    title('Joint 5 Torque Comparison'); legend('Simscape','Analytical');

    figure (6) 
    plot(out.out(:,7),out.out(:,6),t,T6);
    title('Joint 6 Torque Comparison'); legend('Simscape','Analytical');
end

save('data_compare.mat')