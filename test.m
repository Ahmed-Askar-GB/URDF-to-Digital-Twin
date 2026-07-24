clc; clear; close all;

%% 1. Extract Parameters from the URDF File
% Provide the filename of your UR16e robot URDF
urdf_file = 'universalUR5.urdf'; 

fprintf('Parsing geometry and inertial profiles from "%s"...\n', urdf_file);
[DHpram_numeric, P_Ci_extracted, I_matrix_stack, m_extracted] = urdf_to_dh_matlab(urdf_file);

N = size(DHpram_numeric, 1);
fprintf('Successfully loaded %d moving links from URDF.\n\n', N);

%% 2. Extract Robot Geometric Constants (DH Links/Offsets)
% Dynamically reads whatever dimensions are present in the URDF file
% For a standard UR arm, these will map to your a2, a3, d1, d4, d5, d6 variables
a_links = DHpram_numeric(:, 1);
d_links = DHpram_numeric(:, 4);

% Example explicit mapping if needed for downstream code:
% d1 = d_links(1); a2 = a_links(2); a3 = a_links(3); 
% d4 = d_links(4); d5 = d_links(5); d6 = d_links(6);

%% 3. Extract Dynamic Variables (Mass, Center of Mass, and Diagonal Inertias)
% Assign masses dynamically
m = m_extracted; % Vector of size N x 1 [m1; m2; ...; m6]

% Assign Center of Mass positions
% Maps to your Pc1, Pc2, ... Pc6 variables
Pc = P_Ci_extracted; % Matrix of size N x 3 where row i is Pci

% Extract the primary diagonal inertias [Ixx, Iyy, Izz] from the full 3x3 matrices
I_vectors = zeros(N, 3);
for i = 1:N
    I_vectors(i, :) = diag(I_matrix_stack(:, :, i))';
end

% Example explicit structural breakdown to match your variable naming style:
I1 = I_vectors(1, :); I2 = I_vectors(2, :); I3 = I_vectors(3, :);
I4 = I_vectors(4, :); I5 = I_vectors(5, :); I6 = I_vectors(6, :);

Pc1 = Pc(1, :); Pc2 = Pc(2, :); Pc3 = Pc(3, :);
Pc4 = Pc(4, :); Pc5 = Pc(5, :); Pc6 = Pc(6, :);

g = -9.81; % Constant gravity assignment

%% 4. Define Trajectory Time-Series Arrays
t = linspace(0, 2, 10000);

% Compute base 5th-order polynomial trajectory profile vectors
s1 = (3*t.^5)/16 - (15*t.^4)/16 + (5*t.^3)/4 + (3*t)/1125899906842624;
ds1 = (15*t.^4)/16 - (15*t.^3)/4 + (15*t.^2)/4 + 3/1125899906842624;
dds1 = (15*t.^3)/4 - (45*t.^2)/4 + (15*t)/2;

% Construct State Trajectory Matrices (Each column represents a joint axis 1 to N)
q   = repmat(s1',   1, N); 
dq  = repmat(ds1',  1, N);
ddq = repmat(dds1', 1, N);


