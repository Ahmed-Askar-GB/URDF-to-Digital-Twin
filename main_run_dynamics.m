% main_run_dynamics.m
clc; clear; close all;

%% 1. Extract Parameters From the URDF File
urdf_file = 'universalUR5.urdf';
fprintf('Parsing geometry and inertial profiles from "%s"...\n', urdf_file);
[DHpram_numeric, P_Ci, I, m] = urdf_to_dh_matlab(urdf_file);

N = size(DHpram_numeric, 1);
fprintf('Successfully loaded %d moving links.\n\n', N);
%% 2. Initialize Symbolic Variables for Dynamic Equation Resolution
q   = sym('q',   [N, 1], 'real');  % Joint Positions
dq  = sym('dq',  [N, 1], 'real');  % Joint Velocities
ddq = sym('ddq', [N, 1], 'real');  % Joint Accelerations

syms g_acc 'real';
g_vector = [0; 0; -g_acc]; % Define gravity vector

%% 3. CRITICAL FIX: Convert DH matrix to symbolic type first
DHpram = sym(DHpram_numeric); 

% Inject Symbolic Variable State References Into the DH Parameter Table
for i = 1:N
    if DHpram(i, 5) == 1 % If joint is revolute, position adds to theta
        DHpram(i, 3) = DHpram(i, 3) + q(i);
    else % If joint is prismatic, position adds to displacement d
        DHpram(i, 4) = DHpram(i, 4) + q(i);
    end
end

%% 4. Compute Dynamic Formulation Matrices via DH_dyn
fprintf('Executing Lagrange dynamic model generation (this may take a moment)... \n');
[M, C, G, Ro] = DH_dyn(DHpram, P_Ci, q, dq, I, m, g_vector);

%% 5. Display Equation Properties 
fprintf('=== INERTIA MATRIX M(q) SIZE ===\n');
disp(size(M));

fprintf('=== CORIOLIS MATRIX C(q,dq) SIZE ===\n');
disp(size(C));

fprintf('=== GRAVITY VECTOR G(q) SIZE ===\n');
disp(size(G));

fprintf('Computing Joint Torque Expression (Tau)... \n');
tau = M * ddq + C * dq + G;