% extract_parameters.m
cd('c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven\URDF Dynamic');
addpath(pwd);

run('test.m');
disp('=== EXTRACTED DH PARAMETERS ===');
disp(DHpram_numeric);

disp('=== EXTRACTED CENTER OF MASS VECTORS (P_Ci) ===');
for i = 1:6
    fprintf('Pc%d: [%.6f, %.6f, %.6f]\n', i, Pc(i, 1), Pc(i, 2), Pc(i, 3));
end

disp('=== EXTRACTED DIAGONAL INERTIAS (I_vectors) ===');
for i = 1:6
    fprintf('I%d: [%.6e, %.6e, %.6e]\n', i, I_vectors(i, 1), I_vectors(i, 2), I_vectors(i, 3));
end
