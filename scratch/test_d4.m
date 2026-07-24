d1 = 0.089159;
a2 = 0.425;
a3 = 0.39225;
d4 = -0.10915; % test negative d4
d5 = 0.09465;
d6 = 0.0823;

DH_standard = [
    0,   pi/2,  0, d1;
    a2,  0,     0, 0;
    a3,  0,     0, 0;
    0,   pi/2,  0, d4;
    0,  -pi/2,  0, d5;
    0,   0,     0, d6
];

Ai_fn = @(a, alpha, theta, d) [
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1
];

T_dh = cell(6, 1);
T_accum = eye(4);
for i = 1:6
    T_accum = T_accum * Ai_fn(DH_standard(i,1), DH_standard(i,2), DH_standard(i,3), DH_standard(i,4));
    T_dh{i} = T_accum;
end

disp('New T_dh{4} origin:');
disp(T_dh{4}(1:3, 4)');

% Build links structure from URDF
xmlData = xmlread('universalUR5.urdf');
links = struct();
linkNodes = xmlData.getElementsByTagName('link');
for i = 0:linkNodes.getLength()-1
    node = linkNodes.item(i);
    name = char(node.getAttribute('name'));
    com = [0;0;0];
    ineNode = node.getElementsByTagName('inertial');
    if ineNode.getLength() > 0
        oNode = ineNode.item(0).getElementsByTagName('origin');
        if oNode.getLength() > 0
            xyzStr = char(oNode.item(0).getAttribute('xyz'));
            if ~isempty(xyzStr), com = str2num(xyzStr)'; end
        end
    end
    links.(name) = struct('com', com);
end

% URDF frame configuration at q = 0
joint_xyz = [
    0, 0, 0.089159;
    0, 0.13585, 0;
    0, -0.1197, 0.425;
    0, 0, 0.39225;
    0, 0.093, 0;
    0, 0, 0.09465
];

joint_rpy = [
    0, 0, 0;
    0, 1.57079632679, 0;
    0, 0, 0;
    0, 1.57079632679, 0;
    0, 0, 0;
    0, 0, 0
];

% Helper to convert rpy to R
rpy_to_R = @(rpy) [1 0 0; 0 cos(rpy(2)) sin(rpy(2)); 0 -sin(rpy(2)) cos(rpy(2))]; % pitch only simplified

T_urdf_accum = eye(4);
T_urdf = cell(6, 1);
for i = 1:6
    T_j = eye(4);
    % We use exact R for pitch rotation
    p = joint_rpy(i, 2);
    T_j(1:3, 1:3) = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
    T_j(1:3, 4) = joint_xyz(i, :)';
    T_urdf_accum = T_urdf_accum * T_j;
    T_urdf{i} = T_urdf_accum;
end

child_list = {'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'};
for i = 5:6
    child_name = child_list{i};
    com_urdf = links.(child_name).com;
    p_com_global = T_urdf{i}(1:3, 1:3) * com_urdf + T_urdf{i}(1:3, 4);
    R_curr_dh = T_dh{i}(1:3, 1:3);
    P_Ci_new = R_curr_dh' * (p_com_global - T_dh{i-1}(1:3, 4));
    fprintf('New P_C%d: [%.6f, %.6f, %.6f]\n', i, P_Ci_new(1), P_Ci_new(2), P_Ci_new(3));
end
