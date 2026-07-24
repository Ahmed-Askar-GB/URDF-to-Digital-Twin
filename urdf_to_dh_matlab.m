function [DHpram, P_Ci, I, m] = urdf_to_dh_matlab(filename)
    if ~exist(filename, 'file')
        error('File %s not found.', filename);
    end
    
    try
        xmlData = xmlread(filename);
    catch
        error('Failed to parse XML file.');
    end
    
    %% 1. Extract Links from URDF
    links = struct();
    linkNodes = xmlData.getElementsByTagName('link');
    
    for i = 0:linkNodes.getLength()-1
        node = linkNodes.item(i);
        name = char(node.getAttribute('name'));
        
        mass = 0; com = [0;0;0]; inertia_mat = zeros(3,3);
        inertialNode = node.getElementsByTagName('inertial');
        if inertialNode.getLength() > 0
            ine = inertialNode.item(0);
            mNode = ine.getElementsByTagName('mass');
            if mNode.getLength() > 0
                mass = str2double(char(mNode.item(0).getAttribute('value')));
            end
            oNode = ine.getElementsByTagName('origin');
            if oNode.getLength() > 0
                xyzStr = char(oNode.item(0).getAttribute('xyz'));
                if ~isempty(xyzStr), com = str2num(xyzStr)'; end
                
                % Capture orientation shift of the inertia frame at the CoM
                rpyInertialStr = char(oNode.item(0).getAttribute('rpy'));
                if ~isempty(rpyInertialStr)
                    R_inertial = rpyxyz_to_rot(str2num(rpyInertialStr));
                else
                    R_inertial = eye(3);
                end
            else
                R_inertial = eye(3);
            end
            iNode = ine.getElementsByTagName('inertia');
            if iNode.getLength() > 0
                ixx = str2double(char(iNode.item(0).getAttribute('ixx')));
                ixy = str2double(char(iNode.item(0).getAttribute('ixy')));
                ixz = str2double(char(iNode.item(0).getAttribute('ixz')));
                iyy = str2double(char(iNode.item(0).getAttribute('iyy')));
                iyz = str2double(char(iNode.item(0).getAttribute('iyz')));
                izz = str2double(char(iNode.item(0).getAttribute('izz')));
                raw_inertia = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
                % Rotate inertia tensor from CoM axes to the local URDF link frame
                inertia_mat = R_inertial * raw_inertia * R_inertial';
            end
        end
        links.(name) = struct('mass', mass, 'com', com, 'inertia', inertia_mat);
    end
    
    %% 2. Extract Joints from URDF
    joints = struct([]);
    jointNodes = xmlData.getElementsByTagName('joint');
    
    for i = 0:jointNodes.getLength()-1
        node = jointNodes.item(i);
        
        if ~node.hasAttribute('type')
            continue; 
        end
        
        type = char(node.getAttribute('type'));
        name = char(node.getAttribute('name'));
        
        pNode = node.getElementsByTagName('parent');
        cNode = node.getElementsByTagName('child');
        
        if pNode.getLength() == 0 || cNode.getLength() == 0
            continue; 
        end
        
        parent = char(pNode.item(0).getAttribute('link'));
        child = char(cNode.item(0).getAttribute('link'));
        
        oNode = node.getElementsByTagName('origin');
        xyz = [0 0 0]; rpy = [0 0 0];
        if oNode.getLength() > 0
            xyzStr = char(oNode.item(0).getAttribute('xyz'));
            rpyStr = char(oNode.item(0).getAttribute('rpy'));
            if ~isempty(xyzStr), xyz = str2num(xyzStr); end
            if ~isempty(rpyStr), rpy = str2num(rpyStr); end
        end
        T = rpyxyz_to_ht(rpy, xyz);
        
        aNode = node.getElementsByTagName('axis');
        axis_vec = [1 0 0]'; 
        if aNode.getLength() > 0
            axisStr = char(aNode.item(0).getAttribute('xyz'));
            if ~isempty(axisStr)
                axis_vec = str2num(axisStr)'; 
                if norm(axis_vec) > 1e-6
                    axis_vec = axis_vec / norm(axis_vec); 
                end
            end
        end
        
        joints(end+1).name = name;
        joints(end).type = type;
        joints(end).parent = parent;
        joints(end).child = child;
        joints(end).T = T;
        joints(end).axis = axis_vec;
    end
    
    %% 3. Build Moving Kinematic Chain (Compounding Fixed Joints)
    chain = struct([]);
    current_link = 'base_link'; 
    T_fixed_accum = eye(4); 
    
    while true
        next_j_idx = find(strcmp({joints.parent}, current_link) & ~strcmp({joints.type}, 'fixed'), 1);
        
        if isempty(next_j_idx)
            fixed_j_idx = find(strcmp({joints.parent}, current_link) & strcmp({joints.type}, 'fixed'), 1);
            if isempty(fixed_j_idx)
                break; 
            else
                T_fixed_accum = T_fixed_accum * joints(fixed_j_idx).T;
                current_link = joints(fixed_j_idx).child;
                continue; 
            end
        end
        
        next_joint = joints(next_j_idx);
        next_joint.T = T_fixed_accum * next_joint.T;
        
        chain(end+1).name = next_joint.name;
        chain(end).type = next_joint.type;
        chain(end).parent = next_joint.parent;
        chain(end).child = next_joint.child;
        chain(end).T = next_joint.T;
        chain(end).axis = next_joint.axis;
        
        T_fixed_accum = eye(4);
        current_link = next_joint.child;
    end
    
    % Add a virtual terminal structure to cleanly capture the final link's parameters
    N = numel(chain);
    
    %% 4. Generate DH Parameters and Exact Lagrangian Frame Matching
    DHpram = zeros(N, 5); 
    P_Ci = zeros(N, 3);
    I = zeros(3, 3, N);
    m = zeros(N, 1);
    
    T_urdf_accum = eye(4);
    T_dh_accum = eye(4);
    
    z_prev = [0; 0; 1];
    o_prev = [0; 0; 0];
    
    for i = 1:N
        T_joint = chain(i).T;
        axis_local = chain(i).axis;
        T_urdf_accum = T_urdf_accum * T_joint;
        
        z_curr = T_urdf_accum(1:3, 1:3) * axis_local;
        o_curr = T_urdf_accum(1:3, 4);
        
        if i == 1
            alpha = 0; a = 0; d = 0; theta = 0;
        else
            n = cross(z_prev, z_curr);
            if norm(n) < 1e-5
                n = [1; 0; 0]; n = n - dot(n, z_prev)*z_prev; n = n / norm(n);
                alpha = 0;
            else
                alpha = acos(max(-1, min(dot(z_prev, z_curr), 1)));
                if dot(cross(z_prev, z_curr), n) < 0, alpha = -alpha; end
            end
            v_link = o_curr - o_prev;
            a = dot(v_link, n);
            d = dot(v_link, z_prev);
            theta = atan2(dot(cross(z_prev, n), z_curr), dot(n, n));
        end
        
        if strcmp(chain(i).type, 'revolute')
            j_type = 1;
        else
            j_type = 2;
        end
        
        DHpram(i, :) = [a, alpha, theta, d, j_type];
        
        % Save the origin position of DH frame (i-1) before accumulating frame i
        o_prev_dh = T_dh_accum(1:3, 4);
        
        % Advance the DH frame chain context
        A_dh = Ai_local(DHpram(i, :));
        T_dh_accum = T_dh_accum * A_dh;
        
        % Orientation matrix of current DH frame (i) relative to world base
        R_curr_dh = T_dh_accum(1:3, 1:3);
        
        child_name = chain(i).child;
        m(i) = links.(child_name).mass;
        
        % Compute absolute global coordinate position of the link's Center of Mass
        P_Ci_urdf = links.(child_name).com;
        p_CoM_global = T_urdf_accum * [P_Ci_urdf; 1];
        
        % CRITICAL MATCH FOR DH_DYN: 
        % Calculate the vector from DH origin (i-1) to the CoM, 
        % then rotate it into the orientation frame layout of DH frame (i)
        P_Ci(i, :) = (R_curr_dh' * (p_CoM_global(1:3) - o_prev_dh))';
        
        % Orient the core CoM inertia tensor to match DH frame (i) axes
        % DH_dyn's jw handles the rest without needing parallel axis translation shifts
        % because its jv is evaluated natively at the center of mass.
        T_urdf_to_dh_rot = R_curr_dh' * T_urdf_accum(1:3, 1:3);
        I_urdf = links.(child_name).inertia;
        I(:, :, i) = T_urdf_to_dh_rot * I_urdf * T_urdf_to_dh_rot';
        
        z_prev = z_curr;
        o_prev = o_curr;
    end
end

%% --- Kinematic Transformation Helpers ---
function R = rpyxyz_to_rot(rpy)
    roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
end

function T = rpyxyz_to_ht(rpy, xyz)
    T = eye(4);
    T(1:3, 1:3) = rpyxyz_to_rot(rpy);
    T(1:3, 4) = xyz';
end

function A = Ai_local(Dh)
    ai = Dh(1); alphai = Dh(2); si = Dh(3); di = Dh(4);
    A = [cos(si), -sin(si)*cos(alphai), sin(si)*sin(alphai), ai*cos(si); 
         sin(si), cos(si)*cos(alphai), -cos(si)*sin(alphai), ai*sin(si); 
         0, sin(alphai), cos(alphai), di; 
         0, 0, 0, 1];
end