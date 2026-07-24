classdef ur5_dynamics_casadi < handle
    properties
        % Link masses
        m1 = 3.7;
        m2 = 8.393;
        m3 = 2.275;
        m4 = 1.219;
        m5 = 1.219;
        m6 = 0.1879;
        
        % Link CoM offsets (relative to local link frames)
        rc1 = [0.0; 0.0; 0.0];
        rc2 = [0.0; 0.0; 0.28];
        rc3 = [0.0; 0.0; 0.25];
        rc4 = [0.0; 0.0; 0.0];
        rc5 = [0.0; 0.0; 0.0];
        rc6 = [0.0; 0.0; 0.0];
        
        % Link inertia tensors (relative to CoM, aligned with local link frames)
        I1 = diag([0.010267495893, 0.010267495893, 0.00666]);
        I2 = diag([0.22689067591, 0.22689067591, 0.0151074]);
        I3 = diag([0.049443313556, 0.049443313556, 0.004095]);
        I4 = diag([0.111172755531, 0.111172755531, 0.21942]);
        I5 = diag([0.111172755531, 0.111172755531, 0.21942]);
        I6 = diag([0.0171364731454, 0.0171364731454, 0.033822]);
        
        % Joint translation offsets (xyz) from parent link frame
        p_j1 = [0.0; 0.0; 0.089159];
        p_j2 = [0.0; 0.13585; 0.0];
        p_j3 = [0.0; -0.1197; 0.425];
        p_j4 = [0.0; 0.0; 0.39225];
        p_j5 = [0.0; 0.093; 0.0];
        p_j6 = [0.0; 0.0; 0.09465];
        
        % Joint fixed rotations (rpy) from parent link frame
        rpy_j1 = [0.0; 0.0; 0.0];
        rpy_j2 = [0.0; 1.57079632679; 0.0];
        rpy_j3 = [0.0; 0.0; 0.0];
        rpy_j4 = [0.0; 1.57079632679; 0.0];
        rpy_j5 = [0.0; 0.0; 0.0];
        rpy_j6 = [0.0; 0.0; 0.0];
        
        % Joint rotation axes (in joint frame)
        a1 = [0; 0; 1];
        a2 = [0; 1; 0];
        a3 = [0; 1; 0];
        a4 = [0; 1; 0];
        a5 = [0; 0; 1];
        a6 = [0; 1; 0];
        
        % CasADi functions
        torque_fun
        M_fun
        C_dq_fun
        g_fun
    end
    
    methods
        function obj = ur5_dynamics_casadi()
            obj.build_model();
        end
        
        function R = rot_z(~, angle)
            c = cos(angle);
            s = sin(angle);
            R = [c, -s, 0; ...
                 s, c, 0; ...
                 0, 0, 1];
        end
        
        function R = rot_y(~, angle)
            c = cos(angle);
            s = sin(angle);
            R = [c, 0, s; ...
                 0, 1, 0; ...
                 -s, 0, c];
        end
        
        function R = rot_x(~, angle)
            c = cos(angle);
            s = sin(angle);
            R = [1, 0, 0; ...
                 0, c, -s; ...
                 0, s, c];
        end
        
        function R = rot_axis(obj, axis, angle)
            % Compute rotation matrix about a coordinate axis
            if all(abs(axis - [0; 0; 1]) < 1e-5)
                R = obj.rot_z(angle);
            elseif all(abs(axis - [0; 1; 0]) < 1e-5)
                R = obj.rot_y(angle);
            elseif all(abs(axis - [1; 0; 0]) < 1e-5)
                R = obj.rot_x(angle);
            else
                error('Unsupported axis for rot_axis');
            end
        end
        
        function R = rot_rpy(obj, rpy)
            % Extrinsic ZYX rotation order: Rz(yaw)*Ry(pitch)*Rx(roll)
            R = obj.rot_z(rpy(3)) * obj.rot_y(rpy(2)) * obj.rot_x(rpy(1));
        end
        
        function build_model(obj)
            import casadi.*
            
            % Symbolic variables
            q = MX.sym('q', 6);
            dq = MX.sym('dq', 6);
            ddq = MX.sym('ddq', 6);
            g_val = MX.sym('g_val', 3); % Gravity vector, e.g., [0; 0; -9.81]
            
            origins = {obj.p_j1, obj.p_j2, obj.p_j3, obj.p_j4, obj.p_j5, obj.p_j6};
            rpys = {obj.rpy_j1, obj.rpy_j2, obj.rpy_j3, obj.rpy_j4, obj.rpy_j5, obj.rpy_j6};
            axes = {obj.a1, obj.a2, obj.a3, obj.a4, obj.a5, obj.a6};
            masses = {obj.m1, obj.m2, obj.m3, obj.m4, obj.m5, obj.m6};
            rcs = {obj.rc1, obj.rc2, obj.rc3, obj.rc4, obj.rc5, obj.rc6};
            inertias = {obj.I1, obj.I2, obj.I3, obj.I4, obj.I5, obj.I6};
            
            % Kinematic initialization
            R = {MX.eye(3)};
            p = {MX.zeros(3, 1)};
            omega = {MX.zeros(3, 1)};
            v = {MX.zeros(3, 1)};
            
            T_kinetic = 0;
            V_potential = 0;
            
            for i = 1:6
                q_i = q(i);
                dq_i = dq(i);
                a_i = axes{i};
                p_ji = origins{i};
                rpy_ji = rpys{i};
                
                % Fixed transformation from parent link frame to joint frame
                R_fixed = obj.rot_rpy(rpy_ji);
                
                % Variable transformation from joint frame to child link frame
                R_var = obj.rot_axis(a_i, q_i);
                
                % Compute child orientation and origin
                R_i = R{end} * R_fixed * R_var;
                R{end+1} = R_i;
                
                p_i = p{end} + R{end-1} * p_ji;
                p{end+1} = p_i;
                
                % Angular velocity
                omega_i = omega{end} + R{end-1} * R_fixed * a_i * dq_i;
                omega{end+1} = omega_i;
                
                % Translational velocity of link origin
                v_i = v{end} + cross(omega{end-1}, R{end-1} * p_ji);
                v{end+1} = v_i;
                
                % Center of mass
                p_ci = p_i + R_i * rcs{i};
                v_ci = v_i + cross(omega_i, R_i * rcs{i});
                
                % Local angular velocity in child link frame
                omega_local_i = R_i' * omega_i;
                
                % Kinetic energy
                I_i_sym = MX(inertias{i});
                T_i = 0.5 * masses{i} * (v_ci' * v_ci) + 0.5 * (omega_local_i' * I_i_sym * omega_local_i);
                T_kinetic = T_kinetic + T_i;
                
                % Potential energy
                V_i = -masses{i} * g_val' * p_ci;
                V_potential = V_potential + V_i;
            end
            
            % Derive Lagrangian dynamics
            % M(q)*ddq + C(q,dq)*dq + G(q) = tau
            M = jacobian(jacobian(T_kinetic, dq), dq);
            obj.M_fun = Function('M', {q}, {M});
            
            g_vec = jacobian(V_potential, q)';
            obj.g_fun = Function('g_vec', {q, g_val}, {g_vec});
            
            M_dq = M * dq;
            C_dq = jacobian(M_dq, q) * dq - jacobian(T_kinetic, q)';
            obj.C_dq_fun = Function('C_dq', {q, dq}, {C_dq});
            
            % Total joint torque function
            tau = M * ddq + C_dq + g_vec;
            obj.torque_fun = Function('torque_fun', {q, dq, ddq, g_val}, {tau});
        end
    end
end
