classdef Visors < handle
    properties
        
        % Initial conditions
        ICs % same as visorsStruct
        w0 
        q0
        
        % Struct containing various options/flags 
        opts
        
        % truth
        true
        
        % spacecraft onboard computer
        est
        
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        function obj = Visors(w0, q0, opts)
            obj.ICs = visorsStruct();
            obj.w0 = w0;
            obj.q0 = q0;
            
            if ~exist('opts', 'var')
                opts = obj.get_default_opts;
            end
            obj.opts = opts;
            
            % Spacecraft's true state ("nature")
            obj.true = struct();
            
            % Spacecraft's onboard estimates
            obj.est = struct();
            obj.est.q = [];
            obj.est.q_from_w = [];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Calculate desired quaternion for Sun-pointing
        function q_des = calc_q_des(obj, t_arr)
            
            N = length(t_arr);
            q_des = zeros(4, N);
            
            for i = 1:N
                
                % Get current Sun direction in ECI frame
                JD = obj.ICs.JD_epoch + (t_arr(i)/86400);
                SUN_HAT_ECI = unitVec(get_sun_position(JD));
                
                % Complete triad of sun frame
                Z_HAT = [0;0;1] - SUN_HAT_ECI(3)*SUN_HAT_ECI;
                Z_HAT = Z_HAT / norm(Z_HAT);
                Y_HAT = cross(Z_HAT, SUN_HAT_ECI);
                DCM_ECI_TO_SUN = [SUN_HAT_ECI Y_HAT Z_HAT]'; 
                
                % Calculate desired quaternion such that body frame is
                % aligned with Sun frame
                quat = dcm2quat(obj.ICs.A_rot * DCM_ECI_TO_SUN);
                q_des(:,i) = quat;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Attitude determination from angular velocity measurements
        function q_out = calc_q_from_w(obj, w_est, q, t_prop, options)
            [~, q_out] = ode45(@(t,y) int_quaternion(t,y,w_est), t_prop, q, options);
            q_out = q_out(end,:)'./norm(q_out(end,:)');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % q-method for statistical attitude determination
        function q_out = calc_q_stat(obj, m1_m, m2_m, m1_t, m2_t)
            
            % make W matrix of measurements (in principal axes)
            % Weights all currently set to 1
            w_1 = m1_m;
            w_2 = cross(m1_m,m2_m) ./ norm(cross(m1_m,m2_m));
            w_3 = cross(w_1,w_2);
            W = obj.ICs.A_rot * [w_1, w_2, w_3];

            % Make V matrix of true reference directions (in ECI)
            % Weights all currently set to 1
            v_1 = m1_t;
            v_2 = cross(m1_t,m2_t) ./ norm(cross(m1_t,m2_t));
            v_3 = cross(v_1,v_2);
            V = [v_1, v_2, v_3];
            
            % Calculation of intermediate values used in q-method
            B = W * V';
            S = B + B';
            Z = [B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)];
            sigma = trace(B);
            K = [S-sigma*eye(3), Z; Z', sigma];
            
            % Calculation of output q through eigenvalue/eigenvector
            % problem
            [eig_vec, eig_val] = eig(K);
            q_out = eig_vec(:,4);
            
            % To account for and get rid of sign flips
            q_out = dcm2quat(quat2dcm(q_out));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % deterministic attitude determination
        function q_out = calc_q_det(obj, m1_m, m2_m, m1_t, m2_t)

            % If flag set to 1, use fictitious measurements
            if obj.opts.calc_q_det_flag == 1
                m1_m_tilde = (m1_m + m2_m) ./ 2;
                m2_m_tilde = (m1_m - m2_m) ./ 2;
                m1_m = m1_m_tilde;
                m2_m = m2_m_tilde;
                
                m1_t_tilde = (m1_t + m2_t) ./ 2;
                m2_t_tilde = (m1_t - m2_t) ./ 2;
                m1_t = m1_t_tilde;
                m2_t = m2_t_tilde;
            end
            
            % make M matrix of measurements (in principal axes)
            p_m = m1_m;
            q_m = cross(m1_m,m2_m) ./ norm(cross(m1_m,m2_m));
            r_m = cross(p_m,q_m);
            M = obj.ICs.A_rot * [p_m, q_m, r_m];

            % Make V matrix of true reference directions (in ECI)
            p_v = m1_t;
            q_v = cross(m1_t,m2_t) ./ norm(cross(m1_t,m2_t));
            r_v = cross(p_v,q_v);
            V = [p_v, q_v, r_v];

            % Calculate quaternion
            A = M * inv(V);
            q_out = dcm2quat(A);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Get measurements of ref unit vectors for attitude determination
        % Currently gets measurment directions to Sun and Alpha Centauri A
        function [m1_meas, m2_meas, m1_true, m2_true] = get_ref_vecs_meas(obj, q, no_noise)
            
            [m1_true, m2_true] = obj.get_ref_vecs_true();
            
            % Get measurements in spacecraft body frame
            m1_meas = obj.ICs.A_rot' * quat2dcm(q) * m1_true;
            m2_meas = obj.ICs.A_rot' * quat2dcm(q) * m2_true;
            
            if nargin == 2
                no_noise = false;
            end
            
            % If not corrupting measurements, return now
            if obj.opts.corrupt_measurements == 0 || no_noise
                return
            end
            
            % Noise characteristics for sensor 1
            one_sigma = deg2rad(40/3600);
            Q_1 = one_sigma^2 * eye(3);
            mu_1 = [0;0;0];
            
            % Noise characteristics for sensor 2
            one_sigma = deg2rad(40/3600);
            Q_2 = one_sigma^2 * eye(3);
            mu_2 = [0;0;0];
            
            % Corrupt measurements with noise
            noise_1 = sqrtm(Q_1)*randn(3,1) + mu_1;
            noise_2 = sqrtm(Q_2)*randn(3,1) + mu_2;
            m1_meas = m1_meas + noise_1;
            m2_meas = m2_meas + noise_2;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Get measurements of angular velocity for attitude determination
        function w_meas = get_w_meas(obj, w_true)
            
            % Get angular velocity measurments from true values
            w_meas = w_true;
            
            % If not corrupting measurements, return now
            if obj.opts.corrupt_measurements == 0
                return
            end
            
            % Noise characteristics for IMUs
            Q = deg2rad(0.01)^2 * eye(3);
            mu = [0;0;0];
            
            % Corrupt measurements with noise
            noise = sqrtm(Q)*randn(3,1) + mu;
            w_meas = w_meas + noise;
        end
    end
    methods (Static)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Get truth reference unit vectors for attitude determination
        % Currently gets truth direction to Sirius and Alpha Centauri A
        function [m1_true, m2_true] = get_ref_vecs_true()
            
            % Right ascention of Sirius A in [HH, MM, SS]
            ra = [06, 45, 8.9];

            % Declination of Sirius A in [deg, min, sec]
            de = [-16, 42, 58];

            % Spherical coords
            phi = ra(1) + ra(2)/60 + ra(2)/3600; 
            phi = (phi/24)*2*pi;
            theta = de(1) + sign(de(1))*(de(2)/60 + de(3)/3600);
            theta = deg2rad(90 - theta);

            % Cartesian coords of unit vector to Sirius A in ECI frame
            m1_true = [cos(phi)*sin(theta); sin(phi)*sin(theta); cos(theta)];
            
            % Right ascention of Alpha Centauri A in [HH, MM, SS]
            ra = [14, 39, 35.06311];

            % Declination of Alpha Centauri A in [deg, min, sec]
            de = [-60, 50, 15.0992];

            % Spherical coords
            phi = ra(1) + ra(2)/60 + ra(2)/3600; 
            phi = (phi/24)*2*pi;
            theta = de(1) + sign(de(1))*(de(2)/60 + de(3)/3600);
            theta = deg2rad(90 - theta);

            % Cartesian coords of unit vector to Alpha Centauri A in ECI frame
            m2_true = [cos(phi)*sin(theta); sin(phi)*sin(theta); cos(theta)];
        end
    end
end