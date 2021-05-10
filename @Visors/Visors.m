classdef Visors < handle
    properties
        
        % Initial conditions
        ICs % same as visorsStruct
        w0 
        q0
        
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        function obj = Visors(w0, q0)
            obj.ICs = visorsStruct();
            obj.w0 = w0;
            obj.q0 = q0;
        end
        
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
        
        function [m1_true, m2_true] = get_ref_vecs_true(obj, JD)
            
            % Cartesian coords of unit vector to Sun in ECI frame
            m1_true = unitVec(get_sun_position(JD));
            
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
        
        function [m1_meas, m2_meas] = get_ref_vecs_meas(obj, JD, q, A)
            
            [m1_true, m2_true] = obj.get_ref_vecs_true(JD);
            
            % Get measurements in spacecraft body frame
            m1_meas = A' * quat2dcm(q) * m1_true;
            m2_meas = A' * quat2dcm(q) * m2_true;
            
            % Noise characteristics for sensor 1
            Q_1 = 0.00 * eye(3);
            mu_1 = [0;0;0];
            noise_1 = sqrtm(Q_1)*randn(3,1) + mu_1;
            
            % Noise characteristics for sensor 2
            Q_2 = 0.00 * eye(3);
            mu_2 = [0;0;0];
            noise_2 = sqrtm(Q_2)*randn(3,1) + mu_2;
            
            
            % Corrupt measurements with noise
            m1_meas = m1_meas + noise_1;
            m2_meas = m2_meas + noise_2;
            
        end
    end
end