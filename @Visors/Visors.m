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
                SUN_HAT_ECI = s3_em_thirdbody_sun(JD);
                SUN_HAT_ECI = SUN_HAT_ECI ./ norm(SUN_HAT_ECI);
                
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
    end
end