function srp_torque = get_srp_torque(visors, JD, R_ECI, V_ECI, quat)
% Calculate torque due to SRP in principal axes 

% First check for eclipse
SUN_HAT_ECI = s3_em_thirdbody_sun(visors.JD_epoch);
SUN_HAT_ECI = SUN_HAT_ECI ./ norm(SUN_HAT_ECI);
r_par = dot(R_ECI, SUN_HAT_ECI);
r_perp = R_ECI - (r_par .* SUN_HAT_ECI);

% If eclipse, SRP torque is 0
if (norm(r_perp)-s3_constants('R_EARTH_WGS84') < 0) && (r_par < 0)
    srp_torque = [0;0;0];
    return;
end

% SRP Pressure
P = 1358/(3e8); 

% Coefficients (idk if these are right at all)
Ca = 1/3;
Cs = 1/3;
Cd = 1/3;

% Sun direction
SUN_HAT_PRIN = quat2dcm(quat) * SUN_HAT_ECI;
SUN_HAT_BODY = visors.A_rot' * SUN_HAT_PRIN; 

% Faces that are lit by Sun
active_faces = get_active_faces(visors.faces, SUN_HAT_BODY);

% Calculate force on each face
SUN_HAT_ARR = repmat(SUN_HAT_BODY', active_faces.N, 1);
cos_theta_arr = dot(SUN_HAT_ARR, active_faces.norm, 2); 
s_term = (1 - Cs) .* SUN_HAT_ARR; 
n_term = 2 .* (Cs.*cos_theta_arr + (Cd/3)) .* active_faces.norm;
df_arr = -P .* (s_term + n_term) .* cos_theta_arr .* active_faces.area;

% Calculate total torque in principal axes
torques_arr = cross(active_faces.bary, df_arr, 2); 
srp_torque_body = sum(torques_arr)';
srp_torque = visors.A_rot * srp_torque_body;
end