function drag_torque = get_drag_torque(visors, JD, R_ECI, V_ECI, quat)
% Calculate torque due to drag in principal axes 

% Calculate velocity direction in body coordinates
V_PRIN = quat2dcm(quat) * V_ECI;
V_BODY = visors.A_rot' * V_PRIN;
V_HAT_BODY = V_BODY ./ norm(V_BODY);

% Get faces that contribute to drag
active_faces = get_active_faces(visors.faces, V_HAT_BODY);

% Cd for a flat plate
Cd = 1.28;

% Exponential atmospheric density model
% rho_0 = 1.225; % kg/m^3 
% H = 10000; % m - characteristic height
% rho = rho_0 * exp(-1 * (norm(R_ECI)-s3_constants('R_EARTH_WGS84')) / H);

% From Wertz Appendix L.3 (pg 820) for 600 km altitude
rho = 1.454e-13;

% Get force contribution from each surface
V_mag = norm(V_BODY);
V_HAT_BODY_ARR = repmat(V_HAT_BODY', active_faces.N, 1);
df_arr = -0.5*Cd*rho*(V_mag^2) .* dot(V_HAT_BODY_ARR, active_faces.norm, 2) ...
         .* active_faces.area .* V_HAT_BODY_ARR;

% Calculate total torque in principal axes
torques_arr = cross(active_faces.bary, df_arr, 2); 
drag_torque_body = sum(torques_arr)';
drag_torque = visors.A_rot * drag_torque_body;
end