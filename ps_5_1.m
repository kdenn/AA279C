% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

% Initialize struct
visors = visorsStruct();

% Initial conditions for testing
JD = visors.JD_epoch;
R_ECI = visors.r_ECI_0;
V_ECI = visors.v_ECI_0; 
quat = [0; 0; 0; 1];

% Test functions
env_torques = get_env_torques(visors, JD, R_ECI, V_ECI, quat)

function env_torques = get_env_torques(visors, JD, R_ECI, V_ECI, quat)
% Wrapper function for calculating environmental torques (magnetic, SRP,
% and drag)

% Individual contributions
srp_torque = get_srp_torque(visors, JD, R_ECI, V_ECI, quat);
drag_torque = get_drag_torque(visors, JD, R_ECI, V_ECI, quat);

% Output struct
env_torques = struct();
env_torques.srp = srp_torque;
env_torques.drag = drag_torque;
env_torques.all = srp_torque + drag_torque;
end

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

function active_faces = get_active_faces(faces, ref_vec)
% Returns a struct only containing the 'active' or 'wet' faces. I.e. the
% faces that are lit (for SRP torques) or faces that can impact with
% atmosphere particles (for drag torques)

% Number of faces
N = faces.N; 

% Nx3 matrix where each row is ref_vec
ref_vec_arr = repmat(ref_vec', N, 1);

% Nx1 vector of dot products between ref vector and unit normal of face
dot_prods = dot(ref_vec_arr, faces.norm, 2); 

% Indices of 'active' faces
ind_active = dot_prods > 0;

% Return struct
active_faces = struct(); 
active_faces.bary = faces.bary(ind_active,:);
active_faces.area = faces.area(ind_active);
active_faces.norm = faces.norm(ind_active,:);
active_faces.N = length(active_faces.area);
end