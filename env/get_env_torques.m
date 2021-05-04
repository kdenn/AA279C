function env_torques = get_env_torques(visors, JD, R_ECI, V_ECI, quat, flags)
% Wrapper function for calculating environmental torques (magnetic, SRP,
% drag, and gravity gradient)

grav_torque = zeros(3,1);
srp_torque = zeros(3,1);
drag_torque = zeros(3,1);
mag_torque = zeros(3,1);

% Individual contributions
if flags(1); grav_torque = grav_grad(visors.I_princ, visors.mu, quat2dcm(quat)*R_ECI); end
if flags(2); srp_torque = get_srp_torque(visors, JD, R_ECI, V_ECI, quat); end
if flags(3); drag_torque = get_drag_torque(visors, JD, R_ECI, V_ECI, quat); end
if flags(4); mag_torque = get_mag_torque(visors, JD, R_ECI, quat, 4); end

% Output struct
env_torques = struct();
env_torques.grav = grav_torque;
env_torques.srp = srp_torque;
env_torques.drag = drag_torque;
env_torques.mag = mag_torque;
env_torques.all = grav_torque + srp_torque + drag_torque + mag_torque;

end
