function env_torques = get_env_torques(visors, JD, R_ECI, V_ECI, quat)
% Wrapper function for calculating environmental torques (magnetic, SRP,
% drag, and gravity gradient)

% Individual contributions
grav_torque = grav_grad(visors.I_princ, visors.mu, quat2dcm(quat)*R_ECI);
srp_torque = get_srp_torque(visors, JD, R_ECI, V_ECI, quat);
drag_torque = get_drag_torque(visors, JD, R_ECI, V_ECI, quat);
mag_torque = get_mag_torque(visors, JD, R_ECI, quat, 4);

% Output struct
env_torques = struct();
env_torques.grav = grav_torque;
env_torques.srp = srp_torque;
env_torques.drag = drag_torque;
env_torques.mag = mag_torque;
env_torques.all = grav_torque + srp_torque + drag_torque + mag_torque;
end
