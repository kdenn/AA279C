function env_torques = get_env_torques(visors, JD, R_ECI, V_ECI, quat)
% Wrapper function for calculating environmental torques (magnetic, SRP,
% drag, and gravity gradient)

% Individual contributions
srp_torque = get_srp_torque(visors, JD, R_ECI, V_ECI, quat);
drag_torque = get_drag_torque(visors, JD, R_ECI, V_ECI, quat);

% Output struct
env_torques = struct();
env_torques.srp = srp_torque;
env_torques.drag = drag_torque;
env_torques.all = srp_torque + drag_torque;
end