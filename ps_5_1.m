% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

% % Initialize struct
% visors = visorsStruct();
% 
% % Initial conditions for testing
% JD = visors.JD_epoch;
% R_ECI = visors.r_ECI_0;
% V_ECI = visors.v_ECI_0; 
% quat = [0; 0; 0; 1];
% 
% % Test functions
% env_torques = get_env_torques(visors, JD, R_ECI, V_ECI, quat);


%%
% Initial Conditions
w0 = deg2rad([0;0.1;1]);
q0 = [0; 0; 0; 1];

% Sim time parameters
t0 = 0; dt = 0.5; tf = 60*10; t_arr = (t0:dt:tf)';

% Visors class (not struct)
visors = Visors(w0, q0);

[omega_out, quat_out, rv_ECI_out, M_out] = visors.propagate(t_arr);

plot_attitude_3D(visors.ICs, quat_out);

