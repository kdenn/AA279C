% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 60*10; t_arr = (t0:dt:tf)'; N = numel(t_arr);

% Visors class
vsrs = Visors(w0, q0);
[omega_out, quat_out, rv_ECI_out, M_out, EKF_out] = vsrs.propagate(t_arr,[0,0,0,0,0]);

plot_attitude_3D(vsrs.ICs, quat_out);