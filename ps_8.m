% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 60*30; t_arr = (t0:dt:tf)';

% Visors class
vsrs = Visors(w0, q0);

M_act = vsrs.actuate_RW([0.0001; 0.0002; 0.0003])