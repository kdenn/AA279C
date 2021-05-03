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

