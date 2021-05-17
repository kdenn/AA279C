% Define simulation constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Random number generator
rng(1);

% Sim time parameters
t0 = 0; dt = 0.01; tf = 20; t_arr = (t0:dt:tf)'; N = length(t_arr);

% Constants and initial conditions
dim_s = 3;                  % state dimension
dim_m = 1;                  % measurement dimension