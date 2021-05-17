function A = get_A(x, u, dt)
% Purpose: calculate dynamics Jacobian

% Renaming variables
px = x(1); py = x(2); theta = x(3);
v = u(1); phi = u(2);

% Calculate Jacobian at current timestep
A = [1, 0, -dt*v*sin(theta); ...
     0, 1, +dt*v*cos(theta); ...
     0, 0,               1];
end