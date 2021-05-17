function x_tp1 = f(x, u, dt)
% Purpose: calculate x_{t+1} = f(x_t, u_t)

% Renaming variables
px = x(1); py = x(2); theta = x(3);
v = u(1); phi = u(2);

% X at time t+1 
x_tp1 = [0; 0; 0];
x_tp1(1) = px + dt*v*cos(theta);
x_tp1(2) = py + dt*v*sin(theta);
x_tp1(3) = theta + dt*phi;
end