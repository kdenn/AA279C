function x_tp1 = f(x, u, dt)
% Purpose: calculate x_{t+1} = f(x_t, u_t)

% Renaming variables
wx = x(1); wy = x(2); wz = x(3);
q1 = x(4); q2 = x(5); q3 = x(6); q4 = x(7); q_vec = [q1; q2; q3; q4];
Mx = u(1); My = u(2); Mz = u(3);

% Get inertia properties
visorsInertia;

% Calculating Omega array
Omega = [  0, +wz, -wy, +wx; ...
         -wz,   0, +wx, +wy; ...
         +wy, -wx,   0, +wz; ...
         -wx, -wy, -wz,   0];

% X at time t+1 
x_tp1 = zeros(7,1);
x_tp1(1) = wx + (dt * (Mx - (Iz - Iy) * wy * wz) / Ix);
x_tp1(2) = wy + (dt * (My - (Ix - Iz) * wz * wx) / Iy);
x_tp1(3) = wz + (dt * (Mz - (Iy - Ix) * wx * wy) / Iz);
x_tp1(4:7) = eye(4)*q_vec + 0.5.*dt.*( Omega * q_vec );
end