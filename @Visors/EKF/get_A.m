function A = get_A(x, u, dt)
% Purpose: calculate STM Phi (AKA dynamics Jacobian A)

% Renaming variables
wx = x(1); wy = x(2); wz = x(3);
q1 = x(4); q2 = x(5); q3 = x(6); q4 = x(7);

% Get inertia properties
visorsInertia;

% Calculating Omega array
Omega = [  0, +wz, -wy, +wx; ...
         -wz,   0, +wx, +wy; ...
         +wy, -wx,   0, +wz; ...
         -wx, -wy, -wz,   0];

% Xi array of quaternions
Xi = [q4, -q3, q2; ...
      q3, q4, -q1; ...
      -q2, q1, q4; ...
      -q1, -q2, -q3];
  
% Upper left hand quadrant
H_1 = [                 1, ((Iy-Iz)/Ix)*dt*wz, ((Iy-Iz)/Ix)*dt*wy; ...
       ((Iz-Ix)/Iy)*dt*wz,                  1, ((Iz-Ix)/Iy)*dt*wx; ...
       ((Ix-Iy)/Iz)*dt*wy, ((Ix-Iy)/Iz)*dt*wx,                  1];


% Calculate Jacobian at current timestep
A = [        H_1,                zeros(3,4); ...
     0.5.*dt.*Xi, (eye(4) + 0.5.*dt.*Omega)];
end