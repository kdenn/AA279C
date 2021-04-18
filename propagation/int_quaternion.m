function ydot = int_quaternion(t,y,omega)
% Integrate kinematic equations of motion for quaternion attitude
% parameterization

% Renaming variables
wx = omega(1); wy = omega(2); wz = omega(3);

% Calculating Omega array
Omega = [  0, +wz, -wy, +wx; ...
         -wz,   0, +wx, +wy; ...
         +wy, -wx,   0, +wz; ...
         -wx, -wy, -wz,   0];

% State space form
ydot = zeros(4,1);
ydot = 0.5 .* (Omega * y);
end