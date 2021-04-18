function ydot = int_Euler_Angles(t,y,omega)
% Integrate kinematic equations of motion for Euler Angles attitude
% parameterization

% Renaming variables
wx = omega(1); wy = omega(2); wz = omega(3);
phi = y(1); theta = y(2); psi = y(3);

% State space form
ydot = zeros(3,1);
ydot(1) = (wx*sin(psi) + wy*cos(psi)) / sin(theta);
ydot(2) = wx*cos(psi) - wy*sin(psi);
ydot(3) = wz - (wx*sin(psi) + wy*cos(psi)) / tan(theta);
end