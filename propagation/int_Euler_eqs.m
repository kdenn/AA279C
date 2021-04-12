function ydot = int_Euler_eqs(t,y,I_princ)
% Joshua Geiser

% Renaming variables
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);
wx = y(1); wy = y(2); wz = y(3);
Mx = 0; My = 0; Mz = 0;

% State space form
ydot = zeros(3,1);
ydot(1) = (Mx - (Iz - Iy) * wy * wz) / Ix;
ydot(2) = (My - (Ix - Iz) * wz * wx) / Iy;
ydot(3) = (Mz - (Iy - Ix) * wx * wy) / Iz;
end