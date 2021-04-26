function ydot = int_Euler_eqs(t,y,I_princ,M)
% Integrate Euler's Equations of Motion for angular momentum in principal
% axes

% Renaming variables
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);
wx = y(1); wy = y(2); wz = y(3);
if nargin == 3
    Mx = 0; My = 0; Mz = 0;
else
    Mx = M(1); My = M(2); Mz = M(3);
end

% State space form
ydot = zeros(3,1);
ydot(1) = (Mx - (Iz - Iy) * wy * wz) / Ix;
ydot(2) = (My - (Ix - Iz) * wz * wx) / Iy;
ydot(3) = (Mz - (Iy - Ix) * wx * wy) / Iz;
end