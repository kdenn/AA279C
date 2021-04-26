function ydot = int_Euler_eqs_rotor(t,y,I_princ,Ir,r)
% Integrate Euler's Equations of Motion for angular momentum in principal axes

% Renaming variables
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);
wx = y(1); wy = y(2); wz = y(3); wr = y(4); 
rx = r(1); ry = r(2); rz = r(3);
Mx = 0; My = 0; Mz = 0; Mr = 0;

% State space form
ydot = zeros(4,1);
ydot(4) = Mr / Ir; 
ydot(1) = (Mx - (Iz-Iy)*wy*wz - Ir*wr*(wy*rz-wz*ry) - Ir*ydot(4)*rx) / Ix;
ydot(2) = (My - (Ix-Iz)*wz*wx - Ir*wr*(wz*rx-wx*rz) - Ir*ydot(4)*ry) / Iy;
ydot(3) = (Mz - (Iy-Ix)*wx*wy - Ir*wr*(wx*ry-wy*rx) - Ir*ydot(4)*rz) / Iz;
end