function ydot = int_Euler_eqs_grav(t,y,I_princ,r,mu)
% Integrate Euler's Equations of Motion for angular momentum in principal
% axes
%{
    y:          3x1 double, angular velocity (omega) in rad/s
    I_princ:    3x3 double, moment of inertia of principal axis
    r:          3x1 double, vector from CB to SC in the principal axis frame
    mu:         scalar double, gravitational constant of CB
%}

% Renaming variables
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);
wx = y(1); wy = y(2); wz = y(3);

% Gravity gradient
R = norm(r);
c = r./R;
cx = c(1); cy = c(2); cz = c(3);
M = (3*mu)/(R^3) * [(Iz - Iy)*cy*cz;
                    (Ix - Iz)*cz*cx;
                    (Iy - Ix)*cx*cy];
Mx = M(1); My = M(2); Mz = M(3);

% State space form
ydot = zeros(3,1);
ydot(1) = (Mx - (Iz - Iy) * wy * wz) / Ix;
ydot(2) = (My - (Ix - Iz) * wz * wx) / Iy;
ydot(3) = (Mz - (Iy - Ix) * wx * wy) / Iz;
end