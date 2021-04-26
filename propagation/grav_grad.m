function M = grav_grad(I_princ,mu,r)
% Calculate torque dues to gravity gradient
%{
    I_princ:    3x3 double, moment of inertia of principal axis
    mu:         scalar double, gravitational constant of CB
    r:          3x1 double, vector from CB to SC in the principal axis frame
%}

Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);
R = norm(r);
c = r./R;
cx = c(1); cy = c(2); cz = c(3);
M = ((3*mu)/(R^3)) * [(Iz - Iy)*cy*cz;
                      (Ix - Iz)*cz*cx;
                      (Iy - Ix)*cx*cy];
                
end