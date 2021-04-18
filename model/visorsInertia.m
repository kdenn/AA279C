% Inertia Properties of VISORS model in body and principal axes

% Body Frame inertia tensor
I_body = [+0.20502, -0.10457, +0.00000; ...
          -0.10457, +0.23232, +0.00000; ...
          +0.00000, +0.00000, +0.25733];
      
% Principal axes 
[A_rot, I_princ] = eig(I_body);
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);