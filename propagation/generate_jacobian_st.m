syms q1 q2 q3 q4 Si Sj Sk

visorsInertia;

%DCM from ECI -> prin
DCM = [2*q1^2 - q2^2 - q3^2 - q1^2 + q4^2, 2*q1*q2 + 2*q3*q4,                  2*q1*q3 - 2*q2*q4;...
       2*q1*q2 - 2*q3*q4,                  2*q2^2 - q2^2 - q3^2 - q1^2 + q4^2, 2*q1*q4 + 2*q2*q3;...
       2*q1*q3 + 2*q2*q4,                  2*q2*q3 - 2*q1*q4,                  2*q3^2 - q2^2 - q3^2 - abs(q1)^2 + q4^2];

S_body = A_rot'*DCM*[Si;Sj;Sk];

dSdq = jacobian(S_body,[q1 q2 q3 q4]);

matlabFunction(dSdq,'File','propagation/jacobian_st');