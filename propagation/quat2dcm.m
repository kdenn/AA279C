function DCM = quat2dcm(quat)
% Given input quaternion attitude, outputs the DCM from inertial
% coordinates to principal axes

% q1 = quat(1); q2 = quat(2); q3 = quat(3); q4 = quat(4);
% qv = quat(1:3); qs = quat(4); 
% q_cross = [  0, -q3, +q2; ...
%            +q3,   0, -q1; ...
%            -q2, +q1,   0];
% DCM = diag(q4^2 - qv.^2) + 2.*(qv*qv') - 2.*q4.*q_cross;

% Method used in Diebel Attitude document
q1 = quat(1); q2 = quat(2); q3 = quat(3); q0 = quat(4);
DCM =  [(q0^2+q1^2-q2^2-q3^2), (2*q1*q2+2*q0*q3), (2*q1*q3-2*q0*q2); ...
        (2*q1*q2-2*q0*q3), (q0^2-q1^2+q2^2-q3^2), (2*q2*q3+2*q0*q1); ...
        (2*q1*q3+2*q0*q2), (2*q2*q3-2*q0*q1), (q0^2-q1^2-q2^2+q3^2)];
end