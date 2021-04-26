function DCM = quat2dcm(q)
% Given input quaternion attitude, outputs the DCM from reference frame to
% the principal axis

% q1 = quat(1); q2 = quat(2); q3 = quat(3); q4 = quat(4);
% qv = quat(1:3); qs = quat(4);
% q_cross = [  0, -q3, +q2; ...
%            +q3,   0, -q1; ...
%            -q2, +q1,   0];
% DCM = diag(q4^2 - qv.^2) + 2.*(qv*qv') - 2.*q4.*q_cross;

% Method used in Diebel Attitude document

	N = size(q,2);
	DCM = zeros(3,3,N);

	for n = 1:N
        qvec = [q(1); q(2); q(3)];
        qcross = [ 0 -q(3) q(2);...
                   q(3) 0 -q(1);...
                   -q(2) q(1) 0];
        A =(q(4)^2-norm(qvec)^2)*eye(3)+2*(qvec*qvec.')-2*q(4)*qcross;
        column1 = A(:,1); column2 = A(:,2); column3 = A(:,3); 
        DCM(:,:,n)=[column1/norm(column1) column2/norm(column2) column3/norm(column3)];
	end
end