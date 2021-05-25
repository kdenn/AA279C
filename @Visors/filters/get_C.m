function C = get_C(x, u, dt)
% Purpose: calculate measurement Jacobian (aka H matrix in 279C notation)

% Renaming variables
wx = x(1); wy = x(2); wz = x(3);
q1 = x(4); q2 = x(5); q3 = x(6); q4 = x(7);

% Get ECI directions to Sirius and Alpha Centauri
[Sir_ECI, Alp_ECI] = Visors.get_ref_vecs_true();

% For Sx or Ax component
q_mtx_1 = [+q1, +q2, +q3; ...
           -q2, +q1, -q4; ...
           -q3, +q4, +q1; ...
           +q4, +q3, -q2];
    
% For Sy or Ay component
q_mtx_2 = [+q2, -q1, +q4; ...
           +q1, +q2, +q3; ...
           -q4, -q3, +q2; ...
           -q3, +q4, +q1];
    
% For Sz or Az component
q_mtx_3 = [+q3, -q4, -q1; ...
           +q4, +q3, -q2; ...
           +q1, +q2, +q3; ...
           +q2, -q1, +q4];
      
% Jacobian matrix for Sirius with respect to quaternion
J_Sir = [(2 .*  q_mtx_1 * Sir_ECI)'; ...
         (2 .*  q_mtx_2 * Sir_ECI)'; ...
         (2 .*  q_mtx_3 * Sir_ECI)'];
 
% Jacobian matrix for Alpha Centauri with respect to quaternion
J_Alp = [(2 .*  q_mtx_1 * Alp_ECI)'; ...
         (2 .*  q_mtx_2 * Alp_ECI)'; ...
         (2 .*  q_mtx_3 * Alp_ECI)'];
     
% Since taking Jacobians with the additional rotation from principal to
% body axes makes everything more difficult
visorsInertia;
DCM = A_rot'; % prin->body
AA = DCM(1,1); BB = DCM(1,2); CC = DCM(1,3);
DD = DCM(2,1); EE = DCM(2,2); FF = DCM(2,3);
GG = DCM(3,1); HH = DCM(3,2); II = DCM(3,3);

% Jacobian matrix for Sirius with respect to quaternion
J_Sir = [(2 .*  (AA.*q_mtx_1 + BB.*q_mtx_2 + CC.*q_mtx_3) * Sir_ECI)'; ...
         (2 .*  (DD.*q_mtx_1 + EE.*q_mtx_2 + FF.*q_mtx_3) * Sir_ECI)'; ...
         (2 .*  (GG.*q_mtx_1 + HH.*q_mtx_2 + II.*q_mtx_3) * Sir_ECI)'];
 
% Jacobian matrix for Alpha Centauri with respect to quaternion
J_Alp = [(2 .*  (AA.*q_mtx_1 + BB.*q_mtx_2 + CC.*q_mtx_3) * Alp_ECI)'; ...
         (2 .*  (DD.*q_mtx_1 + EE.*q_mtx_2 + FF.*q_mtx_3) * Alp_ECI)'; ...
         (2 .*  (GG.*q_mtx_1 + HH.*q_mtx_2 + II.*q_mtx_3) * Alp_ECI)'];

J_Sir = jacobian_st(Sir_ECI(1),Sir_ECI(2),Sir_ECI(3),q1,q2,q3,q4);
J_Alp = jacobian_st(Alp_ECI(1),Alp_ECI(2),Alp_ECI(3),q1,q2,q3,q4);
     
% Output measurement sensitivity matrix C (AA273) or H (AA279C)
C = [    eye(3), zeros(3,4); ...
     zeros(3,3),      J_Sir; ...
     zeros(3,3),      J_Alp];
end