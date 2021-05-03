function [omega_out, quat_out, rv_ECI_out, M_out] = propagate_attitude(t_arr, w0, q0, flags, vsr)
% Propagate angular velocity and quaternion according to timesteps
% contained in t_arr and with initial angular velcocity w0 and initial
% quaternion q0

% Propagation happens in the principle axis frame so w0 and q0 should
% be represented in a frame aligned with the principle axis

% These attitude parameters describe orientation of principal axes relative
% to inertial axes.

% If you want gravity gradient, q0 needs to be ECI > princ

if nargin ==  3
    flags = zeros(1,1);
    vsr = visorsStruct();
elseif nargin == 4
    vsr = visorsStruct();
end

flag_grav = flags(1);

% Numerical integrator options
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

% Number of indices 
N = length(t_arr);

% Output vars
omega_out = zeros(3, N);
quat_out = zeros(4, N);
rv_ECI_out = zeros(6, N);
M_out = zeros(3,N);
omega_out(:,1) = w0; % Princ
quat_out(:,1) = q0; % ECI > Princ
rv_ECI_out(:,1) = [vsr.r_ECI_0;vsr.v_ECI_0];

for i = 1:N-1
    % Next timestep
    t_prop = [t_arr(i) t_arr(i+1)];
    
    if flag_grav
        % ECI position
        rv0 = rv_ECI_out(:,i);
        [~,rv_out] = ode45(@FODEint, t_prop, rv0, options);
        rv_ECI_out(:,i+1) = rv_out(end,:)';
        R_ECI2princ = quat2dcm(q0);
        
        % Gravity gradient torque
        r_princ = R_ECI2princ*rv0(1:3);
        M0 = grav_grad(vsr.I_princ,vsr.mu,r_princ);
        M_out(:,i) = M0;
    else
        M0 = zeros(3,1);
    end
    
    % Step angular velocity (Euler propagates in princ)
    w0 = omega_out(:,i);
    [~, w_out] = ode45(@(t,y) int_Euler_eqs(t,y,vsr.I_princ,M0), t_prop, w0, options);
    omega_out(:,i+1) = w_out(end,:)';
    
    % Step quaternion
    q0 = quat_out(:,i);
    [~, q_out] = ode45(@(t,y) int_quaternion(t,y,w_out(end,:)'), t_prop, q0, options);
    quat_out(:,i+1) = q_out(end,:)'./norm(q_out(end,:)');
end

if flag_grav
    R_ECI2princ = quat2dcm(q0);
    r_princ = R_ECI2princ*rv0(1:3);
    M0 = grav_grad(vsr.I_princ,vsr.mu,r_princ);
    M_out(:,N) = M0;
end

end