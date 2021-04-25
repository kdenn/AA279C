function [omega_out, alpha_out] = propagate_attitude_Eul_ang(t_arr, w0, q0)
% Propagate angular velocity and quaternion according to timesteps
% contained in t_arr and with initial angular velcocity w0 and initial
% quaternion q0

% Propagation happens in the principle axis frame so w0 and q0 should
% be represented in a frame aligned with the principle axis

visorsInertia;

% Numerical integrator options
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

% Number of indices 
N = length(t_arr);

% Output vars
omega_out = zeros(3, N);
alpha_out = zeros(3, N);
omega_out(:,1) = w0;
alpha_out(:,1) = q0;

for i = 1:N-1
    % Next timestep
    t_prop = [t_arr(i) t_arr(i+1)];
    
    % Step angular velocity 
    w0 = omega_out(:,i);
    [t_out, w_out] = ode45(@(t,y) int_Euler_eqs(t,y,I_princ), t_prop, w0, options);
    omega_out(:,i+1) = w_out(end,:)';
    
    % Step Euler Angle
    q0 = alpha_out(:,i);
    [t_out, q_out] = ode45(@(t,y) int_Euler_Angles(t,y,w_out(end,:)'), t_prop, q0, options);
    alpha_out(:,i+1) = wrapToPi(q_out(end,:)');
end

end