function [omega_out, quat_out] = propagate_attitude(t_arr, w0, q0)
% Propagate angular velocity and quaternion according to timesteps
% contained in t_arr and with initial angular velcocity w0 and initial
% quaternion q0

visorsInertia;

% Numerical integrator options
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

% Number of indices 
N = length(t_arr);

% Output vars
omega_out = zeros(3, N);
quat_out = zeros(4, N);
omega_out(:,1) = w0;
quat_out(:,1) = q0;

for i = 1:N-1
    % Next timestep
    t_prop = [t_arr(i) t_arr(i+1)];
    
    % Step angular velocity 
    w0 = omega_out(:,i);
    [t_out, w_out] = ode45(@(t,y) int_Euler_eqs(t,y,I_princ), t_prop, w0, options);
    omega_out(:,i+1) = w_out(end,:)';
    
    % Step quaternion
    q0 = quat_out(:,i);
    [t_out, q_out] = ode45(@(t,y) int_quaternion(t,y,w_out(end,:)'), t_prop, q0, options);
    quat_out(:,i+1) = q_out(end,:)'./norm(q_out(end,:)');
end

end