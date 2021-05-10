function [omega_out, quat_out, rv_ECI_out, M_out] = propagate(obj, t_arr, flags)
% Propagate angular velocity and quaternion according to timesteps
% contained in t_arr and with initial angular velcocity w0 and initial
% quaternion q0

% Propagation happens in the principle axis frame so w0 and q0 should
% be represented in a frame aligned with the principle axis

% These attitude parameters describe orientation of principal axes relative
% to inertial axes.

% If you want gravity gradient, q0 needs to be ECI > princ

% flags: 
    % 1: gravity gradient
    % 2: solar radiation pressure
    % 3: drag
    % 4: magnetic field
if nargin == 2
    flags = ones(1,4);
end

% Numerical integrator options
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

% Number of indices 
N = length(t_arr);

% Output vars
omega_out = zeros(3, N);
quat_out = zeros(4, N);
rv_ECI_out = zeros(6, N);
M_out = zeros(3, N, 5);         % (:,:,1) = gravity gradient
                                % (:,:,2) = solar radiation pressure
                                % (:,:,3) = drag
                                % (:,:,4) = magnetic
                                % (:,:,5) = sum of all

% Append ICs to output arrays
omega_out(:,1) = obj.w0; % Princ
quat_out(:,1) = obj.q0; % ECI > Princ
rv_ECI_out(:,1) = [obj.ICs.r_ECI_0; obj.ICs.v_ECI_0];

for i = 1:N-1
    % Current time 
    t = t_arr(i);
    JD_curr = obj.ICs.JD_epoch + (t/86400); 
    
    % Values at current timestep 
    rv = rv_ECI_out(:,i);
    w = omega_out(:,i);
    q = quat_out(:,i);
    
    % Get environmental torques
    env_torques = get_env_torques(obj.ICs, JD_curr, rv(1:3), rv(4:6), q, flags);
    M0 = env_torques.all;
    
    % Update environmental torques output variable
    M_out(:,i+1,1) = env_torques.grav;
    M_out(:,i+1,2) = env_torques.srp;
    M_out(:,i+1,3) = env_torques.drag;
    M_out(:,i+1,4) = env_torques.mag;
    M_out(:,i+1,5) = env_torques.all;
    
    % Get reference direction measurements and calculate q
    [m1_meas, m2_meas, m1_true, m2_true] = obj.get_ref_vecs_meas(JD_curr, q);
    q_est = obj.opts.est_q(m1_meas, m2_meas, m1_true, m2_true);
    obj.est.q = [obj.est.q, q_est];
    
    % Get angular velocity measurements and calculate q
    w_est = obj.get_w_meas(w);
    if length(obj.est.q_from_w) == 0
        q_in = q;
    else
        q_in = obj.est.q_from_w(:,end);
    end
    q_from_w = obj.calc_q_from_w(w_est, q_in, [t t_arr(i+1)], options);
    obj.est.q_from_w = [obj.est.q_from_w, q_from_w];
    
    % For propagation to next timestep
    t_prop = [t t_arr(i+1)];
    
    % Step ECI position/velocity
    [~,rv_out] = ode45(@FODEint, t_prop, rv, options);
    rv_ECI_out(:,i+1) = rv_out(end,:)';
    
    % Step angular velocity (Euler propagates in princ)
    [~, w_out] = ode45(@(t,y) int_Euler_eqs(t,y,obj.ICs.I_princ,M0), t_prop, w, options);
    omega_out(:,i+1) = w_out(end,:)';
    
    % Step quaternion
    [~, q_out] = ode45(@(t,y) int_quaternion(t,y,w_out(end,:)'), t_prop, q, options);
    quat_out(:,i+1) = q_out(end,:)'./norm(q_out(end,:)');
end

obj.true.w = omega_out;
obj.true.q = quat_out;
obj.true.rv_ECI = rv_ECI_out;
obj.true.M_env = M_out;

end