function [omega_out, quat_out, rv_ECI_out, M_out, EKF_out] = propagate(obj, t_arr, flags)
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
    % 5: EKF
if nargin == 2
    flags = ones(1,5);
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

% EKF initialization
dt = diff(t_arr(1:2));
load('propagation/noises.mat'); % noises
Qw_p = flags(1).*10.*noises.Qw_g + flags(2).*noises.Qw_s + flags(3).*noises.Qw_d + flags(4).*noises.Qw_m;
Qq_p = flags(1).*10.*noises.Qq_g + flags(2).*noises.Qq_s + flags(3).*noises.Qq_d + flags(4).*noises.Qq_m;
Qb_w = 5e-3.*[1,1,1];
Qb_q = 5e-2.*[1,1,1,1];
Q = dt*(diag([Qb_w,Qb_q]) + blkdiag(Qw_p,Qq_p))
R_st = (deg2rad(40/3600))^2 * eye(3);
R_imu = (1.7453E-4)^2 * eye(3);
R = blkdiag(R_imu,R_st,R_st);
% q0 = obj.q0 + 5*sqrtm((deg2rad(40/3600))^2 * eye(4))*randn(4,1);
% q0 = unitVec(q0);
% w0 = obj.w0 + 5*sqrtm(R_imu)*randn(3,1);
mu = [obj.w0;obj.q0] + 2*sqrtm(Q)*randn(7,1);
mu(4:7) = unitVec(mu(4:7));
mu
cov = (1e3)*blkdiag(R_imu,(deg2rad(40/3600))^2 * eye(4))

mu_arr = zeros(7,N);
mu_arr(:,1) = mu;
cov_arr = zeros(7,7,N);
cov_arr(:,:,1) = cov;
obs_rank_arr = zeros(1,N);
z_pre_arr = zeros(9,N);
z_post_arr = zeros(9,N);

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
    M = env_torques.all;
    
    % Update environmental torques output variable
    M_out(:,i+1,1) = env_torques.grav;
    M_out(:,i+1,2) = env_torques.srp;
    M_out(:,i+1,3) = env_torques.drag;
    M_out(:,i+1,4) = env_torques.mag;
    M_out(:,i+1,5) = env_torques.all;
    
    % Get reference direction measurements and calculate q
    [m1_meas, m2_meas, m1_true, m2_true] = obj.get_ref_vecs_meas(q);
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
    dt = diff(t_prop);
    
    % Onboard State Estimation
    if flags(5)
        y = [w_est;m1_meas;m2_meas];
        mu = mu_arr(:,i);
        cov = cov_arr(:,:,i);
        [mu,cov,A,C,z_pre,z_post] = EKFfilter(@f,@get_A,@g,@get_C,Q,R,mu,cov,y,zeros(3,1),dt);
        mu(4:7) = unitVec(mu(4:7));
        mu_arr(:,i+1) = mu;
        cov_arr(:,:,i+1) = cov;
        obs_rank_arr(i+1) = rank(obsv(A,C));
        z_pre_arr(:,i+1) = z_pre;
        z_post_arr(:,i+1) = z_post;
    end
    
    % Step ECI position/velocity
    [~,rv_out] = ode45(@FODEint, t_prop, rv, options);
    rv_ECI_out(:,i+1) = rv_out(end,:)';
    
    % Step angular velocity (Euler propagates in princ)
    [~, w_out] = ode45(@(t,y) int_Euler_eqs(t,y,obj.ICs.I_princ,M), t_prop, w, options);
    omega_out(:,i+1) = w_out(end,:)';
    
    % Step quaternion
    [~, q_out] = ode45(@(t,y) int_quaternion(t,y,w_out(end,:)'), t_prop, q, options);
    quat_out(:,i+1) = q_out(end,:)'./norm(q_out(end,:)');
end

obj.true.w = omega_out;
obj.true.q = quat_out;
obj.true.rv_ECI = rv_ECI_out;
obj.true.M_env = M_out;

EKF_out.mu_arr = mu_arr;
EKF_out.cov_arr = cov_arr;
EKF_out.obs_rank_arr = obs_rank_arr;
EKF_out.z_pre_arr = z_pre_arr;
EKF_out.z_post_arr = z_post_arr;

end