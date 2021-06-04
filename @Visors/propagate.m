function [omega_true, quat_true, rv_ECI_out, M_out, EKF_out, states, L_dot] = propagate(obj, t_arr, flags)
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
    % 5: EKF (1), q-method (2), or true state (0)
    % 6: linear (1), nonlinear (2), or no control (0)
    % 7: control noise
if nargin == 2
    flags = ones(1,5);
end

% Numerical integrator options
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

% Number of indices 
N = length(t_arr);

% Output vars
omega_true = zeros(3, N);
omega_des = zeros(3, N);
omega_est = zeros(3, N);
quat_true = zeros(4, N);
quat_des = zeros(4, N);
quat_est = zeros(4, N);
rv_ECI_out = zeros(6, N);
M_out = zeros(3, N, 7);         % (:,:,1) = gravity gradient
                                % (:,:,2) = solar radiation pressure
                                % (:,:,3) = drag
                                % (:,:,4) = magnetic
                                % (:,:,5) = sum of all env
                                % (:,:,6) = control desired
                                % (:,:,7) = control actual
L_dot = zeros(4,N,2);
                                
% Append ICs to output arrays
omega_true(:,1) = obj.w0; % Princ
quat_true(:,1) = obj.q0; % ECI > Princ
rv_ECI_out(:,1) = [obj.ICs.r_ECI_0; obj.ICs.v_ECI_0];

%% EKF initialization
dt = diff(t_arr(1:2));
load('propagation/noises.mat'); % noises
Qw_p = flags(1).*10.*noises.Qw_g + flags(2).*noises.Qw_s + flags(3).*noises.Qw_d + flags(4).*noises.Qw_m;
Qq_p = flags(1).*10.*noises.Qq_g + flags(2).*noises.Qq_s + flags(3).*noises.Qq_d + flags(4).*noises.Qq_m;
Qb_w = 5e-3.*[1,1,1];
Qb_q = 5e-2.*[1,1,1,1];
Q = dt*(diag([Qb_w,Qb_q]) + blkdiag(Qw_p,Qq_p));
R_st = (deg2rad(40/3600))^2 * eye(3);
R_imu = (1.7453E-4)^2 * eye(3);
R = blkdiag(R_imu,R_st,R_st);
% q0 = obj.q0 + 5*sqrtm((deg2rad(40/3600))^2 * eye(4))*randn(4,1);
% q0 = unitVec(q0);
% w0 = obj.w0 + 5*sqrtm(R_imu)*randn(3,1);
mu = [obj.w0;obj.q0] + 2*sqrtm(Q)*randn(7,1);
mu(4:7) = unitVec(mu(4:7));
mu
cov = (1e3)*blkdiag(R_imu,(deg2rad(40/3600))^2 * eye(4));

mu_arr = zeros(7,N);
mu_arr(:,1) = mu;
cov_arr = zeros(7,7,N);
cov_arr(:,:,1) = cov;
obs_rank_arr = zeros(1,N);
z_pre_arr = zeros(9,N);
z_post_arr = zeros(9,N);

%% Begin Propagation
for i = 1:N-1
    % Current time 
    t = t_arr(i);
    t_prop = [t t_arr(i+1)];
    dt = diff(t_prop);
    JD_curr = obj.ICs.JD_epoch + (t/86400); 
    rv = rv_ECI_out(:,i);
    w = omega_true(:,i);
    q = quat_true(:,i);
    u = M_out(:,i,7);
    
    %% Get environmental torques
    env_torques = get_env_torques(obj.ICs, JD_curr, rv(1:3), rv(4:6), q, flags);
    M = env_torques.all;
    
    % Update environmental torques output variable
    M_out(:,i+1,1) = env_torques.grav;
    M_out(:,i+1,2) = env_torques.srp;
    M_out(:,i+1,3) = env_torques.drag;
    M_out(:,i+1,4) = env_torques.mag;
    M_out(:,i+1,5) = env_torques.all;
    
    %% Get Estimates of q and w
    % Direct estimate (non-filter) method
    [m1_meas, m2_meas, m1_true, m2_true] = obj.get_ref_vecs_meas(q);
    q_est = obj.opts.est_q(m1_meas, m2_meas, m1_true, m2_true);
    obj.est.q = [obj.est.q, q_est];
    w_est = obj.get_w_meas(w);
    if length(obj.est.q_from_w) == 0
        q_in = q;
    else
        q_in = obj.est.q_from_w(:,end);
    end
    q_from_w = obj.calc_q_from_w(w_est, q_in, [t t_arr(i+1)], options);
    obj.est.q_from_w = [obj.est.q_from_w, q_from_w];
    
    % Onboard State Estimation
    switch flags(5)
        case 1
            y = [w_est;m1_meas;m2_meas];
            mu = mu_arr(:,i);
            cov = cov_arr(:,:,i);
            [mu_p,cov,A,C,z_pre,z_post] = EKFfilter(@f,@get_A,@g,@get_C,Q,R,mu,cov,y,u,dt);
            mu_p(4:7) = unitVec(mu_p(4:7));
            mu_arr(:,i+1) = mu_p;
            cov_arr(:,:,i+1) = cov;
            obs_rank_arr(i+1) = rank(obsv(A,C));
            z_pre_arr(:,i+1) = z_pre;
            z_post_arr(:,i+1) = z_post;
        case 2
            mu = [w_est;q_est];
        otherwise
            mu = [w;q];
    end
    omega_est(:,i) = mu(1:3);
    quat_est(:,i) = mu(4:7);
    
    %% Control
    q_des = obj.calc_q_des(t);
    quat_des(:,i) = q_des;
    w_des = obj.w0;
    omega_des(:,i) = w_des;
    if i > 10 % && mod(t,30) == 0
        switch flags(6)
            case 1
                M_c_des = obj.linear_ctrl(q_des, mu(4:7), w_des, mu(1:3));
                [M_c_act,L_dot_act,L_dot_cmd] = obj.actuate_RW(M_c_des,w,dt,flags(7));
                L_dot(:,i+1,1) = L_dot_act;
                L_dot(:,i+1,2) = L_dot_cmd;
            case 2
                M_c_des = obj.nonlinear_ctrl(q_des, mu(4:7), w_des, mu(1:3));
                [M_c_act,L_dot_act,L_dot_cmd] = obj.actuate_RW(M_c_des,w,dt,flags(7));
                L_dot(:,i+1,1) = L_dot_act;
                L_dot(:,i+1,2) = L_dot_cmd;
            otherwise
                M_c_des = [0;0;0];
                M_c_act = [0;0;0];
        end
        M_out(:,i+1,6) = M_c_des;
        M_out(:,i+1,7) = M_c_act;
        M = M + M_c_act;
    end
    
    %% Propagate Forward
    % Step ECI position/velocity
    [~,rv_out] = ode45(@FODEint, t_prop, rv, options);
    rv_ECI_out(:,i+1) = rv_out(end,:)';
    
    % Step angular velocity (Euler propagates in princ)
    [~, w_out] = ode45(@(t,y) int_Euler_eqs(t,y,obj.ICs.I_princ,M), t_prop, w, options);
    omega_true(:,i+1) = w_out(end,:)';
    
    % Step quaternion
    [~, q_out] = ode45(@(t,y) int_quaternion(t,y,w_out(end,:)'), t_prop, q, options);
    quat_true(:,i+1) = q_out(end,:)'./norm(q_out(end,:)');
end
q_des = obj.calc_q_des(t_arr(N));
quat_des(:,N) = q_des;
omega_des(:,N) = w_des;
quat_est(:,N) = quat_est(:,N-1);
omega_est(:,N) = omega_est(:,N-1);

%% Storage

obj.true.w = omega_true;
obj.true.w_des = omega_des;
obj.true.w_est = omega_est;
obj.true.q = quat_true;
obj.true.q_des = quat_des;
obj.true.q_est = quat_est;
obj.true.rv_ECI = rv_ECI_out;
obj.true.M_env = M_out(:,:,1:5);
obj.true.M_ctl = M_out(:,:,6:7);

states.w_true = omega_true;
states.w_des = omega_des;
states.w_est = omega_est;
states.q_true = quat_true;
states.q_des = quat_des;
states.q_est = quat_est;

EKF_out.mu_arr = mu_arr;
EKF_out.cov_arr = cov_arr;
EKF_out.obs_rank_arr = obs_rank_arr;
EKF_out.z_pre_arr = z_pre_arr;
EKF_out.z_post_arr = z_post_arr;

end