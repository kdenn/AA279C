function [mu_arr, cov_arr] = EKF(Y_arr, u_arr, iEKF)
% Purpose: Run an EKF or iEKF

% TODO: EKF currently takes in history of measurements Y_arr and history of
% control inputs u_arr as inputs and loops through each to calculate
% estimates and covariance. Will probably just want to update this to have
% EKF run a single step (i.e. get rid of for loop, inputs will be a single
% measurement (9x1) and single control input (3x1) so the EKF runs
% synchronously with the simulation (as opposed to offline after
% simulating)

% Sim time parameters
t0 = 0; dt = 1; tf = 20; t_arr = (t0:dt:tf)'; N = length(t_arr);

% Constants and initial conditions
dim_s = 7;                  % state dimension
dim_m = 9;                  % measurement dimension

% Time histories of KF parameters
mu_arr = zeros(dim_s, N);
cov_arr = zeros(dim_s, dim_s, N);

% Noise covariances
Q = 0.100*dt*eye(dim_s);      % TODO: update process noise covariance
R = 0.100*eye(dim_m);         % TODO: update measurement noise covariance

% Initial belief
mu_0 = [0; 0; 0];               % TODO
cov_0 = diag([0.01 0.01 0.01]); % TODO: I think this is the P matrix using 
                                %       D'Amicos 279C notation

% Setting initial condition
mu = mu_0;
cov = cov_0;

% Observability matrix
obs_rank_arr = zeros(N-1,1);

for i = 1:N-1
    
    % Updating mean/covariance estimate time histories
    mu_arr(:,i) = mu;
    cov_arr(:,:,i) = cov;
    
    % Get input at timestep
    u = u_arr(:,i);
        
    % Predict Step
    A = get_A(mu, u, dt);       % A = STM in 279C notation
    mu_bar = f(mu, u, dt);
    cov_bar = A * cov * A' + Q;
    
    % Update Step for iEKF
    if iEKF == 1
        mu = mu_bar; 
        for j = 1:10
            C = get_C(mu, u, dt);   % C = H in 279C notation
            K = cov_bar * C' * inv(C * cov_bar * C' + R);
            mu = mu_bar + K*(Y_arr(:,i+1)-g(mu, u, dt)) + K*C*(mu-mu_bar);
        end
        cov = cov_bar - K * C * cov_bar;
        
    % Update step for standard EKF
    else
        C = get_C(mu_bar, u, dt);   % C = H in 279C notation
        K = cov_bar * C' * inv(C * cov_bar * C' + R);
        mu = mu_bar + K * (Y_arr(:,i+1) - g(mu_bar, u, dt));
        cov = cov_bar - K * C * cov_bar;
    end
    
    % Check observability matrix
    obs_rank_arr(i) = rank(obsv(A,C));
end
mu_arr(:,end) = mu;
cov_arr(:,:,end) = cov;

end