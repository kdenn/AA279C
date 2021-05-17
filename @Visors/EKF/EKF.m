function [mu_arr, cov_arr] = EKF(Y_arr, u_arr, iEKF)
% Purpose: Run an EKF or iEKF

% Call file to get sim constants
get_constants;

% Time histories of KF parameters
mu_arr = zeros(dim_s, N);
cov_arr = zeros(dim_s, dim_s, N);

% Noise covariances
Q = 0.100*dt*eye(dim_s);      % process noise covariance
R = 0.100*eye(dim_m);         % measurement noise covariance

% Initial belief
mu_0 = [0; 0; 0];
cov_0 = diag([0.01 0.01 0.01]);

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
    A = get_A(mu, u, dt);
    mu_bar = f(mu, u, dt);
    cov_bar = A * cov * A' + Q;
    
    % Update Step for iEKF
    if iEKF == 1
        mu = mu_bar; 
        for j = 1:10
            C = get_C(mu, u, dt);
            K = cov_bar * C' * inv(C * cov_bar * C' + R);
            mu = mu_bar + K*(Y_arr(:,i+1)-g(mu, u, dt)) + K*C*(mu-mu_bar);
        end
        cov = cov_bar - K * C * cov_bar;
        
    % Update step for standard EKF
    else
        C = get_C(mu_bar, u, dt);
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