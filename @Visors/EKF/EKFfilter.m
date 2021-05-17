function [mu, sig, At, Ct] = EKFfilter(f_fun,A_fun,g_fun,C_fun,Q,R,mut,sigt,yt,ut,dt)
% f: propagate x by dt
% A: df/dx
% g: measurement model: x -> y
% C: dg/dx

    % Predict
    mup = f_fun(mut,ut,dt);
    At = A_fun(mut,ut,dt);
    sigp = At * (sigt * At') + Q;
    
    % Update
    Ct = C_fun(mup,ut,dt);
    Kt = sigp * Ct' * inv((Ct * sigp * Ct') + R);
    mu = mup + (Kt * (yt-g_fun(mup,ut,dt)));
    sig = sigp - (Kt * Ct * sigp);
    
end