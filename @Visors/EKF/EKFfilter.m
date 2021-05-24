function [x, P, At, Ht, z_pre, z_post] = EKFfilter(f_fun,A_fun,g_fun,H_fun,Q,R,xt,Pt,yt,ut,dt)
% f: propagate x by dt
% A: df/dx
% g: measurement model: x -> y
% H: dg/dx
    N = numel(xt);

    % Predict
    At = A_fun(xt,ut,dt);
    xp = At*xt;
    Pp = At * (Pt * At') + Q;

    z_pre = g_fun(xp) - yt;

    % Update
    Ht = H_fun(xp,ut,dt);
    Kt = Pp * Ht' * inv((Ht * Pp * Ht') + R);
    x = xp + (Kt * (yt-g_fun(xp)));
    P = (eye(N) - Kt*Ht)*Pp*(eye(N) - Kt*Ht)' + Kt*R*Kt';

    z_post = g_fun(x) - yt;

end