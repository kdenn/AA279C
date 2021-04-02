function [r_ECI,v_ECI] = OE2MCI(a,e,i,Om,w,nu,mu)
    % find r in pqw
    p = a*(1-e^2);
    rmag = p/(1+e*cos(nu));
    rp = rmag*cos(nu);
    rq = rmag*sin(nu);
    r_PQW = [rp;rq;0];
    v_PQW = sqrt(mu/p)*[-sin(nu);e+cos(nu);0];
    % Rotation matrix PQW to ECI
    RzO = [cos(-Om) sin(-Om) 0; -sin(-Om) cos(-Om) 0; 0 0 1];
    Rxi = [1 0 0; 0 cos(-i) sin(-i); 0 -sin(-i) cos(-i)];
    Rzw = [cos(-w) sin(-w) 0; -sin(-w) cos(-w) 0; 0 0 1];
    R_PQW_ECI = RzO*Rxi*Rzw;
    r_ECI = R_PQW_ECI*r_PQW;
    v_ECI = R_PQW_ECI*v_PQW;
end