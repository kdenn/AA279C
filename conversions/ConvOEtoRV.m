function [r, v] = ConvOEtoRV(oe,mu,JD_epoch,JD)
    % Propagate from singular OEs to cartesian to particular JD
    %{
    ---------------------------------------------------------------
    INPUT:
        oe:         6x1 double, orbital elements in inertial space
                        [a e i Om w M_0]'
                        (m - rad rad rad rad)
        mu:         double, gravitational param of CB
        JD_epoch:   double, initial Julian date of OE's (optional)
        JD:         double, Julian date (days) (optional)
    ---------------------------------------------------------------
    OUTPUT:
        r:          3x1 double, position in the frame of OE's (m)
        v:          3x1 double, velocity in the frame of OE's (m/s)
    ---------------------------------------------------------------
    AUTHOR: Kaitlin Dennison, 2020
    ---------------------------------------------------------------
    %}

    %% Initialize
    if nargin < 3
        t = 0;
    else
        t = (JD - JD_epoch)*86400;
    end
    a = oe(1);
    e = oe(2);
    i = oe(3);
    Om = oe(4);
    w = oe(5);
    n = sqrt(mu/(a^3));
    M = mod(oe(6) + n*t, 2*pi);

    %% Get Ec
    er = 100;
    Eco = M;
    iter = 0;
    while er >= 1E-8 && iter <= 100
        del = -(Eco-e*sin(Eco)-M)/(1-e*cos(Eco));
        Ec = Eco + del;
        er = abs(Ec-Eco);
        Eco = Ec;
        iter = iter + 1;
    end

    %% Get PQW
    a = (mu/n^2)^(1/3);
    rPQW = [a*(cos(Ec)-e),a*sqrt(1-e^2)*sin(Ec),0]';
    vPQW = (a*n)/(1-e*cos(Ec)).*[-sin(Ec),sqrt(1-e^2)*cos(Ec),0]';

    %% Get Rotation
    RzOm = [cos(-Om),sin(-Om),0;...
            -sin(-Om),cos(-Om),0;...
            0,0,1];
    Rxi = [1,0,0;...
           0,cos(-i),sin(-i);...
           0,-sin(-i),cos(-i)];
    Rzw = [cos(-w),sin(-w),0;...
            -sin(-w),cos(-w),0;...
            0,0,1];
    R_PQW2wld = RzOm*Rxi*Rzw;

    %% Final ECI values
    r = R_PQW2wld*rPQW;
    v = R_PQW2wld*vPQW;

end