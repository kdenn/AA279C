function [oe_qns]=ConvRVtoQNS(r,v,mu_centerbody)
    %this function that calculates the inertial radial and velocity
    %position about a central body

    %{
    ---------------------------------------------------------------
    INPUT:
    cart [1x6] m m m m/s m/s m/s       inertial coordinates
                                        position and velocity

    mu_centerbody m^3/s^2             center body gravity parameter

    ---------------------------------------------------------------
    OUTPUT:
    oe_qns [1 x 6]      quasi-nonsingular orbital elements
    [a (m) u (rad) ex (-) ey(-) i (rad) OMEGA (rad)]

    ---------------------------------------------------------------
    NOTES:
        called within PropDerivative
    ---------------------------------------------------------------
    AUTHOR: Corinne Lippe, 2019
    ---------------------------------------------------------------
    %}

    h = cross(r,v);
    hnorm = norm(h);
    h_normvec = h./hnorm;

    n = cross([0; 0; 1],h);
    n_norm = n/norm(n);

    evec = (((norm(v))^2-mu_centerbody/norm(r)).*r-(dot(r,v)).*v)/mu_centerbody;
    e = norm(evec);

    zeta = (norm(v))^2/2-mu_centerbody/norm(r);
    a = -mu_centerbody/2/zeta;
    p = a*(1-e^2);

    cosi = h(3)/norm(h);
    i = acos(cosi);

    cosO = n(1)/norm(n);
    O = acos(cosO);

    if n(2) < 0
        O = 2*pi - O;
    end

    ex = dot(evec,n_norm);
    ey = cross(n_norm,evec);

    if dot(ey/norm(ey),h/norm(h)) > 0
        ey = norm(ey);
    else
        ey = -norm(ey);
    end
    u = atan2(r(3)/sin(i), r(1)*cos(O)+r(2)*sin(O));
    e = norm([ex ey]);
    %% if eccentric, u needs to be converted from w+v to w+M
    % if e > 0.1
    mean_motion = sqrt(mu_centerbody/(a*a*a));
    E = atan2(dot(r, v)/(a*a*mean_motion), (1-norm(r)/a));
    M = E - e*sin(E);
    nu = atan2(sqrt(1-e*e)*sin(E), cos(E)-e);
    u = u-nu+M;
    % end

    if u < 0
        u = u+2*pi;
    end
    oe_qns = [a, u, ex, ey, i, O];
end