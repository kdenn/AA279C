function B = magnetic_field(k,r_ECI,JD)
% Wertz SADC Ap. H
    % k: highest order of n and m
    % r_ECI: ECI position of the spacecraft (m)
    % theta: coelevation (colatitude?) (rad)
    % phi: east longitude (rad)
    % JD: julian date (days)
    % B: 3x1 magnetic field in geocentric inertial components
    
    r = norm(r_ECI); % geocentric distance to the spacecraft
    a = s3_constants('R_EARTH_WGS84'); % equatorial radius of Earth
    GC = ECItoGC(r_ECI,JD);
    GD = GCtoGD(GC,1E-9,100);
    phi = deg2rad(GD(2));
    theta = deg2rad(90 - GD(1));
    
    P = gauss_funs(k,theta);
    dP = d_gauss_funs(P,k,theta);
    S = schmidt(k);
    [g,h] = igrf_coeffs();
    gs = S.*g;
    hs = S.*h;
    
    Br = 0;
    Bt = 0;
    Bp = 0;
    for n = 1:k
        ni = n+1; % adjust for 1-based indexing
        Cr = 0;
        Ct = 0;
        Cp = 0;
        for m = 0:n
            mi = m+1; % adjust for 1-based indexing
            Cr = Cr + (gs(ni,mi)*cos(m*phi) + hs(ni,mi)*sin(m*phi))*P(ni,mi);
            Ct = Ct + (gs(ni,mi)*cos(m*phi) + hs(ni,mi)*sin(m*phi))*dP(ni,mi);
            Cp = Cp + (-gs(ni,mi)*cos(m*phi) + hs(ni,mi)*sin(m*phi))*P(ni,mi);
        end
        Br = Br + (a/r)^(n+2)*(n+1)*Cr;
        Bt = Bt + (a/r)^(n+2)*Ct;
        Bp = Bp + (a/r)^(n+2)*Cp;
    end
    Bt = -Bt;
    Bp = -Bp/sin(theta);
    
    delta = pi/2 - theta;
    alpha_G = deg2rad(JD2GMST(JD)); % alpha_G is the right ascension of the Greenwich meridian (sidereal time at Greenwich)
    alpha = phi + alpha_G;
    
    Bx = (Br*cos(delta) + Bt*sin(delta))*cos(alpha) - Bp*sin(alpha);
    By = (Br*cos(delta) + Bt*sin(delta))*sin(alpha) + Bp*cos(alpha);
    Bz = Br*sin(delta) - Bt*cos(delta);
    
    B = [Bx;By;Bz]; 
    
end

function [g,h] = igrf_coeffs()
% return the IGRF coefficients from Wertz Table H-1
    % remember indexing starts at 0!
    % g(n,m+1) etc

    g = zeros(5);
    h = zeros(5);

    % n=1
    g(1+1,0+1) = -30186;
    g(1+1,1+1) = -2036;
    h(1+1,1+1) = 5735;

    % n=2
    g(2+1,0+1) = -1898;
    g(2+1,1+1) = 2997;
    h(2+1,1+1) = -2124;
    g(2+1,2+1) = 1551;
    h(2+1,2+1) = -37;
    
    % n=3
    g(3+1,0+1) = 1299;
    g(3+1,1+1) = -2144;
    h(3+1,1+1) = -361;
    g(3+1,2+1) = 1296;
    h(3+1,2+1) = 249;
    g(3+1,3+1) = 805;
    h(3+1,3+1) = -253;
    
    % n=4
    g(4+1,0+1) = 951;
    g(4+1,1+1) = 807;
    h(4+1,1+1) = 148;
    g(4+1,2+1) = 462;
    h(4+1,2+1) = -264;
    g(4+1,3+1) = -393;
    h(4+1,3+1) = 37;
    g(4+1,4+1) = 235;
    h(4+1,4+1) = -307;
    
    g = g.*1E-9;
    h = h.*1E-9;

end

function S = schmidt(k)
% equation H-7
    
    S = ones(k+1);
    for n = 1:k
        ni = n+1;
        for m = 1:n
            mi = m+1;
            if m == 0
                S(ni,1) = S(ni-1,1)*((2*n-1)/n);
            else
                S(ni,mi) = S(ni,mi-1)*sqrt((n-m+1)*((m==1)+1)/(n+m));
            end
        end
    end

end

function dP = d_gauss_funs(P,k,theta)
% equation H-10

    dP = zeros(k+1);
    for n = 1:k
        ni = n+1;
        for m = 0:n
            mi = m+1;
            if n == m
                dP(ni,mi) = sin(theta)*dP(ni-1,ni-1) + cos(theta)*P(ni-1,ni-1);
            elseif n == 1
                dP(ni,mi) = cos(theta)*dP(ni-1,mi) - sin(theta)*P(ni-1,mi);
            else
                dP(ni,mi) = cos(theta)*dP(ni-1,mi) - sin(theta)*P(ni-1,mi) - get_K(n,m)*dP(ni-2,mi);
            end
        end
    end

end

function P = gauss_funs(k,theta)
% equation H-8

    P = ones(k+1);
    for n = 1:k
        ni = n+1;
        for m = 0:n
            mi = m+1;
            if n == m
                P(ni,ni) = sin(theta) * P(ni-1,ni-1);
            elseif n == 1
                P(ni,mi) = cos(theta)*P(ni-1,mi);
            else
                P(ni,mi) = cos(theta)*P(ni-1,mi) - get_K(n,m)*P(ni-2,mi);
            end
        end
    end
    
end

function K = get_K(n,m)
% equation H-9

    if n == 1
        K = 0;
    else
        K = ((n-1)^2 - m^2)/((2*n-1)*(2*n-3));
    end
end