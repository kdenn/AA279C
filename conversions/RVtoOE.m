function [a,p,e,i,Om,w,nu,M,Ec,m,ang] = RVtoOE(rECI,vECI,mu)
% Following sec 2.4 of Fundamentals of Astrodynamics by Bate and Mueller.
% Valid only for Earth-orbiting objects
% Input must be in km and km/s respectively.

%% Vectors/Norms
r = norm(rECI);
v = norm(vECI);
hVec = cross(rECI,vECI);
h = norm(hVec);
K = [0 0 1];
nVec = cross(K,hVec);
n = norm(nVec);
eVec = (1/mu)*((norm(vECI)^2-mu/norm(rECI))*rECI-dot(rECI,vECI)*vECI);
e = norm(eVec);

%% Orbit Size
energy = 0.5*v^2 - mu/r;

if e ~=1
    a = -(mu/2)*((v^2)/2-mu/r)^(-1);
    p = a*(1-e^2);
else
    a = inf;
    p = h^2/mu;
end

%% Orbit Orientation
i = acos(hVec(3)/h);
if i > pi
    i = i - pi;
end
Om = acos(nVec(1)/n);
if nVec(2) < 0
    Om = 2*pi - Om;
end
w = real(acos(dot(nVec,eVec)/(n*e)));
if eVec(3) < 0
    w = 2*pi - w;
end
nu = real(acos(dot(eVec,rECI)/(e*r)));
if dot(rECI,vECI) < 0
    nu = 2*pi-nu;
end
Ec = real(acos((e+cos(nu))/(1+e*cos(nu))));
if nu > pi
    Ec = 2*pi-Ec;
end
M = real(Ec-e*sin(Ec));

%% Special Cases
if i < 1E-7 && e > 1E-4
    % Elliptical  equatorial
    % Provide the longitude of periapsis (PI = Om + w)
    ang = real(acos(eVec(1)/e));
    if eVec(2) < 0
        ang = 2*pi-ang;
    end
elseif i > 1E-7 && e < 1E-4
    % Circular  inclined
    % Provide the argument of latitude (u = w + nu)
    ang = real(acos(dot(nVec,rECI)/(n*r)));
    if rECI(3) < 0
        ang = 2*pi - ang;
    end
    nu = ang;
    Ec = acos((e+cos(nu))/(1+e*cos(nu)));
    if nu > pi
        Ec = 2*pi-Ec;
    end
    M = Ec-e*sin(Ec);
    w = ang;
elseif i < 1E-7 && e < 1E-4
    % Circular Equatorial
    % Provide the true longitude (l = Om + w + anom)
    ang = acos(rECI(1)/r);
    if rECI(2) < 0
        ang = 2*pi - ang;
    end
    nu = ang;
    Ec = acos((e+cos(nu))/(1+e*cos(nu)));
    if nu > pi
        Ec = 2*pi-Ec;
    end
    M = Ec-e*sin(Ec);
    w = ang;
else
    ang = NaN;
end

m = sqrt(mu/(a^3));

end