function [a, e, i, Om , w, anom , ang ] = ECI2OE (r_ijk , v_ijk , mu)
% ECI2OE Converts r and v in the ECI frame to orbital elements
%
% Inputs :
% r_ijk - 3x1 vector of radius in ECI frame [km]
% v_ijk - 3x1 vector of velocity in ECI frame [km]
% mu - gravitational parameter of central body [km ^3/s ^2]
%
% Outputs :
% a - semi - major axis of orbit [km]
% e - eccentricity of orbit
% i - inclination of orbit [ deg ]
% Om - right ascension of the ascending node [deg]
% w - argument of periapsis [ deg ]
% v - true anomaly [ deg ]
% ang - extra placeholder angle for special cases [deg]
%
% This function is able to handle any orbit type , including
%equatorial and
% circular orbits . The output ang will output the following
%values for each
% orbit type
% 1) Elliptical equatorial
% Longitude of periapsis , Pi = Om + w
% 2) Circular inclined
% Argument of latitude , u = w + anom
% 3) Circular equatorial
% True latitude , lambda = Om + w + anom
% Otherwise , ang will be undefined ( NaN )
r = norm ( r_ijk );
v = norm ( v_ijk );
% Create all necessary vectors
hVec = cross (r_ijk , v_ijk );
h = norm ( hVec );
nVec = cross ([0 , 0, 1], hVec );
n = norm ( nVec );
eVec = (1/ mu) *(( v^2 - mu/r)* r_ijk - dot (r_ijk , v_ijk )* v_ijk );
e = norm ( eVec );
% Compute the size of the orbit
mechEnergy = 0.5* v^2 - mu/r;
if e ~= 1
a = -mu /(2* mechEnergy );
p = a*(1 - e ^2) ;
else
a = inf ;
p = h^2/ mu;
end
% Compute the orientation of the orbit
i = acosd ( hVec (3) /h);
Om = acosd ( nVec (1)/n);
w = acosd ( dot (nVec , eVec )/(n*e));
anom = acosd ( dot (eVec , r_ijk )/(e*r));
% Place angles in the correct domains
if nVec (2) < 0
Om = 360 - Om;
end
if eVec (3) < 0
w = 360 - w;
end
if dot (r_ijk , v_ijk ) < 0
anom = 360 - anom ;
end
% Account for any special cases
if i == 0 && e ~= 0 % Elliptical equatorial
% Provide the longitude of periapsis (PI = Om + w)
ang = acosd ( eVec (1) /e);
if eVec (2) < 0
    ang = 360 - ang;
end
elseif i ~= 0 && e == 0 % Circular inclined
% Provide the argument of latitude (u = w + anom )
ang = acosd ( dot (nVec , r_ijk )/(n*r));
if r_ijk (3) < 0
ang = 360 - ang;
end
elseif i == 0 && e == 0 % Circular equatorial
% Provide the true latitude ( lambda = Om + w + anom )
ang = acosd ( r_ijk (1) /r);
if r_ijk (2) < 0
ang = 360 - ang;
end
else
% Default output for ang
ang = NaN ;
end
end