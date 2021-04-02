function [AD,N] = ACtoAD(AC,tol,imax)
r = GCtoMCMF(AC);
lat = AC(1);
lng = AC(2);
rXY = sqrt(r(1)^2+r(2)^2);
% Springer: A new reference equipotential surface, and reference ellipsoid
% for the planet Mars.

Req = 3397; % equatorial radius [km]
Rpol = 3375; % polar radius [km]
eM = 1-(Rpol/Req)^2;
N = Req/sqrt(1-eM^2*sind(lat)^2);
err = 100;
iter = 0;
while err > tol && iter < imax
    lat_p = atand((r(3)+N*eM^2*sind(lat)^2)/rXY);
    err = abs(lat_p-lat);
    lat = lat_p;
    N = Req/sqrt(1-eM^2*sind(lat)^2);
    iter = iter+1;
end
h = rXY/cosd(lat)-N;
AD = [lat;lng;h];
end