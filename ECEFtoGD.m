function [GD,N] = ECEFtoGD(r,tol,imax)
% Convert ECEF to Geodetic coordinates. 
% ECEF = XYZ
% Geocentric = ellipsoid lat/longlat
lat = rad2deg(asin(r(3)/norm(r)));
lng = rad2deg(atan2(r(2),r(1)));
rXY = sqrt(r(1)^2+r(2)^2);
eE = 0.0818;
N = 3397/sqrt(1-eE^2*sind(lat)^2);
err = 100;
iter = 0;
while err > tol && iter < imax
    lat_p = atand((r(3)+N*eE^2*sind(lat)^2)/rXY);
    err = abs(lat_p-lat);
    lat = lat_p;
    N = 3397/sqrt(1-eE^2*sind(lat)^2);
    iter = iter+1;
end
h = rXY/cosd(lat)-N;
GD = [lat,lng,h];
end
