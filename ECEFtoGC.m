function GC = MCMFtoGC(r)
% Convery ECEF to Geocentric coordinates. 
% ECEF = XYZ
% Geocentric = spherical lat/long
lat = rad2deg(asin(r(3)/norm(r)));
lng = rad2deg(atan2(r(2),r(1)));
h = norm(r) - 6371; %km
GC = [lat,lng,h];
end