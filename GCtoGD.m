function [GD,N] = GCtoGD(GC,tol,imax)
r = GCtoECEF(GC);
lat = GC(1);
lng = GC(2);
rXY = sqrt(r(1)^2+r(2)^2);
eE = 0.0818;
N = 6371/sqrt(1-eE^2*sind(lat)^2);
err = 100;
iter = 0;
while err > tol && iter < imax
    lat_p = atand((r(3)+N*eE^2*sind(lat)^2)/rXY);
    err = abs(lat_p-lat);
    lat = lat_p;
    N = 6371/sqrt(1-eE^2*sind(lat)^2);
    iter = iter+1;
end
h = rXY/cosd(lat)-N;
GD = [lat,lng,h];
end