function [GC,rECEF] = ECItoGC(rECI,JD)
% [Y,MO,D,HR,MI,S] = datevec(JD - 1721058.5);
% sec = S + MI*60 + HR*60*60;
% GMST = UTtoGMST(MO,D,Y,sec);
GMST = deg2rad(JD2GMST(JD));
rECEF = rotECItoECEF(GMST)*rECI;
lat = rad2deg(asin(rECEF(3)/norm(rECEF)));
lng = rad2deg(atan2(rECEF(2),rECEF(1)));
h = norm(rECEF) - 6371; %km
GC = [lat,lng,h];
rECEF = rECEF';
end