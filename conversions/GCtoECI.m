function rECI = GCtoECI(GC,JD)
[Y,MO,D,HR,MI,S] = datevec(JD - 1721058.5);
sec = S + MI*60 + HR*60*60;
GMST = UTtoGMST(MO,D,Y,sec);
lat = GC(1);
lon = GC(2);
R = 6378.1366 + GC(3);
rECEF = [R*cosd(lat)*cosd(lon);
         R*cosd(lat)*sind(lon);
         R*sind(lat)];
rECI = rotECItoECEF(GMST)'*rECEF;
end