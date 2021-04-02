function rECI = GDtoECI(GD,JD,N) 
lat = GD(1); %deg (phi)
lng = GD(2); %deg (lambda)
h = GD(3); %km
Req = 6378.1366; % equatorial radius [km]
Rpol = 6356; % polar radius [km]
eM = 1-(Rpol/Req)^2; % eccetricity of Mars ellipsoid
if nargin == 2
    N = Req/sqrt(1-eM^2*sind(lat)^2); % modified radius of curvature
end

[Y,M,D,H,MN,S] = datevec(JD - 1721058.5);
D = D + (H*60*60 + MN*60 + S)/86400;
GMST = UTtoGMST(M,D,Y,0);

rECEF = [(N+h)*cosd(lat)*cosd(lng);
         (N+h)*cosd(lat)*sind(lng);
         (N*(1-eM)+h)*sind(lat)];
rECI = rotECItoECEF(GMST)'*rECEF;
end