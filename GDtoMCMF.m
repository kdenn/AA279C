function rMCMF = GDtoMCMF(AD,N) 
lat = AD(1); %deg (phi)
lng = AD(2); %deg (lambda)
h = AD(3); %km
Req = 3397; % equatorial radius [km]
Rpol = 3375; % polar radius [km]
eM = 1-(Rpol/Req)^2; % eccetricity of Mars ellipsoid
if nargin == 1
    N = Req/sqrt(1-eM^2*sind(lat)^2); % modified radius of curvature
end
rMCMF = [(N+h)*cosd(lat)*cosd(lng);
         (N+h)*cosd(lat)*sind(lng);
         (N*(1-eM)+h)*sind(lat)];
end