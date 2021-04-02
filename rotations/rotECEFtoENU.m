function R = rotECEFtoENU(GD_ground)
lat = GD_ground(1); %deg (phi)
lng = GD_ground(2); %deg (lambda)
E = [-sind(lng);cosd(lng);0];
N = [-sind(lat)*cosd(lng);
     -sind(lat)*sind(lng);
     cosd(lat)];
U = [cosd(lat)*cosd(lng);
     cosd(lat)*sind(lng);
     sind(lat)];
R = [E,N,U]';
end