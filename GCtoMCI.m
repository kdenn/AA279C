function rMCI = GCtoMCI(GC,t)
w_mar = 2*pi/(24.6229*60*60); 

lat = GC(1); %deg (phi)
lng = GC(2); %deg (lambda)
h = GC(3); %km
rm = 3.395428e+03; % km

rMCMF = [(rm+h)*cosd(lat)*cosd(lng);
         (rm+h)*cosd(lat)*sind(lng);
         (rm+h)*sind(lat)];
     
GMST = w_mar.*t;

R = [cos(GMST) sin(GMST) 0;
     -sin(GMST) cos(GMST), 0;
     0 0 1]';

rMCI = R*rMCMF;
end