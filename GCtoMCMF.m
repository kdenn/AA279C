function rMCMF = GCtoMCMF(GC) 
lat = GC(1); %deg (phi)
lng = GC(2); %deg (lambda)
h = GC(3); %km
rm = 3.395428e+03; % km
rMCMF = [(rm+h)*cosd(lat)*cosd(lng);
         (rm+h)*cosd(lat)*sind(lng);
         (rm+h)*sind(lat)];
end