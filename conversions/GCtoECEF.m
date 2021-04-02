function rECEF = GCtoECEF(GC) 
lat = GC(1); %deg (phi)
lng = GC(2); %deg (lambda)
h = GC(3); %km
rE = 6371; % km
rECEF = [(rE+h)*cosd(lat)*cosd(lng);
         (rE+h)*cosd(lat)*sind(lng);
         (rE+h)*sind(lat)];
end