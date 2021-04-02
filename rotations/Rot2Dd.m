function R = Rot2Dd(theta)
R = [cosd(theta) -sind(theta);
     sind(theta) cosd(theta)];
end