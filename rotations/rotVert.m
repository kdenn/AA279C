function v_rot = rotVert(v,alp,del,W_0,W_d,d)
% Rotate the vertices from ACAF -> ACI frame
% INPUT:
%   v: matrix of vertices [nx3] [x y z]
%   alp: alpha, right ascension [deg]
%   del: delta, declination [deg]
%   W_0: prime meridian [deg]
%   W_d: rotation rate [deg/day]
%   d: days since standard epoch
% OUTPUT:
%   v_rot: matrix of vertices [nx3] [x y z]
% NOTES:
%   Standard epoch: J2000.0 = JD 2451545.0 (2000 Jan 1 12:00)
% -------------------------------------------------------------------------
% Kaitlin Dennison [5/1/2018]
% Stanford University - Space Rendezvous Laboratory
% -------------------------------------------------------------------------

n = size(v,1);
v_rot = zeros(n,3);
R = R3(deg2rad(-alp-90))*R1(deg2rad(del-90))*R3(deg2rad(-W_0-W_d*d));

for i = 1:n
    v_rot(i,:) = (R*v(i,:)')';
end

end