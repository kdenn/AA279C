function nu = iterMtoNu(M_0,n,d,d_epoch,tol)
% Get nu from M and d
% INPUT:
%   M_0: mean motion at epoch, deg
%   n: mean anomaly, deg/day
%   d: MJD days since J2000
%   d_epocj: MJD dayssince J2000 at epoch
%   tol: tolerance for newton's method
% OUTPUT:
%   nu: true anomaly, deg
%--------------------------------------------------------------------------
% Kaitlin Dennison [5/1/2018]
% Stanford University - Space Rendezvous Laboratory
% -------------------------------------------------------------------------
M = deg2rad(M_0+n*(d-d_epoch));
er = 100;
Eco = M;
iter = 0;
while er >= tol && iter <= 100
    % Newton's method
    del = -(Eco-e*sin(Eco)-M)/(1-e*cos(Eco));
    Ec = Eco + del;
    er = abs(Ec-Eco);
    Eco = Ec;
    iter = iter + 1;
end
Ec = iterMtoEc(M_0+n*(d-d_epoch),e,1E-8);
nu = acos((cos(Ec)-e)/(1-e*cos(Ec)));
if Ec > pi
    nu = 2*pi-nu;
end
nu = rad2deg(nu);
end