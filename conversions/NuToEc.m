function Ec = NuToEc(nu,e)
% Convert ue anomaly (and eccentricity) to Eccentric anomaly

Ec = acos((e+cos(nu))/(1+e*cos(nu)));
if nu > pi
    Ec = 2*pi-Ec;
end

end