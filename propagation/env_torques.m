function M = env_torques(r_ECI,JD)


end

function M_m = mag_torque(r_ECI,JD)

V = scalar_potential(r_ECI,JD);




end

function V = scalar_potential(r_ECI,JD)
[GC,r_ECEF] = ECItoGC(r_ECI,JD);
R = norm(r_ECEF);
lambda = GC(2);
theta = 90 - GC(1); % https://en.wikipedia.org/wiki/Colatitude

[g,h,gd,hd] = igrf_coeffs(); 
Re = s3_constants('R_EARTH');

k = 2;

V = 1;
for n = 1:k
    B = (Re/R)^(n+1);
    C = 0;
    P = legendre(n,theta);
    for m = 1:(n+1)
        C = C + (g(n,m)*cosd(m*lambda) + h(n,m)*sind(m*lambda))*P(m);
    end
    V = V + B*C;
end
V = Re*V;

end

function [g,h,gd,hd] = igrf_coeffs()
% remember m starts at 0!
% g(n,m+1) etc

g = zeros(2,3);
h = zeros(2,3);
gd = zeros(2,3);
hd = zeros(2,3);

g(1,0+1) = -30186;
gd(1,0+1) = 25.6;

g(1,1+1) = -2036;
h(1,1+1) = 5735;
gd(1,1+1) = 10.0;
hd(1,1+1) = -10.2;

g(2,0+1) = -1898;
gd(2,0+1) = -24.9;

g(2,1+1) = 2997;
h(2,1+1) = -2124;
gd(2,1+1) = 0.7;
hd(2,1+1) = -3;

g(2,2+1) = 1551;
h(2,2+1) = -37;
gd(2,2+1) = 4.3;
hd(2,2+1) = -18.9;

end