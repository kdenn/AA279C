function mars = getMarsConst()
G = 6.67259*10^-20;                 % gravitational constant
mars.m = 6.4185*10^23;              % mass, [kg]
mars.mu = G*mars.m;                 % gravitational parameter, [km^3/s^2]
mars.Req = 3397;                    % equatorial radius, [km]
mars.Rpol = 3375;                   % polar radius, [km]
mars.w = 2*pi/(24.6229*60*60);      % angular velocity, [rad/s]
end