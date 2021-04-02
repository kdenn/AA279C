function earth = getEarthConst()

% From hpiers.ebsom/fr/eop-pc/models/constants_mobile.html
earth.m = 5.972E24;             % mass, kg
earth.mu = 398600.442;          % grav param, km^3/s^2
earth.R = 6378.1366;            % equatorial R, km
earth.Req = 6378.1366;          % equatorial R, km
earth.Rpo = 6356;               % polar R, km
earth.J2 = 1.0826E-3;           % oblateness, non-dim
earth.w = 7.292115E-5;          % angular velocity, rad/s
earth.D = 86400;                % solar day duration, s
earth.Ds = 86164.091;           % siderial day duration, s
earth.Y = 365.256363004;        % siderial year, Ds
earth.tilt = 23.4119;           % axial tilt from ecliptic, deg

% Earth TLE or OEs here...