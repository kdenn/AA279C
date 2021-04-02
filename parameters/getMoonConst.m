function moon = getMoonConst()

AU = 149597870.700; % km/AU

% From "Lunar Constants and Models Document" - NASA
moon.mu = 4902.801076; % grav param, km^3/s^2
moon.R = 1738.0; % mean radius, km
moon.J2 = 2.033542482111609E-4; % oblateness, non-dim

% From JPL HORIZONS
moon.e = 6.476694137484437E-02;
moon.Om = deg2rad(1.239837037681769E+02); % rad
moon.n = deg2rad(1.335975862260855E+01)/86400; %rad/s
moon.a = 2.548289534512777E-03*AU; % km
moon.w = deg2rad(3.081359025079810E+02); % rad
moon.M0 = deg2rad(1.407402568949268E+02); % rad
moon.JD0 = 2451544.5; 
moon.i = deg2rad(5.240010960708354); % rad


