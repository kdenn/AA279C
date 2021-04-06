% Orbit Specifications for VISORS
%{
    Nominal orbit for mission design is 600 km, 98 deg. Sun Synchronous with 10 am LTAN
     - Altitude can range between 500 km and 650 km circular
     - 500 km minimum is to reduce atmospheric drag
     - Sun synchronous preferred to have a consistent beta angle
     - LTAN can vary between 1-4 and 8-11 AM/PM
%}

%% Earth
mu = s3_constants('GM_EARTH');

%% Orbit Elements
a = (650*1000) + s3_constants('R_EARTH'); % (m)
e = 0.001;
i = deg2rad(98);
Om = deg2rad(210);
w = deg2rad(45);
M_0 = deg2rad(45);
T = 2*pi/sqrt(mu/a^3);

oe = [a e i Om w M_0]';

%% Timing
Y_epoch = 2021;
M_epoch = 3;
D_epoch = 20.5;
JD_epoch = DateToJD(Y_epoch,M_epoch,D_epoch,0,0,0); % Noon on Spring Equinox
