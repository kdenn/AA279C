% s3_constants.m
% [S3_CONSTANTS] = s3_constants()
% -------------------------------------------------------------------------
% USAGE: s3_constants
%        s3_constants(varname)
% -------------------------------------------------------------------------
% DESCRIPTION: Loads fundamental constants into the calling workspace or
%              returns and individual variable when requested by name
% -------------------------------------------------------------------------
% INPUTS: varname - (string) variable name, see below for full list.
% -------------------------------------------------------------------------
% OUTPUTS: S3_CONSTANTS - (struct) of s3 constants is returned when no
%                         input argument is provided.
% -------------------------------------------------------------------------
% VARIABLE NAME LIST: 
% OMEGA_EARTH
% R_EATH
% GM_EARTH
% J2_EARTH
% 
% -------------------------------------------------------------------------
% AUTHOR: Duncan Eddy, Connor Beierle
%         2015-08-08  DOE: initial release
%         2017-02-24  CRB: changed the output to be a structure containting
%                          constants when no input argument is provided
% -------------------------------------------------------------------------
% COPYRIGHT:
% Built using S3: SLAB Satellite Software, (c) 2016
% https://slab.stanford.edu/s3/
% -------------------------------------------------------------------------


function var = s3_constants(varname)

%% Fundamental Parameters
C_LIGHT = 299792458.0;
AU = 149597870700;                                                         % AU conversion [m/AU]
G = 6.67408*10^-11;                                                        % Gravitational Constant [m^3/(kg s^2)]

%% Planetary Body Constants
OMEGA_EARTH = 7.292115146706979e-5;                                        % Taken from Vallado 4th Ed page 222
R_EARTH = 0.6378136300e7;                                                  % GGM05s value [m]
GM_EARTH = 0.3986004415e15;                                                % GGM05s value [m^3/s^2]
J2_EARTH = 0.0010826358191967;                                             % GGM05s value [m^3/s^2]
J3_EARTH = -2.53244425358897e-6;                                           % GGM05s value

% WGS84 Definitions
R_EARTH_WGS84 = 6378137.0;                                                 % Equitorial raidus (semi-major axis) [m]
e_EARTH_WGS84 = 8.1819190842622e-2;                                        % First Eccentricity [-]
f_EARTH_WGS84 = 1/298.257223563;                                           % Flattening
OMEGA_EARTH_WGS84 = 7292115.0e-11;                                         % Angular Velocity of Earth [rad/s]
GM_EARTH_WGS84 = 3986004.418e8;                                            % Earth's Gravitational constant [m^3/s^2]
J2_EARTH_WGS84 = 1.082629821313e-03;                                       % Earth Oblateness Form Factor

% Gravitational constants
GM_SUN = 132712440041.939400*1e9;
GM_MOON = 4902.800066*1e9;
GM_MERCURY = 22031.780000*1e9;
GM_VENUS = 324858.592000*1e9;
GM_MARS = 42828.37521*1e9;
GM_JUPITER = 126712764.8*1e9;
GM_SATURN = 37940585.2*1e9;
GM_URANUS = 5794548.6*1e9;
GM_NEPTUNE = 6836527.100580*1e9;
GM_PLUTO = 977.000000*1e9;

%% GNSS Constants
% Used s3_getGNSSwavelength or s3_getGNSSfrequency

% Frequencies
fL1  = 1575.42e6; % [1/s] Hz. L1 var. IS-GPS-200H, p. 14
fL2  = 1227.60e6; % [1/s] Hz. L2 var. IS-GPS-200H, p. 14
fL5  = 1176.46e6; % [1/s] Hz. L5 var. IS-GPS-200H, p. 50

fE1  = 1575.420e6; % [1/2] Hz. E1 var. Galileo-OS-SIS-ICS-1.2, p. 3
fE5  = 1191.795e6; % [1/2] Hz. E1 var. Galileo-OS-SIS-ICS-1.2, p. 3
fE5a = 1176.450e6; % [1/2] Hz. E1 var. Galileo-OS-SIS-ICS-1.2, p. 3
fE5b = 1207.140e6; % [1/2] Hz. E1 var. Galileo-OS-SIS-ICS-1.2, p. 3
fE6  = 1278.750e6; % [1/2] Hz. E1 var. Galileo-OS-SIS-ICS-1.2, p. 3

fB1  = 1561.098e6; % [1/s] Hz. B1 var. BDS-SIS-ICD-2.0, p. 4
fB2  = 1207.140e6; % [1/s] Hz. B2 var. BDS-SIS-ICD-2.0, p. 4
fB3  = 1268.520e6; % [1/s] Hz. B3 var. Navipedia.net Jul 20, 2016

% Wavelengths
lamL1 = C_LIGHT/fL1;
lamL2 = C_LIGHT/fL2;
lamL5 = C_LIGHT/fL5;

lamE1  = C_LIGHT/fE1;
lamE5  = C_LIGHT/fE5;
lamE5a = C_LIGHT/fE5a;
lamE5b = C_LIGHT/fE5b;
lamE6  = C_LIGHT/fE6;

lamB1 = C_LIGHT/fB1;
lamB2 = C_LIGHT/fB2;
lamB3 = C_LIGHT/fB3;

%% Parse Input

if ~exist('varname','var') % No frequency specified. Assign all to caller
    
    S3_CONSTANTS.GM.SUN            = GM_SUN;
    S3_CONSTANTS.GM.MOON           = GM_MOON;
    S3_CONSTANTS.GM.MERCURY        = GM_MERCURY;
    S3_CONSTANTS.GM.VENUS          = GM_VENUS;
    S3_CONSTANTS.GM.EARTH          = GM_EARTH;
    S3_CONSTANTS.GM.EARTH_WGS84    = GM_EARTH_WGS84;
    S3_CONSTANTS.GM.MARS           = GM_MARS;
    S3_CONSTANTS.GM.JUPITER        = GM_JUPITER;
    S3_CONSTANTS.GM.SATURN         = GM_SATURN;
    S3_CONSTANTS.GM.URANUS         = GM_URANUS;
    S3_CONSTANTS.GM.NEPTUNE        = GM_NEPTUNE;
    S3_CONSTANTS.GM.PLUTO          = GM_PLUTO;
    S3_CONSTANTS.C_LIGHT           = C_LIGHT;
    S3_CONSTANTS.G                 = G; % added by KD
    S3_CONSTANTS.AU                = AU; % added by KD
    S3_CONSTANTS.R_EARTH           = R_EARTH;
    S3_CONSTANTS.R_EARTH_WGS84     = R_EARTH_WGS84;
    S3_CONSTANTS.J2_EARTH          = J2_EARTH;
    S3_CONSTANTS.J2_EARTH_WGS84    = J2_EARTH_WGS84;
    S3_CONSTANTS.J3_EARTH          = J3_EARTH;
    S3_CONSTANTS.OMEGA_EARTH       = OMEGA_EARTH;
    S3_CONSTANTS.OMEGA_EARTH_WGS84 = OMEGA_EARTH_WGS84;
    S3_CONSTANTS.e_EARTH_WGS84     = e_EARTH_WGS84;
    S3_CONSTANTS.f_EARTH_WGS84     = f_EARTH_WGS84;
    
    assignin('caller', 'S3_CONSTANTS', S3_CONSTANTS);
    
elseif strcmp(varname,'C_LIGHT') || strcmp(varname,'C')
    var = C_LIGHT;
elseif strcmp(varname,'G')
    var = G;
elseif strcmp(varname,'AU')
    var = AU;
elseif strcmp(varname,'OMEGA_EARTH')
    var = OMEGA_EARTH;
elseif strcmp(varname,'R_EARTH')
    var = R_EARTH;
elseif strcmp(varname,'GM_EARTH')
    var = GM_EARTH;
elseif strcmp(varname,'J2_EARTH')
    var = J2_EARTH;
elseif strcmp(varname,'J3_EARTH')
    var = J3_EARTH;
elseif strcmp(varname,'R_EARTH_WGS84')
    var = R_EARTH_WGS84;
elseif strcmp(varname,'e_EARTH_WGS84')
    var = e_EARTH_WGS84;
elseif strcmp(varname,'f_EARTH_WGS84')
    var = f_EARTH_WGS84;
elseif strcmp(varname,'OMEGA_EARTH_WGS84')
    var = OMEGA_EARTH_WGS84;
elseif strcmp(varname,'GM_EARTH_WGS84')
    var = GM_EARTH_WGS84;
elseif strcmp(varname,'J2_EARTH_WGS84')
    var = J2_EARTH_WGS84;
elseif strcmp(varname,'GM_SUN')
    var = GM_SUN;
elseif strcmp(varname,'GM_MOON')
    var = GM_MOON;
elseif strcmp(varname,'GM_MERCURY')
    var = GM_MERCURY;
elseif strcmp(varname,'GM_VENUS')
    var = GM_VENUS;
elseif strcmp(varname,'GM_MARS')
    var = GM_MARS;
elseif strcmp(varname,'GM_JUPITER')
    var = GM_JUPITER;
elseif strcmp(varname,'GM_SATURN')
    var = GM_SATURN;
elseif strcmp(varname,'GM_URANUS')
    var = GM_URANUS;
elseif strcmp(varname,'GM_NEPTUNE')
    var = GM_NEPTUNE;
elseif strcmp(varname,'GM_PLUTO')
    var = GM_PLUTO;
elseif strcmp(varname,'fL1') || strcmp(varname,'L1')
    var = fL1;
elseif strcmp(varname,'fL2') || strcmp(varname,'L2')
    var = fL2;
elseif strcmp(varname,'fL5') || strcmp(varname,'L5')
    var = fL5;
elseif strcmp(varname,'fE1') || strcmp(varname,'E1')
    var = fE1;
elseif strcmp(varname,'fE5') || strcmp(varname,'E5')
    var = fE5;
elseif strcmp(varname,'fE5a') || strcmp(varname,'E5a')
    var = fE5a;
elseif strcmp(varname,'fE5b') || strcmp(varname,'E5b')
    var = fE5b;
elseif strcmp(varname,'fE6') || strcmp(varname,'E6')
    var = fE6;
elseif strcmp(varname,'fB1') || strcmp(varname,'B1')
    var = fB1;
elseif strcmp(varname,'fB2') || strcmp(varname,'B2')
    var = fB2;
elseif strcmp(varname,'fB3') || strcmp(varname,'B3')
    var = fB3;
end 

end