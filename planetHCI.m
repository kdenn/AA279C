function [rHCI, vHCI] = planetHCI(p, JD)
% Returns Heliocentric position and velocity of the given planet
% INPUT:
%   p: planet number, 1-8 (1 being Mercury, 8 being Neptune)
%   JD: Julian date
% Output:
%   r: HCI position [x y z] [km]
%   v: HCI velocity [x' y' z'] [km/s]
% -------------------------------------------------------------------------
% Kaitlin Dennison
% AA279B Spring 2018
% -------------------------------------------------------------------------

%% Constants and time
mu_sun = 1.3271244004193938E11; % [km^3/s^2]
AU = 149597870.7; % AU conversion [km/AU]
cty = 1 / 36525; % Julian Century conversion [cty/days]
JD_epoch = 2451545.0; % J2000 [days]
t = (JD - JD_epoch) * cty; % given time in cty since J2000

%% Get planet data
[elements, rates] = AA279j2000_planetary_elements(p);

% J2000 rates
a_dot = rates(1)*AU; % [km/cty]
e_dot = rates(2); % [1/cty]
i_dot = rates(3); % [deg/cty]
Om_dot = rates(4); % [deg/cty]
w_bar_dot = rates(5); % [deg/cty]
L_dot = rates(6); % [deg/cty]

% elements
a = elements(1) * AU + a_dot * t; % semimajor axis [km]
e = elements(2) + e_dot * t; % eccentricity
i = deg2rad(elements(3) + i_dot * t); % inclination [rad]
Om = deg2rad(elements(4) + Om_dot * t); % longitude of the ascending node [rad]
w_bar = deg2rad(elements(5) + w_bar_dot * t); % longitude of perihelion [rad]
L = deg2rad(elements(6) + L_dot * t); % mean longitude [rad] 
w = w_bar - Om;  % longitude of periapsis [rad]
M_0 = L - w_bar; % mean anomaly [rad]

%% Get r and v
n = sqrt(mu_sun/a^3);
M = mod(M_0,2*pi); % mod(M_0+n*(t-t_0),2*pi);
[rHCI,vHCI] = OEtoRVv2(e,i,Om,w,M,n,mu_sun);
