function sd = stateDeriv(t,s)
% State derivative 

%% Problem constants
mu = 398600.4418; % [km^3/s^2] (Earth)
% mu = 1.3271244004193938E11; % [km^3/s^2] (Sun)

%% Calculations
sd = zeros(6,1);
r = s(1:3);
v = s(4:6);

f_CB = -mu.*r./(norm(r,2)^3); % FODE

sd(1:3) = v; % rdot = vel
sd(4:6) = f_CB;

end