function sd = stateDerivJ2(t,s)
% State derivative 

%% Problem constants
mu = 398600.4418; % km^3/s^2 (Earth)
J2 = 1.0826267E-3; % (Earth oblateness)
R = 6378.137; % km (Earth equatorial radius)

%% Calculations
sd = zeros(6,1);
r = s(1:3);
v = s(4:6);

f_CB = -mu.*r./(norm(r,2)^3); % FODE

% Acceleration due to oblateness
normr = norm(r);
rhat = r/normr;
khat = [0 0 1]';
z = r(3);
accJ2 = - (mu*J2*R*R/2)*((6*z/(normr^5))*khat+((3/(normr^4))-...
    (15*z*z/(normr^6)))*rhat);

sd(1:3) = v; % rdot = vel
sd(4:6) = f_CB+accJ2;

end