function sd = stateDerivPert(t,s)
% s = [x y z dx dy dz]' in ECI;

%% Problem constants
earth = getEarthConst();
moon = getMoonConst();
sat = getSatConst();
mu_sun = 1.3271244004193938E11;     % Grav Param of Sun, km^3/s^2
pSRP = 4.57E-6;                     % Solar radiation pressure, N/m^2
h0 = 0;
rho0 = 1.225*(1000^3);              % atm density, kg/km^3
H = 10;                             % characteristic height for atm, km

%% Calculations
JD = t/86400 + 2451544.5;
sd = zeros(6,1);
r = s(1:3); % rEarSat
v = s(4:6);
normr = norm(r);
h = normr - earth.R;
rhat = r/normr;
khat = [0 0 1]';
rSunEar = planetHCI(3, JD);
moonM = moon.M0 + moon.n*(JD-moon.JD0)/86400;
rEarMoo = OEtoRVv2(moon.e,moon.i,moon.Om,moon.w,moonM,moon.n,earth.mu);
rSatMoo = r - rEarMoo;
rSunSat = rSunEar + r;
rSatSun = -rSunSat;
rEarSun = -rSunEar;

% FODE
f_CB = - earth.mu .* r ./ (normr^3); 

% Earth Oblateness
f_J2 = -(earth.mu*earth.J2*earth.R^2/2)*((6*r(3)/(normr^5))*khat+...
       ((3/(normr^4))-(15*r(3)^2/(normr^6)))*rhat);
   
% Lunar 3rd Body
f_M = rSatMoo.*moon.mu/(norm(rSatMoo)^3) - ...
      rEarMoo.*moon.mu/(norm(rEarMoo)^3);

% Solar 3rd Body
f_S = rSatSun.*mu_sun/(norm(rSatSun)^3) - ...
      rEarSun.*mu_sun/(norm(rEarSun)^3);

% Atmospheric Drag
v_rel = v-cross([0;0;earth.w],r);
rho = rho0*exp(-(h-h0)/H);
B = sat.CD*sat.A/sat.m;
f_D = -0.5*B*rho*(norm(v_rel)^2)*(v_rel./norm(v_rel));

% Solar Radiation Pressure
f_SRP = pSRP*sat.CSRP*(sat.A/sat.m)*rSunSat./norm(rSunSat);

%% State Derivative
sd(1:3) = v; % rdot = vel
sd(4:6) = f_CB + f_J2 + f_M + f_S + f_D + f_SRP;

end