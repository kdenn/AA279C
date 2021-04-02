% Final Exam

in = 135;
e = 0.2;
r0 = [1000;0;0];

Rc = 473;
J2 = 0.0303;
G = (6.6741E-11)/(1000^3);
mc = 8.958E20;
Tc = (9*60*60)+(4*60);
nc = (2*pi)/Tc;
wc = 0.0111; %deg/s

%% 
nu0 = 0;
rp = r0(1);
a = rp/(1-e);
Om = 0;
w0 = 0;
mu = G*mc;
T = 2*pi*sqrt(a^3/mu);
n = 2*pi/T;
dwdt = (3/4)*n*J2*(Rc/(a*(1-e^2)))^2*(5*cosd(in)^2-1);
dOmdt = -(3/2)*n*J2*(Rc/(a*(1-e^2)))^2*cosd(in);

dw = dwdt*T;
dOm = dOmdt*T;

%% Feature position after one orbit
rf0 = [Rc;0;0];
ang1 = mod(deg2rad(wc*T),2*pi);
rf1 = R3(-ang1)*rf0;

%% Relative position to the feature
lw = rp*dw;
lOm = rp*dOm;
beta2 = deg2rad(90-(180-in));
dy = lOm - lw*sin(beta2);
dz = lw*cos(beta2);
r1 = [r0(1),dy,dz]';
rdf1 = rf1-r1;

%% Triangles
dr = norm(r1-r0);
beta1 = asind(r1(3)/dr);
beta2 = 180-(180-in)-beta1;
Om = (dr*sind(beta2)/sind(180-in))/norm(r1);
w = (dr*sind(beta1)/sind(180-in))/norm(r1);

%% J2
J2p = (-2*(Om/T)/(3*n*cosd(in)))*(Rc/(a*(1-e^2)))^(-2)


