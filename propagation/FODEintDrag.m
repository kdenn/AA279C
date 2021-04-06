function statedot = FODEintDrag(t,state)
% state: [rx ry rz vx vy vz]'

Re = 6378.1370; %km
H = 10.000; %km
mu = 3.986E5; %km^3/s^2
rho0 = 1.225E9; %kg/km^3
m = 1500; %kg
Cd = 2.3; %unitless
A = 2E-5; %km^2

statedot = zeros(6,1);

f_CB = -mu.*state(1:3)./(norm(state(1:3),2)^3);
rho = rho0*exp(-((norm(state(1:3),2)-Re)/H));
f_D = -(0.5*Cd*A*rho/m)*norm(state(4:6),2).*state(4:6);

statedot(1:3) = state(4:6); % rdot = vel
statedot(4:6) = f_CB + f_D;

end