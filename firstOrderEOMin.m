function ds = firstOrderEOMin(t,s)
% 1st order state-space EOMs in inertial frame for ode113
% s = [r_i3,v_i3]' = [x,y,z,x',y',z']'

%% Problem constants
% Change these for the problem
mu_1 = 398600; % km^3/s^2
mu_2 = 4903; % km^3/s^2
R_12 = 384400; % km

%% Calculations
R_1 = R_12*mu_2/(mu_1+mu_2);
R_2 = R_12-R_1;
ws = [0;0;sqrt((mu_1+mu_2)/((R_1+R_2)^3))];

r_i1 = [-R_1*cos(ws(3)*t),-R_1*sin(ws(3)*t),0]';
r_i2 = [R_2*cos(ws(3)*t),R_2*sin(ws(3)*t),0]';
r_i3 = s(1:3);
v_i3 = s(4:6);
r_13 = r_i3-r_i1;
r_23 = r_i3-r_i2;

dv = -(mu_1/norm(r_13)^3).*r_13-(mu_2/norm(r_23)^3).*r_23;

%% EOMs
ds = zeros(6,1);
ds(1) = v_i3(1); % x'
ds(2) = v_i3(2); % y'
ds(3) = v_i3(3); % z'
ds(4) = dv(1); % x''
ds(5) = dv(2); % y''
ds(6) = dv(3); % z''

end