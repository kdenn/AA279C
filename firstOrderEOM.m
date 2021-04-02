function ds = firstOrderEOM(t,s)
% 1st order state-space EOMs in synodic frame for ode113
% s = [r_s3,v_s3]' = [x,y,z,x',y',z']'

%% Problem constants
% Change these for the problem
mu_1 = 398600; % km^3/s^2
mu_2 = 4903; % km^3/s^2
R_12 = 384400; % km

%% Calculations
R_1 = R_12*mu_2/(mu_1+mu_2);
R_2 = R_12-R_1;
r_s1 = [-R_1,0,0]';
r_s2 = [R_2,0,0]';
r_s3 = s(1:3);
v_s3 = s(4:6);
r_13 = r_s3-r_s1;
r_23 = r_s3-r_s2;
ws = [0;0;sqrt((mu_1+mu_2)/((R_1+R_2)^3))];
dv = -(mu_1/norm(r_13)^3).*r_13-(mu_2/norm(r_23)^3).*r_23-...
    2*cross(ws,v_s3)-cross(ws,cross(ws,r_s3));

%% EOMs
ds = zeros(6,1);
ds(1) = v_s3(1); % x'
ds(2) = v_s3(2); % y'
ds(3) = v_s3(3); % z'
ds(4) = dv(1); % x''
ds(5) = dv(2); % y''
ds(6) = dv(3); % z''

end