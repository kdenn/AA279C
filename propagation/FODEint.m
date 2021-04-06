function statedot = FODEint(t,state)
% state: [rx ry rz vx vy vz]'

mu = s3_constants('GM_EARTH'); % m^3/s^2

statedot = zeros(6,1);

f_CB = -mu.*state(1:3)./(norm(state(1:3),2)^3);

statedot(1:3) = state(4:6); % rdot = vel
statedot(4:6) = f_CB;

end