% AA279A Space Mechanics
% Kaitlin Dennison

%% Problem 3

mu = 1.327E11; %km3/s2
AU = 1.496E8; %km
a = 17.834*AU;
e = 0.967;
% a)
    n = sqrt(mu/a^3); %rad/s
    T = 2*pi/n; %s
    Ty = T/(365*86400); %years
% b)
    %Ec = iterMtoEc(M,e,tol);
% c)
    t0 = 0;
    tf = T;
    tol = 1E-10;
    M0 = 0; %(because t0=0)
    dT = 0.04*T;
    t = sim('PS_03_sim.slx');
% d)
    figure(); hold on
    title('PQW of Halleys Comet')
    plot(xPQW,yPQW,'o')
    plot(0,0,'g*')
    xlabel('x')
    ylabel('y')
    axis equal
    hold off
    
%% Problem 4
    