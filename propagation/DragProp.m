function [ydot] = DragProp(t,Y,mu,M,cd,A,w_mars)
    % Drag Propagator.
    % Shouldn't be given rho, should calculate here.
    ydot = zeros(6,1);
    r = [Y(1);Y(2);Y(3)];
    v = [Y(4);Y(5);Y(6)];
    h = norm(r)-3397;                               % Start accounting for drag at 150 km? Compare w/Mars first.
    rho = 0.0156 * exp(-h /11);                      % Scale height equation for Mars. For scale height of 11 km: http://planetfacts.org/the-atmosphere-of-mars/ 
    [avec,~] =  WhatADrag(r,v,M,cd,A,rho,w_mars);
    ydot(1:3) = v;
    ydot(4:6) = -mu.*r./norm(r)^3+avec(1:3).';
end