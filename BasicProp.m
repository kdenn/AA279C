function [ydot] = BasicProp(t,Y,mu)
    % Basic Propagator.
    ydot = zeros(6,1);
    r = [Y(1);Y(2);Y(3)];
    v = [Y(4);Y(5);Y(6)];
    ydot(1:3) = v;
    ydot(4:6) = -mu.*r./norm(r)^3;
end