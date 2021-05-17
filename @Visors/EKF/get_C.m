function C = get_C(x, u, dt)
% Purpose: calculate measurement Jacobian

% Renaming variables
px = x(1); py = x(2); p = x(1:2);

C = [px/norm(p), py/norm(p), 0];
end