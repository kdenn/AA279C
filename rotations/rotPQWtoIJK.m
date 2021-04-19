function R = rotPQWtoIJK(Om,i,w,e)
% Rotation Matrices
       RzOm = [cos(-Om),sin(-Om),0;...
              -sin(-Om),cos(-Om),0;...
              0,0,1];
       Rxi = [1,0,0;...
              0,cos(-i),sin(-i);...
              0,-sin(-i),cos(-i)];
       Rzw = [cos(-w),sin(-w),0;...
              -sin(-w),cos(-w),0;...
              0,0,1];
       if i == 0 && e ~= 0
       % Equatorial & Elliptical
       R = Rzw;
       elseif e == 0 && i ~= 0
       % Circular & Inclined
       R = RzOm*Rxi;
       elseif i == 0 && e == 0
       % Equatorial & Circular
       R = eye(3);
       else
       R = RzOm*Rxi*Rzw;
       end
end