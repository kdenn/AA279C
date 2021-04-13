function [x,y,w_out] = polhodeAnalytical(I,w0,t)

Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

T = ((w0(1)^2 * Ix) + (w0(2)^2 * Iy) + (w0(3)^2 * Iz)) / 2;
L = sqrt((w0(1)^2 * Ix^2) + (w0(2)^2 * Iy^2) + (w0(3)^2 * Iz^2));

if ~exist('t','var')
    t = linspace(0,2*pi,100)';
end

x = zeros(numel(t),6);
y = zeros(numel(t),6);

dxy = abs(Ix-Iy);
dxz = abs(Ix-Iz);
dzy = abs(Iz-Iy);

if dxz < 1E-4
    % Axially symmetric about y
elseif dxy < 1E-4
    % Axially symmetric about z
elseif dzy < 1E-4
    % Axially symmetric about x
    
    Im = Iz;
    wx0 = w0(1);
    wy0 = w0(2);
    wz0 = w0(3);
    lambda = wx0*(Ix-Im)/Im;
    
    wx_d0 = (-(Iz-Iy)*wy0*wz0)/Ix;
    wy_d0 = (-(Ix-Iz)*wx0*wz0)/Iy;
    wz_d0 = (-(Iy-Ix)*wx0*wy0)/Iz;
    
    c1 = (wy0/2) - (wy_d0/(2*1i*lambda));
    c2 = (wy0/2) + (wy_d0/(2*1i*lambda));
    c3 = (wz0/2) - (wz_d0/(2*1i*lambda));
    c4 = (wz0/2) + (wz_d0/(2*1i*lambda));
    
    wx = ones(size(t)).*wx0;
    wy = c1*exp(1i*lambda.*t)+c2*exp(-1i*lambda.*t);
    wz = c3*exp(1i*lambda.*t)+c4*exp(-1i*lambda.*t);
    w_out = [wx,wy,wz];
    
    x(:,1) = ones(size(t)).*wx0;
    y(:,1) = wy;
    x(:,2) = wy;
    y(:,2) = wz;
    x(:,3) = x(:,1);
    x(:,4) = x(:,1);
    x(:,5) = -x(:,1);
    x(:,6) = -x(:,1);
    y(:,3) = wz;
    y(:,4) = -wz;
    y(:,5) = wz;
    y(:,6) = -wz;
    
else
    % xy plane
    a = sqrt( (L^2-2*T*Iz) / ((Ix-Iz)*Ix) );
    b = sqrt( (L^2-2*T*Iz) / ((Iy-Iz)*Iy) );
    x(:,1) = a*cos(t); %wx
    y(:,1) = b*sin(t); %wy
    
    % yz plane 
    a = sqrt( (L^2-2*T*Ix) / ((Iy-Ix)*Iy) );
    b = sqrt( (L^2-2*T*Ix) / ((Iz-Ix)*Iz) );
    x(:,2) = a*cos(t); %wy
    y(:,2) = b*sin(t); %wz
    
    % xz plane
    a = - sqrt( (L^2-2*T*Iy) / ((Ix-Iy)*Ix) );
    b = sqrt( -(L^2-2*T*Iy) / ((Iz-Iy)*Iz) );
    x(:,3) = linspace(a, 2*a, numel(t));
    x(:,4) = x(:,3);
    x(:,5) = -x(:,3);
    x(:,6) = -x(:,3);
    y(:,3) = b.*sqrt(x(:,3).^2 ./ a.^2 - 1);
    y(:,4) = -b.*sqrt(x(:,3).^2 ./ a.^2 - 1);
    y(:,5) = y(:,3);
    y(:,6) = y(:,4);
    
    w_out = [];

end

