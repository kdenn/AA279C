function plotParabola(rp)
e = 1;
v = -150:150;
p = rp*(1+e);
r = p./(1+e.*cosd(v));
x = r.*cosd(v);
y = r.*sind(v);
figure(); hold on
    plot(x,y)
    plot(0,0,'x')
    axis equal
hold off
end