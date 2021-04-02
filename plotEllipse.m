function plotEllipse(e,a)
v = 0:360;
c = e*a;
rp = a-c;
b = a*sqrt(1-e^2);
x = a*cosd(v)-a+rp;
y = b*sind(v);
figure(); hold on 
    plot(x,y)
    plot(0,0,'x')
    axis equal
hold off
end