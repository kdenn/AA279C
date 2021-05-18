function plotFillRegion(x,y1,y2,c,alpha)

if nargin == 4
    alpha = 1;
end

plot(x,y1,'Color',c);
hold on;
plot(x,y2,'b','Color',c);
x2 = [x;flipud(x)];
inBetween = [y1;flipud(y2)];
patch(x2,inBetween,c,'EdgeColor','none','FaceAlpha',alpha);

end