function [h m s] = timeJD(JD)

D = floor(JD); % day
f = JD - D; % fraction of day
if f >= 0.5
    % after midnight
    h = (f-0.5)*24;
else
    h = f*24;
end
m = (h-floor(h))*60;
s = (m-floor(m))*60;
h = floor(h);
m = floor(m);
end