function GMST = UTtoGMST(M,D,Y,sec)
if (Y < 1582) || (Y == 1582 && M < 10) || (Y == 1582 && M == 10 && D <= 4)
    B = -2+((Y+4716)/4)-1179;
else
    B = (Y/400)-(Y/100)+(Y/4);
end
if M <= 2
    % adjust for leap years
    Y = Y-1;
    M = M+12;
end
D = D + (sec/86400); % fraction of a day
MJD = 365*Y-679004+floor(B)+floor(30.6001*(M+1))+D;
GMST = 280.4606+360.9856473*(MJD-51544.5);
if GMST > 360 
    GMST = GMST - floor(GMST/360)*360;
end
GMST = deg2rad(GMST);
end