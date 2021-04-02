function [year,day,sec] = JDtoYDS(JD)
% Convert a Julian Date to year-day-sec format
    V = datevec(JD - 1721058.5);
    year = V(1);
    sec = (V(4)*60 + V(5))*60 + V(6);
    month_days = [31,28,31,30,31,30,31,31,30,31,30,31];
    if mod(year,4) == 0 && (mod(year,100) ~= 0 || mod(year,400) == 0)
        month_days(2) = 29;
    end
    day = V(3);
    for m = 1:(V(2)-1)
        day = day + month_days(m);
    end
end