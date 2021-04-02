function TLE = readTLE(file)
% Source: https://www.mathworks.com/matlabcentral/fileexchange/56904-read-two-line-element-ephemeris-files?focused=6262366&tab=function
earth = getEarthConst();

fd = fopen(file,'r');
if fd < 0, fd = fopen([file '.tle'],'r'); end
assert(fd > 0,['Can''t open file ' file ' for reading.'])

A1 = fgetl(fd);
A2 = fgetl(fd);

satNum = A1(3:7);
class = A1(8);

disp(['Satellite ',satNum,class])

N = 1;

while ischar(A2)
    assert(chksum(A1), 'Checksum failure on line 1')
    assert(chksum(A2), 'Checksum failure on line 2')
    epY = str2double(['20',A1(19:20)]);
    epD = str2double(A1(21:32));
    JD0 = datenum(epY,00,epD,00,00,00)+1721058.5;
    i = str2double(A2(9:16));
    Om = str2double(A2(18:25));
    e = str2double(['.' A2(27:33)]);
    w = str2double(A2(35:42));
    M = str2double(A2(44:51));
    n = str2double(A2(53:63));
    T = 86400/n;
    a = ((T/(2*pi))^2*earth.mu)^(1/3);
    b = a*sqrt(1-e^2);

    TLE(N,:) = [a,e,i,Om,w,M,n,T,b,JD0];
    N = N + 1;

    A1 = fgetl(fd);
    A2 = fgetl(fd);
end

fclose(fd);

TLE = sortrows(TLE,10,'decend');

end

% Checksum (Modulo 10)
% Letters, blanks, periods, plus signs = 0; minus signs = 1
function result = chksum(str)
  result = false; c = 0;
  
  for k = 1:68
    if str(k) > '0' && str(k) <= '9'
      c = c + str(k) - 48;
    elseif str(k) == '-'
      c = c + 1;
    end
  end

  if mod(c,10) == str(69) - 48
    result = true;
  end
  
end