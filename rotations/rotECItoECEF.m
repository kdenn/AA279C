function R = rotECItoECEF(GMST)
R = [cos(GMST) sin(GMST) 0;
     -sin(GMST) cos(GMST), 0;
     0 0 1];
end