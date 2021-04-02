function R = rotECItoRTN(Om,i,w,nu)
R = R3(nu)*rotPQWtoIJK(Om,i,w)';
end