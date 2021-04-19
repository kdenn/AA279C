function R = rotECItoRTN(Om,i,w,e,nu)
R = R3(nu)*rotPQWtoIJK(Om,i,w,e)';
end