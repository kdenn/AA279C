function R = rotECItoRTN(Om,incl,w,e,nu)
R = R3(nu)*rotPQWtoIJK(Om,incl,w,e)';
end