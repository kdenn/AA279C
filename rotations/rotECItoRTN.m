function R = rotECItoRTN(Om,incl,w,e,nu_or_r)

R_P2E = rotPQWtoIJK(Om,incl,w,e);

if numel(nu_or_r) > 1
    r_E = nu_or_r;
    r_P = R_P2E'*r_E;
    nu = atan2(r_P(2),r_P(1));
else
    nu = nu_or_r;
end 
 
R = R3(nu)*R_P2E';

end