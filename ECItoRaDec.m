function Y = ECItoRaDec(rECI,q)
rObsSat = rECI - q;
rho = norm(rObsSat);
rHat = rObsSat./rho;

delta = asind(rHat(3));
if rHat(2) > 0
    alpha = acosd(rHat(1)/cosd(delta));
else
    alpha = 360 - acosd(rHat(1)/cosd(delta));
end
Y = [alpha;delta];
end