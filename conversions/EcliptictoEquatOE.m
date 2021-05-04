function oe = EcliptictoEquatOE(oe)
    % Convert orbital elements from the ecliptic plane to the equatorial
    %{
    ---------------------------------------------------------------
    INPUT:
        oe:         6x1 double, orbital elements in ecliptic plane
                        [a e i Om w M_0]'
                        (m - rad rad rad rad)
    ---------------------------------------------------------------
    From: 
     - https://astronomy.stackexchange.com/questions/36954/converting-orbital-elements-from-equatorial-reference-frame-to-ecliptic
     - http://www.astro.utoronto.ca/~astrolab/files/AST326_Lab4_2017.pdf
    ---------------------------------------------------------------
    AUTHOR: Kaitlin Dennison, 2021
    ---------------------------------------------------------------
    %}

    R_ec2eq = R1(deg2rad(-23.43929111))'; 
    % rotating the ref frame instead of the vector so the angle is negative
    
    i_ec = oe(3);
    Om_ec = oe(4);
    w_ec = oe(5);

    R_ec2orb = R3(Om_ec)*R1(i_ec)*R3(w_ec);
    R_eq2orb = R_ec2eq*R_ec2orb;
    
    if all(abs([R_eq2orb(1,3),R_eq2orb(2,3),R_eq2orb(3,1),R_eq2orb(3,2)]) < 1E-9)
        Om_eq = atan2(-R_eq2orb(2,1),R_eq2orb(1,1));
        i_eq = acos(R_eq2orb(3,3));
        w_eq = 0;
    else
        Om_eq = atan2(R_eq2orb(1,3),R_eq2orb(2,3));
        i_eq = acos(R_eq2orb(3,3));
        w_eq = atan2(R_eq2orb(3,1),-R_eq2orb(3,2));
    end

    oe(3) = i_eq;
    oe(4) = Om_eq;
    oe(5) = w_eq;

end

%{
% test
mu = s3_constants('GM_SUN');
AU = s3_constants('AU');
R_ec2eq = R1(deg2rad(23.43929111))'; % for transofrming a vector

JD = 2451170.5; % 1998 Dec 23 00:00:00 UTC

% Eros from HORIZONS ephemeris
oe_HCI = [1.458260038106518*AU;
            0.2228858603247133;
            deg2rad(10.83015266864554);
            deg2rad(304.4308844737856);
            deg2rad(178.6132327246327);
            deg2rad(208.1235381788443)];
        
[r_HCI,v_HCI] = ConvOEtoRV(oe_HCI,mu);
    % validated with ephemeris

r_SCI_true = R_ec2eq*r_HCI;

oe_SCI = EcliptictoEquatOE(oe_HCI);

[r_SCI,v_SCI] = ConvOEtoRV(oe_SCI,mu);

r_SCI - r_SCI_true
%}