function visors = visorsStruct()
    visors(1) = struct();

    %% Orbit
    visorsOrbit;
    visors.oe = oe;
    visors.a = a;
    visors.e = e;
    visors.incl = incl;
    visors.Om = Om;
    visors.w = w;
    visors.M_0 = M_0;
    visors.T = T;
    visors.n = n;
    visors.mu = mu;
    visors.JD_epoch = JD_epoch;
    
    [rECI,vECI,rPQW,vPQW,nu] = OEtoRVv2(visors.e,visors.incl,visors.Om,visors.w,visors.M_0,visors.n,...
    visors.mu);

    visors.r_ECI_0 = rECI;
    visors.v_ECI_0 = vECI;
    visors.r_PQW_0 = rPQW;
    visors.v_PQW_0 = vPQW;
    visors.nu_0 = nu;

    %% Inertia
    visorsInertia
    visors.I_body = I_body;
    visors.I_princ = I_princ;
    visors.A_rot = A_rot;
    visors.Ix = I_princ(1,1);
    visors.Iy = I_princ(2,2);
    visors.Iz = I_princ(3,3);

end