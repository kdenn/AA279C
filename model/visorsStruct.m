function visors = visorsStruct()
    visors(1) = struct();

    %% Orbit
    visorsOrbit;
    visors.oe = oe;
    visors.a = a;
    visors.e = e;
    visors.i = i;
    visors.Om = Om;
    visors.w = w;
    visors.M_0 = M_0;
    visors.T = T;
    visors.n = n;
    visors.mu = mu;
    visors.JD_epoch = JD_epoch;

    %% Inertia
    visorsInertia
    visors.I_body = I_body;
    visors.I_princ = I_princ;
    visors.A_rot = A_rot;
    visors.Ix = I_princ(1,1);
    visors.Iy = I_princ(2,2);
    visors.Iz = I_princ(3,3);

end