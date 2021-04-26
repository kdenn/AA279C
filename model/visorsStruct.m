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
    visors.A_rot = A_rot; % body to princ
    visors.Ix = I_princ(1,1);
    visors.Iy = I_princ(2,2);
    visors.Iz = I_princ(3,3);
    
    %% Model + Mass
    visorsModel;
    % f_bus, v_bus, f_pan, v_pan_1, v_pan_2
    % Get CoMs in body frame
    [com_bus,bary_bus,area_bus,norm_bus] = spacecraftGeom(f_bus,v_bus);
    [com_pan_1,bary_pan_1,area_pan_1,norm_pan_1] = spacecraftGeom(f_pan,v_pan_1);
    [com_pan_2,bary_pan_2,area_pan_2,norm_pan_2] = spacecraftGeom(f_pan,v_pan_2);
    m_bus = 10.92; %kg
    m_panel = 0.54; %kg
    visors.m = m_bus + 2*m_panel;
    com_body = (m_bus.*com_bus' + m_panel.*com_pan_1' + ...
        m_panel.*com_pan_2')./visors.m;
    if all(abs(com_body) < 1E-9)
        % below computer precision, effectively 0
        visors.com_body = zeros(3,1);
    else
        visors.com_body = com_body;
    end
    visors.com_princ = A_rot*visors.com_body;
    

end