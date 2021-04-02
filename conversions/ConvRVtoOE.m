function oe = ConvRVtoOE(r,v,mu)
    % Convert cartesian inertial to kelperian OEs
    %{
    ---------------------------------------------------------------
    INPUT:
        r:          3x1 double, position in the frame of OE's (m)
        v:          3x1 double, velocity in the frame of OE's (m/s)
        mu:         double, gravitational param of CB
    ---------------------------------------------------------------
    OUTPUT:
        oe:         6x1 double, orbital elements in inertial space
                        [a e i Om w M_0]'
                        (m - rad rad rad rad)
    ---------------------------------------------------------------
    AUTHOR: Corinne Lippe, 2020
    ---------------------------------------------------------------
    %}

    %calculate h and unit vector in h direction
    h=cross(r,v);
    wvec=h/norm(h);

    %calculate i and RAAN using h
    i=atan2(sqrt(wvec(1)^2+wvec(2)^2),wvec(3));
    RAAN=atan2(wvec(1),-wvec(2));

    %calculate needed paramenters
    p=norm(h)^2/mu;
    a=(2/norm(r)-norm(v)^2/mu)^(-1);
    n=sqrt(mu/a^3);

    %calculate e
    e=sqrt(1-p/a);
    evec=cross(v,h)/mu-r/norm(r);

      %calculate all angle paramenters
      E=atan2(dot(r,v)/(a^2*n),1-norm(r)/a); %returns radians
      M=E-e*sin(E); %returns radians, I don't think this M is actually necessary
      %though despite the notes from Professor D'amico's lecture
      cos_nu=(cos(E)-e)/(1-e*cos(E));
      sin_nu=sin(E)*sqrt(1-e^2)/(1-e*cos(E));
      true_anomaly=(atan2(sin_nu, cos_nu));

      %compute u
      u=atan2(r(3)/sind(i),r(1)*cosd(RAAN)+r(2)*sind(RAAN));

      %computer omega
      omega=u-true_anomaly;

    %Convert all angles to ensure they are between 0 and 2*pi
    if M <0;
        M=M+2*pi;
    end
    if omega <0
        omega=omega+2*pi;
    end
    oe = [a, e, i, RAAN, omega, M];