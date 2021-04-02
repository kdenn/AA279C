function roe = ConvOEtoROE(oe_chief,oe_dep)
    % convert orbital elements of deputy spacecraft to ROE
    %{
    ---------------------------------------------------------------
    INPUT:
        oe_chief:   oe of chief satellite
                        [a e i Om w M_0]'
                        (m - rad rad rad rad)
        oe_dep:     oe of deputy satellite
                        [a e i Om w M_0]'
                        (m - rad rad rad rad)
    ---------------------------------------------------------------
    OUTPUT:
        roe:      6x1 double [ada, adl, adex, adey, adix, adiy]
                                [m, m, m, m, m, m]
    ---------------------------------------------------------------
    AUTHOR: Corinne Lippe, 2019
    ---------------------------------------------------------------
    %}
    %pull chief orbital elements
    a_c = oe_chief(1);
    e_c = oe_chief(2);
    i_c = oe_chief(3);
    O_c = oe_chief(4);
    w_c = oe_chief(5);
    M_c = oe_chief(6);

    %pull deputy orbital elements
    a_d = oe_dep(1);
    e_d = oe_dep(2);
    i_d = oe_dep(3);
    O_d = oe_dep(4);
    w_d = oe_dep(5);
    M_d = oe_dep(6);

    %inititalize ROE vector
    roe = zeros(1,6);

    %ada
    roe(1) = (a_d-a_c);

    %adl
    roe(2) = (M_d-M_c)+w_d-w_c+cos(i_c)*(O_d-O_c);
    if roe(2) < - pi
        roe(2) = roe(2)+2*pi;
    end
    if roe(2) > pi
        roe(2) = roe(2) - 2*pi;
    end
    roe(2) = roe(2)*a_c;

    %adex
    roe(3) = a_c*(e_d*cos(w_d)-e_c*cos(w_c));

    %adey
    roe(4) = a_c*(e_d*sin(w_d)-e_c*sin(w_c));

    %adix
    roe(5) = i_d - i_c;
    if roe(5) < - pi
        roe(5) = roe(5)+2*pi;
    end
    if roe(5) > pi
        roe(5) = roe(5) - 2*pi;
    end
    roe(5) = roe(5)*a_c;

    %adiy
    roe(6) = sin(i_c)*(O_d-O_c);
    if roe(6) < - pi
        roe(6) = roe(6)+2*pi;
    end
    if roe(6) > pi
        roe(6) = roe(6) - 2*pi;
    end
    roe(6) = a_c*roe(6);

end