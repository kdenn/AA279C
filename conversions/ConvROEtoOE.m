function oe = ConvROEtoOE(oe_chief,roe)
    % convert relative orbital elements of deputy spacecraft to classical OE
    %{
    ---------------------------------------------------------------
    INPUT:
        oe_chief:       6x1 double, keplerian OE of chief
                            [a, e, i, OM, w, M]
                            (m, - , rad, rad ,rad rad)
        roe:            6x1 double, relative OE of deputy
                            [ada adlam ade_x ade_y adi_x adi_y]
                            (m m m m m m)
    ---------------------------------------------------------------
    OUTPUT:
        oe:             6x1 double, keplerian OE of deputy
                            [a, e, i, OM, w, M]
                            (m, - , rad, rad ,rad rad)
    ---------------------------------------------------------------
    AUTHOR: Corinne Lippe, 2019
    ---------------------------------------------------------------
    %}

    % pull chief data
    a_c = oe_chief(1);
    e_c = oe_chief(2);
    i_c = oe_chief(3);
    OM_c = oe_chief(4);
    w_c = oe_chief(5);
    M_c = oe_chief(6);
    u_c = w_c+M_c;

    % pull ROE
    ada = roe(1);
    adl = roe(2);
    adex = roe(3);
    adey = roe(4);
    adix = roe(5);
    adiy = roe(6);

    % unnormalize ROE
    dl = adl/a_c;
    dex = adex/a_c;
    dey = adey/a_c;
    dix = adix/a_c;
    diy = adiy/a_c;

    % transform
    a_d = ada + a_c;
    e_d = sqrt((dex + e_c.*cos(w_c)).^2 + (dey + e_c.*sin(w_c)).^2);
    i_d = dix + i_c;
    OM_d = mod(diy./sin(i_c) + OM_c, 2*pi);
    w_d = atan2(dey + e_c.*sin(w_c), dex + e_c.*cos(w_c));
    u_d = mod(dl - (OM_d - OM_c).*cos(i_c) + u_c, 2*pi);
    M_d = mod(u_d - w_d, 2*pi);

    %produce output vector
    oe = [a_d; e_d; i_d; OM_d; w_d; M_d];

end