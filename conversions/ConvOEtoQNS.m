function oe_qns = ConvOEtoQNS(oe)
    % convert classical orbital elements to quasinonsingular
    %{
    ---------------------------------------------------------------
    INPUT:
        oe:         6x1 double, classical oes
                    elements (navigation frame)
                        [a e i Om w M_0]'
                        (m - rad rad rad rad)
    ---------------------------------------------------------------
    OUTPUT:
        oe_qns:      6x1 double, qns oes
                        [a, u, ex, ey, i, OM]
                        (m, rad , -,- ,rad rad)
    ---------------------------------------------------------------
    AUTHOR: Corinne Lippe, 2019
    ---------------------------------------------------------------
    %}

    oe_qns = zeros(6,1);
    oe_qns(1) = oe(1); %a is same
    oe_qns(2) = oe(5)+oe(6); % u = M+ w
    oe_qns(3) = oe(2)*cos(oe(5)); % ex = ecosw
    oe_qns(4) = oe(2)*sin(oe(5)); % ey = esinw
    oe_qns(5) = oe(3); %i
    oe_qns(6) = oe(4); %OM
end