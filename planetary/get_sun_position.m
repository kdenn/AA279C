function r_sun = get_sun_position(JD)
% get the ECI position of the sun (m)

    % sun position in ICRF (J2000 ecliptic)
    r_ecliptic = planetEphemeris(JD,'Earth','Sun')'*1000;

    % convert to equatorial (ECI)
    r_sun = R1(deg2rad(23.43929111))*r_ecliptic;

end