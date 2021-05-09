function r_sun = get_sun_position(JD)
% get the ECI position of the sun (m)

    flag = 3;
    
    % Use Matlab 'planetEphemeris' toolbox (seems to run slow)
    % Average run time ~ 0.25 sec
    if flag == 1
        % sun position in ICRF (J2000 ecliptic)
        r_ecliptic = planetEphemeris(JD,'Earth','Sun')'*1000;

        % convert to equatorial (ECI)
        r_sun = R1(deg2rad(23.43929111))*r_ecliptic;
        
    % Use s3 ephemeris function to get r_sun (causes dependency on s3)
    % Average run time ~ 0.0002 sec
    elseif flag == 2
        r_sun = s3_em_thirdbody_sun(JD);
        
    % Use Josh's code for propagating Sun orbit assuming Keplerian motion
    % and using initial orbital elements at J2000 from JPL SSD website (not
    % as precise as ephemeris models)
    % Average run time ~ 0.005 sec
    elseif flag == 3
        rv_Earth = get_rv_Earth(get_ICs(), JD-2400000.5);
        r_sun = -rv_Earth(1:3).*1000;
    end
    
end