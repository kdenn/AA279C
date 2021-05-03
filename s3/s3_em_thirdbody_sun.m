% s3_em_thirdbody_sun.m
% [r_sun_eci] = s3_em_thirdbody_sun(jd_tt)
% -------------------------------------------------------------------------
% DESCRIPTION: computes position vector from Earth to Sun (ECI)
% -------------------------------------------------------------------------
% INPUTS: jd_tt - (1,1) Julian Date, Terrestrial Time
% -------------------------------------------------------------------------
% OUTPUTS: r_sun_eci - (3,1) position vector from Earth to Sun (ECI) [m]
% ------------------------------------------------------------------------
% EXAMPLE: jd_tt = cal2jd([2016 5 5 0 0 0]);
%          r_sun_eci = s3_em_thirdbody_sun( jd_tt );
% ------------------------------------------------------------------------
% REFERENCE: "Satellite Orbits: Models, Methods, Applications"
%            O. Montenbruck, E. Gill 2000-05-01
% -------------------------------------------------------------------------
% AUTHOR: Connor Beierle 2017-04-12
%         Duncan Eddy    2017-05-02
% -------------------------------------------------------------------------
% COPYRIGHT: 2016 SLAB Group
%            S3 Function
% -------------------------------------------------------------------------