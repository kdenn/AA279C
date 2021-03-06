function M = get_mag_torque(visors,JD,r_ECI,q,k)

R_ECI2prin = quat2dcm(q);

% approximate coil as a cylinder with h = 30cm, r = 7cm
mu = 4*pi*10^(-7);
n = 1;
S_sat = 2*pi*0.07*0.3;
I = 0.1;
m_body = mu*n*S_sat*I*[0; 0; 1];

B_ECI = magnetic_field(k,r_ECI,JD);

M = cross(visors.A_rot*m_body,R_ECI2prin*B_ECI);

% Magnitude check: 2*norm(m_body)*(30710E-9)*s3_constants('R_EARTH_WGS84')^3/norm(r_ECI)^3 = 1.8012e-19
    
end