function R_WldtoCam = RotWldtoCam(r_sat,v_sat,eul_sat)
    % Generate the rotation matrix to transform from wld to Cam
    %{
    ---------------------------------------------------------------
    INPUT: R_WldtoCam = RotWldtoCam(r_sat_wld,v_sat_wld,eul_sat)
        r_sat:      3x1 double, wld position (m)
        v_sat:      3x1 double, wld velocity (m/s)
        eul_sat:    3x1 double, Euler angles  in 3-2-1
    ---------------------------------------------------------------
    OUTPUT:
        R_WldtoCam:  3x3 double, rot matrix from wld to the camera
                    frame
    ---------------------------------------------------------------
    NOTES:
        - This should be replaced by a rotation matrix from the
          star-tracker
        - Assumes the camera is pointing directly at the COM of the
          asteroid
    ---------------------------------------------------------------
    %}

    h = cross(r_sat,v_sat); % angular momentum of sat
    % AA273 AR lec 5 s10
    kc = -r_sat./norm(r_sat); % Directly towards asteroid CM
    jc = h./norm(h); % Aligned with angular momentum
    ic = cross(jc,kc)./norm(cross(jc,kc)); % Complete the RH triad
    if nargin < 3
        R_WldtoCam = [ic';jc';kc'];
    else
        % Rotate from looking directly at CM via 3-2-1
        Rc = R1(eul_sat(3))*R2(eul_sat(2))*R3(eul_sat(1));
        iu = Rc*ic;
        ju = Rc*jc;
        ku = Rc*kc;
        iw = [1 0 0];
        jw = [0 1 0];
        kw = [0 0 1];
        R_WldtoCam = [dot(iw,iu) dot(jw,iu) dot(kw,iu);
             dot(iw,ju) dot(jw,ju) dot(kw,ju);
             dot(iw,ku) dot(jw,ku) dot(kw,ku)];
    end
end