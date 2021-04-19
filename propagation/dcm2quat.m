function q = dcm2quat(A)
    % convert the dcm to quaterion 4x1 representation

    phi = acos(0.5 * (trace(A) - 1));

    if phi == 0
        q = [0;0;0;1];
    else
        e1 = (A(2,3) - A(3,2))/(2*sin(phi));
        e2 = (A(3,1) - A(1,3))/(2*sin(phi));
        e3 = (A(1,2) - A(2,1))/(2*sin(phi));
        q = [e1*sin(phi/2);
             e2*sin(phi/2);
             e3*sin(phi/2);
             cos(phi/2)];
    end
    
    q = q./norm(q);

end