function [e_vec,phi] = dcm2eulax(A)
    % convert the dcm to Eular axis/angle representation

    phi = acos(0.5 * (trace(A) - 1));

    if phi == 0
        % assume spin about z-axis
        e_vec = [0;0;1];
    else
        e1 = (A(2,3) - A(3,2))/(2*sin(phi));
        e2 = (A(3,1) - A(1,3))/(2*sin(phi));
        e3 = (A(1,2) - A(2,1))/(2*sin(phi));
        e_vec = [e1;e2;e3];
    end

end