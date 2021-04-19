function q = dcm2quat(A)
    % convert the dcm to quaterion 4x1 representation

    N = size(A,3);
    q = zeros(4,N);

    for n = 1:N
        phi = acos(0.5 * (trace(A(:,:,n)) - 1));
        if phi == 0
            qns = [0;0;0;1];
        else
            e1 = (A(2,3) - A(3,2,n))/(2*sin(phi));
            e2 = (A(3,1) - A(1,3,n))/(2*sin(phi));
            e3 = (A(1,2) - A(2,1,n))/(2*sin(phi));
            qns = [e1*sin(phi/2);
                  e2*sin(phi/2);
                  e3*sin(phi/2);
                  cos(phi/2)];
        end
        q(:,n) = qns./norm(qns);
    end

end