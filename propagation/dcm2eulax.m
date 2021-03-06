function e_vec = dcm2eulax(A)
    % convert the dcm to Eular axis/angle representation

    N = size(A,3);
    e_vec = zeros(4,N);

    for n = 1:N
        phi = acos(0.5 * (trace(A(:,:,n)) - 1));
        if phi == 0
            % assume spin about z-axis
            e1 = 0;
            e2 = 0;
            e3 = 1;
        else
            e1 = (A(2,3,n) - A(3,2,n))/(2*sin(phi));
            e2 = (A(3,1,n) - A(1,3,n))/(2*sin(phi));
            e3 = (A(1,2,n) - A(2,1,n))/(2*sin(phi));
        end
        ev = [e1;e2;e3]./norm([e1;e2;e3]);
        e_vec(:,n) = [ev;phi];
    end

end