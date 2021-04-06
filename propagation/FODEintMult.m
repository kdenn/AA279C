function statedot = FODEintMult(t,state)

N = length(state)/7;

statedot = zeros(N*7,1);

for i = 0:(N-1)
    statedot(1+i*7) = 0;
    statedot((2:4)+i*7) = state((5:7)+i*7);
    r_i = state((2:4)+i*7);
    a_i = 0;
    mu_i = state(i*7+1);
    for j = 0:(N-1)
        if i ~= j
            r_j = state((2:4)+j*7);
            mu_j = state(j*7+1);
            a_i = a_i - mu_j.*(r_i-r_j)./(norm((r_i-r_j),2)^3);
        end
    end
    statedot((5:7)+i*7) = a_i;
end

end