function x = cheby(n,a,b,rev)
if nargin == 3
    rev = false;
end
x = zeros(1,n);
for k = 1:n
    x(k) = (a + b)/2 + ((b - a)/2)*cos((2*k - 1)*pi/(2*n));
end

if rev
    x(1:floor(n/2)) = x(1:floor(n/2))-b;
end
end