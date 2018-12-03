function diffn = differencing(x,n,dt)
diffn(1) = x(1);
for i = 1:n-1
    diffn_all = diff(x,i)./dt.^i;
    diffn(i+1) = diffn_all(1);
end
