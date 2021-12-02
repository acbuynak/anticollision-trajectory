function cdAB = cdist(A,B) % Cartesian distance starting from A ending at B

    As = sym(ones(size(A)));    Bs = sym(ones(size(B)));
    As(:) = A(:);    Bs(:) = B(:);
    n = length(As);
    sAB = sym(ones(1,n));
    
    for ii = 1:n
        del = Bs(ii) - As(ii);
        sAB(ii) = simplify((del)^2);    
    end
    
    sAB = simplify(sum(sAB));
    cdAB = (sAB)^(1/2);

end