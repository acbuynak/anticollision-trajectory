function AdT = Adjoint(T)
% from lynch

[R, p] = TransToRp(T);
AdT = [R, zeros(3); VecToso3(p) * R, R];
end