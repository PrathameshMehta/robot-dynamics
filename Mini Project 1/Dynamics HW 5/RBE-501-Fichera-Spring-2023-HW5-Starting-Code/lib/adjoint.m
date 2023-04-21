function Vtrans = adjoint(V,T)
    R = T(1:3,1:3);
    p = [0 -1*T(3,4) T(2,4); T(3,4) 0 -1*T(1,4); -1*T(2,4) T(1,4) 0];
    Adj = [R zeros(3); p*R R];
    Vtrans = Adj * V;
end
