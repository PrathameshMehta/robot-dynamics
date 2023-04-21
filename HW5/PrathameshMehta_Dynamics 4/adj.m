function ADJOINT = adj(T)
    R = T(1:3,1:3);
    P = T(1:3,4);
    Ps= skew(P);
    ST = Ps*R;
    O = zeros(3);
    ADJOINT = [R O;ST R];
end
