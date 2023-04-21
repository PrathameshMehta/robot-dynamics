function Vb = adjoint(Va,T)
    % Your code here
    R = T(1:3,1:3);
    P = T(1:3,4);
    skew_p = [0 -P(3) P(2) ; P(3) 0 -P(1) ; -P(2) P(1) 0 ];
    r_p = skew_p*R;
    adjtrans = [R zeros(3,3);
                r_p R];
    Vb = adjtrans*Va;
end

