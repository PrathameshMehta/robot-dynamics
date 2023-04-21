function J_v = jacobe(S,M,q)    
% Your code here
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    R = T(1:3,1:3);
    P = T(1:3,4);
    skew_p = skew(P);
    r_p = skew_p*R;
    adjtrans = [R zeros(3,3);
                r_p R];
    J_v = inv(adjtrans)*J_s;
end
