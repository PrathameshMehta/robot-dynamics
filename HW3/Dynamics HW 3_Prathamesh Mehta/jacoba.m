function J_a = jacoba(S,M,q)    
% Your code here
    T = fkine(S,M,q);
    R = T(1:3,1:3);
    J_s = jacob0(S,q);
    P = T(1:3,4);
    skew_p = skew(P);
    r_p = skew_p*R;
    adjtrans = [R zeros(3,3);
                r_p R];
    J_b = inv(adjtrans)*J_s;
    J_vb = J_b(4:6,:);
    J_a = R*J_vb;
end