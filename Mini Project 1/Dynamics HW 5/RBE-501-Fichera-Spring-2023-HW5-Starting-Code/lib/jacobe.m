function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');  
    Ti = inv(T);
    R = Ti(1:3,1:3);
    p = [0 -1*Ti(3,4) Ti(2,4); Ti(3,4) 0 -1*Ti(1,4); -1*Ti(2,4) Ti(1,4) 0];
    Adj = [R zeros(3); p*R R];
    J_b = Adj * J_s;
end