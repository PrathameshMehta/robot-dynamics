function adV = ad(V)
    % your code here
    omega = skew(V(1:3));
    V = skew(V(4:6));
    adV = [omega , zeros(3,3); V , omega ];
    
end
