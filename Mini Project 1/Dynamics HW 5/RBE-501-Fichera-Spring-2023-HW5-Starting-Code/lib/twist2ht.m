function T = twist2ht(S,theta)
    omega = S(1:3);
    SS = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
    I = eye(3);
    R = I + sin(theta)* SS + (1-cos(theta))*SS^2;
    V = S(4:6);
    Z = ((I*theta) + ((1-cos(theta))*SS)+ (((theta - sin(theta))*SS^2)))*V;
    T = [ R Z ; 0 0 0 1];
end