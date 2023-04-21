function R = axisangle2rot(omega,theta)
    I = eye(3);
    omega = [ 0 -omega(1,3) omega(1,2) ; omega(1,3) 0 -omega(1,1) ; -omega(1,2) omega(1,1) 0];
    R = I + sin(theta)*omega + (1-cos(theta))*omega*omega;
end