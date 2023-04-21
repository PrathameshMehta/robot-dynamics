function R = axisangle2rot(omega,theta)
 % your code here
 omega = [0   -omega(3)   omega(2) ; omega(3)   0  -omega(1); -omega(2)   omega(1)   0];
 R = eye(3)+ sin(theta)*omega + (1-cos(theta))*omega^2;
end
