function T = twist2ht(S,theta)
% Your code here
    omega = [S(1) S(2) S(3)];
    skew_omega = [0 -omega(3) omega(2) ; omega(3) 0 -omega(1) ; -omega(2) omega(1) 0 ];
    % If needed, you can calculate a rotation matrix with:
    R = axisangle2rot(omega,theta);
    v = [S(4) S(5) S(6)]';
    translation = (theta*eye(3) + (1 - cos(theta))*skew_omega + (theta - sin(theta))*skew_omega^2)*v;
    T = [R(1,1) R(1,2) R(1,3) translation(1); R(2,1) R(2,2) R(2,3) translation(2); R(3,1) R(3,2) R(3,3) translation(3); 0 0 0 1];
end