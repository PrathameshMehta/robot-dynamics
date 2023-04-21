function T = twist2ht(S,theta)
    w = [S(1) S(2) S(3)];
    W_skew = [0 -w(3) w(2) ; w(3) 0 -w(1) ;-w(2) w(1) 0];
    R = axisangle2rot(w,theta);
    v = [S(4) S(5) S(6)]';
    P = (eye(3)*theta + (1-cos(theta))*(W_skew) + (theta - sin(theta))*(W_skew)^2)*v;
    T= [ R P; 0 0 0 1];
end