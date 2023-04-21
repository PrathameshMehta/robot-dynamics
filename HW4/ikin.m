function q = ikin(twist, M, currentQ, targetPose)
    currentPose = MatrixLog6(M);
    currentPose = [currentPose(3, 2) currentPose(1, 3) currentPose(2, 1) currentPose(1:3, 4)']';
    
    count = 0;

    for  count = 1:1000
       if  norm(targetPose - currentPose) < 1e-10;
           break 
       end
    
        count = count + 1;
       

        J = jacob0(twist, currentQ);        
        deltaQ = pinv(J) * (targetPose - currentPose);
        currentQ = currentQ + deltaQ' - 2*pi*floor( (currentQ + deltaQ'+pi)/(2*pi) );

        T = fkine(twist, M, currentQ);

        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3, 2) ...
                       currentPose(1, 3) ...
                       currentPose(2, 1) ...
                       currentPose(1:3, 4)']';

    end
    q = currentQ;
end

function R = axisangle2rot(omega_val, theta)
    bracket_omega = [0             -omega_val(3)       omega_val(2) ;
                 omega_val(3)       0             -omega_val(1) ;
                -omega_val(2)       omega_val(1)       0       ];
    R = eye(3) + sin(theta) * bracket_omega + (1 - cos(theta)) * bracket_omega * bracket_omega;
end

function T = twist2ht(twist, theta)
 omega_val = [twist(1); twist(2); twist(3)];
 rot = axisangle2rot(omega_val, theta);
 bracket_omega = [0 -twist(3) twist(2); twist(3) 0 -twist(1); -twist(2) twist(1) 0];
 v = [twist(4); twist(5); twist(6)];
 translation = (eye(3)  * theta + (1 - cos(theta)) * bracket_omega + (theta - sin(theta)) * bracket_omega * bracket_omega) * v;
 concat = [rot translation];
 T = [concat; 0 0 0 1];
end

function T = fkine(twist, M, q)
    [~, n] = size(twist);
    T = eye(4);
    for i = 1 : n
        s = twist(:, i);
        theta = q(i);
        H  = twist2ht(s, theta);
        T = T * H;
    end
    T = T * M;
end

function Vb = adjoint(Va,T)
    omega_val = [Va(1); Va(2) ;Va(3)];
    linear = [Va(4); Va(5); Va(6)];
    bracket_omega = [0 -omega_val(3) omega_val(2) ; omega_val(3) 0 -omega_val(1) ; -omega_val(2) omega_val(1) 0];
    bracket_V = [bracket_omega linear ; 0 0 0 0];
    bracket_V_new = T * bracket_V / T;
    Vb = [-bracket_V_new(2,3); bracket_V_new(1, 3); -bracket_V_new(1,2); bracket_V_new(1,4);bracket_V_new(2, 4); bracket_V_new(3, 4)];
end

function J = jacob0(twist,q)
    [m, n] = size(q);
    m = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for i = 1 : n
        k = 1;
        if i > 1
            k = m(i-1);
        end
        m(i) = k * twist2ht(twist(:, i), q(i));
    end
    J = twist(:, 1);
    for i = 2 : n
        J = [J adjoint(twist(:, i), m(i-1))];
    end
end

function so3mat = MatrixLog3(R)    
    acosinput = (trace(R) - 1) / 2;
    if acosinput >= 1
        so3mat = zeros(3);
    elseif acosinput <= -1
        if ~NearZero(1 + R(3, 3))
            omg = (1 / sqrt(2 * (1 + R(3, 3)))) ...
                  * [R(1, 3); R(2, 3); 1 + R(3, 3)];
        elseif ~NearZero(1 + R(2, 2))
            omg = (1 / sqrt(2 * (1 + R(2, 2)))) ...
                  * [R(1, 2); 1 + R(2, 2); R(3, 2)];
        else
            omg = (1 / sqrt(2 * (1 + R(1, 1)))) ...
                  * [1 + R(1, 1); R(2, 1); R(3, 1)];
        end
        so3mat = skew(pi * omg);
    else
	    theta = acos(acosinput);
        so3mat = theta * (1 / (2 * sin(theta))) * (R - R');
    end
end

function expmat = MatrixLog6(T)    
    [R, p] = TransToRp(T);
    omgmat = MatrixLog3(R);
    if isequal(omgmat, zeros(3))
        expmat = [zeros(3), T(1: 3, 4); 0, 0, 0, 0];
    else
        theta = acos((trace(R) - 1) / 2);
        expmat = [omgmat, (eye(3) - omgmat / 2 ...
                          + (1 / theta - cot(theta / 2) / 2) ...
                            * omgmat * omgmat / theta) * p;
                  0, 0, 0, 0];    
    end
end

function judge = NearZero(near)
    judge = norm(near) < 1e-6;
end

function [R, p] = TransToRp(T)
    R = T(1: 3, 1: 3);
    p = T(1: 3, 4);
end