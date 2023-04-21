function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
% YOUR CODE HERE 
function AdT = adjoint(T)
    % your code here
    R = T(1:3,1:3);
    omega = T(1:3,4)';
    P = [ 0 -omega(1,3) omega(1,2) ; omega(1,3) 0 -omega(1,1) ; -omega(1,2) omega(1,1) 0];
    AdT = [R, zeros(3,3); P*R, R];
end

function T = twist2ht(S,theta)
    omega = S(1:3);
    SS = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
    I = eye(3);
    R = I + sin(theta)* SS + (1-cos(theta))*SS^2;
    K = S(4:6);
    Z = ((I*theta) + ((1-cos(theta))*SS)+ (((theta - sin(theta))*SS^2)))*K;
    T = [ R Z ; 0 0 0 1];
end

g = params.g;
S = params.S;       
M = params.M;
G = params.G;
jointPos = params.jointPos;
jointVel = params.jointVel;
jointAcc = params.jointAcc; 
Ftip = params.Ftip;

n = size(S,2);
V = zeros(6, n+1);
Vdot = zeros(6, n+1);
Vdot(:,1) = [0 0 0 -g(1) -g(2) -g(3)];

% M1 = M(:,:,1);
% M2 = M1 * M(:,:,2);
% M3 = M2 * M(:,:,3);
% M4 = M3 * M(:,:,4);
% 
% A1 = adjoint(pinv(M1))*S(:,1);
% T1 = twist2ht(A1,-jointPos(1))*pinv(M(:,:,1));
% V(:,2) = A1.*jointVel(1) + adjoint(T1)*V(:,1);
% Vdot(:,2) = A1.*jointAcc(1) + adjoint(T1)*Vdot(:,1) + ad(V(:,1))*A1*jointVel(1);
% 
% A2 = adjoint(pinv(M2))*S(:,2);
% T2 = twist2ht(A2,-jointPos(2))*pinv(M(:,:,2));
% V(:,3) = A2.*jointVel(2) + adjoint(T2)*V(:,2);
% Vdot(:,3) = A2.*jointAcc(2) + adjoint(T2)*Vdot(:,2) + ad(V(:,2))*A2*jointVel(2);
% 
% A3 = adjoint(pinv(M3))*S(:,3);
% T3 = twist2ht(A3,-jointPos(3))*pinv(M(:,:,3));
% V(:,4) = A3.*jointVel(3) + adjoint(T3)*V(:,3);
% Vdot(:,4) = A3.*jointAcc(3) + adjoint(T3)*Vdot(:,3) + ad(V(:,4))*A3*jointVel(3);

 Mii = zeros(4,4,n);
 Aii = zeros(6,n);
 for ii = 1:n
     Mi = eye(4);
     for j = 1:ii
         Mi = Mi*M(:,:,j);
     end
     Mii(:,:,ii) = Mi;
     Ai = adjoint(pinv(Mi))*S(:,ii);
     Aii(:,ii) = Ai;
     Ti = twist2ht(Ai,-jointPos(ii))*pinv(M(:,:,ii));
     V(:,ii+1) = Ai.*jointVel(ii) + adjoint(Ti)*V(:,ii);
     Vdot(:,ii+1) = Ai*jointAcc(ii) + adjoint(Ti)*Vdot(:,ii) + ad(V(:,ii+1))*Ai*jointVel(ii);  
 end

% Backward iterations
% YOUR CODE HERE
% tau = zeros(3,1);
% M1 = M(:,:,1);
% M2 = M1 * M(:,:,2);
% M3 = M2 * M(:,:,3);
% M4 = M3 * M(:,:,4);
% 
% A3 = adjoint(pinv(M3))*S(:,3);
% T4 = pinv(M(:,:,4));
% F3 = G(:,:,3)*Vdot(:,4) - ad(V(:,4))'*G(:,:,3)*V(:,4) + adjoint(T4)'*Ftip;
% tau(3,1) = F3'*A3;
% 
% A2 = adjoint(pinv(M2))*S(:,2);
% T3 = twist2ht(A3,-jointPos(3))*pinv(M(:,:,3));
% F2 = G(:,:,2)*Vdot(:,3) - ad(V(:,3))'*G(:,:,2)*V(:,3) + adjoint(T3)'*F3;
% tau(2,1) = F2'*A2;
% 
% A1 = adjoint(pinv(M1))*S(:,1);
% T2 = twist2ht(A2,-jointPos(2))*pinv(M(:,:,2));
% F1 = G(:,:,1)*Vdot(:,2) - ad(V(:,2))'*G(:,:,1)*V(:,2) + adjoint(T2)'*F2;
% tau(1,1) = F1'*A1;
% 
tau = zeros(n,1);

for jj = n:-1:1

    if jj == n
        Ti = pinv(M(:,:,n+1));
        Fi = G(:,:,jj)*Vdot(:,jj+1) - ad(V(:,jj+1))'*G(:,:,jj)*V(:,jj+1) + adjoint(Ti)'*Ftip;
        F_previous = Fi;
    else
        Ti = twist2ht(Aii(:,jj+1),-jointPos(jj+1))*pinv(M(:,:,jj+1));
        Fi = G(:,:,jj)*Vdot(:,jj+1) - ad(V(:,jj+1))'*G(:,:,jj)*V(:,jj+1) + adjoint(Ti)'*F_previous;
        F_previous = Fi;
    end
    tau(jj) = Fi'*Aii(:,jj);
end
    
    
       
        
    


% M = cat(3, M1, M2, M3, M4);


end