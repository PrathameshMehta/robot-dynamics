%% Robot Definition:
n = 2;    % Number of links in the kinematic chain
L1 = 0.3; % [m] Length of the first link
L2 = 0.4; % [m] Length of the second link
m1 = 1;   % [kg] Mass of the first link
m2 = 1;   % [kg] Mass of the second link
g = -9.8;  % [m/s2] Gravity acceleration (aligned with the Y axis)

% Calculate the home configurations of each link, expressed w.r.t. the previous link frame
M01 = eye(4); M01(1:3,4) = [L1 0 0]' % pose of frame {1} expressed in the {0} (space) reference frame
M12 = eye(4); M12(1:3,4) = [L2 0 0]' % pose of frame {2} expressed in the {1} reference frame
M23 = eye(4) % pose of frame {3} expressed in the {2} reference frame

%% *** STEP 1 ***
% Calculate the home configurations of each link, expressed in the space frame                
M1 = M01 % pose of frame {1} expressed in the {0} (space) reference frame
M2 = M1*M12 % pose of frame {2} expressed in the {0} (space) reference frame
M3 = M2*M23 % pose of frame {3} expressed in the {0} (space) reference frame

% Calculate the home configurations of each link, expressed w.r.t. the previous link frame
% M01 = eye(4); M01(1:3,4) = [L1 0 0]'; % pose of frame {1} expressed in the {0} (space) reference frame
% M12 = eye(4); M01(1:3,4) = [L2 0 0]'; % pose of frame {2} expressed in the {1} reference frame
% M23 = eye(4); % pose of frame {3} expressed in the {2} reference frame

% Define the screw axes of each joint, expressed in the space frame
S = zeros(6,n);
S(:,1) = [0 0 1 0 0 0]';
S(:,2) = [0 0 1 -cross([0 0 1], [L1 0 0])]' ;
% Calculate the screw axes of each joint, expressed in the local link frame
A = zeros(6,n);
A(:,1) = adjoint(inv(M1))*S(:,1);
A(:,2) = adjoint(inv(M2))*S(:,2);

% Initialize the twists and accelerations of each link
V1 = zeros(6,1);
V2 = zeros(6,1);
Vd1 = zeros(6,1);
Vd2 = zeros(6,1);
Vd0 = [0 0 0 0 -g 0]';

% Initialize the joint positions and velocities
q = zeros(2,1);
qd = ones(2,1);
qdd = ones(2,1);
T1 = twist2ht(-A(:,1),q(1)) * (inv(M1));
T2 = twist2ht(-A(:,2),q(2)) * (inv(M2)*M1);
%% *** STEP 2 ***
% Forward Iteration - First Link
V1 = A(:,1)*qd(1) ;% Link Velocity
Vd1 =A(:,1)*qdd(1) +(ad(V1)*A(:,1))+adjoint(T1)*Vd0; % Link Acceleration
     
% Forward Iteration - Second Link
T21 = eye(4); T21(1:3,4) = [-L2 0 0]';
V2 =A(:,2) + (adjoint(T21)*V1); % Link Velocity
Vd2 = A(:,2)+ (adjoint(T21)*Vd1) + (ad(V2)*A(:,2)) ; % Link Acceleration