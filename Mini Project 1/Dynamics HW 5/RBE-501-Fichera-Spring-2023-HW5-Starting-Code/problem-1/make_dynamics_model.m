function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link


%% Create the manipulator
L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.3; % Lenght of Link 2 [m]
L3 = 0.15; % Lenght of Link 3 [m]
w  = 0.04; % Link Width [m]
l  = 0.04; % Link Depth [m]

% Link poses when the robot is in the home configuration
% M01 = ...
% M12 = ...
% M23 = ...
% M34 = ...

M1 = [eye(3) [0 0 L1/2]'; 0 0 0 1];

M2 = [ 1 0 0 0;
       0 0 1 L2/2;
       0 -1 0 L1;
       0 0 0 1];

M3 = [ 1 0 0 0;
       0 -1 0 L2;
       0 0 -1 L1-(L3/2);
       0 0 0 1];

M4 = [0 1 0 0;
      0 0 -1 L2;
      -1 0 0 L1-L3;
       0 0 0 1];

M01 = M1;
M12 = pinv(M1)*M2;
M23 = pinv(M2)*M3;
M34 = pinv(M3)*M4;


Mlist = cat(3, M01, M12, M23, M34);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
m1 = 5;   % Mass of Link 1 [kg]
m2 = 1;   % Mass of Link 2 [kg]
m3 = 1;   % Mass of Link 3 [kg]

% Spatial Inertia Matrices
% G1 = ...
% G2 = ...
% G3 = ...

Ib1 = diag([m1*(w^2+L1^2)/12, m1*(l^2 + L1^2)/12, m1*(l^2+w^2)/12]);
Ib2 = diag([m2*(w^2+L2^2)/12, m1*(l^2 + L2^2)/12, m1*(l^2+w^2)/12]);
Ib3 = diag([m3*(w^2+L3^2)/12, m1*(l^2 + L3^2)/12, m1*(l^2+w^2)/12]);
G1 = [Ib1 zeros(3); zeros(3) m1*eye(3)];% Spatial Inertia Matrix for link 1
G2 = [Ib2 zeros(3); zeros(3) m2*eye(3)];% Spatial Inertia Matrix for link 2
G3 = [Ib3 zeros(3); zeros(3) m3*eye(3)];% Spatial Inertia Matrix for link 3

% G1 = [zeros(3) zeros(3); zeros(3) m1*eye(3)];% Spatial Inertia Matrix for link 1
% G2 = [zeros(3) zeros(3); zeros(3) m2*eye(3)];% Spatial Inertia Matrix for link 2
% G3 = [zeros(3) zeros(3); zeros(3) m3*eye(3)];% Spatial Inertia Matrix for link 3

Glist = cat(3, G1, G2, G3);

end