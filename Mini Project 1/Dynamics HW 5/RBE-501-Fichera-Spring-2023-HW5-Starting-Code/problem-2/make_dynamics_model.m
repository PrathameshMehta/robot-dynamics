function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Link poses when the robot is in the home configuration
[M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

%% Spatial Inertia Matrices

m1 = 3.7;  % [kg] Mass of Link 1
m2 = 8.4;  % [kg] Mass of Link 2
m3 = 2.33; % [kg] Mass of Link 3
m4 = 1.22; % [kg] Mass of Link 4
m5 = 1.22; % [kg] Mass of Link 5
m6 = 0.19; % [kg] Mass of Link 6

% *** Define the link inertial properties ***
Ib1 = diag([0.010267495893, 0.010267495893, 0.00666]);  % Rotational Inertia Matrix of Link 1
Ib2 = diag([0.22689067591, 0.22689067591, 0.0151074]);   % Rotational Inertia Matrix of Link 2
Ib3 = diag([0.049443313556, 0.049443313556, 0.004095]);  % Rotational Inertia Matrix of Link 3
Ib4 = diag([0.111172755531, 0.111172755531, 0.21942]);   % Rotational Inertia Matrix of Link 4
Ib5 = diag([0.111172755531, 0.111172755531, 0.21942]);   % Rotational Inertia Matrix of Link 5
Ib6 = diag([0.0171364731454, 0.0171364731454, 0.033822]);% Rotational Inertia Matrix of Link 6

% Spatial Inertia Matrices
G1 = [Ib1 zeros(3);
      zeros(3) m1*eye(3)]; 
G2 = [Ib2 zeros(3);
      zeros(3) m2*eye(3)];
G3 = [Ib3 zeros(3);
      zeros(3) m3*eye(3)]; 
G4 = [Ib4 zeros(3);
      zeros(3) m4*eye(3)]; 
G5 = [Ib5 zeros(3);
      zeros(3) m5*eye(3)]; 
G6 = [Ib6 zeros(3);
      zeros(3) m6*eye(3)]; 

Glist = cat(3, G1, G2, G3, G4, G5, G6);

end