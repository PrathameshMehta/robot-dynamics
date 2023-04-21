% RBE 501 - Robot Dynamics - Spring 2023
% Homework 4, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 03/20/2023
clear, clc, close all
addpath('utils');
plotOn = true; 
nTests = 10;

%% Create the manipulator
mdl_stanford
stanf
qlim = stanf.qlim;
if plotOn
   stanf.teach(zeros(1,6)); 
end

%% YOUR CODE HERE

%Link lengths
L1 = 0.412;
L2 = 0.154;
L3 = 0;
L4 = 0;
L5 = 0;
L6 = 0;
Tt = 0.263;%z-component of the tool tip 

% Display the manipulator in the home configuration
q = zeros(1,6);
stanf.teach(q);

%% Part A - Forward Kinematics via PoE in the body frame
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S_space = [0 0 1 0 0 0;
           0 1 0 -cross([0 1 0], [0 0 L1]);
           0 0 0 0 0 1;
           0 0 1 -cross([0 0 1], [0 L2 L1]);
           1 0 0 -cross([1 0 0], [0 L2 L1]);
           0 0 1 -cross([0 0 1], [0 L2 L1])]';

%Screw axis wrt the body frame

Sb1 = [0 0 1 -cross([0 0 1], [L2 0 -(L1+Tt)])]';
Sb2 = [-1 0 0 -cross([-1 0 0], [L2 0 -Tt])]';
Sb3 = [0 0 0 0 0 1]';
Sb4 = [0 0 1 -cross([0 0 1], [0 0 -Tt])]';
Sb5 = [0 1 0 -cross([0 1 0], [0 0 -Tt])]';
Sb6 = [0 0 1 -cross([0 0 1], [0 0 -Tt])]';
S_body = [Sb1, Sb2, Sb3, Sb4, Sb5, Sb6];

% Let us calculate the homogeneous transformation matrix M for the
% home configuration
% R_home = [0 0 -1; 0 -1 0;-1 0 0]';
% t_home = [L2 0 L1-L3]';
% M = [R_home t_home; 0 0 0 1];
Mx = [0;-1;0];
My = [1;0;0];
Mz = [0;0;1];
Rm = [Mx,My,Mz];
T = [0;L2;L1+Tt];
M = [Rm ,T; 0 0 0 1];


fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
    
    T =fkine(S_body,M,q,'body');
    
    if plotOn
        stanf.teach(q);
        title('Forward Kinematics Test');
    end
    assert(all(all(abs(double(stanf.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the Jacobian in the body frame
    J_b = jacobe(S_space,M,q); 
    
    if plotOn
        stanf.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    assert(all(all(abs(double(stanf.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

% Part C - Calculate the Analyical Jacobian of the manipulator
fprintf('---------------------Analytical Jacobian Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Analytical Jacobian for 10 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Gen-serate a random configuration
     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    % Calculate the Analytical Jacobian
    J_a = jacoba(S_space,M,q);
    
    if plotOn
        stanf.teach(q);
        title('Analytical Jacobian Test');
    end
    
    % Test the correctness of the Jacobian
    Jref = stanf.jacob0(q);
    Jref = Jref(1:3,:);
    assert(all(all(abs(double(Jref) - J_a) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');
           

% Set the current joint variables
      
currentQ = zeros(1,6);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.25 * cos(t);
y = 0.25 * sin(t);
z = 0.2 * ones(1,nTests);
path = [x; y; z];

if plotOn
    stanf.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
plotOn = true; 
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
     T = fkine(S_body, M, currentQ, 'body');
     currentPose = T(1:3,4);
plotOn = true; 

    
    while norm(targetPose - currentPose) > 1e-3
        % YOUR INVERSE KINEMATICS CODE HERE
            J_a = jacoba(S_space,M,currentQ);
            er = targetPose - currentPose;
            deltaQ = pinv(J_a)*er;
            currentQ = currentQ + 0.5*deltaQ';
             if currentQ(3)<0
            currentQ(3)=0;
            end
            T = fkine(S_body,M,currentQ, 'body');
            currentPose = T(1:3,4);

            if plotOn
            try
                stanf.teach(currentQ);
                h = plot_ellipse(J_a*J_a',T(1:3,4), 'alter',h)
                drawnow;
            catch e
                continue;
            end
        end
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
    end
end