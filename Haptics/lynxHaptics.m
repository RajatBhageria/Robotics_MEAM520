% Fill this script in with your position tracking, force computation,
% and graphics for the virtual environment

close all

%% Run on hardware or simulation
hardwareFlag = false;

%% Plot end effector in environment
global qs % configuration (NOTE: This is only 3 angles now)
global posEE % position of end effectr

figClosed = 0;
qs = [0,0,0]; % initialize robot to zero pose
posEE = [0,0,0];  % initialize position of end effector

hold on; scatter3(0, 0, 0, 'kx', 'Linewidth', 1); % plot origin
h1 = scatter3(0, 0, 0, 500,'r.'); % plot end effector position
h2 = quiver3(0, 0, 0, 0, 0, 0, 'b'); % plot output force
if ~hardwareFlag
    h_fig = figure(1);
    set(h_fig, 'Name','Haptic environment: Close figure to quit.' ,'KeyPressFcn', @(h_obj, evt) keyPressFcn(h_obj, evt));
end

%% Create Environment here:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create static objects and interactive objects in their initial state

% Create a flat box with the base of the robot situated at 0,0,0. 

% define a length of the size of the cube 
s = 1000; 

% define the points on our cube 
p1 = [0,-s/2, s];
p2 = [0,s/2, s]; 
p3 = [s, s/2, s];
p4 = [s,-s/2, s];
p5 = [0,-s/2, 0];
p6 = [0,s/2, 0];
p7 = [s,s/2, 0];
p8 = [s,-s/2, 0];

%Back wall  
xBackWall = [p1(1) p2(1) p6(1) p5(1)];
yBackWall = [p1(2) p2(2) p6(2) p5(2)];
zBackWall = [p1(3) p2(3) p6(3) p5(3)];
fill3(xBackWall, yBackWall, zBackWall, 1);

%Left wall 
xLeftWall = [p1(1) p5(1) p8(1) p4(1)];
yLeftWall = [p1(2) p5(2) p8(2) p4(2)];
zLeftWall = [p1(3) p5(3) p8(3) p4(3)];
fill3(xLeftWall, yLeftWall, zLeftWall, 2);

%Right Wall 
xRightWall = [p2(1) p3(1) p7(1) p6(1)];
yRightWall = [p2(2) p3(2) p7(2) p6(2)];
zRightWall = [p2(3) p3(3) p7(3) p6(3)];
fill3(xRightWall, yRightWall, zRightWall, 3);

%Floor  
xFloor = [p5(1) p6(1) p7(1) p8(1)];
yFloor = [p5(2) p6(2) p7(2) p8(2)];
zFloor = [p5(3) p6(3) p7(3) p8(3)];
fill3(xFloor, yFloor, zFloor, 4);

%

% Example of a flat plane
%hFloor = fill3([200 200 200 200], [-300 -300 300 300], [-300 300 300 -300], [0.7 0 0], 'facealpha', 0.3);

hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set camera properties
axis([-1000 1000 -1000 1000 -1000 1000]);
view([75,30]);

i = 0; frameSkip = 3; % plotting variable - set how often plot updates
while(1)
    %% Read potentiometer values, convert to angles and end effector location
    if hardwareFlag
        qs = lynxGetAngles();
    end
    
    %% Calculate current end effector position
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    posEE = computeEEposition();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Calculate desired force based on current end effector position
    % Check for collisions with objects in the environment and compute the total force on the end effector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F = computeForces();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Plot Environment
    if i == 0
        figClosed = drawLynx(h1, h2, F);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set handles for interactive objects you make here
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
        drawnow
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute torques from forces and convert to currents for servos
    Tau = computeTorques(F);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if hardwareFlag
        if figClosed % quit by closing the figure
            lynxDCTorquePhysical(0,0,0,0,0,0);
            return;
        else
            currents = torquesToCurrents(Tau);
            lynxDCTorquePhysical(currents(1),currents(2),currents(3),0,0,0);
        end
    end
    
    if (figClosed) % quit by closing the figure
        return;
    end
    
    %% Debugging
    %[posEE, qs, F', Tau']
    i = mod(i+1, frameSkip);
end