% Fill this script in with your position tracking, force computation,
% and graphics for the virtual environment

close all

%% Run on hardware or simulation
hardwareFlag = false;

%% Define global time increment
global interval;
interval = 0.0001;

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

%% Create a flat box with the base of the robot situated at 0,0,0

% define a length of the size of the cube 
global s; 
s = 300; %if doesnt work do 1000
% define the points on our cube 
global p1;
p1 = [0,-s/2, s];
global p2 
p2 = [0,s/2, s]; 
global p3;
p3 = [s, s/2, s];
global p4;
p4 = [s,-s/2, s];
global p5;
p5 = [0,-s/2, 0];
global p6;
p6 = [0,s/2, 0];
global p7;
p7 = [s,s/2, 0];
global p8;
p8 = [s,-s/2, 0];

%Back wall  
xBackWall = [p1(1) p2(1) p6(1) p5(1)];
yBackWall = [p1(2) p2(2) p6(2) p5(2)];
zBackWall = [p1(3) p2(3) p6(3) p5(3)];
fill3(xBackWall, yBackWall, zBackWall, 'g');
hold on; 

%Left wall 
xLeftWall = [p1(1) p5(1) p8(1) p4(1)];
yLeftWall = [p1(2) p5(2) p8(2) p4(2)];
zLeftWall = [p1(3) p5(3) p8(3) p4(3)];
fill3(xLeftWall, yLeftWall, zLeftWall, 'b');
hold on; 

%Right Wall 
xRightWall = [p2(1) p3(1) p7(1) p6(1)];
yRightWall = [p2(2) p3(2) p7(2) p6(2)];
zRightWall = [p2(3) p3(3) p7(3) p6(3)];
fill3(xRightWall, yRightWall, zRightWall, 'o');
hold on; 

%Floor  
xFloor = [p5(1) p6(1) p7(1) p8(1)];
yFloor = [p5(2) p6(2) p7(2) p8(2)];
zFloor = [p5(3) p6(3) p7(3) p8(3)];
fill3(xFloor, yFloor, zFloor, 'y');
hold on; 

xlabel('x'); ylabel('y'); zlabel('z');  

%% Create button on the back wall 
%Create the cylinder 
global radCylinder;
global heightButton; 
radCylinder = s/10; 
xBase = 0; 
yBase = s/4; 
zBase = 3*s/4; 
heightButton = s/8; 
[X,Y,Z] = cylinder(radCylinder);
surf(Z*heightButton+xBase,Y+yBase,X+zBase,'FaceColor','red'); 
hold on; 

%Create a circular button head 
theta=-pi:0.01:pi;
z=zBase + radCylinder*cos(theta);
x=xBase + heightButton + zeros(1,numel(z)); 
y=yBase + radCylinder*sin(theta);
patch(x,y,z,'red');
hold on; 

%% Create a ball 
global radBall; 
global posOfBall;

radBall = s/10; 
x0Ball = s/2; 
y0Ball = 2*s/5; 
z0Ball = 2*s/5; 
[xSphere,ySphere,zSphere] = sphere(radBall); 
surf(xSphere*radBall+x0Ball,ySphere*radBall+y0Ball,zSphere*radBall+z0Ball,'FaceColor','m');
hold on; 

posOfBall = [x0Ball,y0Ball,z0Ball];

%% Black hole 
global x0Black; 
global y0Black; 
global z0Black; 
global radSphereBlack; 

% make the point 
x0Black = 0; 
y0Black = -s/4; 
z0Black = 3*s/4; 
scatter3(x0Black,y0Black,z0Black, s/15,'black');hold on; 

% create the sphere for the black hole 
radSphereBlack = s/10; 
posOfBlackHole = [x0Black, y0Black, z0Black];
[xSphereBlk,ySphereBlk,zSphereBlk] = sphere(radSphereBlack); 
surf(xSphereBlk*radSphereBlack+x0Black,ySphereBlk*radSphereBlack+y0Black,zSphereBlk*radSphereBlack+z0Black,'FaceAlpha',0.5,'FaceColor','c');
hold off;


%% Example of a flat plane
%hFloor = fill3([200 200 200 200], [-300 -300 300 300], [-300 300 300 -300], [0.7 0 0], 'facealpha', 0.3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% set camera properties
axis([-s s -s*1 s*1 -s*1 s*1]);
view([75,30]);

i = 0; frameSkip = 3; % plotting variable - set how often plot updates

% define currPos and a prevSmoothVelocity so that we can find the velocity 
currPos = 0; 
prevSmoothVelocity = [0,0,0]; 

%% Star the control loop 
while(1)
    %% start a timer to measure the control loop 
    tic; 
    
    %% Read potentiometer values, convert to angles and end effector location
    if hardwareFlag
        qs = lynxGetAngles();
    end
    
    %% Calculate current end effector position
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    currPos = computeEEposition();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Calculate the velocity using exponential moving average 
    distance = (currPos - posEE); 
    weight = 0.90; 
    currentRawVelocity = distance/interval;
    velocity = weight * currentRawVelocity + (1-weight) * prevSmoothVelocity; 
    
    % set the global pos of the EE to the current position 
    posEE = currPos; 
    
    % set the previous smooth velocity of the current velocity 
    prevSmoothVelocity = velocity; 
    
    %% Calculate desired force based on current end effector position
    % Check for collisions with objects in the environment and compute the total force on the end effector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F = computeForces(posEE,velocity);
    disp(F); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Plot Environment
    if i == 0
        figClosed = drawLynx(h1, h2, F);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set handles for interactive objects you make here
        
        % Redraw the ball
%         [xSphere,ySphere,zSphere] = sphere(radBall); 
%         surf(xSphere*radBall+posOfBall(1),ySphere*radBall+posOfBall(2),zSphere*radBall+posOfBall(3),'FaceColor','m');
%         hold on
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
    
    interval = toc;
    disp(interval);
end