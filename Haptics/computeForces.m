% Check for collisions with objects in the environment and compute the
% total force on the end effector

function F = computeForces(posEE,velocity)
% @param: velocity is a 1x3 vector of velocities in the x y and z
% @param: posEE is a 1x3 vector of positions of the end effector  
% @return: F the force that the EE feels 

%define all the globals 
global s; 
global radCylinder;
global heightButton; 
global x0Black; 
global y0Black; 
global z0Black; 
global radSphereBlack; 
global radBall; 
global posOfBall;
global interval;

%% instantiate the positions of the end effectors 
x = posEE(1); %mm
y = posEE(2); %mm
z = posEE(3); %mm

%% Spring flat wall 
% Floor 

% define when the flat wall happens
springWall = (z <= 0); 

% define variable used to define the distance of the wall from EE
x0Wall = [x y 0];

% find the Force of the flat wall 
kWall = .003; %kg/s^2
FflatWall = -kWall * (posEE - x0Wall);

%% Texture wall 
% right wall 

% define when the texture wall happens 
textureWall = (y >= s/2); 

% find the normal force
x0WallTexture = [x s/2 z]; 
FNormalTexture = -kWall * (posEE-x0WallTexture);

% Vary the constant with position using the sin wave 
cTexture = abs(.2*sin(norm(x+z))); %constant/position

% Find the force friction of the textured wall 
FFricTexture = - cTexture * cross(FNormalTexture,velocity);  

%find the total force by atdding with normal force 
Ftexture = FFricTexture + FNormalTexture; 

%% Viscous wall 
% left wall 
% define when the viscous wall happens 
viscousWall = (y <= -s/2); 

% find thre position of the viscous wall 
x0WallViscous = [x -s/2 z];

% find the normal force 
FNormalViscous = abs(-kWall * (posEE - x0WallViscous)); 

% find the F of viscous; 
cViscous = .0005; %constant

%find the force of the friction; 
FFriction =  - cViscous * cross(FNormalViscous,velocity);

%define total force as the force of the friction + the normal force 
%so that the user is not able to leave the haptic enviornment. 
Fviscous = FFriction + FNormalViscous; 

%% Button 
%to reach: 6 key "s" joint 2, 6 key "e" joint 3, use
%joint 1 to push button. 
% top right of back wall 
% define when the button happens

%face of the button 
withinY = (y < s/4 + radCylinder && y > s/4 - radCylinder); 
%face of the button 
withinZ = (z < 3*s/4 + radCylinder && z > 3*s/4 - radCylinder); 
%height of the button 
withinX = (x > 0 && x < heightButton); 
%we're at the button when the x,y, and z coordinates of the button match. 
button = withinY && withinZ && withinX; 

%find the force of the button 
kPressing = kWall*1.5;  %kg/s^2
kPressed = kWall*2;  %kg/s^2
Fbutton =[0,0,0];

%find the distance you're pressing 
distPressing = posEE-[heightButton, y, z]; 
%pressing button
if (x > heightButton/2 && x <= heightButton)
    Fbutton = - kPressing * (distPressing); 
%force is zero 
elseif (x > heightButton/4 && x <= heightButton/2) 
    Fbutton = [0,0,0];
%higher force
elseif (x <  heightButton/4)
    Fbutton = - kPressed * (distPressing); 
end
        
%% Black hole 
% top left of back wall
blackhole = ((x-x0Black)^2+(y-y0Black)^2 + (z-z0Black)^2) <= (radSphereBlack)^2; 

%define the k 
kBlackhole = .0005; %mm/s^2

%find the distance from the EE to the surfce of the button. 
distFromCenter = norm([x0Black y0Black z0Black] - posEE);
distFromSurface = radSphereBlack - distFromCenter;
directionToCenter = ([x0Black y0Black z0Black] - posEE)/distFromCenter;

% Force of blackhole
Fblackhole = kBlackhole * distFromSurface * directionToCenter;

%% Ball 
%choose spring constant for the ball
kBall = .003;  %kg/s^2

%figure out whether the EE is touching the ball or within the ball. 
ball = ((x-posOfBall(1))^2+(y-posOfBall(2))^2 + (z-posOfBall(3))^2) <= (radBall)^2;

%find the distance into the ball that the EE is located. 
%Or the distance between the surface and the EE 
distFromBallCenter = norm(posOfBall - posEE);
distFromBallSurface = abs(radBall - distFromBallCenter);
dirToBallCenter = (posOfBall-posEE)/distFromBallCenter;

%% if the EE is touching the wall, then 
if (ball)
    
    % Force of the ball on the EE 
    Fball = -kBall * distFromBallSurface * dirToBallCenter;

    %% simulate ball movement 
    FEEOnBall = - Fball; 

    %define virtual mass 
    massBall = .000001; %kg

    %find the acceleration that the ball will move when the EE collides with
    %it.
    accelBall = FEEOnBall/massBall; %m/s^2

    % xf = x0 + v0t + 0.5at^2    
    posOfBall = posOfBall + (0.5 * accelBall*interval^2);
    
    % Interaction of ball with spring flat wall the ball is touching touching the floor 
    FfloorOnBall = [0,0,0];
    if(posOfBall(3) - radBall <= 0)
        FfloorOnBall = -kWall * (posOfBall- [0,0,radBall] - x0Wall);
        %find the new force on the ball after it hits the floor 
        finalForceBall = FEEOnBall+FfloorOnBall;
        accelBall = finalForceBall/massBall; %m/s^2

        % xf = x0 + v0t + 0.5at^2    
        posOfBall = posOfBall  + (0.5 * accelBall*interval^2);
    end 
end 

%% Free space 
FfreeSpace = [0,0,0];

%% Do the switch cases 
F = [];

if (springWall)
    disp('springWall');
    F = FflatWall;
elseif (textureWall)
    disp('texturewall');
    F = Ftexture; 
elseif (viscousWall)
    disp('viscouswall');
    F = Fviscous;
elseif (button)
    disp('button');
    F = Fbutton; 
elseif (blackhole)
    disp('blackhole');
    F = Fblackhole;  
elseif (ball)
    disp('ball');
    F = Fball;
else%freespace 
    disp('freespace');
    F = FfreeSpace;
end 

%F needs to be a 3x1 vector not a 1x3 vector
F = F';

end