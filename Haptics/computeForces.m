% Check for collisions with objects in the environment and compute the
% total force on the end effector

function F = computeForces(posEE,velocity,posOfBall)
% @param: velocity is a 1x3 vector of velocities in the x y and z
% directions 

global s; 
global radCylinder;
global heightButton; 
global x0Black; 
global y0Black; 
global z0Black; 
global radSphereBlack; 
global radBall; 

x = posEE(1); 
y = posEE(2); 
z = posEE(3); 

%% Spring flat wall 
% Floor 
% define when the flat wall happens
x0Wall = [x y 0];
% find the Force of the flat wall 
kWall = 32;
springWall = (z <= 0); 
FflatWall = -kWall * (posEE - x0Wall);

%% Texture wall 
% right wall 
% define when the texture wall happens 
textureWall = (y >= s/2); 
% % find the normal force
% x0WallTexture = [x s/2 z]; 
% FNormalTexture = abs(-kWall * (posEE - x0WallTexture));
% % Vary the constant with position 
% % ////// Currently trying to figure how to change texture with position
% cTexture = sin(norm(x+z)); 
% % Find the force of the textured wall 
% Ftexture = - cTexture * cross(FNormalTexture,velocity); 

%% Viscous wall 
% left wall 
% define when the viscous wall happens 
viscousWall = (y <= -s/2); 
% % find the normal force 
% x0WallViscous = [x -s/2 z];
% FNormalViscous = abs(-kWall * (posEE - x0WallViscous)); 
% % find the F of viscous; 
% cViscous = 32; 
% Fviscous = - cViscous * cross(FNormalViscous,velocity); 

%% Button 
% top right of back wall 
% define when the button happens 

%face of the button (square######delete!!!!)
withinY = (y < s/4 + radCylinder && y > s/4 - radCylinder); 
%face of the button (square######delete!!!!)
withinZ = (z < 3*s/4 + radCylinder && z > 3*s/4 - radCylinder); 
%height of the button 
withinX = (x > 0 && x < heightButton); 
button = withinY && withinZ && withinX; 

%find the force of the button 
%Fbutton =;

%% Black hole 
% top left of back wall
blackHole = ((x-x0Black)^2+(y-y0Black)^2 + (z-z0Black)^2) <= (radSphereBlack)^2; 

% define when the black hole happens
%FblackHole = 0; 

%% Ball 
%choose spring constant for the ball
kBall = 32;

%find the distance into the ball that the EE is located. 
%Or the distance between the surface and the EE 
distanceFromCenter = ((x-posOfBall(1))^2+(y-posOfBall(2))^2 + (z-posOfBall(3))^2)^.5; 
distInSurface = (radBall - distanceFromCenter); 

%find the force
Fball = [0,0,0];
% if the EE within the surface of the ball
if (distInSurface > 0)
    Fball = -kBall*distInSurface;
end 

%simulate ball movement 
FEEOnBall = - Fball; 

%define virtual mass 
massBall = 100; 

%find the acceleration that the ball will move when the EE collides with
%it.
acceleration = FEEOnBall/massBall;



%% Interaction of ball with spring flat wall 

%% Free space 
FfreeSpace = [0,0,0];

%% Do the switch cases 
F = [];

if (springWall)
    F = FflatWall;
elseif (textureWall)
    F = Ftexture; 
elseif (viscousWall)
    F = Fviscous;
elseif (button)
    F = Fbutton; 
elseif (blackHole)
    F = Fblackhole;  
else%freespace 
    F = FfreeSpace;
end 

%F needs to be a 3x1 vector not a 1x3 vector
F = F';

end