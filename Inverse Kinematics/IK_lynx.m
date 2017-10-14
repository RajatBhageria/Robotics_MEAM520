% Fill in this function with the mathematics for computing the positions of
% each of the joints of the Lynx robot and of the gripper.
%
% input: T = 4x4 transformation matrix representing the pose of the end
%             effector frame
% output: q = 1x5 vector of joint variable values that bring the end
%             effector frame to pose T
%         is_possible = true/false whether it is possible for the end
%             effector to move to the pose described by T
%

function [q, is_possible] = IK_lynx(T)

%% NOTE: BASED ON THE WRITEUP, THE PARAMETER T HERE IS T_0^6. 
%THUS WE TRANSPOSED IT to make it T_6^0.
T06 = T;
%T = T';
disp(T);

%% Initialize distances in mm
d1 = 3 * 25.4;
a2 = 5.75 * 25.4;
a3 = 7.375 * 25.4;
d5 = 3 * 25.4;
l  = 1.125 * 25.4;

%% Find positions of center of wrist based on end effector 

% find the x position of wrist center
r13 = T(1,3); 
tx = T(1,4); 
xc = tx - (d5 + l) * r13;

% find the y position of wrist center
r23 = T(2,3); 
ty = T(2,4); 
yc = ty - (d5 + l) * r23;

% find the z position of wrist center
r33 = T(3,3); 
tz = T(3,4); 
zc = tz - (d5 + l) * r33;

%% Find theta1-3

% find theta1
theta1 = atan2(yc,xc);

% find c, the distance from ref1 to center of wrist
c = sqrt(xc^2+yc^2+(zc-d1)^2); 

% find bigAngle, the angle between line c and xy plane in frame 0
bigAngle = acos((zc-d1)/c);

% find rho, the angle formed by a2 and line c
rho = acos((a3^2-a2^2-c^2)/(-2*a2*c));

% find theta2
theta2 = bigAngle - rho;

% find theta3
theta3 = pi/2 - acos((xc^2+yc^2+(zc-d1)^2-a2^2-a3^2)/(-2*a2*a3)); 

PI = pi(); 
%% Inserting previously found thetas 1-3 into the transformation matrices

%Frame 1 w.r.t Frame 0
A1 = [cos(theta1) -sin(theta1)*cos(-PI/2)  sin(theta1)*sin(-PI/2)  0;
      sin(theta1)  cos(theta1)*cos(-PI/2) -cos(theta1)*sin(-PI/2)  0;
              0            sin(-PI/2)            cos(-PI/2)        d1;
              0                     0                  0           1];
          
%Frame 2 w.r.t Frame 1          
A2 = [cos(theta2 -(PI/2)) -sin(theta2 -(PI/2))  0   a2*cos(theta2 -(PI/2));
      sin(theta2 -(PI/2))  cos(theta2 -(PI/2))  0   a2*sin(theta2 -(PI/2));
              0                        0      1                     0;
              0                        0      0                     1];

%Frame 3 w.r.t Frame 2
A3 = [cos(theta3 + (PI/2)) -sin(theta3 + (PI/2))  0   a3*cos(theta3 + (PI/2));
      sin(theta3 + (PI/2))  cos(theta3 + (PI/2))  0   a3*sin(theta3 + (PI/2));
              0                        0      1                     0;
              0                        0      0                     1];

% find T03 populated with actual values
T03 = A1*A2*A3;

% % isolate the rotation matrix of R03 from T03 and transpose to get R30
% R03 = T03(1:3,1:3);
% R30 = transpose(R03);
% 
% % isolate the rotation matrix R06 from T06
% R06 = T06(1:3,1:3);
% 
% % multiply R30 and R06 to obtain R36
% R36 = R30 * R06;

T36 = T03' * T;

%% Reference Chart of A34, A45, A56 matrices multiplied as equations
% A36 = [cos(q(5))*cos(q(4)-pi/2) -sin(q(5))*cos(q(4)-pi/2)   -sin(q(4)-pi/2)   L6*(-sin(q(4)-pi/2))+(L4+L5)*(-sin(q(4)-pi/2));
%        cos(q(5))*sin(q(4)-pi/2) -sin(q(5))*sin(q(4)-pi/2)   cos(q(4)-pi/2)    L6*(cos(q(4)-pi/2))+(L4+L5)*(cos(q(4)-pi/2));
%        -sin(q(5))               -cos(q(5))                  0                 0;
%        0                        0                           0                 1]

%% Find theta4 and theta 5

theta5 = -asin(T36(3,1));

if isreal(-acos(T36(2,3)) + pi/2)
    theta4 = -acos(T36(2,3)) + pi/2;
elseif isreal(-asin(T36(1,3)) + pi/2)
    theta4 = -asin(T36(1,3)) + pi/2;
else  
    theta4 = asin(T36(2,2)/(-sin(theta5))) + pi/2;
end

%% Set Theta6 to 0
theta6 = 0; 

%% Return the results of the q vector
q = [theta1, theta2, theta3, theta4, theta5, theta6];

%% Test whether is possible
isItReal = isreal(theta1) && isreal(theta2)  && isreal(theta3)  && isreal(theta4)  && isreal(theta5);
is_possible = isItReal && (q(1) >= -1.4 && q(1) <= 1.4) && (q(2) >= -1.2 && q(2) <= 1.2) && (q(3) >= -1.8 && q(3) <= 1.7) && (q(4) >= -1.9 && q(4) <= 1.7) && (q(5) >= -2 && q(5) <= 1.5); 

end