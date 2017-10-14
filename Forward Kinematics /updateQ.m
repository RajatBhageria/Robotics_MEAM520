% Fill in this function with the mathematics for computing the positions of
% each of the joints of the Lynx robot and of the gripper.
%
% input: q = 1x6 vector [q1, q2, q3, q4, q5, q6] of joint angles (q1-q5)
%            and grip distance (q6)
% output: X = 6x3 matrix of values [x1 y1 z1; x2 y2 z2; ...] containing the
%             positions of each of the joints and the center of the gripper
%             got the given input q
%         T = 4x4x6 matrix of transformation matrices, where each slice
%             T(:,:,i) is the coordinate transformation between links i-1
%             and i
%

function [X, T] = updateQ(q)
%% Enter in the DH parameters 

% ai, alphai, di, thetai on the rows
% columns are Link1-Link6 
% Link 1 is base 
% Link 6 is gripper and di is the gripper distance 
% Note that these have - thetas at time and offsets of pi/2 and -pi/2 
% since while we were testing Legend, we found that the zero position 
% seemed to be off. 
DH = [0, pi/2, 76.2, -q(1); 
      146.05,0,0, -q(2)+pi/2;
      187.325, 0,0, -q(3)-pi/2;
      0, pi/2, 0, -q(4)+pi/2; 
      0, 0,50.8, q(5);
      0,0,q(6),0];
  
%% Calculate all the individual joint transformations 
%Aij -> Transformation to j in relation to frame i. 
% the transformation function is saved as transformation.m inside the
% included files

A01= transformation(DH(1,:));
A12= transformation(DH(2,:));
A23= transformation(DH(3,:));
A34= transformation(DH(4,:));
A45= transformation(DH(5,:));
A56= transformation(DH(6,:));

%%Calculate Homogeneous matrix T0e

T = cat(3,A01, A12, A23, A34, A45, A56);

%%Calculate X 

H01 = A01;
H02 = A01 * A12; 
H03 = A01 * A12 * A23;
H04 = A01 * A12 * A23 * A34;
H05 = A01 * A12 * A23 * A34 * A45;
display(A01 * A12 * A23 * A34 * A45 * A56);

X = [0,0,0;
    H01(1,4), H01(2,4), H01(3,4);
    H02(1,4), H02(2,4), H02(3,4);
    H03(1,4), H03(2,4), H03(3,4);
    H04(1,4), H04(2,4), H04(3,4);
    H05(1,4), H05(2,4), H05(3,4)];


end