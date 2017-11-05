function [qdot] = inverseVelocityKinematics(velocity,q)
%% Finds the qdot vector given the velocity of the end effector and a particular joint configuration 
% @param a 6x1 vector of the velocties of the end effector where the top 
% 3x1 are the linear velocities of the end effector and the bottom 3x1 
% are the angular velocities of the end effector. 
% @param a 6x1 vector of positions of configurations 
% @return a 6x1 vector of the velocities of each of the joints 
syms q1 q2 q3 q4 q5 q6
%% Find the Jacobian given q
J = findJacobian(q,6);

%% Jacobian velocities [3x6]
J = J(1:3,:);

%% Substitute the configurations in q with the symbolic representation in J
J = subs(J, q1, q(1));
J = subs(J, q2, q(2)); 
J = subs(J, q3, q(3)); 
J = subs(J, q4, q(4)); 
J = subs(J, q5, q(5)); 
J = subs(J, q6, q(6)); 

disp (rank(J)); 

%% Find the Pseudoinverse (6x3) (3x6) (6x3)
JPseudoInverse = (J'*J)\J';

%% Find qdot 
qdot = double(JPseudoInverse * velocity);

end 