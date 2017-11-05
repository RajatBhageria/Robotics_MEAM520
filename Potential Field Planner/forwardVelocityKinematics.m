function [velocity] = forwardVelocityKinematics(q, qdot)
%% Finds the forward velocity kinematics  
% @param: q: a 6x1 vector of joint configurations 
% @param: qdot: a 6x1 vector of joint velocity configurations 
% @param: velocity: a 6x1 vector of the velocity of the end effector 

syms q1 q2 q3 q4 q5 q6

%% Find the Jacobian given q
J = findJacobian(q,6);

%% Substitute the configurations in q with the symbolic representation in J
J = subs(J, q1, q(1));
J = subs(J, q2, q(2)); 
J = subs(J, q3, q(3)); 
J = subs(J, q4, q(4)); 
J = subs(J, q5, q(5)); 
J = subs(J, q6, q(6)); 

%% Return the velocity of the end effector 
velocity = J * qdot;

end 