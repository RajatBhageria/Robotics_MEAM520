% Convert end effector forces into joint torques
% For 10V, and i = [1,2,1].*tau, we have tauLim = [0.7,0.5,0.9];

% Fill in the necessary inputs
function Tau = computeTorques(f)

global qs; 

%% Find the lengths of the arms 
% MAKE SURE TO CHECK THESE VALUES!!! 
a1 = 3*25.4; 
a2 = 5.75*25.4; 
a3 = 7.375*25.4; 

%% Compute the Jacobian 
Jv = computeJacobian(qs(1),qs(2),qs(3),a1,a2,a3); 

%% Find the torque of the end effector  
Tau = Jv' * f; 

end