% Compute end effector position based on the current configuration

% Fill in the necessary inputs
function posEE = computeEEposition()

global qs; 

[X,~] = updateQ(qs); 

posEE = X(3,:);

end