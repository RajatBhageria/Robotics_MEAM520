% Compute end effector position based on the current configuration

% Fill in the necessary inputs
function posEE = computeEEposition()

global qs; 

%% Find the positions of each of the joints using FK 
[X,~] = updateQ(qs); 

%% Return the position of the end effector 
posEE = X(4,:);

end