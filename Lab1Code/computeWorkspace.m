% Reachable Workspace

% Place your code for computing the workspace of the Lynx robot here.

% Below is series of 4 nested for loops, one for each joint in the shoulder; 
% Each joint loops from its lower limit its upper limit 
% In the inner-most loop, we set these parameters into q and sent it to updateQ 
% which returns the X matrix
% We then take the x, y, z variables of each iteration and plot it in 3D

% Joint 1
for i = -1.4:1:1.4
    % Joint 2
    for j = -1.2:1:1.4
        %Joint 3
         for k = -1.8:1:1.7
             %Joint 4
             for l = -1.9:1:1.7
                         % input the joint thetas into q
                         q = [i, j, k, l, 0,0];
                         % update the position of all shoulder joints
                         [X,~] = updateQ(q); 
                         % append new point to matrix of points  
                         xyzPoints = [xyzPoints; X(6,:)];
            end
         end
    end
end 

% Plot all of the collected points
figure; 
scatter3(xyzPoints(:,1), xyzPoints(:,2),xyzPoints(:,3));
view(40,35);


ptCloud = pointCloud(xyzPoints);
pcshow(ptClound); 