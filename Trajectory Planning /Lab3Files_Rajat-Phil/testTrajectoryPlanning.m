function [path] = testTrajectoryPlanning(posOfObstacles,ri,p_start,p_end,G)
    %find the free c-s
    [c_space,workspace] = freeCSpaceCalc(posOfObstacles, ri);
    
    %create the graph (maintain indexes of c-space)
    G = createGraph(workspace); 
    
    %find the shortest path between pstart and pend 
    path = findShortestPath(G,c_space,workspace,p_start,p_end); 

    lynxStart();
    
    %simulate the movement of the robot by looping through path sequence
    for time = 1:size(path,1)
        config = path(time,:); 
        lynxServoSim(config(1,1),config(1,2),config(1,3),config(1,4),config(1,5),config(1,6));
        pause(1);
    end

end
