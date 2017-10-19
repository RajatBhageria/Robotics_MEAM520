function [path] = findShortestPath(G,c_space,workspace,p_start,p_end)
    %%Step 1: Find index of qa closest to qstart 
    closestPointAIndex = findIndexOfClosestPoints(workspace, p_start);

    %%Step 2: Find index of qb closest to qend 
    closestPointBIndex = findIndexOfClosestPoints(workspace, p_end); 

    %%Step 3: Find path between qa and qb in the workspace
    [workspacePathAtoB] = shortestpath(G,closestPointAIndex,closestPointBIndex);

    %%Step 4: Find the path between qa and qb in the configuration space 
    configPathAtoB = c_space(workspacePathAtoB,:);

    path = configPathAtoB;
end


%%find the index of the closest point to pointEntry 
function [index] = findIndexOfClosestPoints(workspace, point)
    index = 0; 
    minDist = 1E50; 
    for comparisonIndex  = 1:size(workspace,1)
        comparisonPoint = workspace(comparisonIndex,:);
        dist = norm(point-comparisonPoint); 
        if (dist < minDist)
            minDist = dist; 
            index = comparisonIndex;
        end 
    end 
end 
