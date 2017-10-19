function [G] = createGraph(workspace, c_space)
    [n,~] = size(workspace); 
    A = zeros(n,n); 

    for row = 1:n
        for col = 1:n 
            point1 = workspace(row,:);
            point2 = workspace(col,:); 
            % calculate the euclidian dist between two points in workspace
            dist = norm(point1-point2);
            indexDistance = norm(row - col); 
            
            % calculate cost that penalizes large movements in individual 
            % steps between thetas
            cost = 50*dist/1000 + 200*indexDistance/20000; 
            % penalize very large distances that could result in jerkiness
            % penalize very small distances that could result in
            % sub-optimal movements
            if (cost < 10 && cost > 3)
                A(row,col) = cost; 
            end 
        end 
        if (mod(row,100)==0)
             disp(row); 
        end 
    end 
    
    %return graph 
    G = graph(A); 

end

