function [c_space, workspace] = freeCSpaceCalc(pos, ri)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   p_start the start point in the workspace
%   p_end the end point in the worspace
%   pos: [xi,yi,zi]' representing the position of obstacle i 
%   ri: radius of obstacle i 

c_space = []; %this is an n x 6 matrix 
workspace = []; %n x 3 matrix of resulting points
counter = 0; 
for theta1 = -1.4:.28:1.4 %10 increments = mag of difference of min and max thetas / 10
    for theta2 = -1.2:.26:1.4 %10 increments
         for theta3 = -1.8:.35:1.7 %10 increments
             for theta4 = -1.9:.36:1.7 %10 increments
                theta5 = 0; 
                theta6 = 0; 
                q = [theta1 theta2 theta3 theta4 theta5 theta6];
                              
                %Do FK to find the positions in the workspace of each of
                %the joints and the end effector (these are in X) 
                [X,~] = updateQ(q);
                j1Pos = X(1,:); 
                j2Pos = X(2,:); 
                j3Pos = X(3,:);
                j4Pos = X(4,:); 
                endEffPos = X(6,:);
                
                linkBeg = [j1Pos; j2Pos; j3Pos; j4Pos]; %4x3
                linkEnd = [j2Pos; j3Pos; j4Pos; endEffPos]; %4x3
                linkVectors = linkEnd - linkBeg; %4x3
                
                %Interpolate the array of points between each of the joint                                
                %each gives us a 4x3 vector of points 1/3 and 2/3 of the
                %distance between two consecutive joints. 
                oneThirdLinks = (1/3)*linkVectors + linkBeg;
                twoThirdLinks = (2/3)*linkVectors + linkBeg;
                
                %adding all the points in the workspace of the joints
                %(theta1-theta4 and the end effector) along with two points
                %in between each of the joints to a nx3 matrix that
                %represents the geometry of the robot and the points the
                %obstacles cannot hit. 
                robotGeometry = [X(1:4,:); endEffPos; oneThirdLinks; twoThirdLinks];
                
                obstacleDetected = isWithinObstacle(robotGeometry);
                
                if (~obstacleDetected)
                    c_space  = [c_space; q];
                    workspace = [workspace; endEffPos];
                else
                    counter = counter + 1 %number of collision points in the c -space
                end 
            end
         end
    end
end 

function [robotIsWithinObstacle] = isWithinObstacle(robotGeometry)
    WIDTH_ROBOT = 35; %the lynx robot has a max radius of 35mm at any given arm 
    
    % adding buffer zone around the existing obstacle space
    newRad = ri + WIDTH_ROBOT; 
    
    xsObstacle = pos(:,1); 
    ysObstacle = pos(:,2); 
    zsObstacle = pos(:,3); 
    
    xsRobot = robotGeometry(:,1); 
    ysRobot = robotGeometry(:,2); 
    zsRobot = robotGeometry(:,3); 
    
    robotIsWithinObstacle = false; 
    
    % testing all collected points representative of the robot
    % configuration and seeing whether it collides with any of the space
    % occupied by the obstacles + obstacle strings
    for obstaclei = 1:size(pos,1)
        xsWithinObstacle = ((xsObstacle(obstaclei,1)-newRad)<xsRobot) & (xsRobot<(xsObstacle(obstaclei,1)+newRad));
        ysWithinObstacle = ((ysObstacle(obstaclei,1)-newRad)<ysRobot) & (ysRobot<(ysObstacle(obstaclei,1)+newRad));
        zsWithinObstacle = (zsObstacle(obstaclei,1) - newRad) < zsRobot;
        robotWithinObstacleI= xsWithinObstacle & ysWithinObstacle & zsWithinObstacle; 
        collides = (size(find(robotWithinObstacleI==1),1))>1;
        if (collides)
            robotIsWithinObstacle = 1; 
            break; 
        end
    end

end

end

