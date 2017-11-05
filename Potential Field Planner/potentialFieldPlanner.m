function [] = potentialFieldPlanner(posOfObstacles,ri,qStart,qEnd)
%% @param: posOfObstacles a mx3 matrix with the xyz positions of each of the m obstacles 
% @param: ri: a mx1 matrix of radii for the m obstacles 
% @param: qStart: a 6x1 matrix of the joint configurations of starting pos
% @param: qEnd: a 6x1 matrix of the joint configurations of ending pos

%start the lynx
lynxStart();        

% set the epsilon for what a reasonable distance to stop is 
epsilon = .15; 

%Draw the obstacles 
for i = 1:size(posOfObstacles)
    pos = posOfObstacles(i,:); 
    rad = ri(i);
    [x,y,z] = sphere();
    surf(rad*x+pos(1,1), rad*y+pos(1,2), rad*z+pos(1,3)); hold on;
end 

%assign qCurr to be qStart. qCurr will be updated throughout the while loop
qCurr = qStart; 

%loop while the distance between the configuration spaces have not reached
%the goal configuration 
while any((qEnd(1,1:5) - qCurr(1,1:5))>epsilon)
    %% Calculate the attractive force
    [XCurr,~] = updateQ(qCurr); 
    [XEnd, ~ ] = updateQ(qEnd); 
    
    %% Calculate the attractive force, a 6x1 vector 

    %initialize taua
    tauaTotal = zeros(6,1); 
    taurTotal = zeros(6,1); 
    
    %Find all the J matrices using qCurr 
    JAll = [];
    
    syms q1 q2 q3 q4 q5 q6
    for joint = 1:6
        %% Calculate the attractive joint effort for joint i 
        %Find the positions of the current position and the end goal of the
        % joint i. 
        currPosJointI = XCurr(joint,:);
        endPosJointI = XEnd(joint,:);
        
        %instantiate Fa 
        Fa = zeros(1,3); 
        
        %if case over whether the current pos of joint i is at it's goal 
        if( norm(currPosJointI - endPosJointI) ~= 0)
            %if joint i is not at its goal position, then calculate Fa
            Fa = - (currPosJointI - endPosJointI) / norm(currPosJointI - endPosJointI);
        else
            Fa = zeros(1,3); 
        end 
        
        %find the jacobian for joint i 
        J = findJacobian(qCurr,joint);
        
        %substitute values of qCur into symbolic J 
        J = subs(J, q1, qCurr(1));
        J = subs(J, q2, qCurr(2));
        J = subs(J, q3, qCurr(3));
        J = subs(J, q4, qCurr(4));
        J = subs(J, q5, qCurr(5));
        J = subs(J, q6, qCurr(6));
        
        % Convert sym matrix back to numeric
        J = double(J);
        
        %Only take the first joint columns 
        Jnew = J(:,1:joint); 
        
        %%add zeros if Jnew is not a 6x5
        colsOfZerosToAdd = 6 - joint;
        Jnew = [Jnew, zeros(6,colsOfZerosToAdd)];
        
        %calculate the taua for this joint 
        Jv = Jnew(1:3,:);
        taua = Jv'*Fa';
        %6x1   =    [6x3][3x1]
            
        %find the total Taua by adding the taua for this joint 
        tauaTotal = tauaTotal + taua; %final result of 5x6 matrix
    
        %% Calculate the repulsive forces 
        %initialize a Fr vector;
        Fr = zeros(1,3);
        
        %add 20mm to our ris to have a larger sphere of influence
        additionalSphereOfInfluence = 20; %mm
        ri = ri + additionalSphereOfInfluence;
        
        %loop over all the obstacles we have
        for obstacle = 1:size(posOfObstacles,1)
            %find the center point of the particular obstacles
            posOfObstacle = posOfObstacles(obstacle,:);
            
            %find the radius of the particular obstacle
            radiusi = ri(obstacle,1);
            
            %find the distance between the vector of joints and the current
            %obstacle
            distBetweenJointAndObstacle = norm(currPosJointI-posOfObstacle);
            
            %figure out whether the joint falls within the sphere of
            %influence of the obstacle. this is a boolean 
            jointIsWithinSphereOfInfluence = distBetweenJointAndObstacle < radiusi;
            
            %Robot is not within the sphere of influence of the obstacle
            if (~jointIsWithinSphereOfInfluence)
                %Fr is still zeros.
                Fr = zeros(1,3); 
            else
                %Robot is within the sphere of influence of the obstacle
                nu = 13;
                %find the expression Fr
                firstPart = zeros(1,3); 
                secondPart = zeros(1,3); 
                thirdPart = zeros(1,3); 
                if (currPosJointI>0 & posOfObstacle >0)
                    firstPart = 1 ./ currPosJointI - 1 ./posOfObstacle;
                end 
                if (currPosJointI>0)
                    secondPart = 1./(currPosJointI .^ 2);
                end 
                b = zeros(1,3);
                if (norm(currPosJointI - posOfObstacle)~=0)
                    b = (currPosJointI - posOfObstacle)/norm(currPosJointI - posOfObstacle) * radiusi + posOfObstacle;
                end 
                if (norm(currPosJointI - b)~=0)
                    thirdPart = (currPosJointI - b)/norm(currPosJointI - b);
                end 
                Fr = nu * firstPart .* secondPart .* thirdPart;
            end
            
            %find the taur for obstacle j on joint i
            taur = Jv'*Fr';
            %add taur to taurTotal
            taurTotal = taurTotal + taur; 
        end
        
    end 

    %set the tau = to the tauatotal + the taurtotal
    tau = tauaTotal + taurTotal;
    
    %set the step rate
    alpha = 0.02;
    
    %ensure there is not
    if (norm(tau)~=0)
        qCurr = qCurr' + alpha * (tau / norm(tau));
    else
        qCurr = qCurr';
    end
    
    %change qCurr to be 1x6 not 6x1 to match with input
    qCurr = qCurr' ;
    disp(qCurr);
    lynxServo(qCurr);
    scatter3(XCurr(6,1),XCurr(6,2),XCurr(6,3)); hold on;
end

end