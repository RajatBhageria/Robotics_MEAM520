%% Used to generate the helix where all the joints are moving 
%% at a constant velocity 
% lynxStart()
% %%
% for i = -100:1:100
%     %% set the initial position of the robot
%     th = 1.2/100*i;
%     q = th*ones(6,1);
%     [X,T] = updateQ(q); 
% 
% %     %% set qdot 
% %     qdot = [0 0 0 0 0 0]';
% 
% %     %% find the velocity 
% %     [velocity] = forwardVelocityKinematics(q', qdot);
%     
%     x = X(6,1);
%     y = X(6,2);
%     z = X(6,3);
% 
%     lynxServo(q); hold on;
%     scatter3(x,y,z); hold on;
%     pause(.01);
% end 


%% Used to trace out circle
lynxStart()

qCurr = [0 0 0 0 0 0];
qEnd = [1.2 0 0 0 0 0];

for t = 1:1:100
    velocity = [0;2*cos(2*t*pi/100);2*sin(2*t*pi/100)];
    % find joint velocity vectors using invVelKinematics
    [qdot] = inverseVelocityKinematics(velocity,qCurr);
    
    % add joint velocities to current configuration to find new config
    qCurr = qCurr + qdot';
    
    [X,T] = updateQ(qCurr); 
    
    x = X(6,1);
    y = X(6,2);
    z = X(6,3);

    lynxServo(qCurr); hold on;
    scatter3(x,y,z); hold on;
    pause(.01);
end 