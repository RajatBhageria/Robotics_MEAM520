function [] = lynxServoPhysical(th1, th2, th3, th4, th5, grip, microseconds)
% Commands the real Lynx robot to the angles defined by the input (in radians)
% Has the same limits as lynxServo.m

% INPUTS:
%   th1...th6 : Joint variables for the six DOF Lynx arm (th6 = grip)
%   microseconds (optional) : sets the duration of the motion\

global ttl robotName
q = [th1 th2 th3 th4 th5 grip]; % Position vector

%% Adjusting for out of range positions
lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm)
upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in mm)
maxOmegas = [1 1 1 2 3 20];

%Instructions said 1000us => 90degrees.  If so, 636.62 converts to
%microseconds.
maxSpeedCommands = maxOmegas*636.62;

for i=1:length(q)
    if q(i) < lowerLim(i)
        q(i) = lowerLim(i);
        fprintf('Joint %d was sent below lower limit, moved to boundary %0.2f\n',i,lowerLim(i))
    elseif q(i) > upperLim(i)
        q(i) = upperLim(i);
        fprintf('Joint %d was sent above upper limit, moved to boundary %0.2f\n',i,upperLim(i))
    end
end

%% Collision Detection

% Preventing forearm from hitting base of robot
if (q(3) > (-0.135 * q(2) + 1.15))
    q(3) = (-0.135 * q(2)) + 1.15;
    disp('Position would have caused collision, moved to closest safe position')
end


%% Serial Command Conversion

% Find which robot is being controled, and adjust to offsets appropriately
% if strcmpi(robotName, 'Legend')
%     % Servo offsets
%     servoOffsets = [-4.02, -1.13, -0.30, -5.55, 0.95, -50];
% elseif  strcmpi(robotName, 'Lucky')
%     % Servo offsets
%     servoOffsets = [-4.31425728, -1.16, -0.34906585, -5.75, 1.05, -50];
% elseif strcmpi(robotName, 'Lyric')
%     % Servo offsets
%     servoOffsets = [-3.96425728, -1.083529864, -0.35906585, -5.6, 1.308996939, -50];
% else
%     error('Invalid robot name.')
% end
if strcmpi(robotName, 'Legend')
    % Servo offsets
    servoOffsets = [1380, 1480, 1500, 1450, 1420, 2000];
    servoDirection = [-1,-1,1,-1,-1,1];
elseif  strcmpi(robotName, 'Lucky')
    % Servo offsets
    servoOffsets = [1550, 1500, 1470, 1560, 1460, 2000];
    servoDirection = [-1,-1,1,-1,-1,1];
elseif strcmpi(robotName, 'Lyric')
    % Servo offsets
    servoOffsets = [1370, 1400, 1500, 1450, 1640, 2000];
    servoDirection = [-1,-1,1,-1,-1,1];
else
    error('Invalid robot name.')
end

P1 = servoDirection(1) * q(1) * (180/pi/0.102) + servoOffsets(1);
P2 = servoDirection(2) * q(2) * (180/pi/0.105) + servoOffsets(2);
P3 = servoDirection(3) * q(3) * (180/pi/0.109) + servoOffsets(3);
P4 = servoDirection(4) * q(4) * (180/pi/0.095) + servoOffsets(4);
P5 = servoDirection(5) * q(5) * (180/pi/0.102) + servoOffsets(5);
P6 = servoDirection(6) * q(6) / -0.028 + servoOffsets(6);

%% Sending commands to lynx
if(nargin == 6)
    fprintf(ttl, '%s\r', ...
        ['#1P' num2str(P1,'%.0f') ' S' num2str(maxSpeedCommands(1),'%.0f')  ...
        '#2P' num2str(P2,'%.0f') ' S' num2str(maxSpeedCommands(2),'%.0f') ...
        '#3P' num2str(P3,'%.0f') ' S' num2str(maxSpeedCommands(3),'%.0f') ...
        '#4P' num2str(P4,'%.0f') ' S' num2str(maxSpeedCommands(4),'%.0f') ...
        '#5P' num2str(P5,'%.0f') ' S' num2str(maxSpeedCommands(5),'%.0f') ...
        '#6P' num2str(P6,'%.0f') ' S' num2str(maxSpeedCommands(6),'%.0f')]);
end
if(nargin>6)
    fprintf(ttl, '%s\r', ...
        ['#1P ' num2str(P1,'%.0f') ' S' num2str(maxSpeedCommands(1),'%.0f')  ...
        '#2P ' num2str(P2,'%.0f') ' S' num2str(maxSpeedCommands(2),'%.0f') ...
        '#3P ' num2str(P3,'%.0f') ' S' num2str(maxSpeedCommands(3),'%.0f') ...
        '#4P ' num2str(P4,'%.0f') ' S' num2str(maxSpeedCommands(4),'%.0f') ...
        '#5P ' num2str(P5,'%.0f') ' S' num2str(maxSpeedCommands(5),'%.0f') ...
        '#6P ' num2str(P6,'%.0f') ' S' num2str(maxSpeedCommands(6),'%.0f') ...
        ' T' num2str(microseconds)]);
end

end
