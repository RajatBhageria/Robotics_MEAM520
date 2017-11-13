function [] = lynxDCTorquePhysical(i1,i2,i3,i4,i5,i6)
% Commands the real Lynx robot to the currents defined by the input (in A)
% Has the same limits as lynxDCTorque.m

% INPUTS:
%   i1...i6 : Motor currents for the six DOF Lynx arm (A)

global ttl robotName
i = [i1,i2,i3,i4,i5,i6];

%% Position and Velocity limits (NOT CURRENTLY IMPLEMENTED)

% Read potentiometers
% messageOut = 0;
% pause(2);
% fwrite(ttl,messageOut);
% while(ttl.BytesAvailable<12)
%     disp('Stuck on read');
% end
% messageIn=fread(ttl,ttl.BytesAvailable);

% q = [th1 th2 th3 th4 th5 grip]; % Position vector
% 
% Adjusting for out of range positions
% lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm)
% upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in mm)
% maxOmegas = [1 1 1 2 3 20];
% 
% %Instructions said 1000us => 90degrees.  If so, 636.62 converts to
% %microseconds.
% maxSpeedCommands = maxOmegas*636.62;
% 
% for i=1:length(q)
%     if q(i) < lowerLim(i)
%         q(i) = lowerLim(i);
%         fprintf('Joint %d was sent below lower limit, moved to boundary %0.2f\n',i,lowerLim(i))
%     elseif q(i) > upperLim(i)
%         q(i) = upperLim(i);
%         fprintf('Joint %d was sent above upper limit, moved to boundary %0.2f\n',i,upperLim(i))
%     end
% end
% 
% %% Collision Detection
% 
% % Preventing forearm from hitting base of robot
% if (q(3) > (-0.135 * q(2) + 1.15))
%     q(3) = (-0.135 * q(2)) + 1.15;
%     %disp('Position would have caused collision, moved to closest safe position')
% end

%% Current limits
%These are limits from spec sheet
% lowerCurrentLim = [-1.2, -2, -1.8, 0, 0, 0];
% upperCurrentLim = [1.2, 2, 1.8, 0, 0, 0];

%These are limits when power supply is 10V
lowerCurrentLim = [-0.7, -1, -0.9, 0, 0, 0];
upperCurrentLim = [0.7, 1, 0.9, 0, 0, 0];

%These are limits when power supply is 20V
% lowerCurrentLim = [-1.2, -1.6, -1.6, 0, 0, 0];
% upperCurrentLim = [1.2, 1.6, 1.6, 0, 0, 0];

for j=1:length(i)
    if i(j) < lowerCurrentLim(j)
        i(j) = lowerCurrentLim(j);
        fprintf('Joint %d was sent below lower current limit, moved to boundary %0.2f\n',j,lowerCurrentLim(j))
    elseif i(j) > upperCurrentLim(j)
        i(j) = upperCurrentLim(j);
        fprintf('Joint %d was sent above upper current limit, moved to boundary %0.2f\n',j,upperCurrentLim(j))
    end
end

%% Serial Command Conversion

% Convert currents to pwm values
R = 5; %MAYBE CHANGE
voltages = i*R;

% We'll probably want to make this unique for each motor
% Voltage = (pwm/255 * 5 -2.5) * 4.3
pwm = floor((voltages/4.3 + 2.5)*255/5); %DEFINITELY CHANGE (127 is zero voltage.  The rest depends on the resistors used in the circuit.  Nominal is 0-255 => -10 to 10 V

messageOut = [1,uint8(pwm)];
fwrite(ttl,messageOut);


end
