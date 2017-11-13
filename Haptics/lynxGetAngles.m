function qs = getAngles()

global ttl
global robotName
% maxPotVals = [40, 44, 75];
% minPotVals = [980, 934, 858];
% minAngles = [-2.0, -1.57, 1.71]; % radians
% maxAngles = [1.57, 1.75, -1.4]; % radians

if strcmpi(robotName, 'Lucky')
    potValsPerHalfTurn = [838,840,800];
    potOffsets = [533,520,520];
    potDirection = [-1,-1,1];
elseif strcmpi(robotName, 'Lyric')
    potValsPerHalfTurn = [838,840,800];
    potOffsets = [458,516,577];
    potDirection = [-1,-1,1];
else
    error('Invalid robot name.')
end

radiansPerPotVal = pi./potValsPerHalfTurn;
fwrite(ttl,uint8([0,0,0,0,0,0,0]));
pause(0.001);
while(ttl.BytesAvailable<12)
end
messageIn = fread(ttl,ttl.BytesAvailable);
potVals = messageIn([1,3,5])'*256 + messageIn([2,4,6])';
% qs = (potVals - minPotVals)./(maxPotVals - minPotVals) .* (maxAngles - minAngles) + minAngles;
qs = potDirection .* (potVals - potOffsets) .* radiansPerPotVal;
end