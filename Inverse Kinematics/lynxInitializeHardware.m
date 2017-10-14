%serialPort is a string. e.g. 'com3'
function lynxInitializeHardware(serialPort)

%Close any previous communication.  Shouldn't need this when all is done!
delete(instrfindall);

%Opens serial communication with the lynx
global ttl

%The com port may change.  Go to Device manager on a PC to find the port.
%Alternatively, on a MAC, 
%ttl = serial('/dev/cu.usbserial-AI0484D4'); 
%Note that the portion after the hypen depends on the usb cable.  Type 
%ls /dev/cu.* into the command line to find this name.
ttl = serial(serialPort); 
ttl.BaudRate = 115200;
fopen(ttl);

if(~strcmp(ttl.Status,'open'))
    error('Please connect the Lynx robot via the USB port');
end

end
