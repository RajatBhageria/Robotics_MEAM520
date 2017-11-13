% updates qs (configuration) of robot using keyboard input
function keyPressFcn(h_obj, evt)

global qs 

if evt.Key == 'q'
    qs(1) = qs(1) + 0.1;
elseif evt.Key == 'a'
    qs(1) = qs(1) - 0.1;
elseif evt.Key == 'w'
    qs(2) = qs(2) + 0.1;
elseif evt.Key == 's'
    qs(2) = qs(2) - 0.1;
elseif evt.Key == 'e'
    qs(3) = qs(3) + 0.1;
elseif evt.Key == 'd'
    qs(3) = qs(3) - 0.1;
end

end