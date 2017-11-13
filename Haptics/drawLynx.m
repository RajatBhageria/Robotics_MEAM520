function [figClosed] = drawLynx(h1, h2, F)

figClosed = 0;

global posEE;

if(ishghandle(h1)&&ishghandle(h2))
    set(h1, 'XData', posEE(1), 'YData', posEE(2), 'ZData', posEE(3))
    set(h2, 'XData', posEE(1), 'YData', posEE(2), 'ZData', posEE(3), 'UData', F(1), 'VData', F(2), 'WData', F(3))
else
    figClosed = 1;
end
end