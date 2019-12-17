% laser=rossubscriber('/scan')
tic;
while toc < 50
    scan = receive(laser,3);
    plot(scan);
end
   
% front is where wheels are, defined as 0 and 360 degrees
% angles calculated counter clockwise
% ranges are distance (oh)
% intensities are reflective surface qualities