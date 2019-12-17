% set the speed parameters
spd = 0.18;
maxx = 0.22;

forwardSpeed = spd;
rotationSpeed = 0;

% set the run time
tic;
runtime = 120;    %seconds

% run robot for specified run time
turtlebotSendSpeed(forwardSpeed, rotationSpeed, velocityPublisher);

while toc < runtime
    scan = receive(laser,3);
    plot(scan);
    
    closestObs = min(scan.Ranges(scan.Ranges>0));
    closestLocs = find(scan.Ranges == closestObs);
    closestLoc = mean(closestLocs);
    
    % ------------------------------------------------
    %nearObs = closestObs + 0.1; 
    %     ^ The idea is to create multiple "zones"
    %     * critical zone: stop if anything is super close to bot
    %     * near zone: stop if anything is super close to bot
    
    % ------------------------------------------------
    
    % Critical Zone Emergency stop
%     if closestObs < 0.14
%         break
%     else
%         turtlebotSendSpeed(forwardSpeed, rotationSpeed, velocityPublisher) % go
%     end
    
    % Near Zone -> Pivot away
    if closestLoc > 0 && closestLoc < 45 && closestObs < 0.35    %Front left
         rotationSpeed = -2; % cw
         forwardSpeed = 0;
    elseif  closestLoc > 45 && closestLoc < 135 && closestObs < 0.2
        rotationSpeed = -2; % cw
    elseif closestLoc > (270-45) && closestLoc < (270 + 45) && closestObs < 0.2
        rotationSpeed = 2; % ccw
    elseif closestLoc > (270 + 45) && closestLoc < 359 && closestObs < 0.35  %front right
        rotationSpeed = 2; % ccw
        forwardSpeed = 0;
    elseif closestLoc > (135) && closestLoc < (180) && closestObs < 0.2  %back left
        rotationSpeed = -2; % ccw
        forwardSpeed = maxx;
    elseif closestLoc > (180) && closestLoc < (270-45) && closestObs < 0.2  %back right
        rotationSpeed = 2; % ccw
        forwardSpeed = maxx;
    else
        rotationSpeed = 0;
        forwardSpeed = spd;
    end

turtlebotSendSpeed(forwardSpeed, rotationSpeed, velocityPublisher)

end

turtlebotStop(velocityPublisher)