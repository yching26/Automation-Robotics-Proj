function turtlebotStop(velocityPublisher)

for i = 1:10
    turtlebotSendSpeed(0, 0, velocityPublisher)
    pause(0.01);
end