function turtlebotSendSpeed(forwardSpeed, turnSpeed, velocityPublisher)

velocityMessage = rosmessage(velocityPublisher);
velocityMessage.Linear.X = forwardSpeed;
velocityMessage.Linear.Y = 0;
velocityMessage.Linear.Z = 0;
velocityMessage.Angular.Z = turnSpeed;

send(velocityPublisher,velocityMessage);
