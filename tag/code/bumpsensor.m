function state = bumpsensor(BumpSubscriber)

% retrieve the odometry data from the ROS subscriber
BumpState = receive(BumpSubscriber);
% convert the quaternion nonsense to rotation about Z axis
state = BumpState.Bumper;

end