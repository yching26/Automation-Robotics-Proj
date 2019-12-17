function state = scansensor(ScanSubscriber)

% retrieve the odometry data from the ROS subscriber
ScanState = receive(ScanSubscriber);
% convert the quaternion nonsense to rotation about Z axis
state = ScanState.Ranges;

end


%ScanSubscriber = rossubscriber('/scan');