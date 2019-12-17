%%
tbot = turtlebot('192.168.10.105')
%tbot = turtlebot('192.168.187.129') %ben's
%% MAIN FOR BIG TURTLE BOT 
% set the speed parameters
forwardSpeed = 0;
rotationSpeed = .7;
chasespeed=.7;
chasespin=.6;
spinspeed=.4;
% set the run time
runtime = 200;    %seconds
timeInterval = .1;
n_intervals = floor(runtime/timeInterval);
za=0;
% run robot for specified run time
for i = 1:n_intervals
    BumpSubscriber = rossubscriber('/mobile_base/sensors/core');
    state=bumpsensor(BumpSubscriber)
    if (state ~= 0)
      soundpub = rospublisher('/mobile_base/commands/sound', 'kobuki_msgs/Sound')
      soundmsg = rosmessage('kobuki_msgs/Sound');
      soundmsg.Value = 6;      % Any number 0-6
      send(soundpub,soundmsg);
    end
    turtlebotSendSpeed(forwardSpeed, rotationSpeed, velocityPublisher);
    colorImg = getColorImage(tbot);
    filename='colorImg.jpg';
    imwrite(colorImg,filename);
       %figure(3);
      imshow(colorImg);
       
    cam_capture = imread('colorImg.jpg');
    BW_Img1= createMaskRED(cam_capture);
    BW_Img2= createMaskBLUE(cam_capture);
    BW_Img3= createMaskORANGE(cam_capture);
%     BW_Img4= createMaskYELLOW(cam_capture);
    img_stats1 = regionprops('struct',BW_Img1,'all');
    img_stats2 = regionprops('struct',BW_Img2,'all');
    img_stats3 = regionprops('struct',BW_Img3,'all');
%     img_stats4 = regionprops('struct',BW_Img4,'all');
    index1 = find([img_stats1.Area] == max([img_stats1.Area]));
    index2 = find([img_stats2.Area] == max([img_stats2.Area]));
    index3 = find([img_stats3.Area] == max([img_stats3.Area]));
%     index4 = find([img_stats4.Area] == max([img_stats4.Area]));
    [B1,L,N] = bwboundaries(BW_Img1); 
    [B2,L,N] = bwboundaries(BW_Img2); 
    [B3,L,N] = bwboundaries(BW_Img3); 
%     [B4,L,N] = bwboundaries(BW_Img4); 
         %imshow(BW_Img);   
    if ~isempty(index1) && ~isempty(index2) && ~isempty(index3) 
        boundary1 = B1{index1};
        boundary2 = B2{index2};
        boundary3 = B3{index3};
%         boundary4 = B4{index4};
        figure(4);
        imshow(cam_capture);
        hold on;
        plot(boundary1(:,2), boundary1(:,1), 'g', 'LineWidth', 2);
        plot(boundary2(:,2), boundary2(:,1), 'y', 'LineWidth', 2);
        plot(boundary3(:,2), boundary3(:,1), 'm', 'LineWidth', 2);
%         plot(boundary4(:,2), boundary4(:,1), 'c', 'LineWidth', 2);
        hold off;
        [onesRowLoc1,onesColumnLoc1]=find(BW_Img1~=0);
        [onesRowLoc2,onesColumnLoc2]=find(BW_Img2~=0);
        [onesRowLoc3,onesColumnLoc3]=find(BW_Img3~=0);
       A=median(onesColumnLoc1)
       B=median(onesColumnLoc2)
       C=median(onesColumnLoc3)
       xPos=[A,B,C];
        xCenter=median(xPos)
        if za==0
        [y, Fs] = audioread('house.mp3');
        sound(y, Fs, 16);
        za=1;
        end
       % pause(timeInterval)
        %
        if xCenter > 0 && xCenter < 128 %&& closestObs < 0.35    %Front left
         rotationSpeed = chasespin; % cw
         forwardSpeed = chasespeed;
    elseif  xCenter > 129 && xCenter < 256 %&& closestObs < 0.2
        rotationSpeed = chasespin/2; % cw
         forwardSpeed = chasespeed;
    elseif  xCenter > 257 && xCenter < 384 %&& closestObs < 0.2
        rotationSpeed = 0; % cw
         forwardSpeed = chasespeed;
    elseif  xCenter > 385 && xCenter < 512 %&& closestObs < 0.2
        rotationSpeed = -chasespin/2; % cw
         forwardSpeed = chasespeed;
    elseif  xCenter > 513 && xCenter < 640 %&& closestObs < 0.2
        rotationSpeed = -chasespin; % cw
          forwardSpeed = chasespeed;
        else
        end
    turtlebotSendSpeed(forwardSpeed, rotationSpeed, velocityPublisher)
    else
        clear sound
        za=0;
       rotationSpeed = spinspeed;
       forwardSpeed = 0;
    turtlebotSendSpeed(forwardSpeed, rotationSpeed, velocityPublisher)
    end
    
end
%%  Keyboard control of big one
handles.odomSub = rossubscriber('/odom', 'BufferSize', 25);
receive(handles.odomSub,3);
handles.laserSub = rossubscriber('/scan', 'BufferSize', 50);
receive(handles.laserSub,3);

handles.velPub = rospublisher('/mobile_base/commands/velocity');
%% To actually enable keyboard control
exampleHelperTurtleBotKeyboardControl(handles);

%% Obstacle Avoidance Big    POS don't use
gains.goalTargeting = 5;          % Gain for desire to reach goal
gains.forwardPath = .1;            % Gain for moving forward 
gains.continuousPath = .1;         % Gain for maintaining continuous path
gains.obstacleAvoid = 20;        % Gain for avoiding obstacles

timerHandles.pub = rospublisher('/mobile_base/commands/velocity'); % Set up publisher
timerHandles.pubmsg = rosmessage('geometry_msgs/Twist');

timerHandles.sublaser = rossubscriber('/scan');  % Set up subscribers
timerHandles.subodom = rossubscriber('/odom');
timerHandles.subbump = rossubscriber('/mobile_base/sensors/bumper_pointcloud');

timerHandles.gains = gains;

timer1 = timer('TimerFcn',{@exampleHelperTurtleBotObstacleTimer,timerHandles},'Period',0.1,'ExecutionMode','fixedSpacing');
timer1.StopFcn = {@exampleHelperTurtleBotStopCallback};

start(timer1);
while strcmp(timer1.Running, 'on')
    exampleHelperTurtleBotShowGrid(timerHandles.sublaser);
    pause(0.5);    
end
%%
odomresetpub = rospublisher('/mobile_base/commands/reset_odometry');  % Reset odometry 
odomresetmsg = rosmessage('std_msgs/Empty');
send(odomresetpub,odomresetmsg)
pause(2);     % Wait until odometry is reset
%% Audio?
  soundpub = rospublisher('/mobile_base/commands/sound', 'kobuki_msgs/Sound')
  soundmsg = rosmessage('kobuki_msgs/Sound');
  soundmsg.Value = 6;      % Any number 0-6
  send(soundpub,soundmsg);
%% Audio text to speech?
 soundpub = rospublisher('/mobile_base/commands/sound', 'cob_sound/SayTextResponse')
  soundmsg = rosmessage('cob_sound/SayTextResponse');
  soundmsg.Value = 6;      % Any number 0-6
  send(soundpub,soundmsg);
%% Image processing
for i = 1:20
    colorImg = getColorImage(tbot);
    filename='colorImg.jpg';
    imwrite(colorImg,filename);
       figure(3);
      imshow(colorImg)
cam_capture = imread(colorImg);
    
    
    BW_Img1 = createMask1(cam_capture);
    
    img_stats1 = regionprops('struct',BW_Img1,'all');
    index1 = find([img_stats1.Area] == max([img_stats1.Area]));
    
    
    [B,L,N] = bwboundaries(BW_Img1);
    
    %     imshow(BW_Img);
    
    if ~isempty(index1)
        boundary1 = B{index1};
        figure(1);
        imshow(cam_capture);
        hold on;
        plot(boundary1(:,2), boundary1(:,1), 'g', 'LineWidth', 2);
        hold off;
        pause(0.1)
    end
end














