clear all, close all

% In Gazebo, go to Edit and select Reset Model Poses (shortcut Shift+Ctrl+R).

% Create a ROS node and connect to roscore using rosinit('IP_ADDRESS_OF_VIRTUAL_PC')
rosshutdown
rosinit('10.128.0.100')

% Initial velocities
lin_vel = 0.15; % meters per second 
max_ang_vel = 2.84; % radians per second
init_lin_vel = lin_vel;
Kp = 1.2; % Sets proportional gain

MaxRange = 1; % max scan range of waffle pi is 3.5m
odomcount = 1; % a counter
odomList = zeros(4,100000); % matrix to collect odom readings

%% Subscribers and publishers
robot = rospublisher('/cmd_vel'); 
velmsg = rosmessage(robot); 
odom = rossubscriber('/pose_fusion');
emptyscan = rosmessage('sensor_msgs/LaserScan');
scansub = rossubscriber('/scan');


%% Create a rate object that runs at 10 Hz. T% imgSub = rossubscriber('/raspicam_node/image/compressed');
%Object Tracking
imgSub = rossubscriber('/raspicam_node/image/compressed');
imgMsg = receive(imgSub);
imgMsg.Format = 'bgr8; jpeg compressed bgr8';
rgbImg = readImage(imgMsg);

r = robotics.Rate(10);

%% Waypoints
% TO DO: add the last three waypoints
waypoints = [[0.5;0.5] [3.25;0.5] [3;2.55]];

% TO DO: un-commend the following code block for map generation

for i=2:size(waypoints,2)
    xEnd = waypoints(1,i);
    yEnd = waypoints(2,i);
    waypoint_reached = false;
    while waypoint_reached == false
    odomdata = receive(odom,3);    
    [x,y,phi,t] = GetPose(odomdata);
    scanMsg = receive(scansub, 10);
    lidarData = lidarScan(scanMsg);
    velmsg.Linear.X = lin_vel;
    send(robot,velmsg);
   while (x > 2.5) && (y<1.5) 
       fprintf('in second Obst_avoid')
       obst_avoid(odom,scansub,velmsg,robot,xEnd,yEnd)
       odomdata = receive(odom,3);    
    [x,y,phi,t] = GetPose(odomdata);
       waypoint_reached=true;
       break
   end
   
   while(x > 2) && (y<1) 
       fprintf('In first obst_avoid')
       obst_avoid(odom,scansub,velmsg,robot,xEnd,yEnd)
       odomdata = receive(odom,3);    
    [x,y,phi,t] = GetPose(odomdata);
       waypoint_reached=true;
       break
   end
    end
end
%% Object Tracking
while (y > 1.5) && (x>2.5)
   
    % Parameters for ball position and size
params.horizontalTolerance = 40;
params.sizeGoal = 250;
params.sizeTolerance = 50;
params.positionFlag = 0;
params.sizeFlag = 0;
params.width = size(rgbImg,2); 
load ('classifier.mat');
params.classifier = classifier;

 for i = 1:10000
    % Get latest image, ball postion, and ball size.
    imgMsg = receive(imgSub);
    imgMsg.Format = 'bgr8; jpeg compressed bgr8';
    rgbImg = readImage(imgMsg);
    figure(111),imshow(rgbImg)
    
    [predictedLabels,lin_vel,ang_vel]  = trackCircle(rgbImg, params, x, y, robot, velmsg, odom);

    % Send velocity commands and wait for commands to send.
    velmsg.Linear.X = lin_vel; 
    velmsg.Angular.Z = ang_vel; 
    send(robot,velmsg);
    pause(0.01)
    
 end
 

end

%% stop the robot

lin_vel = 0;
ang_vel = 0; 
velmsg.Linear.X = lin_vel; 
velmsg.Angular.Z = ang_vel; 
send(robot,velmsg);
        
rosshutdown

plot_trajectory(odomList, waypoints)

function plot_pointcloud(scanMsg)
lidarData = lidarScan(scanMsg);

ptCloud = pointCloud([lidarData.Cartesian zeros(size(lidarData.Cartesian,1),1)]);

minDist =0.25;
[labels,numClusters] = pcsegdist(ptCloud,minDist);

colors = hsv(numClusters+1);
figure(49),scatter(ptCloud.Location(:,1),ptCloud.Location(:,2),10,colors(labels+1,:))
xlabel('X [m]')
ylabel('Y [m]')
title('Segmented Obstacles')
end
function [x,y,phi,t] = GetPose(odomdata)
% TO DO: complete the function
%///... write your code here
x = odomdata.X;
y = odomdata.Y;
%q = odomdata.Pose.Pose.Orientation;
phi = odomdata.Theta; %2*atan2(q.Z,q.W); % get the angle from the quaternion
%         time = odomdata.Header.Stamp;
t = 1;%time.Sec + time.Nsec/1e9;

end
function [lin_vel,ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel)

Kp = 1.2;

% use below control logic to calculate phi_desired in four
% quadrant
if xEnd>x
    if yEnd>y % 1st quadrant
        phi_desired = atan((yEnd-y)/(xEnd-x));
    else % 4th quadrant
        phi_desired = 2*pi - atan(abs(yEnd-y)/abs(xEnd-x));
    end
else
    if yEnd>y % 2nd quadrant
        phi_desired = pi - atan(abs(yEnd-y)/abs(xEnd-x));
    else % 3rd quadrant
        phi_desired = pi + atan(abs(yEnd-y)/abs(xEnd-x));
    end
end


% TO DO: write the expression to calculate error
% ///... write your code here
error = phi - phi_desired;
% ///...
error = wrapToPi(error);
ang_vel = -Kp * error;
lin_vel = init_lin_vel*(1-(ang_vel)/(max_ang_vel)); %make lin_vel proportional to ang_vel
if lin_vel>init_lin_vel
    lin_vel = init_lin_vel;
end

if abs(ang_vel) > 0.2
    lin_vel = 0.05;
end
%///
end

%%
function plot_trajectory(robot_poses, waypoints)

max_ind = max(find(robot_poses(4,:)));

figure(10)
if nargin==2
    plot(waypoints(1,:),waypoints(2,:),'r*-')
    hold on
end

plot(robot_poses(1,1:max_ind),robot_poses(2,1:max_ind),'b.')
axis equal
xlabel('x (m)')
ylabel('y (m)')
title('Robot trajectory')

figure(20)
subplot(3,1,1)
plot(robot_poses(4,1:max_ind)-robot_poses(4,1), robot_poses(1,1:max_ind))
ylabel('x (m)')
title('Robot pose over time ')

subplot(3,1,2)
plot(robot_poses(4,1:max_ind)-robot_poses(4,1), robot_poses(2,1:max_ind))
ylabel('y (m)')

subplot(3,1,3)
plot(robot_poses(4,1:max_ind)-robot_poses(4,1),180/pi*robot_poses(3,1:max_ind))
xlabel('Time (sec)')
ylabel('Angle (deg)')
end
%% obstacle avoid function
function obst_avoid(odom, scansub,velmsg,robot,xEnd,yEnd)
while true
MaxRange = 1; % max scan range of waffle pi is 3.5m
odomcount = 1; % a counter
odomList = zeros(4,100000); % matrix to collect odom readings

    lin_vel = 0.15; % meters per second 
    max_ang_vel = 2.84; % radians per second
    init_lin_vel = lin_vel;
    
    scanMsg = receive(scansub, 10);
    lidarData = lidarScan(scanMsg);
    velmsg.Linear.X = lin_vel;
    send(robot,velmsg);
    
    odomdata = receive(odom,3);
    [x,y,phi,t] = GetPose(odomdata);% returns x,y,phi,t
    ranges = lidarData.Ranges;
    angles = lidarData.Angles;
    fwdranges = [ranges(1:30); ranges(331:360)];
    %         Rrange =  [ranges(); ranges()];
    
    % use () brackets
    for i=1:60
        if fwdranges(i) == 0
            fwdranges(i) = NaN;
        end
    end
    
    [minrangeF, minIndexF] = min(fwdranges);
    plot_pointcloud(scanMsg);
    
    minrangeF_current = minrangeF;
    
    if minrangeF_current<0.4
        %collect odometry readingds (x,y,phi and t) to odomList
        odomList(:,odomcount) = [x; y; phi; t];
        odomcount = odomcount + 1;
        lin_vel = 0.01;
        %  ang_vel = 0;
        velmsg.Linear.X = lin_vel;
        % velmsg.Angular.Z = ang_vel;
        send(robot,velmsg);
        
        if (1 < minIndexF) && (minIndexF <= 30) % to the bots left
            % turn right
            fprintf('turning Right')
            phi_start=phi-pi/6;
            
            phi = wrapTo2Pi(phi);
            phi_start = wrapTo2Pi(phi_start);
            
            while (phi>phi_start)
                phi
                phi_start
                fprintf('in turn right\n')
                ang_vel = -0.5;
                velmsg.Angular.Z = ang_vel;
                send(robot,velmsg);
                odomdata = receive(odom,3);
                [x,y,phi,t] = GetPose(odomdata);% returns x,y,phi,t
                
                % collect odometry readingds (x,y,phi and t) to odomList
                odomList(:,odomcount) = [x; y; phi; t];
                odomcount = odomcount + 1;
            end
            fprintf('end turn right')
            [minrangeF, minIndexF] = min(fwdranges);
            plot_pointcloud(scanMsg);
            minrangeF_current = minrangeF;
            
        end
        
        if (31 <= minIndexF) && (minIndexF <= 60) % to the bots right
            % turn left
            fprintf('turning left')
            phi_start=phi+pi/6;
            phi = wrapTo2Pi(phi);
            phi_start = wrapTo2Pi(phi_start);
            
            while (phi < phi_start)
                fprintf('in turn left')
                ang_vel = 0.5;
                velmsg.Angular.Z = ang_vel;
                send(robot,velmsg);
                odomdata = receive(odom,3);
                [x,y,phi,t] = GetPose(odomdata);% returns x,y,phi,t
                
                % collect odometry readingds (x,y,phi and t) to odomList
                odomList(:,odomcount) = [x; y; phi; t];
                odomcount = odomcount + 1;
            end
            
            fprintf('end turn left')
            [minrangeF, minIndexF] = min(fwdranges);
            plot_pointcloud(scanMsg);
            minrangeF_current = minrangeF;
        end
        
    else
        fprintf('else statement')
        lin_vel = 0.15;
        velmsg.Linear.X = lin_vel;
        [lin_vel,ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel);
        velmsg.Angular.Z = ang_vel;
        send(robot,velmsg);
        % collect odometry readingds (x,y,phi and t) to odomList
        odomList(:,odomcount) = [x; y; phi; t];
        odomcount = odomcount + 1;
        
        distance = sqrt((xEnd-x)^2+(yEnd-y)^2)
        % stop the robot if the distance between robot and goal is less
        % than 0.1m
        
        if abs(distance)<0.15
            fprintf('BREAK')
            break
        end
        %while dist loop
    end
    % while true loop
end
%top for loop (waypoints)
end

%% Track
function [predictedLabels,lin_vel,ang_vel]  = trackCircle(rgbImg, params, x, y, robot, velmsg, odom)
    [position,ballSize,boundingBox] = findColorBall(rgbImg);
    
    % Initialize velocities to zero.
    lin_vel = 0; 
    ang_vel = 0;
    
    positionFlag = 0;
    sizeFlag = 0;
      
    % Left and right controls
    if isempty(position)
        ang_vel = 0.0;
        lin_vel = 0;
    elseif (position(1) < (params.width/2)-params.horizontalTolerance)
        ang_vel = 0.3;
    elseif (position(1) > (params.width/2)+params.horizontalTolerance)
        ang_vel = -0.3;
    else
        positionFlag = 1;
    end
      
    % Forward and back control
    if isempty(ballSize)
        ang_vel = 0.0;
        lin_vel = 0;
    elseif ballSize > params.sizeGoal + params.sizeTolerance
        lin_vel = - 0.05;
    elseif ballSize < params.sizeGoal - params.sizeTolerance
        lin_vel = 0.05;
    else
        sizeFlag = 1;
    end
     
    if (positionFlag==1 && sizeFlag==1)
        % run image classifier
        w = boundingBox(3);
        h = boundingBox(4);
        boundingBox = [boundingBox(1)+(w*0.2), boundingBox(2)+w*0.2, w*0.6, h*0.6]; 
        croppedCircle = imcrop(rgbImg, boundingBox);
        % figure, imshow(croppedCircle)
        bwcroppedCircle = rgb2gray(croppedCircle);
        bwcroppedCircle = imbinarize(bwcroppedCircle,0.1);
        figure(100), imshow(bwcroppedCircle)

        predictedLabels = predictNumber(bwcroppedCircle,params.classifier) 
        predicted_numb = string(predictedLabels);
        predicted_numb = double(predicted_numb);
%         predicted_numb = int(predicted_numb);
        ang_vel = 0.0;
        lin_vel = 0.0;
        velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
         send(robot,velmsg);
        odomdata = receive(odom,3);
        
        if predicted_numb == 2
            lin_vel = 0.0;
            ang_vel = 0.0;
            
            send(robot,velmsg);
            odomdata = receive(odom,3);
            
            fprintf('yeet')
            
            filepath = 'map_orig.jpg'; % Provide the path of map image file
            image = imread(filepath);
            figure(1),imshow(image)
            
            % Convert to grayscale and then black and white image based on arbitrary
            % threshold.
            grayimage = rgb2gray(image);
            bwimage = grayimage < 0.5;
            
            % Use black and white image as matrix input for binary occupancy grid.
            % Resolution was selected to be 136 by trial and error, specified in cells per meter.
            map = robotics.BinaryOccupancyGrid(bwimage,136);
            load('map')
            %             figure(2),show(map)
            
            mapInflated = copy(map);
            inflate(mapInflated,0.18); %robot radius is 0.22m
            figure(3),show(mapInflated)
            start_location = [x, y];
            end_location = [0.75, 2.25];
            %  call path plan function
            [path] = FindLocalPath(mapInflated, start_location, end_location);
            init_lin_vel = lin_vel;
            max_ang_vel = 2.84;
            path = path.'
            lin_vel = -0.5;
            velmsg.Linear.X = lin_vel;
            send(robot,velmsg);
            fprintf('pause\n')
            pause(1)
            
            for i=2:size(path,2)
                xEnd = path(1,i);
                yEnd = path(2,i);
                
                
                
                while true
                    lin_vel = 0.15;
                    velmsg.Linear.X = lin_vel;
                    
                    send(robot,velmsg);
                    odomdata = receive(odom,3);
                    
                    [x,y,phi,t] = GetPose(odomdata);% returns x,y,phi,t
                    
                    [lin_vel,ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel); % returns ang_vel
                    
                    velmsg.Angular.Z = ang_vel;
                    send(robot,velmsg);
                    
                    % collect odometry readingds (x,y,phi and t) to odomList
                    distance = sqrt((xEnd-x)^2+(yEnd-y)^2);
                    % stop the robot if the distance between robot and goal is less
                    % than 0.1m
                    if abs(distance)<0.15
                        break
                    end
                    %
                end
            end
            
        end
        lin_vel = 0.0;
        velmsg.Linear.X = lin_vel;
        send(robot,velmsg);
        fprintf('pause\n')
        pause(5)
        
        mapInflated = copy(map);
        inflate(mapInflated,0.18); %robot radius is 0.22m
        figure(3),show(mapInflated)
        
        start_location = [x, y];
        end_location = [0.5, 0.5];
        path = []
        [path] = FindLocalPath(mapInflated,start_location, end_location);
        
        init_lin_vel = lin_vel;
        max_ang_vel = 2.84;
        
        path = path.'
        
        for i=2:size(path,2)
            xEnd = path(1,i);
            yEnd = path(2,i);
            fprintf('goin home BOIS\n')
            while true
                lin_vel = 0.15;
                velmsg.Linear.X = lin_vel;
                
                send(robot,velmsg);
                odomdata = receive(odom,3);
                
                [x,y,phi,t] = GetPose(odomdata);% returns x,y,phi,t
                
                [lin_vel,ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel); % returns ang_vel
                
                velmsg.Angular.Z = ang_vel;
                send(robot,velmsg);
                
                % collect odometry readingds (x,y,phi and t) to odomList
                distance = sqrt((xEnd-x)^2+(yEnd-y)^2);
                % stop the robot if the distance between robot and goal is less
                % than 0.1m
                if abs(distance)<0.15
                    break
                end
                %
            end
        end
    else
        predictedLabels = [];
    end
%    lin_vel = 0;
% ang_vel = 0; 
% velmsg.Linear.X = lin_vel; 
% velmsg.Angular.Z = ang_vel; 
% send(robot,velmsg);
%         
% rosshutdown 
end
function path = FindLocalPath(mapInflated,startLocation,endLocation)
% TO DO: Complete the function
%///... write your code here
prm = robotics.PRM; % Create a probabilistic roadmap path planner.
prm.Map = mapInflated; % Assign the inflated map to the PRM object
prm.NumNodes = 1000; % Define the number of PRM nodes to be used during PRM construction.
prm.ConnectionDistance = 0.5; % Define the maximum allowed distance between two connected nodes on the map.


path = findpath(prm, startLocation, endLocation); % Search for a feasible path with the updated PRM.
filepath = 'map_orig.jpg'; % Provide the path of map image file
image = imread(filepath);
figure(1),imshow(image);

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Use black and white image as matrix input for binary occupancy grid.
% Resolution was selected to be 136 by trial and error, specified in cells per meter.
map = robotics.BinaryOccupancyGrid(bwimage,136);
load('map');
%             figure(2),show(map)

mapInflated = copy(map);
inflate(mapInflated,0.19); %robot radius is 0.22m
figure(3),show(mapInflated);

while isempty(path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 50;
    
    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);
    
    % Search for a feasible path with the updated PRM
    path = findpath(prm, startLocation, endLocation);
    
    path1  = path;
end
hold on
plot(path(:,1),path(:,2),'r--d');

figure(5),show(map);
hold on
plot(path(:,1), path(:,2),'r-d')
hold off
%///
end
function predictedLabels = predictNumber(bwcroppedCircle,classifier)

I = imresize(bwcroppedCircle, [16 16]);

testFeatures = extractHOGFeatures(I,'CellSize',[4 4]);

% Make class predictions using the test features.
predictedLabels = predict(classifier, testFeatures);
        
end
function [position, ballSize, boundingBox] = findColorBall(rgbImg)
% This function finds the position of circle using the binarized image.

bwImg = extractColorAreas(rgbImg);
% figure, imshow(bwImg),impixelinfo

blobDetector = vision.BlobAnalysis('MinimumBlobArea',5000);
[~,centroids,boundingBox] = blobDetector(bwImg);

if isempty(boundingBox)
    ballSize = [];
else
    ballSize = boundingBox(1,3);
end
position = centroids;

end
function bwImg = extractColorAreas(img)
% This function converts an RGB image to a binary image based on 
% red/green/blue color ratio thresholds.

% Calculate the ratio of blue (channel 3) to the mean of all channels
colorRatio = double(img(:,:,1)) ./ sum(img,3); 

% Set the binary image based on the threshold below.
colorThresh = 0.6;
bwImg = colorRatio > colorThresh;
end

% function fin_location(predicted_numb)
% 
%     if predicted_numb == 1
%         end_location =[0.75, 2.25];
%     
%     end
%     
%      if predicted_numb == 2
%         end_location =[0.75, 2.25];
%     
%      end
%      if predicted_numb == 3
%         end_location =[0.75, 2.25];
%     
%      end
%      if predicted_numb == 4
%         end_location =[0.75, 2.25];
%     
%     end
%     
% 
% end



