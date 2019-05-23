clear all, close all

% Create a ROS node and connect to roscore using rosinit('IP_ADDRESS_OF_VIRTUAL_PC')
rosshutdown
rosinit('10.128.0.100')

% Initial velocities
lin_vel = 0.15; % meters per second 
max_ang_vel = 2.84; % radians per second
init_lin_vel = lin_vel;
% Kp = 1.2; % Sets proportional gain

MaxRange = 1; % max scan range of waffle pi is 3.5m
odomcount = 1; % a counter
odomList = zeros(4,100000); % matrix to collect odom readings
predicted = true

%% Subscribers and publishers
robot = rospublisher('/cmd_vel'); 
velmsg = rosmessage(robot); 
odom = rossubscriber('/pose_fusion');
emptyscan = rosmessage('sensor_msgs/LaserScan');
scansub = rossubscriber('/scan');

%Object Tracking
imgSub = rossubscriber('/raspicam_node/image/compressed');
imgMsg = receive(imgSub);
imgMsg.Format = 'bgr8; jpeg compressed bgr8';
rgbImg = readImage(imgMsg);

r = robotics.Rate(10);

%% Waypoints
waypoints = [];
init_waypoints = [[0.5;0.5] [3.25;0.5] [3;2.55]];
waypoints = init_waypoints;

for i=2:size(waypoints,2)
    xEnd = waypoints(1,i);
    yEnd = waypoints(2,i);
    
    odomdata = receive(odom,3);
    [x,y,phi,t] = GetPose(odomdata);
    waypoint_reached = false;
    while waypoint_reached == false
        [lin_vel, ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel, robot, velmsg);
        odomdata = receive(odom,3);
        [x,y,phi,t] = GetPose(odomdata);
        while (x >= 2) && (y <= 1)
            fprintf('osbt_avoid - x >= 2\n')
            % do obstical avoidance
            obst_avoid(odom,scansub,velmsg,robot,xEnd,yEnd,lin_vel, init_lin_vel,max_ang_vel,odomcount,odomList)
            odomdata = receive(odom,3);
            [x,y,phi,t] = GetPose(odomdata);
            waypoint_reached=true;
            break
        end%While first condition (x >= 2) && (y <= 1)
        
        
        
        while (x >= 2.5) && (y <1.5)
            fprintf('osbt_avoid - x >= 2.5\n')
            % do obstical avoidance
            obst_avoid(odom,scansub,velmsg,robot,xEnd,yEnd,lin_vel, init_lin_vel,max_ang_vel,odomcount,odomList)
            odomdata = receive(odom,3);
            [x,y,phi,t] = GetPose(odomdata);
            waypoint_reached=true;
            break
        end %While second condition (x >= 2.5) && (y <1.5)
        
        
        %         distance = sqrt((xEnd-x)^2+(yEnd-y)^2);
        %             % stop the robot if the distance between robot and goal is less
        %             % than 0.1m
        %
        %             if abs(distance)<0.15
        %                 break
        %             end
        %
    end  % ends while true
end  %For waypoints Statement

%% Look at picture

while (predicted == true)

    while (y > 1.5) && (x>=2.5)

        fprintf('looking @ Piccy\n')

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
            figure(111),imshow(rgbImg);
            predicted = false;
            [predictedLabels,lin_vel,ang_vel]  = trackCircle(rgbImg, params, x, y, robot, velmsg, odom);
%              break
         end
         break
    end % ends while (y > 1.5) && (x>=2.5)
end
fprintf('out of double break')

%%
  if predictedLabels == 1
        endLocation =[0.75, 2.25];
        startLocation = [x, y];
      waypoints =  FindLocalPath(mapInflated, startLocation,endLocation, x, y, odom, robot, velmsg);
       
  end
 
for i=1:size(waypoints,2)
    waypoints
    
    odomdata = receive(odom,3);
    [x,y,phi,t] = GetPose(odomdata);
    
    xEnd = waypoints(i,1);
    yEnd = waypoints(i,2);
    
    while true
        odomdata = receive(odom,3);
        [x,y,phi,t] = GetPose(odomdata);
        [lin_vel, ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel,robot, velmsg);
        velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
        send(robot,velmsg);
        distance = sqrt((xEnd-x)^2+(yEnd-y)^2);
        % stop the robot if the distance between robot and goal is less
        % than 0.1m
        
        if abs(distance)<0.2
           fprintf('Next wayyyypoint')
            break
        end
    end  % ends while true else
end  %For waypoints Statement
% [predictedLabels,lin_vel,ang_vel]  = trackCircle(rgbImg, params, x, y, robot, velmsg, odom);
%% Function: GetPose()
function [x,y,phi,t] = GetPose(odomdata)
x = odomdata.X;
y = odomdata.Y;
%q = odomdata.Pose.Pose.Orientation;
phi = odomdata.Theta; %2*atan2(q.Z,q.W); % get the angle from the quaternion
%         time = odomdata.Header.Stamp;
t = 1;%time.Sec + time.Nsec/1e9;

end
%% Function: Drive forward & P_Controller
function [lin_vel,ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel,robot, velmsg)
% does errror correction for driving in a straight line.
Kp = 1.2;

% fprintf('in P_Control\n\n')

lin_vel = 0.15;
velmsg.Linear.X = lin_vel; 
send(robot,velmsg);

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

error = phi - phi_desired;
error = wrapToPi(error)
ang_vel = -Kp * error;

%make lin_vel proportional to ang_vel
lin_vel = init_lin_vel*(1-(ang_vel)/(max_ang_vel)); 

if lin_vel>init_lin_vel
    lin_vel = init_lin_vel;
end

if abs(ang_vel) > 0.1
    lin_vel = 0.05;
end

end

%% Function: Obstical Avoidance
function obst_avoid(odom,scansub,velmsg,robot,xEnd,yEnd,lin_vel, init_lin_vel,max_ang_vel,odomcount,odomList)
% this will avoid obstacles

fprintf('-- in FUNCTION : osbt avoid -- \n')

while true
    %         odomcount = 1; % a counter
    %         odomList = zeros(4,100000); % matrix to collect odom readings
    %         lin_vel = 0.10; % meters per second
    
    scanMsg = receive(scansub, 10);
    lidarData = lidarScan(scanMsg);
    velmsg.Linear.X = lin_vel;
    send(robot,velmsg);
    
    odomdata = receive(odom,3);
    [x,y,phi,t] = GetPose(odomdata);
    
    % sets up range stuff from lidar
    ranges = lidarData.Ranges;
    angles = lidarData.Angles;
    fwdranges = [ranges(1:30); ranges(331:360)];
    
    % use () brackets
    for i=1:60
        % removes 0s from array
        if fwdranges(i) == 0
            fwdranges(i) = NaN;
        end
    end% ends the for i=1:60
    
    % sets min distances with zeros removed
    [minrangeF, minIndexF] = min(fwdranges);
    minrangeF_current = minrangeF;
    
    if minrangeF_current<0.4
        %collect odometry readingds (x,y,phi and t) to odomList
        odomList(:,odomcount) = [x; y; phi; t];
        odomcount = odomcount + 1;
        
        lin_vel = 0.01;
        velmsg.Linear.X = lin_vel;
        send(robot,velmsg);
        
        if (1 < minIndexF) && (minIndexF <= 30) % to the bots left
            % turn right
            fprintf('turning Right\n')
            
            phi_start=phi-pi/6;
            phi = wrapTo2Pi(phi);
            phi_start = wrapTo2Pi(phi_start);
            
            while (phi>phi_start)
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
            fprintf('end turn right\n')
            [minrangeF, minIndexF] = min(fwdranges);
            minrangeF_current = minrangeF;
            
        end
        
        if (31 <= minIndexF) && (minIndexF <= 60) % to the bots right
            % turn left
            fprintf('turning left\n')
            phi_start=phi+pi/6;
            phi = wrapTo2Pi(phi);
            phi_start = wrapTo2Pi(phi_start);
            
            while (phi < phi_start)
                fprintf('in turn left\n')
                ang_vel = 0.5;
                velmsg.Angular.Z = ang_vel;
                send(robot,velmsg);
                odomdata = receive(odom,3);
                [x,y,phi,t] = GetPose(odomdata);% returns x,y,phi,t
                
                % collect odometry readingds (x,y,phi and t) to odomList
                odomList(:,odomcount) = [x; y; phi; t];
                odomcount = odomcount + 1;
            end % ends (31 <= minIndexF) && (minIndexF <= 60) % to the bots right
            
            fprintf('end turn left\n')
            [minrangeF, minIndexF] = min(fwdranges);
            minrangeF_current = minrangeF;
            odomdata = receive(odom,3);
            [x,y,phi,t] = GetPose(odomdata);
            
        end
    
    
    else
        fprintf('else statement\n')
        lin_vel = 0.1;
        velmsg.Linear.X = lin_vel;
        [lin_vel,ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel,robot, velmsg);
        velmsg.Angular.Z = ang_vel;
        send(robot,velmsg);
        % collect odometry readingds (x,y,phi and t) to odomList
        odomList(:,odomcount) = [x; y; phi; t];
        odomcount = odomcount + 1;
        odomdata = receive(odom,3);
        [x,y,phi,t] = GetPose(odomdata);
        
        distance = sqrt((xEnd-x)^2+(yEnd-y)^2)
        % stop the robot if the distance between robot and goal is less
        % than 0.1m
        
        if abs(distance)<0.15
            fprintf('BREAK\n')
            break
        end % if abs(distance)<0.15
    end %ends if minrange < 0.4
end % ends the while true
end % ends the function: obst_avoid

%% Function: Track Circle    
function [predictedLabels,lin_vel,ang_vel] = trackCircle(rgbImg, params, x, y, robot, velmsg, odom)
    [position,ballSize,boundingBox] = findColorBall(rgbImg);
    
    % Initialize velocities to zero.
    lin_vel = 0;
    ang_vel = 0;
    velmsg.Linear.X = lin_vel;
    velmsg.Angular.Z = ang_vel;
    send(robot,velmsg);
    pause(0.01)
    positionFlag = 0;
    sizeFlag = 0;
    
    % Left and right controls
    if isempty(position)
        ang_vel = 0.0;
        lin_vel = 0;
    elseif (position(1) < (params.width/2)-params.horizontalTolerance)
        ang_vel = 0.3;
          velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
         send(robot,velmsg);
    elseif (position(1) > (params.width/2)+params.horizontalTolerance)
        ang_vel = -0.3;
          velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
         send(robot,velmsg);
    else
        positionFlag = 1;
    end
      
    % Forward and back control
    if isempty(ballSize)
        ang_vel = 0.0;
        lin_vel = 0.0;
    elseif ballSize > params.sizeGoal + params.sizeTolerance
        lin_vel = - 0.05;
          velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
         send(robot,velmsg);
    elseif ballSize < params.sizeGoal - params.sizeTolerance
        lin_vel = 0.05;
          velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
         send(robot,velmsg);
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
        % shows the predicted number in B+W
        figure(100), imshow(bwcroppedCircle)
        predictedLabels = predictNumber(bwcroppedCircle,params.classifier);
        predicted_numb = string(predictedLabels);
        predicted_numb = double(predicted_numb);
        class(predicted_numb);
        fprintf('flags=1')
        predicted_numb
        
        
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
        end_location = [];
        %  call path plan function

       
       % [path] = FindLocalPath(mapInflated, start_location, end_location, x, y);
        
        fprintf('Yep')
        
    else
        predictedLabels = [];
    end
    positionFlag=0;
    sizeFlag=0;
end % function end  

%% Function: Find Final Location Based On Prediction
function final_loc(predictedLabels, x, y, mapInflated, robot, velmsg, odom)
        fprintf('Fin Loc\n')
        
        
     if predictedLabels == 1
        endLocation =[0.75, 2.25];
        startLocation = [x, y];
        FindLocalPath(mapInflated, startLocation,endLocation, x, y, odom, robot, velmsg)
       
    end
    
     if predictedLabels == 2
        end_location =[0.75, 1.25];
        start_location = [x, y];
        FindLocalPath(start_location, end_location);
     end
     
     if predictedLabels == 3
        end_location =[1.25, 1.75];
        start_location = [x, y];
        FindLocalPath(start_location, end_location);
     end
     
     if predictedLabels == 4
        end_location =[1.75, 1.25];
        start_location = [x, y];
        FindLocalPath(start_location, end_location);
     end

end


%% Function: Find Path
function waypoints = FindLocalPath(mapInflated, startLocation,endLocation, x, y, odom, robot, velmsg)
waypoints = [];
fprintf('Local path\n')

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
mapInflated = copy(map);
inflate(mapInflated,0.19); %robot radius is 0.22m
figure(3),show(mapInflated);

prm = robotics.PRM; % Create a probabilistic roadmap path planner.
prm.Map = mapInflated; % Assign the inflated map to the PRM object
prm.NumNodes = 1000; % Define the number of PRM nodes to be used during PRM construction.
prm.ConnectionDistance = 0.5; % Define the maximum allowed distance between two connected nodes on the map.
path = findpath(prm, startLocation, endLocation); % Search for a feasible path with the updated PRM.

% Search for a feasible path with the updated PRM
path = findpath(prm, startLocation, endLocation);
waypoints = [];
waypoints = path;


% w = [x, y]
% fprintf('about to call move to end')
%     final_loc(predicted_numb, x, y, lin_vel, ang_vel);

    
while isempty(path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 50;
    
    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);
    
    % Search for a feasible path with the updated PRM
    path = findpath(prm, startLocation, endLocation);  
end
  hold on
            plot(path(:,1),path(:,2),'r--d');
            
            figure(5),show(map);
            hold on
            plot(path(:,1), path(:,2),'r-d')
            hold off
            
fprintf('about to call move to end')   


end
%% Function: Predicted Labels
  
 function predictedLabels = predictNumber(bwcroppedCircle,classifier)

I = imresize(bwcroppedCircle, [16 16]);

testFeatures = extractHOGFeatures(I,'CellSize',[4 4]);

% Make class predictions using the test features.
predictedLabels = predict(classifier, testFeatures);
        
 end
%% Function: Ball Size

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
%% Function: Extract Colour

function bwImg = extractColorAreas(img)
% This function converts an RGB image to a binary image based on 
% red/green/blue color ratio thresholds.

% Calculate the ratio of blue (channel 3) to the mean of all channels
colorRatio = double(img(:,:,3)) ./ sum(img,3); 

% Set the binary image based on the threshold below.
colorThresh = 0.6;
bwImg = colorRatio > colorThresh;
end
    
%% Function: Move to end

function MoveToEnd(waypoints, odom, robot, velmsg)
fprintf('in move to end')
init_lin_vel = 0.1;
max_ang_vel = 2.84;


for i=1:size(waypoints,2)
    waypoints
    
    odomdata = receive(odom,3);
    [x,y,phi,t] = GetPose(odomdata);
    
    xEnd = waypoints(i,1)
    yEnd = waypoints(i,2)
    
    while true
        odomdata = receive(odom,3);
        [x,y,phi,t] = GetPose(odomdata);
        [lin_vel, ang_vel] = P_Controller(x,y,phi,xEnd,yEnd,init_lin_vel,max_ang_vel,robot, velmsg);
        velmsg.Linear.X = lin_vel;
        velmsg.Angular.Z = ang_vel;
        send(robot,velmsg);
        distance = sqrt((xEnd-x)^2+(yEnd-y)^2);
        % stop the robot if the distance between robot and goal is less
        % than 0.1m
        
        if abs(distance)<0.2
           fprintf('Next wayyyypoint')
            break
        end
    end  % ends while true else
end  %For waypoints Statement



end 

    
    
    
    
    
    
    
    
    
    
    
    
    
    