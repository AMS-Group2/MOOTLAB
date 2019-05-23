clear all, close all

% In Gazebo, go to Edit and select Reset Model Poses (shortcut Shift+Ctrl+R).

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
        
        distance = sqrt((xEnd-x)^2+(yEnd-y)^2);
        % stop the robot if the distance between robot and goal is less
        % than 0.1m
        
        if abs(distance)<0.15
            break
        end
        
    end  % ends while true
end  %For waypoints Statement

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