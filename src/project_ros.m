function project_ros()
    % for image stuff: for sim use webcam, for real use usb_cam topic
    %% Ros Setup
    rosinit
    %% Initialse Camera ROS Subscriber
    camSub = rossubscriber('/usb_cam/image_raw');
    pause(1)
    %% Initialise AR Tag ROS Subscriber
    tagSub = rossubscriber('/tags');
    pause(1)
    %% AR Tag detection
    tagMsg = receive(tagSub ,5); %Set to 5 second wait currently ----
    tagData = tagMsg(1).Poses;
    tagSize = size(tagData);
    tagSize = tagSize(2);
    if tagSize > 1
        tags = tagMsg(1).Markers;
        %Need to sort which ar tag is which based of id ----------
        for i=1:tagSize
            if tags(i).Id == 1 %------------------------------------
                wireTag = tags(i).pose.pose;
            elseif tags(i).Id == 2 %--------------------------------
                robotTag = tags(i).pose.pose;
            end
        
        end
        
        %Pose Calculation - Please check this
%        quat = [x y z w]; %CHECK THIS IS CORRECT FORMAT
        wireRotm = quat2rotm([wireTag.orientation.x wireTag.orientation.y wireTag.orientation.z wireTag.orientation.w])
        wireRotm = [wireRotm; ones(1,3)]
        wireRotm = [wireRotm ones(4,1)]
        robotRotm = quat2rotm([robotTag.orientation.x robotTag.orientation.y robotTag.orientation.z robotTag.orientation.w])
        robotRotm = [robotRotm; ones(1,3)]
        robotRotm = [robotRotm ones(4,1)]
        
        %%Find the transforms from each ar tag------------------------
        wire2RobotPose = transl(robotTag.position.x, robotTag.position.y, robotTag.position.z)*robotRotm/transl(wireTag.position.x, wireTag.position.y, wireTag.position.z)*wireRotm;
        robot2WirePose = transl(wireTag.position.x, wireTag.position,y, wireTag.position.z)*wireRotm/transl(robotTag.position.x, robotTag.position.y, robotTag.position.z)*robotRotm;
    end
    %% Get image from camera
    camMsg = receive(camSub,0.1);
    camData = readImage(camMsg);
    %%
    rosshutdown
end