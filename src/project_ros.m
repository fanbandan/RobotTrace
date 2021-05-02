function project_ros()
    % for image stuff: for sim use webcam, for real use usb_cam topic
    %% Ros Setup
    rosinit
    %% Initialise Arduino ROS Subscriber 
    % Running this in rosserial might cause errors since dobot uses it too
    arduinoSub = rossubscriber('/pushed');
    pause(1)
    %% Initialse Camera ROS Subscriber
    camSub = rossubscriber('/usb_cam/image_raw');
    pause(1)
    %% Initialise AR Tag ROS Subscriber
    tagSub = rossubscriber('/ar_pose_marker');
    pause(1)
    %% Arduino eStop
    arduinoMsg = receive(arduinoSub,0.1);
    arduinoData = arduinoMsg(1).Data;
    if arduinoData == 1
        return
    end
    %% AR Tag detection
    tagMsg = receive(tagSub ,5); %Set to 5 second wait currently ----
    tagData = tagMsg(1).Data;
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
        
        %%Find the transforms from each ar tag------------------------
        wire2RobotPose;
        robot2WirePose;
    end
    %% Get image from camera
    camMsg = receive(camSub,0.1); %Maybe change wait amount?--------------
    camData = camMsg(1).Data.image; %it's either image or data
    %for testing
%     imshow(camData)
    %%
    rosshutdown
end