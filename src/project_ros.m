function project_ros()
    % for image stuff: for sim use webcam, for real use usb_cam topic
    %% Ros Setup
    rosinit
    %     L.mlog = {L.DEBUG,'Main','ROS Initialised'}; 
    %% Create and initialise robot
    dobot = DobotMagician();
    dobot.SetRobotOnRail(true);
    dobot.InitialiseRobot();
    % Initialise Rail
    [railStatusPublisher, railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
    railStatusMsg.Data = true;
    send(railStatusPublisher, railStatusMsg);
    [railPosPub, railPosMsg] = rospublisher('/dobot_magician/target_rail_position');
    %%
    position = 0.5;
    railPosMsg.Data = position;
    send(railPosPub, railPosMsg);
    %% Initialise ROS for Arduino eStop
    % rosrun rosserial_python serial_node.py /dev/tty<USB#> - For running
    % arduino serial node
    sub = rossubscriber('/pushed');
    pause(1);
    
    %Estop usage
%     %Arduino eStop
%             msg = receive(sub,0.1);
%             msgData = msg(1).Data;
%             if msgData == 1
%             dobot.EStopRobot();
%                 return %Maybe remove the return?
%             end

    %% Initialse Camera ROS Subscriber
    camSub = rossubscriber('/usb_cam/image_raw');
    pause(1);
    %% Initialise AR Tag ROS Subscriber
    tagSub = rossubscriber('/tags');
    pause(1);

    %% Vision
    compV = ComputerVision.Interface();
    % Get image from camera
    cameraImage = compv.getImage(camSub);
    edges = compV.GetPathPixel(cameraImage);
    
    %% AR Tag detection
    tagMsg = receive(tagSub ,5); %Set to 5 second wait currently ----
    tagData = tagMsg(1).Poses;
    tagSize = size(tagData , 2);
    if tagSize > 1
        tags = tagMsg(1).Markers;
        %Need to sort which ar tag is which based of id ----------
        gameTag = tags(1).pose.pose;
        robotTag = tags(2).pose.pose;
        fprintf('gameTag: %d,%d,%d \n',gameTag.pose.position.x, gameTag.pose.position.y, gameTag.pose.position.z);
        fprintf('robotTag: %d,%d,%d \n',robotTag.pose.position.x, robotTag.pose.position.y, robotTag.pose.position.z);
        % for i=1:tagSize
        %     if tags(i).Id == 1 %------------------------------------
        %         gameTag = tags(i).pose.pose;
        %     elseif tags(i).Id == 2 %--------------------------------
        %         robotTag = tags(i).pose.pose;
        %     end   
        % end    
    end


    %% Path Extraction
    
    %% Run RMRC
    
    %% Move robot
    sz = size(qMatrix,1);
    for i = 1:sz
        joint_target = [qMatrix(i,2) qMatrix(i,3) qMatrix(i,4) qMatrix(i,5)];
        dobot.MoveRailToPosition(qMatrix(i,1));
        dobot.PublishTargetJoint(joint_target);
        pause(0.05);
    end
    %%
    rosshutdown
end