function project_ros()
    % for image stuff: for sim use webcam, for real use usb_cam topic
    %% Ros Setup
    rosinit
    %%
    rosMode = true;
    debug = false;    
    gui = GUI.GUI(rosMode, debug);
    gui.SetManualStartGuess([-0.204,-0.190,1.80]);
    gui.SetPathingParameters(8000, 0.025, 1, 0.6);
    
%     %% Create and initialise robot
%     dobot = DobotMagician();
%     dobot.SetRobotOnRail(true);
%     dobot.InitialiseRobot();
%     % Initialise Rail
%     [railStatusPublisher, railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
%     railStatusMsg.Data = true;
%     send(railStatusPublisher, railStatusMsg);
%     [railPosPub, railPosMsg] = rospublisher('/dobot_magician/target_rail_position');
%     %%
%     position = 0.5;
%     railPosMsg.Data = position;
%     send(railPosPub, railPosMsg);
    %% Initialise ROS for Arduino eStop
    % rosrun rosserial_python serial_node.py /dev/tty<USB#> - For running
    % arduino serial node
%     sub = rossubscriber('/pushed');
%     pause(1);
    
    %Estop usage
%     %Arduino eStop
%             msg = receive(sub,0.1);
%             msgData = msg(1).Data;
%             if msgData == 1
%             dobot.EStopRobot();
%                 return %Maybe remove the return?
%             end

    %%
    rosshutdown
end