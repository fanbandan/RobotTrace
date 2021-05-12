function demo()
%%
rosinit
%%
dobot = DobotMagician();
%% Initialise robot
dobot.InitaliseRobot();
%% test
dobot.SetRobotOnRail(true);
%% Initialise robot
dobot.InitaliseRobot();
%% Intialise rail
[railStatusPublisher, railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
railStatusMsg.Data = true;
send(railStatusPublisher,railStatusMsg);
%% Initialise rail
position = 0.5 % Move to the position of 0.5
[railPosPub,railPosMsg] = rospublisher('/dobot_magician/target_rail_position');
railPosMsg.Data = position;
send(railPosPub,railPosMsg);

%% Publish custom joint target
sz = size(qMatrix,1); % n by 5
% sz = sz(1);
for i = 1:sz
    joint_target = [qMatrix(i,2) qMatrix(i,3) qMatrix(i,4) qMatrix(i,5)];
    dobot.MoveRailToPosition(qMatrix(i,1));
    dobot.PublishTargetJoint(joint_target);
    pause(0.05);
end
%% EStop - Only use if roobt is in motion
dobot.EStopRobot();

%%
rosshutdown
end

