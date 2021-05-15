%% For One AR Tag
clf;

% AR Retreat variables
ARDist = 0.2;
noOfARPoses = 100;

% Get AR Pose array (in 3D coords)
ARPose = zeros(4,4,noOfARPoses);
for i = 1:noOfARPoses
    ARPose(:,:,i) = transl(i/100, 0.15 + 0.1*sin(i*2*pi*0.05), 0.15 +  0.1*cos(i*2*pi*0.05));
end
    
% Create dobot sim
dobot = RMRC.Dobot();
workspace = [-1,1,-1,1,-1,1];
q0 = [0.3, deg2rad([0, 5, 0, 0])];
dobot.PlotRobot(workspace, q0);
hold on
axis equal
TDobot = dobot.fkine(dobot.GetPos());

% Loop to execute safety retreat 
% for loop to loop through AR tag array
for i = 1:noOfARPoses
    % Distance between AR pose and end effector
    xAR = ARPose(1,4,i);
    yAR = ARPose(2,4,i);
    zAR = ARPose(3,4,i);
    xDobot = TDobot(1,4);
    yDobot = TDobot(2,4);
    zDobot = TDobot(3,4);
    currentDistance = sqrt((xDobot - xAR)^2 + (yDobot - yAR)^2 + (zDobot - zAR)^2);
    plot3(xAR,yAR,zAR,'r*');
    
    if currentDistance < ARDist
        % Calculate pose ARDist from the AR tag ----> (T)
        TNewDobotPose = ARPose(:,:,i) * transl(0.5, 0.5, 0);
        % Use dobot.ikcon(T, q0) to find joint pose q
        qNewDobotPose = dobot.ikcon(TNewDobotPose, q0);
        % Animate robot to new q
        dobot.Animate(qNewDobotPose);
    % else do nothing 
    % test 
    elseif currentDistance > ARDist
        % Calculate pose ARDist from the AR tag ----> (T)
        TNewDobotPose = ARPose(:,:,i) * transl(0.5, 0.5, 0);
        % Use dobot.ikcon(T, q0) to find joint pose q
        qNewDobotPose = dobot.ikcon(TNewDobotPose, q0);
        % Animate robot to new q
        dobot.Animate(qNewDobotPose);
    end
    
    q0 = qNewDobotPose;
    TDobot = TNewDobotPose;
    pause(0.1);
end
