%% This script allows for the jogging of the Dobot with an XBoz controller
clf;
%% Setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

%% Set up robot

rosinit

[railStatusPublisher, railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
[railPosPub,railPosMsg] = rospublisher('/dobot_magician/target_rail_position');

dobot = DobotMagician();
% Initialise robot
dobot.InitaliseRobot();
% test
dobot.SetRobotOnRail(true);
% Initialise robot
dobot.InitaliseRobot();
% Intialise rail
railStatusMsg.Data = true;
send(railStatusPublisher,railStatusMsg);
% Reinitialise
dobot.InitialiseRobot();
% Move robot in loop
    joint_target = [qMatrix(i,2) qMatrix(i,3) qMatrix(i,4) qMatrix(i,5)];
    dobot.MoveRailToPosition(qMatrix(i,1));
    dobot.PublishTargetJoint(joint_target);
    
% Move to the position of qMatrix(1,i)
railPosMsg.Data = position;
send(railPosPub,railPosMsg);

%% Start "real-time" simulation
q = qCurrent;                 % Set initial robot configuration 'q'

HF = figure(1);         % Initialise figure to display robot
dobot.Animate(q);          % Plot robot in initial configurationq
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 1000;  % Set duration of the simulation (seconds)  
dt = 1;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count
    
    % read joystick
    [axes, buttons, povs] = read(joy);
    
    % Turn joystick input into an end-effector velocity command
    lambda = 0.1;
    I = eye(4);
    
    % Gain values for linear and angular velocity control
    K_lin = 0.02;
    K_ang = 0.05;
    
    vx = K_lin*axes(4); % x
    % wx = K_ang*(buttons(5)-buttons(6)); % roll
    vy = K_lin*axes(2); % y
    % wy = K_ang*(axes(6)-axes(3)); % pitch
    vz = K_lin*axes(1); % z
    wz = K_ang*axes(5); % yaw
    % x = [vx; vy; vz; wx; wy; wz];
    x = [vx; vy; vz; wz];
    
    J = dobot.jacob(q(end,:));
    J(4:5, :) = [];
    J(:,5) = [];

    JDLS = inv(J'*J + lambda*I)*J';
    qdot = inv(JDLS) * x;
    q(5) = [];

    q = q + (qdot * dt)';
    
    % Update plot
    q = [q 0];
    pause(0.01);
    dobot.Animate(q);
    pause(0.01);
        
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n)
        % wait until loop time (dt) has elapsed
    end
end