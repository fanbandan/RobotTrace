%% This script allows for the jogging of the Dobot with an XBoz controller
clf;
%% Setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

%% Set up robot
dobot = RMRC.Dobot();
workspace = [-1,1,-1,1,-1,1];
qn = [0, deg2rad([0, 5, 0, 0])];
% q is optional.
dobot.PlotRobot(workspace,qn);

%% Start "real-time" simulation
q = qn;                 % Set initial robot configuration 'q'

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
    K_lin = 0.3;
    K_ang = 0.8;
    
    vx = K_lin*axes(5); % x
    wx = K_ang*axes(1); % roll
    vy = K_lin*axes(4); % y
    wy = K_ang*axes(2); % pitch
    vz = K_lin*(axes(6)-axes(3)); % z
    wz = K_ang*(buttons(5)-buttons(6)); % yaw
%   x = [vx; vy; vz; wx; wy; wz];
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