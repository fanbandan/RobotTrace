%% This script allows for the jogging of the Dobot with an XBoz controller

%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information


%% Set up robot
dobot = ...;                    % Load dobot
robot = ...;                   % Create copy called 'robot'
robot.tool = transl(0.1,0,0);   % Define tool frame on end-effector


%% Start "real-time" simulation
q = qn;                 % Set initial robot configuration 'q'

HF = figure(1);         % Initialise figure to display robot
robot.plot(q);          % Plot robot in initial configurationq
robot.delay = 0.001;    % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 1000;  % Set duration of the simulation (seconds)  -------------------------------------- Change this at some stage to fix it ------------------
dt = 0.15;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count
    
    % read joystick
    [axes, buttons, povs] = read(joy);
    
    % Turn joystick input into an end-effector velocity command
    lambda = 0.1;
    I = eye(6);
    
    % Gain values for linear and angular velocity control
    K_lin = 0.3;
    K_ang = 0.8;
    
    vx = K_lin*axes(5); % x
    wx = K_ang*axes(1); % roll
    vy = K_lin*axes(4); % y
    wy = K_ang*axes(2); % pitch
    vz = K_lin*(axes(6)-axes(3)); % z
    wz = K_ang*(button(5)-button(6)); % yaw
    x = [vx; vy; vz; wx; wy; wz];
        
    J = ...jacob0(q(end,:));
    
    % Use J inverse to calculate joint velocity
    qdot = inv(J)*x;
    
    % Apply joint velocity to step robot joint angles
    % q = q + qdot * dt;
    
    JDLS = inv(J'*J + lambda*I)*J';
    qdot = inv(JDLS) * x;
    q = q + (qdot * dt)';
    
    % Update plot
    robot.animate(q);
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n)
     % wait until loop time (dt) has elapsed
    end
end

