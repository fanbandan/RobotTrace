classdef ResMotion < handle
    % Resolved Motion Rate Control of Dobot for path
    % Calculates joint positions for the dobot to move through the
    % trajectory
    properties (Access = public)
        qMatrix = zeros(steps,3)       % Array for joint anglesR
    end
    properties (Access = private)
        % Path Extraction properties
        xDot
        traj
        
        % Dobot property
        dobot
        
        % Sim parameters
        t                              % Total time (s)
        deltaT                         % Control frequency
        steps = t/deltaT               % Number of steps
        delta = 2*pi/steps             % Angle change
        T                              % End Effector pose
        q0 = zeros(1,3)                % Initial joint angles
        lambda                         % Damping coefficient
        angularVelocity
        linearVelocity
        J                              % Joint Jacobian
        
        % Array data
        m = zeros(steps,1)             % Array for Measure of Manipulability
        qdot = zeros(steps,3)          % Array for joint velocities
        theta = zeros(3,steps)         % Array for roll-pitch-yaw angles
        x = zeros(3,steps)             % Array for x-y-z trajectory
        positionError = zeros(3,steps) % For plotting trajectory error
        angleError = zeros(3,steps)    % For plotting trajectory error
    end
    properties (Constant)
        epsilon = 0.1
        W = diag([1 1 1 0.1 0.1 0.1]); % Weighting matrix for the velocity vector
    end
    methods (Access = public)
        function self = ResMotion(x_dot, traj, dobot)
            self.xDot = x_dot;
            self.dobot = dobot;
            self.traj = traj;
        end
        function qMat = RateControl(self)
            self.T =  [rpy2r(self.theta(1,1), self.theta(2,1), self.theta(3,1)) self.x(:,1);zeros(1,3) 1];            % Create transformation of first point and angle
            self.qMatrix(1,:) = self.dobot.model.ikcon(self.T, self.q0); % Replace ikcon               % Solve joint angles to achieve first waypoint
            
            % Track the trajectory with RMRC
            for i = 1:self.steps-1
                self.T = self.dobot.model.fkine(self.qMatrix(i,:));   %  Replace fkine               % Get forward transformation at current joint state
                deltaX = self.x(:,i+1) - self.T(1:3,4);            % Get position error from next waypoint
                Rd = rpy2r(self.theta(1,i+1), self.theta(2,i+1), self.theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
                Ra = self.T(1:3,1:3);                 % Current end-effector rotation matrix
                
                Rdot = (1/self.deltaT)*(Rd - Ra);       % Calculate rotation matrix error
                S = Rdot*Ra';                      % Skew symmetric! S(\omega)
                self.linearVelocity = deltaX / self.deltaT;
                self.angularVelocity = [S(3,2);S(1,3);S(2,1)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities
                
                deltaTheta = tr2rpy(Rd*Ra');% Convert rotation matrix to RPY angles
                self.J = self.dobot.model.jacob0(self.qMatrix(i,:));   % Replace Jacob0                             % Get Jacobian at current joint state
                
                if self.m(i) < self.epsilon  % If manipulability is less than given threshold
                    self.lambda = (1 - self.m(i) / self.epsilon) * 0.05; % Damping coefficient
                else
                    self.lambda = 0;
                end
                
                invJ = inv(self.J' * self.J + self.lambda * eye(3)) * self.J';
                self.qdot(i,:) = (invJ * self.xDot)'; % Solve the RMRC equation
                for j = 1:3 % Loop through joints 1 to 3
                    if self.qMatrix(i, j) + self.deltaT * self.qdot(i, j) < self.dobot.model.qlim(j, 1) % If next joint angle is lower than joint limit...
                        self.qdot(i, j) = 0; % Stop the motor
                    elseif self.qMatrix(i, j) + self.deltaT * self.qdot(i, j) > self.dobot.model.qlim(j, 2) % If next joint angle is greater than joint limit ...
                        self.qdot(i, j) = 0; % Stop the motor
                    end
                end
                self.qMatrix(i+1,:) = self.qMatrix(i, :) + self.deltaT * self.qdot(i, :); % Update next joint state based on joint velocities
                self.positionError(:,i) = self.x(:, i + 1) - self.T(1:3 , 4);  % For plotting
                self.angleError(:,i) = deltaTheta; % For plotting
            end
        end
        % How do I move the dobot???
    end
    methods (Access = private)
        
    end
end