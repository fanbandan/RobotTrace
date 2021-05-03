classdef ResMotion < handle
    % Resolved Motion Rate Control of Dobot for path
    % Calculates joint positions for the dobot to move through the
    % trajectory
    properties (Access = public)
        qMatrix = zeros(steps,3)       % Array for joint anglesR
    end
    properties (Access = private)
        % Path Extraction properties
        xDot PathExtraction.GetXDot
        traj PathExtraction.GetTrajectory
        
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
        function self = ResMotion(x_dot, dobot)
            self.xDot = x_dot;
            self.dobot = dobot;
        end
        function qMat = RateControl(self)
            self.T = ...             % Create transformation of first point and angle
                self.q0 = ...            % Initial guess for joint angles
                self.qMatrix(1,:) = ...  % Solve joint angles to achieve first waypoint
                F
            % Track the trajectory with RMRC
            for i = 1:self.steps-1
                self.T = ...                 % Get forward transformation at current joint state
                    deltaX = ...            % Get position error from next waypoint
                    Rd = rpy2r(...);      % Get next RPY angles, convert to rotation matrix
                    Ra = ...                % Current end-effector rotation matrix
                    
                Rdot = (1/self.deltaT)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
                S = ....;                      % Skew symmetric! S(\omega)
                    self.linearVelocity = (1/deltaT)*deltaX;
                self.angularVelocity = [S(?,?);S(?,?);S(?,?)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)
                
                deltaR = ...        	% Calculate rotation matrix error
                    deltaTheta = tr2rpy(...);% Convert rotation matrix to RPY angles
                    J = ...                 % Get Jacobian at current joint state
                    
                if ...  % If manipulability is less than given threshold
                        self.lambda = ... % Damping coefficient (try scaling it)
                        invJ = ... % Apply Damped Least Squares pseudoinverse
                else
                invJ = ... % Don't use DLS
                    end
                
                self.qdot(i,:) = ... % Solve the RMRC equation (you may need to transpose the         vector)
                    for ... % Loop through joints 1 to 6
                        if ... % If next joint angle is lower than joint limit...
                            ... % Stop the motor
                        elseif ... % If next joint angle is greater than joint limit ...
                            ... % Stop the motor
                        end
                    end
                    self.qMatrix(i+1,:) = ... % Update next joint state based on joint velocities
                        self.m(i) = ...  % Record manipulability
                        self.positionError(:,i) = deltaX;  % For plotting
                    self.angleError(:,i) = deltaTheta; % For plotting
                end
                
            end
            % How do I move the dobot???
        end
        methods (Access = private)
            
        end
    end