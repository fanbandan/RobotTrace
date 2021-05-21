classdef ResMotion < handle
    % Resolved Motion Rate Control of Dobot for path
    % Calculates joint positions for the dobot to move through the
    % trajectory
    properties (Access = public)
        
    end
    properties (Access = private)
        % Dobot property
        robot
        
        % Sim parameters
        deltaT                         % Control frequency
    end
    properties (Constant)
        epsilon = 0.1
        WeightedMatrix = diag([[1 1 1] [1 1 1]]); % Weighting matrix for the velocity vector
    end
    methods (Access = public)
        function self = ResMotion(robot,deltaT)
            self.robot = robot;
            self.deltaT = deltaT;
        end
        function [qMatrix,positionError,angleError] = RateControl(self,x,q0)
            steps = length(x);
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qdot = zeros(steps,5);          % Array for joint velocities
            qMatrix = zeros(steps,5);       % Array for joint anglesR
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            qMatrix(1,:) = q0;
            
            % hold on;
%             trplot(x(:,:,1))
%             trplot(x(:,:,end))
            
            for i = 1:steps-1
                T = self.robot.fkine(qMatrix(i,:));                                     % Get forward transformation at current joint state
                deltaX = transl(x(:,:,i+1) - T);                                        % Get position error from next waypoint
                Rd = t2r(x(:,:,i+1));                                                   % Get next RPY angles, convert to rotation matrix
                
                                                                                        
                Ra = t2r(T);                                                            % Current end-effector rotation matrix
                Rdot = (1/self.deltaT)*(Rd - Ra);                                       % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Matrix is skew symmetric
                linear_velocity = (1/self.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = self.WeightedMatrix*[linear_velocity;angular_velocity];          % Calculate end-effector velocity to reach next waypoint.
                J = self.robot.jacob(qMatrix(i,:));                                     % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < self.epsilon                                                  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/self.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(5))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation
                for j = 1:5                                                             % Loop through joints 1 to 5
                    if qMatrix(i,j) + self.deltaT*qdot(i,j) < self.robot.qlim(j,1)      % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                                  % Stop the motor
                    elseif qMatrix(i,j) + self.deltaT*qdot(i,j) > self.robot.qlim(j,2)  % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                                  % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT*qdot(i,:);                  % Update next joint state based on joint velocities
                positionError(:,i) = transl(x(:,:,i+1) - T);                            % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
        end
    end
    methods (Access = private)
        
    end
end