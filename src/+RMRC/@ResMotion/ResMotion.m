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
        epsilon = 0.22
        WeightedMatrix = diag([1 1 1 0.1 0.1 0.1]); % Weighting matrix for the velocity vector
    end
    methods (Access = public)
        function self = ResMotion(robot,deltaT)
            self.robot = robot;
            self.deltaT = deltaT;
        end
        function [qMatrix,positionError,angleError] = RateControl(self,x,q0)
            % Setting variable sizing and default joint configuration
            steps = length(x);
            m = zeros(steps,1);
            qdot = zeros(steps,5);
            qMatrix = zeros(steps,5);
            positionError = zeros(3,steps);
            angleError = zeros(3,steps);
            qMatrix(1,:) = q0;
            
            for i = 1:steps-1
                % Current joint position
                T = self.robot.fkine(qMatrix(i,:)); 
                % translation error to next waypoint
                deltaX = transl(x(:,:,i+1) - T);
                % Current RPY
                Rd = t2r(x(:,:,i+1));
                %                 Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
                Ra = t2r(T);                                                         
                % Rotation error to next waypoint
                Rdot = (1/self.deltaT)*(Rd - Ra);
                S = Rdot*Ra';                                                           % Matrix is skew symmetric
                linear_velocity = (1/self.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];
                % Velocity calculations
                xdot = self.WeightedMatrix*[linear_velocity;angular_velocity];
                J = self.robot.jacob(qMatrix(i,:));
                % Check Manipulability
                m(i) = sqrt(det(J*J'));
                % Set lambda based of manipulability and the preset epsilon
                if m(i) < self.epsilon
                    lambda = (1 - m(i)/self.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                invJ = inv(J'*J + lambda *eye(5))*J';
                %RMRC equation
                qdot(i,:) = (invJ*xdot)';
                %Account for joint limits
                for j = 1:5
                    if qMatrix(i,j) + self.deltaT*qdot(i,j) < self.robot.qlim(j,1)
                        qdot(i,j) = 0;
                    elseif qMatrix(i,j) + self.deltaT*qdot(i,j) > self.robot.qlim(j,2)
                        qdot(i,j) = 0;
                    end
                end
                % Calculate joint angles
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT*qdot(i,:);
            end
        end
    end
end