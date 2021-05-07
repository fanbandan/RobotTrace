classdef Dobot < RMRC.Robot
    %LinearUR5 Summary of this class goes here
    %   Detailed explanation goes here
    properties (Access = public)
        qlim;
    end
    properties (Access = private)
        d;
        a;
    end
    methods
        function self = Dobot()
            %UR5 Construct an instance of this class
            %   Detailed explanation goes here            
            self@RMRC.Robot();            
            
            d = zeros(1,5);
            a = zeros(1,5);
            qlim = zeros(5,2);
            
            d(2) = 0.08;
            a(3) = 0.135;
            a(4) = 0.160;
            qlim(1,:) = [0,1];
            qlim(2,:) = deg2rad([-135   135]);
            qlim(3,:) = deg2rad([5      80]);
            qlim(4,:) = deg2rad([-5     85]);
            qlim(5,:) = deg2rad([-85    85]);
            
            self.d = d;
            self.a = a;
            self.qlim = qlim;
        end
        function GetRobot(self)
            %GetRobot
            % Create and return Dobot Magician on linear rail robot model
            pause(0.001);
            name = ['Linear Dobot'];
            
            % Create the dobot model mounted on a linear rail
            % Sourced from https://www.researchgate.net/publication/332145679_Adaptive_tracking_control_of_a_nonlinear_teleoperation_system_with_uncertainties_in_kinematics_and_dynamics
            L(1) = Link([0      0       0       pi/2    1]); % PRISMATIC Link
            L(2) = Link([0      0.08    0       pi/2    0]);
            L(3) = Link([0      0       0.135   0       0]);
            L(4) = Link([0      0       0.160   0       0]);
            L(5) = Link([0      0       0       -pi/2    0]);
            L(6) = Link([0      0       0       0       0]);
            
            % Incorporate joint limits
            L(1).qlim = [0 1]; % PRISMATIC Link
            L(2).qlim = deg2rad([-135   135]);
            L(3).qlim = deg2rad([5      80]);
            L(4).qlim = deg2rad([15     170]);
            L(5).qlim = deg2rad([-180   180]);
            L(6).qlim = deg2rad([-85    85]);
            
            L(2).offset = pi;
            L(3).offset = pi/2;
            L(6).offset = pi;
            
            self.Model = SerialLink(L,'name',name);
            
            % Rotate robot to the correct orientation
            self.Model.base = self.Model.base * trotx(-pi/2); % * troty(pi/2);
        end
        function PlotAndColourRobot(self, workspace, q)
        %PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
            PlotRobot(self, workspace, q)
        end
        function PlotRobot(self, workspace, q)
        %PlotRobot tranforms from real q values to model values
        %   Expects a 1x5 q vector
            if nargin <= 2
                q = zeros(1,self.Model.n);
                q(3) = deg2rad(5);
                q(5) = pi - q(4);
                q(4) = pi/2 - q(3) + q(4);
            else 
                q = [q(1:4), 0, q(5)];
                q(5) = pi - q(4);
                q(4) = pi/2 - q(3) + q(4);
            end
            
            % Display robot
            self.Model.plot(q,'tile1color',[1 1 1],'noarrow','workspace',workspace,'scale',0.25);
            self.Model.delay = 0.0;
        end
        function Animate(self, q)
            q = [q(:,1:4), zeros(size(q,1),1), q(:,5)];
            q(:,5) = pi - q(:,4);
            q(:,4) = pi/2 - q(:,3) + q(:,4);
            self.Model.animate(q);
        end
        function q = GetPos(self)
            q = self.Model.getpos();
            q(4) = q(4) - pi/2 + q(3);
            q = [q(1:4), q(6)];
        end
        function T = fkine(self, q)
            %fkine gets real robot end effector pose for real q values.
            %   Expects a 1x5 q vector
            
%             [valid,q] = self.verifyQ(q);
%             if ~valid
%                 warn('q value outside q limits, defaulting to nearest limit');
%             end
                        
            l = self.a(3)*sin(q(3)) + self.a(4)*cos(q(4)) + self.a(5);
            Z = self.d(2) + self.a(3)*cos(q(3)) - self.a(4)*sin(q(4));
            Y = q(1) + l*sin(q(2));
            X = l*cos(q(2));
            Rz = q(2) + q(5);
            Ry = 0;
            Rx = 0;
            T = transl(X,Y,Z)*trotz(Rz);
        end
        function [qstar, error, exitflag] = ikcon(self, T, q0)
            % check if Optimization Toolbox exists, we need it
            if ~exist('fmincon')
                error('rtb:ikcon:nosupport', 'Optimization Toolbox required');
            end
            
            T_sz = size(T, 3);
            qstar = zeros(T_sz, 5);
            error = zeros(T_sz, 1);
            exitflag = zeros(T_sz, 1);
            
            problem.x0 = zeros(1,5);
            if nargin > 2
                % user passed initial joint coordinates
                problem.x0 = q0;
            end
            problem.options = optimoptions('fmincon', ...
                'Algorithm', 'active-set', ...
                'Display', 'off'); % default options for ikcon
            
            problem.lb = self.qlim(:,1);
            problem.ub = self.qlim(:,2);
            problem.solver = 'fmincon';
            
            reach = sum(abs([self.a, self.d]));
            omega = diag([1 1 1 3/reach]);
            
            for t = 1: T_sz
                problem.objective = ...
                    @(x) sumsqr(((T(:,:,t) \ self.fkine(x)) - eye(4)) * omega);
                
                [q_t, err_t, ef_t] = fmincon(problem);
                
                qstar(t,:) = q_t;
                error(t) = err_t;
                exitflag(t) = ef_t;
                
                problem.x0 = q_t;
            end
        end
        function J = jacob(self, q)
            
%             [valid,q] = self.verifyQ(q);
%             if ~valid
%                 warn('q value outside q limits, defaulting to nearest limit');
%             end
            
            l = self.a(3)*sin(q(3)) + self.a(4)*cos(q(4)) + self.a(5);
            
            J = zeros(6,5);
            
            J(1,:) = [0,-1*sin(q(2)),self.a(3)*cos(q(2))*cos(q(3)),-self.a(4)*cos(q(2))*sin(q(4)),0];
            J(2,:) = [1,l*cos(q(2)),self.a(3)*sin(q(2))*cos(q(3)),-self.a(4)*sin(q(2))*sin(q(4)),0];
            J(3,:) = [0,0,-self.a(3)*sin(q(3)),-self.a(4)*cos(q(4)),0];
            J(4,:) = [0,0,0,0,0];
            J(5,:) = [0,0,0,0,0];
            J(6,:) = [0,1,0,0,1];
        end
        function [valid,q] = verifyQ(self, q)
            valid = true;
            for i = 1:length(q)
                if q(i) < self.qlim(i,1)
                    %q is less than q min.
                    %outside of parameters!!!
                    q(i) = self.qlim(i,1);
                    valid = false;
                end
                if q(i) > self.qlim(i,2)
                    %q is larger than q max;
                    %outside of parameters!!!
                    q(i) = self.qlim(i,2);
                    valid = false;
                end
            end
        end
    end
end

function s = sumsqr(A)
    s = sum(A(:).^2);
end