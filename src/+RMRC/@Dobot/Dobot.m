classdef Dobot < RMRC.Robot
    %LinearUR5 Summary of this class goes here
    %   Detailed explanation goes here
    methods
        function self = Dobot()
            %UR5 Construct an instance of this class
            %   Detailed explanation goes here
            self@RMRC.Robot();
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
            
            % Incorporate joint limits
            L(1).qlim = [0 1]; % PRISMATIC Link
            L(2).qlim = deg2rad([-90    90]);
            L(3).qlim = deg2rad([0      85]);
            L(4).qlim = deg2rad([-10    95]);
            
            L(2).offset = pi;
            L(3).offset = pi/2;
            
            self.Model = SerialLink(L,'name',name);
            
            % Rotate robot to the correct orientation
            self.Model.base = self.Model.base * trotx(-pi/2); % * troty(pi/2);
        end
        function PlotAndColourRobot(self, workspace, q)
        %PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
            if nargin <= 2
                q = zeros(1,self.Model.n);
            end
            for linkIndex = 0:self.Model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'.ply'],'tri');
                self.Model.faces{linkIndex+1} = faceData;
                self.Model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.Model.plot3d(q,'tile1color',[1 1 1],'noarrow','workspace',workspace);
            self.Model.delay = 0.0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.Model.n
                handles = findobj('Tag', self.Model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        function PlotRobot(self, workspace, q)
        %PlotRobot
            if nargin <= 2
                q = zeros(1,self.Model.n);
            end
            
            % Display robot
            self.Model.plot(q,'tile1color',[1 1 1],'noarrow','workspace',workspace,'scale',0.5);
            self.Model.delay = 0.0;
        end
    end
end