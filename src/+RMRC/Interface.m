classdef Interface < handle
    %RMRC Interface - Interface for motion control of Dobot.
    %   Interface integrates collision detection, dobot, and RMRC movement.
    properties (Access = public)
        
    end
    properties (Access = private)
        CVI ComputerVision.Interface
        dobot RMRC.Dobot
        motion RMRC.ResMotion
        dobotROS DobotMagician
        deltaT double
        rosMode logical = true;
        debug logical = false;
        execute logical = false;
        initalised logical = false;
        path;
        environmentObjects cell = cell(0);
    end
    methods (Access = public)
        function self = Interface(cvi, deltaT, rosMode, debug)
            self.CVI = cvi;
            self.dobot = RMRC.Dobot();
            self.deltaT = deltaT;
            self.motion = RMRC.ResMotion(self.dobot, self.deltaT);
            if exist('debug','var')
                self.debug = debug;
            end
            if exist('rosMode','var')
                self.rosMode = rosMode;
            end
            if self.rosMode == true
                self.dobotROS = DobotMagician();
            end
        end
        function Initialise(self)
            if self.rosMode == true
                self.dobotROS.InitaliseRobot();
                self.dobotROS.SetRobotOnRail(true);
                self.dobotROS.InitaliseRobot();
            else
                self.GenerateSimEnvironment();
            end
            self.initalised = true;
        end
        function MoveRobot(self,qMatrix)
            for i = 1:size(qMatrix,1)
                if self.execute == true && self.initalised == true
                    if self.rosMode == true
                        joint_target = [qMatrix(i,2) qMatrix(i,3) qMatrix(i,4) qMatrix(i,5)];
                        self.dobotROS.MoveRailToPosition(qMatrix(i,1));
                        self.dobotROS.PublishTargetJoint(joint_target);
                    else
                        self.dobot.Animate(qMatrix(i,:));
                    end
                    pause(self.deltaT);
                end
            end
        end
        function Stop(self)
            self.execute = false;
            self.initalised  = false;
            if self.rosMode == true
                self.dobotROS.EStopRobot();
            end
        end
        function q = GetRobotJoints(self)
            if self.rosMode == true
                q1 = self.dobotROS.GetCurrentRailPos();
                q2 = self.dobotROS.GetCurrentJointState();
                q = [q1, q2'];
            else
                q = self.dobot.GetPos();
            end
        end
        function pose = GetRobotPose(self)
            q = self.GetRobotJoints();
            pose = self.dobot.fkine(q);
        end
        function UpdatePath(self, path)
            %UpdatePath - Updates the game path
            self.path = path;
        end
        function FollowPath(self)
            %FollowPath - begins RMRC and follows path to completion
            transform = self.CVI.GetCamera2RobotTransformationMatrix();
            x = self.path .* transform;
            q0 = self.GetRobotJoints();
            qMatrix = self.motion.RateControl(x, q0);
            self.execute = true;
            self.MoveRobot(qMatrix);
        end
    end
    methods (Access = private)
        function GenerateSimEnvironment(self)
            surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[-0.01,0.01;0.01,0.01]-0.23,'CData',imread([pwd, '//src//+RMRC//Environment//concrete.jpg']),'FaceColor','texturemap'); 
            hold on;
            Fence1 =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//Fence2.ply'],transl(0.5,0.5,0.26), [0.4 0.6 0.7] );
            self.environmentObjects{1} = Fence1;
            Fence2 =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//Fence2.ply'],transl(0.5,-0.5,0.26), [0.4 0.6 0.7] );
            self.environmentObjects{2} = Fence2;
            Table =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//Table.ply'],transl(0.5,0,0), [1 0.3 0.1] );
            self.environmentObjects{3} = Table;
            self.dobot.PlotRobot([-1,1,-1,1,-1,1]);
            axis equal;
            view(30,30);

        end
    end
end