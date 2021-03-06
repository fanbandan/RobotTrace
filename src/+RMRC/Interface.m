classdef Interface < handle
    %   RMRC Interface - Interface for motion control of Dobot.
    %   Interface integrates collision detection, dobot, and RMRC movement.
    properties (Access = private)
        CVI ComputerVision.Interface
        dobot RMRC.Dobot
        motion RMRC.ResMotion
        eStop RMRC.EStop
        collision RMRC.Collision
        dobotROS DobotMagician
        deltaT double
        rosMode logical = true;
        debug logical = false;
        execute logical = false;
        initalised logical = false;
        renderCollisions logical = false;
        checkCollisions logical = true;
        path;
        environmentObjects cell = cell(0);
        simHandle;
        pathHandle;
        arGameH;
        arRobotH;
    end
    methods (Access = public)
        function self = Interface(cvi, deltaT, rosMode, debug)
            self.CVI = cvi;
            self.dobot = RMRC.Dobot();
            self.deltaT = deltaT;
            self.motion = RMRC.ResMotion(self.dobot, self.deltaT);
            self.collision = RMRC.Collision(self.dobot);
            if exist('debug','var')
                self.debug = debug;
            end
            if exist('rosMode','var')
                self.rosMode = rosMode;
            end
            
            if self.rosMode == true
                self.dobotROS = DobotMagician();
                self.eStop = RMRC.EStop(self,self.debug);
            else
                self.simHandle = figure();
                gca(self.simHandle);
            end
        end
        function delete(self)
            delete(self.simHandle);
        end
        function Initialise(self)
            % Initialise dobot
            clf(self.simHandle);
            gca(self.simHandle);
            if self.rosMode == true
                self.dobotROS.InitaliseRobot();
                self.dobotROS.SetRobotOnRail(true);
                self.dobotROS.InitaliseRobot();
            else
                self.GenerateSimEnvironment();
            end
            if self.renderCollisions == true
                q = self.GetRobotJoints();
                self.RenderCollisionEllipsoids(q);
            end
            self.initalised = true;
        end
        function [success] = MoveRobot(self,qMatrix)
            % move robot from a provided qMatrix
            success = false;
            for i = 1:size(qMatrix,1)                
                if self.execute == true && self.initalised == true
                    if self.renderCollisions == true
                        self.RenderCollisionEllipsoids(qMatrix(i,:));
                    end
                    if self.checkCollisions == true
                        if self.collision.CheckCollision(qMatrix(i,:)) == true
                            self.Stop();
                            return;
                        end
                    end
                    if self.rosMode == true
                        joint_target = [qMatrix(i,2) qMatrix(i,3) qMatrix(i,4) qMatrix(i,5)];
                        self.dobotROS.MoveRailToPosition(qMatrix(i,1));
                        self.dobotROS.PublishTargetJoint(joint_target);
                    else
                        self.dobot.Animate(qMatrix(i,:));
                        drawnow;
                    end
                    pause(self.deltaT);
                else
                    return
                end
            end
            success = true;
        end
        function Stop(self)
            self.execute = false;
            self.initalised  = false;
            disp("Stopping Robot");
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
        function qlim = GetRobotJointLimits(self)
            qlim = self.dobot.qlim;
        end
        function SetRobotJoints(self,q)
            if self.rosMode == true
                joint_target = [q(2) q(3) q(4) q(5)];
                self.dobotROS.MoveRailToPosition(q(1));
                self.dobotROS.PublishTargetJoint(joint_target);
            else
                self.dobot.Animate(q);
                if self.renderCollisions == true
                    self.RenderCollisionEllipsoids(q);
                end
                drawnow;
            end
        end
        function pose = GetRobotPose(self)
            q = self.GetRobotJoints();
            pose = self.dobot.fkine(q);
        end
        function [success,endPose] = MoveRobotToPose(self, pose)
            q0 = self.GetRobotJoints();
            q = self.dobot.ikcon(pose, q0);
            qMatrix = jtraj(q0,q,10);
            self.execute = true;
            success = self.MoveRobot(qMatrix);
            endPose = self.dobot.fkine(q);
        end
        function ToggleCollisions(self, checkCollisions, renderCollisions)
            self.checkCollisions = checkCollisions;
            self.renderCollisions = renderCollisions;
            if self.renderCollisions == true
                q = self.GetRobotJoints();
                self.RenderCollisionEllipsoids(q)
            else
                self.collision.ClearEllipsoid()
            end
        end
        function UpdatePath(self, path)
            %UpdatePath - Updates the game path
            self.path = path;
            if self.rosMode == false
                self.RenderPath()
            end
        end
        function FollowPath(self)
            %FollowPath - begins RMRC and follows path to completion
            x = self.TranformPath();
            q0 = self.GetRobotJoints();
            qMatrix = self.motion.RateControl(x, q0);
            self.execute = true;
            self.MoveRobot(qMatrix);
        end
        function ShowARTags(self,game,robot)
            if self.rosMode == false
                self.ARRender(game, robot)
            end
        end
    end
    methods (Access = private)
        function GenerateSimEnvironment(self)
            %Spawning in the required models in the required positions and
            %orientations for the simulated environment
            figure(self.simHandle);
            ax = gca(self.simHandle);
            surf(ax, [-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[-0.01,0.01;0.01,0.01]-0.23,'CData',imread([pwd, '//src//+RMRC//Environment//concrete.jpg']),'FaceColor','texturemap'); 
            hold(ax, 'on');
            self.collision.GenerateBasePlane(0,0,-0.01);
              
            EE = RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//endEffector.ply'],transl(0,0,0), [0.4 0.6 0.7]);
            self.dobot.SetItem(EE, transl(0.075,0,0)*troty(pi)*trotx(pi/2)*trscale(0.1))
            
            Fence1 =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//Fence2.ply'],transl(0.5,0.5,0.26), [0.4 0.6 0.7] );
            self.environmentObjects{1} = Fence1;
            Fence2 =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//Fence2.ply'],transl(0.5,-0.5,0.26), [0.4 0.6 0.7] );
            self.environmentObjects{2} = Fence2;
            Table =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//Table.ply'],transl(0.5,0,-0.02), [1 0.3 0.1] );
            self.environmentObjects{3} = Table;
            Camera =  RMRC.EnvironmentObject([pwd, '//src//+RMRC//Environment//camera.ply'],transl(0.3,0.6,0.2)*trotx(pi/2), [0 0 1] );
            self.environmentObjects{4} = Camera;
            
            self.dobot.PlotRobot([-1,1,-1,1,-0.24,1]);
            axis equal;
            view(ax,[30,30]);
        end
        function x = TranformPath(self)
            transform = self.CVI.GetCamera2RobotTransformationMatrix();
            x = NaN(size(self.path));
            for i = 1:size(self.path,3)
                x(:,:,i) = transform * self.path(:,:,i);
            end
        end
        function RenderPath(self)
            % Rendering path of dobot motion into the sim
            ax = gca(self.simHandle);
            hold(ax, 'on');
            try 
                delete(self.pathHandle);
            end
            tpath = self.TranformPath();
            x = reshape(tpath(1,4,:),1,[]);
            y = reshape(tpath(2,4,:),1,[]);
            z = reshape(tpath(3,4,:),1,[]);
            self.pathHandle = plot3(ax, ...
                x, y, z, '-', 'Color', [0.9290 0.6940 0.1250], ...
                'LineWidth', 2, 'DisplayName', 'trajectory');
            
            xlabel(ax,"X");
            ylabel(ax,"Y");
            zlabel(ax,"Z");
        end
        function RenderCollisionEllipsoids(self,q)
            ax = gca(self.simHandle);
            hold(ax, 'on');
            self.collision.ShowEllpsoid(ax, q)
        end
        function ARRender(self, gameTF, robotTF)
            % Function to transform the AR tags
            set(self.arGameH,'Matrix',gameTF);
            set(self.arRobotH,'Matrix',robotTF);
            drawnow;       
        end
        function GenerateCollisionPoints(self)
            self.collision.GeneratePlane();
        end
    end
end