classdef Controller < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
    end
    properties (Access = protected)
        CVI ComputerVision.Interface
        PEI PathExtraction.Interface
        RMRCI RMRC.Interface
        rosMode logical = true;
        debug logical = false;
        
        image;
        imageMask;
        pathPoints;
        pathStartGuess;
    end
    methods (Access = public)
        function self = Controller(rosMode, debug)
            %CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            deltaT = 0.05;
            if exist('debug','var')
                self.debug = debug;
            end
            if exist('rosMode','var')
                self.rosMode = rosMode;
            end
            self.CVI = ComputerVision.Interface(self.rosMode, self.debug);
            self.PEI = PathExtraction.Interface(self.debug);
            self.RMRCI = RMRC.Interface(self.CVI, deltaT, self.rosMode, self.debug);
            self.pathStartGuess = [0,0,0];
        end
        function Initialise(self)
            self.RMRCI.Initialise();
        end
        function image = AcquireImageMask(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            image = self.CVI.GetImageMask();
            self.image = image;
            self.CVI.UpdateARTags();
            self.imageMask = self.image;
        end
        function [robot,game] = GetPoses(self)
            robot = self.CVI.GetCamera2RobotTransformationMatrix();
            game = self.CVI.GetCamera2GameTransformationMatrix();
            self.RMRCI.ShowARTags(game,robot);
        end
        function points = GeneratePathPoints(self, zDepthOverride)
            cameraMatrix = self.CVI.GetCameraMatrix();
            if exist('zDepthOverride','var')
                [normal, point] = self.CVI.GetGamePlane(zDepthOverride);
            else
                [normal, point] = self.CVI.GetGamePlane();
            end
            if self.debug == true
                figure(104);
            end
            points = self.PEI.UpdatePathMask(self.imageMask, cameraMatrix, normal, point);
        end
        function DownsamplePathPoints(self, downsample)
            self.PEI.DownsamplePoints(downsample);
        end
        function success = GeneratePath(self, maxDistance, averaging, smoothing)
            startGuess = self.pathStartGuess;
            if self.debug == true
                figure(105);
            end
            path = self.PEI.GeneratePath(startGuess, maxDistance);
            
            if length(path) < 5
                warning('No path found!');
                success = false;
                return
            end
            success = true;
            self.PEI.GenerateSpline(averaging, smoothing);
        end
        function UpdatePathStartGuess(self, manualGuess)
            if exist('manualGuess','var')
                self.pathStartGuess = manualGuess;
            else
                self.pathStartGuess = transl(self.RMRCI.GetRobotPose())';
            end
        end
        function startGuess = GetPathStartGuess(self)
            startGuess = self.pathStartGuess;
        end
        function path = GetTrajectory(self, samples)
            path = self.PEI.GetTrajectory(samples);
            self.pathPoints = path;
        end
        function LoadTrajectory(self)
            pathPoses = zeros(4,4,size(self.pathPoints,2));
            for i = 1:size(self.pathPoints,2)
                pathPoses(:,:,i) = transl(self.pathPoints(1,i),self.pathPoints(2,i),self.pathPoints(3,i));
            end
            self.RMRCI.UpdatePath(pathPoses);
        end
        function Run(self)
            self.RMRCI.FollowPath();
        end
        function EStop(self)
            self.RMRCI.Stop();
        end
        function joints = GetDobotJoints(self)
            joints = self.RMRCI.GetRobotJoints();
        end
        function qlim = GetDobotJointLimits(self)
            qlim = self.RMRCI.GetRobotJointLimits();
        end
        function SetDobotJoints(self,q)
            self.RMRCI.SetRobotJoints(q);            
        end
        function [x,y,z] = GetDobotPose(self)
            pose = transl(self.RMRCI.GetRobotPose());
            x = pose(1);
            y = pose(2);
            z = pose(3);
        end
        function [success, endPose] = JogDobot(self, x,y,z)
            pose = transl(x,y,z);
            [success, endPose] = self.RMRCI.MoveRobotToPose(pose);
        end
    end
    methods
        % Debug / Visual methods
        function ShowPathImage(self, ax)
            imshow(self.imageMask, 'Parent', ax);
        end
        function ShowPathPointCloud(self, ax)
            self.PEI.ShowPointCloud(ax);
            hold(ax, 'on');
            scatter3(ax, ...
                self.pathStartGuess(1), self.pathStartGuess(2), self.pathStartGuess(3), ...
                [], [0.4660 0.6740 0.1880], 'o', 'DisplayName', 'start guess');
            hold(ax, 'off');
            legend(ax,'Location','northeast');
        end
        function ShowPathSpline(self, ax)
            self.PEI.PlotSpline(ax);
            legend(ax,'Location','northeast');
        end
    end
    methods (Access = protected)
    end
end

