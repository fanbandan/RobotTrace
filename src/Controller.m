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
            deltaT = 0.2;
            if exist('debug','var')
                self.debug = debug;
            end
            if exist('rosMode','var')
                self.rosMode = rosMode;
            end
            self.CVI = ComputerVision.Interface(self.rosMode, self.debug);
            self.PEI = PathExtraction.Interface(self.debug);
            self.RMRCI = RMRC.Interface(self.CVI, deltaT, self.rosMode, self.debug);
        end
        function image = AcquireImageMask(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            self.image = self.CVI.GetImageMask();
            self.CVI.UpdateARTags();
            self.imageMask = self.image;
        end
        function [robot,game] = GetPoses(self)
            % Not yet implemented
            robot = self.CVI.GetCamera2RobotTransformationMatrix();
            game = self.CVI.GetCamera2GameTransformationMatrix();
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
                self.pathStartGuess = self.RMRCI.GetRobotPose();
            end
        end
        function path = GetTrajectory(self, samples)
            path = self.PEI.GetTrajectory(samples);
            self.pathPoints = path;
        end
        function LoadTrajectory(self)
            pathPoses = zeros(4,4,length(self.pathPoints));
            for i = 1:length(self.pathPoints)
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
    end
    methods
        % Debug / Visual methods
        function ShowPathImage(self, ax)
            imshow(self.imageMask, 'Parent', ax);
        end
        function ShowPathPointCloud(self, ax)
            self.PEI.ShowPointCloud(ax);
        end
        function ShowPathSpline(self, ax)
            self.PEI.PlotSpline(ax);
        end
    end
    methods (Access = protected)
    end
end

