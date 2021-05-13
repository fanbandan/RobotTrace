classdef Controller < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
    end
    properties (Access = protected)
        CVI ComputerVision.Interface
        PEI PathExtraction.Interface
        RMRCI RMRC.Interface
        debug logical = false;
        
        imageMask;
        pathPoints;
    end
    methods (Access = public)
        function self = Controller(debug, rosMode)
            %CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            self.debug = debug;
            self.CVI = ComputerVision.Interface(debug, rosMode);
            self.PEI = PathExtraction.Interface(debug);
            self.RMRCI = RMRC.Interface(self.CVI, debug, rosMode);
        end
        function image = AcquireImageMask(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            image = self.CVI.GetImageMask();
            self.CVI.UpdateARTags();
            self.imageMask = image;
        end
        function [robot,game] = GetPoses(self)
            % Not yet implemented
            robot = self.CVI.GetCamera2RobotTransformationMatrix();
            game = self.CVI.GetCamera2GameTransformationMatrix();
        end
        function GeneratePathPoints(self, zDepthOverride)
            cameraMatrix = self.CVI.GetCameraMatrix();
            if exist('zDepthOverride','var')
                [normal, point] = self.CVI.GetGamePlane(zDepthOverride);
            else
                [normal, point] = self.CVI.GetGamePlane();
            end
            self.PEI.UpdatePathMask(self.imageMask, cameraMatrix, normal, point);
        end
        function GeneratePath(self, downsample, maxDistance, averaging, smoothing)
            startGuess = self.RMRCI.GetRobotPose();
            path = self.PEI.GeneratePath(downsample, startGuess, maxDistance);
            if length(path) < 5
                warning('No path found!');
                return
            end
            self.PEI.GenerateSpline(averaging, smoothing);
        end
        function ShowPath(self)
            self.PEI.PlotSpline(figure(402));
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
    methods (Access = protected)
    end
end

