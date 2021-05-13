classdef Controller < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
    end
    properties (Access = protected)
        CVI ComputerVision.Interface
        PEI PathExtraction.Interface
        RMRCI RMRC.Interface
        Debug logical = false;
        
        imageMask;
        pathPoints;
    end
    methods (Access = public)
        function self = Controller(debug)
            %CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            self.Debug = debug;
            self.CVI = ComputerVision.Interface(debug);
            self.PEI = PathExtraction.Interface(self.CVI, debug);
            self.RMRCI = RMRC.Interface(self.CVI, debug);
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
            [normal, point] = self.CVI.GetGamePlane(self, zDepthOverride);
            self.PEI.UpdatePath(self.imageMask, cameraMatrix, normal, point);
        end
        function GeneratePath(self, downsample, maxDistance, averaging, smoothing)
            startGuess = self.RMRCI.GetRobotPose();
            self.PEI.GeneratePath(downsample, startGuess, maxDistance);
            self.pathPoints = self.PEI.GenerateSpline(averaging, smoothing);
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

