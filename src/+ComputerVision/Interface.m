classdef Interface < handle
    %Vision Interface - Interface for recognising game path in camera image feed.
    %   DETAILED DESCRIPTION GOES HERE
    properties (Access = public) %Delete if not used
    end
    properties (Access = private)
        vision Vision
        ar AR
        Debug logical = false;
        rosMode logical = true;
        gameTag
        gameRotm
        robotTag
        robotRotm
    end
    methods (Access = public)
        function self = Interface(debug, rosMode)
            if ~isempty(debug)
                self.Debug = debug;
            end
            if ~isempty(rosMode)
                self.rosMode = rosMode;
            end
            if self.rosMode == false
                self.vision = ComputerVision.Vision(debug);
                self.ar = ComputerVision.AR(debug);
            end
        end
        function BWImage = GetImageMask(self)
            %GetPathPixel returns a mask of pixels containing copper pipe.
            %   [1] pixel at uv coordinate contains copper pipe.
            %   [0] pixel at uv coordinate does not contain copper pipe.
            image = self.GetImage();
            maskedRGBImage = ComputerVision.Vision.colourMask(image);
            edgeImage = ComputerVision.Vision.edgeDetection(maskedRGBImage);
            BWImage = edgeImage;
        end
        function [cameraMatrix] = GetCameraMatrix(self)
            cameraMatrix = [ ...
                665.578756,    0,          282.225564; ...
                0,              664.605455, 260.138094; ...
                0,              0,          1; ...
                ];
        end
        function [normal, point] = GetGamePlane(self)
            game = self.ar.GetGamePose();
            position = transl(game);
            orientation = [1, 0, 0]';
            point = self.GetCameraMatrix() * position';
            point = point';
            normal = orientation';
        end
        function UpdateARTags(self)
            self.ar.GetARTags(self);
        end
    end
    methods (Access = public)
        function T = GetCamera2GameTransformationMatrix(self)
            T = self.ar.GetGamePose();
        end
        function T = GetGame2CameraTransformationMatrix(self)
            T = inv(self.ar.GetGamePose());
        end
        function T = GetRobot2CameraTransformationMatrix(self)
            T = inv(self.ar.GetRobotPose());
        end
        function T = GetCamera2RobotTransformationMatrix(self)
            T = self.ar.GetRobotPose();
        end
    end
    methods (Access = private)
        function image = GetImage(self)
            if self.rosMode == true
                image = self.vision.getImage();
            else
                % Change this to some default image
                image = imread([pwd, '\data\lab_photos\5.jpg']);
            end
        end
    end
end