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
            if exist('debug','var')
                self.Debug = debug;
            end
            if exist('rosMode','var')
                self.rosMode = rosMode;
            end
            if self.rosMode == true
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
        function [normal, point] = GetGamePlane(self, zDepthOverride)
            if self.rosMode == true
                game = self.ar.GetGamePose();
            else
                game = transl(0,0,0);
                if ~isempty(zDepthOverride)
                    game(3) = zDepthOverride;
                end
            end
            position = transl(game);
            orientation = [1, 0, 0]';
            point = self.GetCameraMatrix() * position';
            point = point';
            normal = orientation';
        end
        function UpdateARTags(self)
            if self.rosMode == true
                self.ar.GetARTags(self);
            end
        end
    end
    methods (Access = public)
        function T = GetCamera2GameTransformationMatrix(self)
            if self.rosMode == true
                T = self.ar.GetGamePose();
            else
                T = transl(0,0,0);
            end
        end
        function T = GetGame2CameraTransformationMatrix(self)
            if self.rosMode == true
                T = inv(self.ar.GetGamePose());
            else
                T = transl(0,0,0);
            end
        end
        function T = GetRobot2CameraTransformationMatrix(self)
            if self.rosMode == true
                T = inv(self.ar.GetRobotPose());
            else
                T = transl(0,0,0);
            end
        end
        function T = GetCamera2RobotTransformationMatrix(self)
            if self.rosMode == true
                T = self.ar.GetRobotPose();
            else
                T = transl(0,0,0);
            end
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