classdef Interface < handle
    %Vision Interface - Interface for recognising game path in camera image feed.
    %   DETAILED DESCRIPTION GOES HERE
    properties (Access = public)
    end
    properties (Access = private)
    end
    methods (Access = public)
        function self = Interface()
        end
        function [M,u,v] = GetPathPixel(self, image)
            %GetPathPixel returns a mask of pixels containing copper pipe.
            %   [1] pixel at uv coordinate contains copper pipe.
            %   [0] pixel at uv coordinate does not contain copper pipe.
            vision = Vision;
            maskedRGBImage = vision.colourMask(image);
            edgeImage = vision.edgeDetection(maskedRGBImage);
            [M,u,v] = edgeImage;
        end
    end
    methods (Access = public)
        %Transformation Matrices
        %   Derived from AR Tags
        function T = GetRobot2GameTransformationMatrix(self)
        end
        function T = GetGame2RobotTransformationMatrix(self)
        end
        function T = GetCamera2GameTransformationMatrix(self)
        end
        function T = GetGame2CameraTransformationMatrix(self)
        end
        function T = GetRobot2CameraTransformationMatrix(self)
        end
        function T = GetCamera2RobotTransformationMatrix(self)
        end
    end
    methods (Access = private)
    end
end