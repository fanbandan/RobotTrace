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
        function [M,u,v] = GetPathPixel(self)
            %GetPathPixel returns a mask of pixels containing copper pipe.
            %   [1] pixel at uv coordinate contains copper pipe.
            %   [0] pixel at uv coordinate does not contain copper pipe.
        end
    end
    methods (Access = public)
        %Transformation Matrices
        %   Derived from AR Tags
        function T = GetRobotGameTransformationMatrix(self)
        end
        function T = GetGameRobotTransformationMatrix(self)
        end
        function T = GetCameraGameTransformationMatrix(self)
        end
        function T = GetGameCameraTransformationMatrix(self)
        end
        function T = GetRobotCameraTransformationMatrix(self)
        end
        function T = GetCameraRobotTransformationMatrix(self)
        end
    end
    methods (Access = private)
    end
end