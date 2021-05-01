classdef Interface < handle
    %PathExtraction Interface - Interface for creating path and XDot for
    %RMRC.
    %   Creates cartesian path from an image in path coordinate frame.
    properties (Access = public)
    end
    properties (Access = private)
        CVI ComputerVision.Interface
    end
    methods (Access = public)
        function self = Interface(cvi)
            self.CVI = cvi;
        end
        function [x] = GetTrajectory(self)
            %GetTrajectory returns the game path in cartesian coordinates.
        end
        function [xDot] = GetXDot(self)
            %GetXDot returns the velocity path for RMRC.
            %   The game path derivate.
            x = GetTrajectory(self);
            deltaTime = 0.1;
            xDot = diff(x) / deltaTime;
        end
    end
    methods (Access = private)
    end
end