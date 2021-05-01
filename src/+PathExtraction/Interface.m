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
        function self = PathExtractionInterface()
        end
    end
    methods (Access = private)
    end
end