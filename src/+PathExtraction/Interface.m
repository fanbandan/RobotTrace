classdef PathExtractionInterface < handle
    %PathExtractionInterface creates cartesian path from image.
    %   
    properties (Access = public)
    end
    properties (Access = private)
        CVI ComputerVision.Interface
    end
    methods
        function self = PathExtractionInterface()
        end
    end
end