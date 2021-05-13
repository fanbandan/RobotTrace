classdef Controller
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
    end
    properties (Access = protected)
        CVI ComputerVision.Interface
        PEI PathExtraction.Interface
        RMRCI RMRC.Interface
        Debug logical = false;
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
        
        function img = AcquireImageMask(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            img = self.CVI.GetImageMask();
        end
        
        function arTags = GetARTags(self)
            arTags = self.CVI.G
        end
    end
    methods (Access = protected)
    end
end

