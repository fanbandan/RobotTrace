classdef Vision < handle
    properties (SetAccess = private)
        % Add a debug variable to spit out data when enabled--------------
        %         debug %Delete----------------------
        CamSub
        Debug logical = false;
    end
    methods (Access = public)
        function self = Vision(debug)
            self.CamSub = rossubscriber('/usb_cam/image_raw');
            if exist('debug','var')
                self.Debug = debug;
            end
        end
        function image = GetImage(self)
            camMsg = receive(self.CamSub, 0.1);
            image = readImage(camMsg);
            if self.Debug == true
                imshow(image);
            end
        end
    end
    methods (Static)
        function maskedRGBImage = colourMask(image)
            
            % Convert RGB image to HSV color space
            I = rgb2hsv(image);
            
            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.000;
            channel1Max = 0.160;
            
            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.050;
            channel2Max = 1.000;
            
            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.726;
            channel3Max = 1.000;
            
            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;
            
            % Initialize output masked image based on input image.
            maskedRGBImage = image;
            
            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

        end   
        %edgeDetection reutrns a Canny Edge Detected image     
        function edgeImage = edgeDetection(image) 
            edgeImage = edge(image(:,:,3),'canny');
        end
    end
end