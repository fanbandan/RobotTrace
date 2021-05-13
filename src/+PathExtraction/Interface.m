classdef Interface < handle
    %PathExtraction Interface - Interface for creating path and XDot for
    %RMRC.
    %   Creates cartesian path from an image in path coordinate frame.
    properties (Access = public)
    end
    properties (Access = private)
        Path PathExtraction.Path
        Debug logical = false;
        
        zNormal;
        zPoint;
    end
    methods (Access = public)
        function self = Interface(debug)
            self.Path = PathExtraction.Path(debug);
            if ~isempty(debug)
                self.Debug = debug;
            end
        end
        function UpdatePathMask(self, image, CameraMatrix, zNormal, zPoint)
            self.zNormal = zNormal;
            self.zPoint = zPoint;
            [pixelY,pixelX,~] = find(image); % check x and y!
            
            points = self.Pixels2Points(pixelX, pixelY, CameraMatrix);
            self.Path.UpdatePointCloud(points(1,:),points(2,:),points(3,:));
        end
        function GeneratePath(self, downsample, startGuess, maxDistance)
            self.Path.DownsamplePointCloud(downsample);
            self.Path.GeneratePath(startGuess, maxDistance);
        end
        function GenerateSpline(averaging, smoothing)
            if smoothing < 0 || smoothing > 1
                error("Smoothing value must be between 0 and 1");
            end
            if averaging < 1
                error("averaging must be greater than 1");
            end
            self.Path.PathSmoothing(averaging);
            self.Path.SplineFitting(smoothing);
        end
        function [x] = GetTrajectory(self, samples)
            x = self.Path.GetSplinePoints(samples);
        end
    end
    methods (Access = private)
        function points = Pixels2Points(self, uMatrix, vMatrix, C)
            points = NaN(3,length(uMatrix));
            invC = inv(C);
            for i = 1:length(uMatrix)
                points(:,i) = self.Pixel2Point(uMatrix(i),vMatrix(i),invC);
            end
        end
        function point = Pixel2Point(self, u, v, invC)
            w = self.PixelDepth(u,v);
            pixel = [u;v;w];
            point = invC*pixel;
        end        
        function Z = PixelDepth(self, u, v)
            x = u;
            y = v;
            Z = self.zPoint(3) + ( ...
                conj(self.zNormal(1))*(self.zPoint(1) - x) + conj(self.zNormal(2))*(self.zPoint(2) - y) ...
                ) / (conj(self.zNormal(3)));
        end
    end
end