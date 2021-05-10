classdef Interface < handle
    %PathExtraction Interface - Interface for creating path and XDot for
    %RMRC.
    %   Creates cartesian path from an image in path coordinate frame.
    properties (Access = public)
    end
    properties (Access = private)
        CVI ComputerVision.Interface
        Path PathExtraction.Path
        debug logical = false;
        
        zNormal;
        zPoint;
    end
    methods (Access = public)
        function self = Interface(cvi, debug)
            self.CVI = cvi;
            self.debug = debug;
            self.Path = PathExtraction.Path(debug);
        end
        function UpdatePath(self, image, CameraMatrix, zNormal, zPoint)
            self.zNormal = zNormal;
            self.zPoint = zPoint;
            [pixelY,pixelX,~] = find(image); % check x and y!
            
            points = Pixels2Points(pixelX, pixelY, CameraMatrix);
            self.Path.UpdatePointCloud(points(1,:),points(2,:),points(3,:));
        end
        function [x] = GetTrajectory(self)
            %GetTrajectory returns the game path in cartesian coordinates.
        end
%         function [xDot] = GetXDot(self)
%             %GetXDot returns the velocity path for RMRC.
%             %   The game path derivate.
%             x = GetTrajectory(self);
%             deltaTime = 0.1;
%             xDot = diff(x) / deltaTime;
%         end
    end
    methods (Access = private)
        function points = Pixels2Points(uMatrix,vMatrix,C)
            points = NaN(3,length(uMatrix));
            invC = inv(C);
            for i = 1:length(uMatrix)
                points(:,i) = self.Pixel2Point(uMatrix(i),vMatrix(i),invC);
            end
        end
        function point = Pixel2Point(u,v,invC)
            w = PixelDepth(u,v);
            pixel = [u;v;w];
            point = invC*pixel;
        end        
        function Z = PixelDepth(u,v)
            x = u;
            y = v;
            Z = self.zPoint(3) + ( ...
                conj(self.zNormal(1))*(self.zPoint(1) - x) + conj(self.zNormal(2))*(self.zPoint(2) - y) ...
                ) / (conj(self.zNormal(3)));
        end
    end
end