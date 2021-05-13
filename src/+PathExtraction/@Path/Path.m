classdef Path < handle
%Path - Interface for creating path and XDot for
properties (Access = public)
end
properties (Access = private)
    debug logical = false
    pCloud;
    pCloudSource;
    pathPoints;
    spline;
    bounds;
end
methods (Access = public)
    function self = Path(debug)
        if ~isempty(debug)
            self.debug = debug;
        end
    end
    function UpdatePointCloud(self, x, y, z)
        self.pCloud = pointCloud([x;y;z]');
        self.pCloudSource = self.pCloud;
    end
    function ShowPointCloud(self, h)
        figure(h);
        pcshow(self.pCloud);
    end
    function ShowPath(self, h)
        figure(h);
        plot3(self.pathPoints(1,:),self.pathPoints(2,:),self.pathPoints(3,:));
    end
    function DownsamplePointCloud(self,pointNumber)
        %DownsamplePointCloud - downsamples the point cloud for faster
        % trajectory construction. pointNumber sets the final number of
        % points.
        self.pCloud = pcdenoise(self.pCloudSource,'Threshold',0.8,'NumNeighbors',8);
        percentage = pointNumber/length(self.pCloud.Location);
        if percentage > 1
            percentage = 1;
        end
        self.pCloud = pcdownsample(self.pCloud,'random',percentage);
        if self.debug
            size(self.pCloud.Location)
        end
    end
    function GeneratePath(self, startGuess, maxDistance)
        P = startGuess;                    % Start location of trajectory
        L = self.pCloud.Location;           % List of all points
        R = NaN(size(L));               % Final sorted list
        i = 1;
        while ~isempty(L)
            % Get distance between point and all other points;
            [dist,I] = min(pdist2(P,L));
            if dist > maxDistance
                % Max distance between points exceeded!!
                %   Either no points in main path left, or sampling is
                %   too sparce.
                break
            end
            % This is the next point in the trajectory based on
            % distance
            PN = L(I,:);
            % Add this point to the sorted list
            R(i,:) = PN;
            i = i + 1;
            % Point is now connected, so remove it from possible distance list
            L(I,:) = [];
            % Use point for next set of distance measurements
            P = PN;
        end
        % Trim result list, not all points may have been connected.
        self.pathPoints = R(1:(i-1),:)';
    end
    function PathSmoothing(self, averagingNumber)
        x = movmean(self.pathPoints(1,:),averagingNumber);
        y = movmean(self.pathPoints(2,:),averagingNumber);
        z = movmean(self.pathPoints(3,:),averagingNumber);
        
        self.pathPoints = [x;y;z];
    end
    function SplineFitting(self, p)
        t = 1:length(self.pathPoints);
        csx = csaps(t,self.pathPoints(1,:),p);
        csy = csaps(t,self.pathPoints(2,:),p);
        csz = csaps(t,self.pathPoints(3,:),p);
        
        self.spline = [csx;csy;csz];
        self.bounds = [t(1);t(end)];
    end
    function PlotSpline(self, h)
        figure(h);
        t = 1:length(self.pathPoints);
        x = fnval(self.spline(1),t);
        y = fnval(self.spline(2),t);
        z = fnval(self.spline(3),t);
        hold on;
        plot3(self.pathPoints(1,:),self.pathPoints(2,:),self.pathPoints(3,:));
        plot3(x,y,z);
    end
    function points = GetSplinePoints(self, samples)
        t = linspace(self.bounds(1),self.bounds(2), samples);
        
        x = fnval(self.spline(1),t);
        y = fnval(self.spline(2),t);
        z = fnval(self.spline(3),t);
        
        points = [x;y;z];
    end
end
methods (Access = private)
end
end