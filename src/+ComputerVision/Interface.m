classdef Interface < handle
    %Vision Interface - Interface for recognising game path in camera image feed.
    %   DETAILED DESCRIPTION GOES HERE
    properties (Access = public) %Delete if not used
    end
    properties (Access = private)
        vision Vision
        Debug logical = false;
        ROSMode logical = true;
        gameTag
        gameRotm
        robotTag
        robotRotm
    end
    methods (Access = public)
        function self = Interface(debug, rosMode)
            self.vision = ComputerVision.Vision;
            if ~isempty(debug)
                self.Debug = debug;
            end
            if ~isempty(rosMode)
                self.ROSMode = rosMode;
            end
        end
        function BWImage = GetImageMask(self)
            %GetPathPixel returns a mask of pixels containing copper pipe.
            %   [1] pixel at uv coordinate contains copper pipe.
            %   [0] pixel at uv coordinate does not contain copper pipe.
            image = self.GetImage();
            maskedRGBImage = self.vision.colourMask(image);
            edgeImage = self.vision.edgeDetection(maskedRGBImage);
            BWImage = edgeImage;
        end
    end
    methods (Access = public)
        %Transformation Matrices
        function SetTags(self, gameTag, robotTag)
            self.gameTag = gameTag;
            self.robotTag = robotTag;
            self.gameRotm = quat2rotm([self.gameTag.pose.orientation.x self.gameTag.pose.orientation.y self.gameTag.pose.orientation.z self.gameTag.pose.orientation.w]);
            self.robotRotm = quat2rotm([self.robotTag.pose.orientation.x self.robotTag.pose.orientation.y self.robotTag.pose.orientation.z self.robotTag.pose.orientation.w]);
        end
        function [gameTag , robotTag] = GetTags(self)
            gameTag = self.gameTag;
            robotTag = self.robotTag;
        end
        %Transformation matrix
        % T= [r11 r12 r13 px;
        %     r21 r22 r23 py;
        %     r31 r32 r33 pz;
        %     0    0    0  1;];
        %   Derived from AR Tags - Remember tag coordinate system is different (z is depth)
        function T = GetRobot2GameTransformationMatrix(self)
            rotm = self.gameRotm - self.robotRotm;
            rotm = [rotm; zeros(1,3)];
            transm = [self.gameTag.pose.position.x - self.robotTag.pose.position.x; self.gameTag.pose.position.y - self.robotTag.pose.position.y; self.gameTag.pose.position.z - self.robotTag.pose.position.z;1;];
            T= [rotm transm];
        end
        function T = GetGame2RobotTransformationMatrix(self)
            rotm = self.robotRotm - self.gameRotm;
            rotm = [rotm; zeros(1,3)];
            transm = [self.robotTag.pose.position.x - self.gameTag.pose.position.x; self.robotTag.pose.position.y - self.gameTag.pose.position.y; self.robotTag.pose.position.z - self.gameTag.pose.position.z;1;];
            T= [rotm transm];
        end
        function T = GetCamera2GameTransformationMatrix(self)
            rotm = self.gameRotm;
            rotm = [rotm; zeros(1,3)];
            transm = [self.gameTag.pose.position.x; self.gameTag.pose.position.y; self.gameTag.pose.position.z;1;];
            T= [rotm transm];
        end
        function T = GetGame2CameraTransformationMatrix(self)
            rotm = -self.gameRotm;
            rotm = [rotm; zeros(1,3)];
            transm = [-self.gameTag.pose.position.x; -self.gameTag.pose.position.y; -self.gameTag.pose.position.z;1;];
            T= [rotm transm];
        end
        function T = GetRobot2CameraTransformationMatrix(self)
            rotm =  -self.robotRotm;
            rotm = [rotm; zeros(1,3)];
            transm = [-self.robotTag.pose.position.x; -self.robotTag.pose.position.y; -self.robotTag.pose.position.z;1;];
            T= [rotm transm];
        end
        function T = GetCamera2RobotTransformationMatrix(self)
            rotm = self.robotRotm;
            rotm = [rotm; zeros(1,3)];
            transm = [self.robotTag.pose.position.x; self.robotTag.pose.position.y; self.robotTag.pose.position.z;1;];
            T= [rotm transm];
        end
    end
    methods (Access = private)
        function image = GetImage(self)
            if self.ROSMode == true
                image = self.vision.getImage();
            else
                % Change this to some default image
                image = imread([pwd, '\data\lab_photos\5.jpg']);
            end
        end
    end
end