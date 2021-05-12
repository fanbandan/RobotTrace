classdef Interface < handle
    %Vision Interface - Interface for recognising game path in camera image feed.
    %   DETAILED DESCRIPTION GOES HERE
    properties (Access = public) %Delete if not used
    end
    properties (Access = private)
        vision Vision
        gameTag
        gameRotm
        robotTag
        robotRotm
    end
    methods (Access = public)
        function self = Interface()            
            self.vision = ComputerVision.Vision;
        end
        function BWImage = GetPathPixel(self, image)
            %GetPathPixel returns a mask of pixels containing copper pipe.
            %   [1] pixel at uv coordinate contains copper pipe.
            %   [0] pixel at uv coordinate does not contain copper pipe.
            maskedRGBImage = self.vision.colourMask(image);
            edgeImage = self.vision.edgeDetection(maskedRGBImage);
            BWImage = edgeImage;
        end
    end
    methods (Access = public)
        %Transformation Matrices
        function SetTags(gameTag, robotTag)
            self.gameTag = gameTag;
            self.robotTag = robotTag;
            self.gameRotm = quat2rotm([self.gameTag.pose.orientation.x self.gameTag.pose.orientation.y self.gameTag.pose.orientation.z self.gameTag.pose.orientation.w])
            self.robotRotm = quat2rotm([self.robotTag.pose.orientation.x self.robotTag.pose.orientation.y self.robotTag.pose.orientation.z self.robotTag.pose.orientation.w])
            
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
            rotm = gameRotm - robotRotm;
            rotm = [rotm; zeroes(1,3)];
            transm = [self.gameTag.pose.position.x - self.robotTag.pose.position.x; self.gameTag.pose.position.y - self.robotTag.pose.position.y; self.gameTag.pose.position.z - self.robotTag.pose.position.z;1;]
            T= [rotm transm];
        end
        function T = GetGame2RobotTransformationMatrix(self)
            rotm = robotRotm - gameRotm;
            rotm = [rotm; zeroes(1,3)];
            transm = [self.robotTag.pose.position.x - self.gameTag.pose.position.x; self.robotTag.pose.position.y - self.gameTag.pose.position.y; self.robotTag.pose.position.z - self.gameTag.pose.position.z;1;]
            T= [rotm transm];
        end
        function T = GetCamera2GameTransformationMatrix(self)
            rotm = gameRotm;
            rotm = [rotm; zeroes(1,3)];
            transm = [self.gameTag.pose.position.x; self.gameTag.pose.position.y; self.gameTag.pose.position.z;1;]
            T= [rotm transm];
        end
        function T = GetGame2CameraTransformationMatrix(self)
            rotm = -gameRotm;
            rotm = [rotm; zeroes(1,3)];
            transm = [-self.gameTag.pose.position.x; -self.gameTag.pose.position.y; -self.gameTag.pose.position.z;1;]
            T= [rotm transm];
        end
        function T = GetRobot2CameraTransformationMatrix(self)
            rotm =  -robotRotm;
            rotm = [rotm; zeroes(1,3)];
            transm = [-self.robotTag.pose.position.x; -self.robotTag.pose.position.y; -self.robotTag.pose.position.z;1;]
            T= [rotm transm];
        end
        function T = GetCamera2RobotTransformationMatrix(self)
            rotm = robotRotm;
            rotm = [rotm; zeroes(1,3)];
            transm = [self.robotTag.pose.position.x; self.robotTag.pose.position.y; self.robotTag.pose.position.z;1;]
            T= [rotm transm];
        end
        function cameraImage = getImage(camSub, debug)
            camMsg = receive(camSub, 0.1);
            cameraImage = readImage(camMsg);
            if debug == true
                imshow(cameraImage);
            end
        end
    end
    methods (Access = private)
    end
end