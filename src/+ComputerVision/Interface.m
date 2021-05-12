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
            %Pose Calculation - Please check this
            %        quat = [x y z w]; %CHECK THIS IS CORRECT FORMAT
            self.gameRotm = quat2rotm([self.gameTag.orientation.x self.gameTag.orientation.y self.gameTag.orientation.z self.gameTag.orientation.w])
            % gameRotm = [gameRotm; ones(1,3)]
            % gameRotm = [gameRotm ones(4,1)]
            self.robotRotm = quat2rotm([self.robotTag.orientation.x self.robotTag.orientation.y self.robotTag.orientation.z self.robotTag.orientation.w])
            % robotRotm = [robotRotm; ones(1,3)]
            % robotRotm = [robotRotm ones(4,1)]
            
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
        %   Derived from AR Tags
        function T = GetRobot2GameTransformationMatrix(self)
            robot2GamePose = transl(gameTag.position.x, gameTag.position,y, gameTag.position.z)*gameRotm/transl(robotTag.position.x, robotTag.position.y, robotTag.position.z)*robotRotm;
            rotm = gameRotm
            T= [r11 r12 r13 px;
                r21 r22 r23 py;
                r31 r32 r33 pz;
                0    0    0  1;];
        end
        function T = GetGame2RobotTransformationMatrix(self)
            game2RobotPose = transl(robotTag.position.x, robotTag.position.y, robotTag.position.z)*robotRotm/transl(gameTag.position.x, gameTag.position.y, gameTag.position.z)*gameRotm;
            T= [r11 r12 r13 px;
                r21 r22 r23 py;
                r31 r32 r33 pz;
                0    0    0  1;];
        end
        function T = GetCamera2GameTransformationMatrix(self)
            T= [r11 r12 r13 px;
                r21 r22 r23 py;
                r31 r32 r33 pz;
                0    0    0  1;];
        end
        function T = GetGame2CameraTransformationMatrix(self)
            T= [r11 r12 r13 px;
                r21 r22 r23 py;
                r31 r32 r33 pz;
                0    0    0  1;];
        end
        function T = GetRobot2CameraTransformationMatrix(self)
            T= [r11 r12 r13 px;
                r21 r22 r23 py;
                r31 r32 r33 pz;
                0    0    0  1;];
        end
        function T = GetCamera2RobotTransformationMatrix(self)
            T= [r11 r12 r13 px;
                r21 r22 r23 py;
                r31 r32 r33 pz;
                0    0    0  1;];
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