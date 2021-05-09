classdef Interface < handle
    %Vision Interface - Interface for recognising game path in camera image feed.
    %   DETAILED DESCRIPTION GOES HERE
    properties (Access = public)
    end
    properties (Access = private)
        vision Vision
    end
    methods (Access = public)
        function self = Interface()
        end
        function BWImage = GetPathPixel(self, image)
            %GetPathPixel returns a mask of pixels containing copper pipe.
            %   [1] pixel at uv coordinate contains copper pipe.
            %   [0] pixel at uv coordinate does not contain copper pipe.
            vision = Vision;
            maskedRGBImage = vision.colourMask(image);
            edgeImage = vision.edgeDetection(maskedRGBImage);
            BWImage = edgeImage;
        end
    end
    methods (Access = public)
        %Transformation Matrices
        %   Derived from AR Tags
        function T = GetRobot2GameTransformationMatrix(self)
            %Pose Calculation - Please check this
            %        quat = [x y z w]; %CHECK THIS IS CORRECT FORMAT
            wireRotm = quat2rotm([wireTag.orientation.x wireTag.orientation.y wireTag.orientation.z wireTag.orientation.w])
            wireRotm = [wireRotm; ones(1,3)]
            wireRotm = [wireRotm ones(4,1)]
            robotRotm = quat2rotm([robotTag.orientation.x robotTag.orientation.y robotTag.orientation.z robotTag.orientation.w])
            robotRotm = [robotRotm; ones(1,3)]
            robotRotm = [robotRotm ones(4,1)]
            
            %%Find the transforms from each ar tag------------------------
            wire2RobotPose = transl(robotTag.position.x, robotTag.position.y, robotTag.position.z)*robotRotm/transl(wireTag.position.x, wireTag.position.y, wireTag.position.z)*wireRotm;
            robot2WirePose = transl(wireTag.position.x, wireTag.position,y, wireTag.position.z)*wireRotm/transl(robotTag.position.x, robotTag.position.y, robotTag.position.z)*robotRotm;
        end
        function T = GetGame2RobotTransformationMatrix(self)
        end
        function T = GetCamera2GameTransformationMatrix(self)
        end
        function T = GetGame2CameraTransformationMatrix(self)
        end
        function T = GetRobot2CameraTransformationMatrix(self)
        end
        function T = GetCamera2RobotTransformationMatrix(self)
        end
    end
    methods (Access = private)
    end
end