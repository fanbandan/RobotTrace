classdef Interface < handle
    %RMRC Interface - Interface for motion control of Dobot.
    %   Interface integrates collision detection, dobot, and RMRC movement.
    properties (Access = public)
        
    end
    properties (Access = private)
        CVI ComputerVision.Interface
        dobot RMRC.Dobot
        motion RMRC.ResMotion
        dobotROS DobotMagician
        rosMode logical = true;
        debug logical = false;
        path;
    end
    methods (Access = public)
        function self = Interface(cvi, deltaT, debug, rosMode)
            self.CVI = cvi;
            self.dobot = RMRC.Dobot();
            self.motion = RMRC.ResMotion(self.dobot, deltaT);
            if ~isempty(debug)
                self.debug = debug;
            end
            if ~isempty(rosMode)
                self.rosMode = rosMode;
            end
            if self.rosMode == true
                self.dobotROS = DobotMagician();
            end
        end
        function GetRobotPose(self)
            
        end
        function UpdatePath(self, path)
            %UpdatePath - Updates the game path
            self.path = path;
        end
        function FollowPath(self)
            %FollowPath - begins RMRC and follows path to completion
            transform = self.CVI.GetCamera2RobotTransformationMatrix();
            
        end
    end
    methods (Access = private)
    end
end