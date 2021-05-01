classdef Interface < handle
    %RMRC Interface - Interface for motion control of Dobot.
    %   Interface integrates collision detection, dobot, and RMRC movement.
    properties (Access = public)
    end
    properties (Access = private)
        PEI PathExtraction.Interface
    end
    methods (Access = public)
        function self = Interface(pei)
            self.PEI = pei;
        end
        function UpdatePath(self)
            %UpdatePath - Updates the game path
        end
        function FollowPath(self)
            %FollowPath - begins RMRC and follows path to completion
        end
    end
    methods (Access = private)
    end
end