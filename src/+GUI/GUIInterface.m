classdef GUIInterace < handle
    properties (PropertyAttributes)
        Move RMRC.RMRCInterface;
    end
    methods (Access = public)
        function self = GUIInterace(move)
            self.Move = move;
        end
        function Launch(self)
            %Launches Matlab .mlapp s
        end
    end
end