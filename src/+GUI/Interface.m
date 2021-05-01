classdef Interface < handle
    %GUI Interface - Interface for the GUI
    %   DETAILED DESCRIPTION GOES HERE
    properties (Access = public)
    end
    properties (Access = private)
        RMRCI RMRC.Interface;
    end
    methods (Access = public)
        function self = GUIInterace(rmrci)
            self.RMRCI = rmrci;
        end
        function Launch(self)
            %Launches Matlab .mlapp gui.
        end
    end
    methods (Access = private)
    end
end