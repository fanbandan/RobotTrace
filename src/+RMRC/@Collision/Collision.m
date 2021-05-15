classdef Collision < handle
    %Collision Generic class
    %   Provides generic functionality robot collision detection.
    properties
        qMatrix;
    end
    methods
        % create ellipses between links
        % fkine to get joint position
        % check collisions between ellipses
        function self = Collision()
            radii = [3,2,1];
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
        end
        function setQMatrix(self, qMatrix)
            self.qMatrix = qMatrix;
        end
        function delete(self)
        end
        
    end
end