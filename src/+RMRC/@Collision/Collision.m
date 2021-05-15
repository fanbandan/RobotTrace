classdef Collision < handle
    %Collision Generic class
    %   Provides generic functionality robot collision detection.
    properties
        qMatrix;
        base;
    end
    methods
        % create ellipses between links
        % fkine to get joint position
        % check collisions between ellipses
        function self = Collision(base)
%             radii = [3,2,1];
            radii = [0.147,0.01,0.01];
            self.base = base;
            %q1 is linear rail therefore not used
            q2 = [self.base dobot.fkine(qMatrix[1])];
            q3 = [dobot.fkine(qMatrix[1]) dobot.fkine(qMatrix[1])];
            q4 = [dobot.fkine(qMatrix[1]) dobot.fkine(qMatrix[1])];
            q5 = [dobot.fkine(qMatrix[1]) dobot.fkine(qMatrix[1])]
            
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
        end
        function setQMatrix(self, qMatrix)
            self.qMatrix = qMatrix;
        end
        function delete(self)
        end
        
    end
end