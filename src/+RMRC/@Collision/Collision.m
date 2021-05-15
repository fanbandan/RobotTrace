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
            q5 = [dobot.fkine(qMatrix[1]) dobot.fkine(qMatrix[1])];
            
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
            ellipsoidAtOrigin_h = surf(X,Y,Z);
            
            % 2.9
            q = [0,0,0]
            tr = robot.fkine(q);
            cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
            updatedCubePoints = cubePointsAndOnes(:,1:3);
            algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
            pointsInside = find(algebraicDist < 1);
            display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);
            
            % 2.10
            q = [0,0,0]
            tr = zeros(4,4,robot.n+1);
            tr(:,:,1) = robot.base;
            L = robot.links;
            for i = 1 : robot.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            
            % Go through each ellipsoid
            for i = 1: size(tr,3)
                cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
                pointsInside = find(algebraicDist < 1);
                display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
            end
        end
        function setQMatrix(self, qMatrix)
            self.qMatrix = qMatrix;
        end
        function delete(self)
        end
    end
end