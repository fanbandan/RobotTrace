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
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
            ellipsoidAtOrigin_h = surf(X,Y,Z);
            
            %             % 2.9
            %             q = [0,0,0]
            %             tr = robot.fkine(q);
            %             cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
            %             updatedCubePoints = cubePointsAndOnes(:,1:3);
            %             algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
            %             pointsInside = find(algebraicDist < 1);
            %             display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);
            %
            %             % 2.10
            %             q = [0,0,0]
            %             tr = zeros(4,4,robot.n+1);
            %             tr(:,:,1) = robot.base;
            %             L = robot.links;
            %             for i = 1 : robot.n
            %                 tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            %             end
            
            %             % Go through each ellipsoid
            %             for i = 1: size(tr,3)
            %                 cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
            %                 updatedCubePoints = cubePointsAndOnes(:,1:3);
            %                 algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
            %                 pointsInside = find(algebraicDist < 1);
            %                 display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
            %             end
            
            [x1,y1,z1] = LinkKinematics(qMatrix(1));
            [x2,y2,z2] = LinkKinematics(qMatrix(2));
            [x3,y3,z3] = LinkKinematics(qMatrix(3));
            [x4,y4,z4] = LinkKinematics(qMatrix(4));
            [x5,y5,z5] = LinkKinematics(qMatrix(5));
            
            baseToq2 = [x2,y2,z2] - [x1,y1,z1];
            q2Toq3 = [x3,y3,z3] - [x2,y2,z2];
            q3Toq4 = [x4,y4,z4] - [x3,y3,z3];
            q4Toq5 = [x5,y5,z5] - [x4,y4,z4];
            radii = [1 2 3];
            %             [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            [X,Y,Z] = ellipsoid( q4Toq5(1), q4Toq5(2), q4Toq5(3), radii(1), radii(2), radii(3));
            ellipsoidAtOrigin_h = surf(X,Y,Z);
            % Need to check collisions with table and itself---------------
            %             Go through each ellipsoid
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
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
        function delete(self)
        end
    end
end