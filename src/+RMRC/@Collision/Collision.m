classdef Collision < handle
    %Collision Generic class
    %   Provides generic functionality robot collision detection.
    properties
        robot;
        collisionPoints;
    end
    methods
        % create ellipses between links
        % fkine to get joint position
        % check collisions between ellipses
        function self = Collision(robot)
            self.robot = robot;
        end
        function collision = CheckCollision(self, q)
            if isempty(self.collisionPoints)
                collision = false;
                return
            end
            [x(1),y(1),z(1)] = self.robot.LinkKinematics(q(1:1));
            [x(2),y(2),z(2)] = self.robot.LinkKinematics(q(1:2));
            [x(3),y(3),z(3)] = self.robot.LinkKinematics(q(1:3));
            [x(4),y(4),z(4)] = self.robot.LinkKinematics(q(1:4));
            [x(5),y(5),z(5)] = self.robot.LinkKinematics(q(1:5));
            
            tr = zeros(4,4,3);
            radii = [0.1,0.025,0.025];
            
            collision = false;
            
            for i = 1:3
                position = ([x(i+1),y(i+1),z(i+1)] + [x(i),y(i),z(i)])/2;
                dx = x(i+1)-x(i);
                dy = y(i+1)-y(i);
                dz = z(i+1)-z(i);
                L = sqrt(dx^2+dy^2);
                angleZ = atan2(dy,dx);
                angleY = atan2(dz,L);
                T = transl(position(1),position(2),position(3))*trotz(angleZ)*troty(-angleY);
                tr(:,:,i) = T;
                
                cubePointsAndOnes = [inv(tr(:,:,i)) * [self.collisionPoints,ones(size(self.collisionPoints,1),1)]']';
                transformedPoints = cubePointsAndOnes(:,1:3);
                algebraicDist = self.GetAlgebraicDist(transformedPoints, position, radii);
                pointsInside = find(algebraicDist < 1);
                if ~isempty(pointsInside)
                    disp(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                    collision = true;
                    return;
                end
            end
        end
        function GeneratePlane(self)
            %% One side of the cube
            [X,Y] = meshgrid(-1:0.05:1,-1:0.05:1);
            sizeMat = size(Y);
            Z = -0.08*ones(size(Y));
            
            % Combine one surface as a point cloud
            cubePoints = [X(:),Y(:),Z(:)];
            
            % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
            self.collisionPoints = cubePoints;
        end
        function algebraicDist = GetAlgebraicDist(self, points, centerPoint, radii)
            
            
%             [X,Y,Z] = ellipsoid(centerPoint(1),centerPoint(2),centerPoint(3),radii(1),radii(2),radii(3));
%             ellipsoidAtOrigin_h = surf(X,Y,Z);
            
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
        function delete(self)
        end
    end
end