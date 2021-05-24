classdef Collision < handle
    %Collision provides generic functionality robot collision detection.
    properties
        ellipsoidHandle;
        pointsHandle;
    end

    properties (Access = private)
        robot;
        collisionPoints;
        debug logical = false;
    end

    methods
        % create ellipses between links
        % fkine to get joint position
        % check collisions between ellipses
        function self = Collision(robot, debug)
            self.robot = robot;

            if exist('debug', 'var')
                self.debug = debug;
            end

        end

        function collision = CheckCollision(self, q)
            %CheckCollision - Uses ellipsoid point detection to determine if a collision will occur.
            % Returns true if a collision occurs.

            % No collision points, return false.
            if isempty(self.collisionPoints)
                collision = false;
                return
            end

            % Calculate link transforms and radii
            [transforms, radii] = self.CalculateRobotEllipsoids(q);

            collision = false;

            % Iterate through each link and check collision points
            for i = 1:size(transforms, 3)
                % Transform points to ellipsoid coordinate frame
                collisionPointsAndOnes = [inv(transforms(:, :, i)) * [self.collisionPoints, ones(size(self.collisionPoints, 1), 1)]']';
                transformedPoints = collisionPointsAndOnes(:, 1:3);
                % find any points inside ellipsoid
                algebraicDist = self.GetAlgebraicDist(transformedPoints, [0, 0, 0], radii(:, i));

                if any(algebraicDist <= 1)
                    collision = true;
                    return;
                end

            end

        end

        function GenerateBasePlane(self, x, y, z)
            %GenerateBasePlane - helper function to generate a 2x2 ground
            %plane. Can be offset from (0,0,0) with x,y,z parameters.
            if ~exist('x', 'var')
                x = 0;
            end

            if ~exist('y', 'var')
                y = 0;
            end

            if ~exist('z', 'var')
                z = 0;
            end

            [X, Y] = meshgrid(-1:0.025:1, -1:0.025:1);
            Z = z * ones(size(Y));
            X = X + x;
            Y = Y + y;

            planePoints = [X(:), Y(:), Z(:)];

            self.collisionPoints = planePoints;
        end

        function ShowEllpsoid(self, ax, q)
            %ShowEllpsoid - Renders ellipsoids for debugging/visualisation.

            % Calculate link transforms and radii
            [transforms, radii] = self.CalculateRobotEllipsoids(q);

            hold(ax, 'on');

            try
                delete(self.ellipsoidHandle);
                delete(self.pointsHandle);
            end

            self.pointsHandle = scatter3(...
                self.collisionPoints(:, 1), self.collisionPoints(:, 2), self.collisionPoints(:, 3), ...
                ones(size(self.collisionPoints(:, 1))), [0.9290 0.6940 0.1250], '.');

            % Iterate through each link and render ellipsoids
            for i = 1:size(transforms, 3)
                % Transform points to ellipsoid coordinate frame
                collisionPointsAndOnes = [inv(transforms(:, :, i)) * [self.collisionPoints, ones(size(self.collisionPoints, 1), 1)]']';
                transformedPoints = collisionPointsAndOnes(:, 1:3);

                % plot ellipsoid
                [X, Y, Z] = ellipsoid(0, 0, 0, radii(1, i), radii(2, i), radii(3, i));
                self.ellipsoidHandle(i) = surf(ax, X, Y, Z);
                alpha(self.ellipsoidHandle(i), 0.6);

                % find any points inside ellipsoid
                algebraicDist = self.GetAlgebraicDist(transformedPoints, [0, 0, 0], radii(:, i));

                if any(algebraicDist <= 1)
                    set(self.ellipsoidHandle(i), 'edgecolor', 'none', 'facecolor', [0.6350 0.0780 0.1840]);
                else
                    set(self.ellipsoidHandle(i), 'edgecolor', 'none', 'facecolor', [0.3010 0.7450 0.9330]);
                end

                % tranform ellipsoid and colour based on collision
                % detection
                HG = hgtransform('Parent', ax);
                set(self.ellipsoidHandle(i), 'Parent', HG);
                set(HG, 'Matrix', transforms(:, :, i));
            end

        end

        function ClearEllipsoid(self)

            try
                delete(self.ellipsoidHandle);
                delete(self.pointsHandle);
            end

        end

    end

    methods (Access = private)

        function [transforms, radii] = CalculateRobotEllipsoids(self, q)
            %CalculateRobotEllipsoids - calculates robot link transforms
            %and uses distances between links for radii.

            x = zeros(1, size(q, 2));
            y = zeros(1, size(q, 2));
            z = zeros(1, size(q, 2));

            for i = 1:size(q, 2)
                [x(i), y(i), z(i)] = self.robot.LinkKinematics(q(1:i));
            end

            links = length(x) - 1;

            transforms = zeros(4, 4, links);
            radii = zeros(3, links);

            for i = 1:links
                position = ([x(i + 1), y(i + 1), z(i + 1)] + [x(i), y(i), z(i)]) / 2;
                dx = x(i + 1) - x(i);
                dy = y(i + 1) - y(i);
                dz = z(i + 1) - z(i);
                L = sqrt(dx^2 + dy^2);
                angleZ = atan2(dy, dx);
                angleY = atan2(dz, L);

                dist = norm([dx, dy, dz]);

                transforms(:, :, i) = transl(position(1), position(2), position(3)) * trotz(angleZ) * troty(-angleY);
                radii(:, i) = [dist / 2, 0.04, 0.04];
            end

        end

        function algebraicDist = GetAlgebraicDist(self, points, centerPoint, radii)

            if self.debug == true
                ax = gca(figure(8));
                cla(ax);
                hold(ax, 'on');
                axis equal
                [X, Y, Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
                surf(ax, X, Y, Z);
                scatter3(points(:, 1), points(:, 2), points(:, 3), '.');
            end

            algebraicDist = ((points(:, 1) - centerPoint(1)) / radii(1)).^2 ...
                + ((points(:, 2) - centerPoint(2)) / radii(2)).^2 ...
                + ((points(:, 3) - centerPoint(3)) / radii(3)).^2;
        end

    end

end
