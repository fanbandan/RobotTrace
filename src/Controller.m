classdef Controller < handle
    %Controller This class is the main interface for launching the project.
    %   It handles all the sub class interfaces.

    properties (Access = public)
    end

    properties (Access = protected)
        CVI ComputerVision.Interface
        PEI PathExtraction.Interface
        RMRCI RMRC.Interface
        rosMode logical = true;
        debug logical = false;
    end

    properties (Access = private)
        image;
        imageMask;
        pathPoints;
        pathStartGuess;
    end

    methods (Access = public)

        function self = Controller(rosMode, debug)
            %Controller creates an instance of the controller.
            %   rosMode and debug are passed to the subclasses to enable
            %   ROS subscribers / publishers and show additional debug outputs.
            deltaT = 0.05;

            if exist('debug', 'var')
                self.debug = debug;
            end

            if exist('rosMode', 'var')
                self.rosMode = rosMode;
            end

            self.CVI = ComputerVision.Interface(self.rosMode, self.debug);
            self.PEI = PathExtraction.Interface(self.debug);
            self.RMRCI = RMRC.Interface(self.CVI, deltaT, self.rosMode, self.debug);
            self.pathStartGuess = [0, 0, 0];
        end

        function Initialise(self)
            %Initialise setups up the robot and environment.
            %   If rosMode is enabled it will initallise the dobot magician driver.
            %   Otherwise it will just create the sim environment
            self.RMRCI.Initialise();
        end

        function image = AcquireImageMask(self)
            %AcquireImageMask Gets an image from the Computer Vision Interface
            %   Gets the latest image from Computer Vision and
            %   updates the AR tag posisitons.
            image = self.CVI.GetImageMask();
            self.image = image;
            self.CVI.UpdateARTags();
            self.imageMask = self.image;
        end

        function [robot, game] = GetPoses(self)
            %GetPoses returns the position of the robot and game from the camera
            % based on their AR tags
            robot = self.CVI.GetCamera2RobotTransformationMatrix();
            game = self.CVI.GetCamera2GameTransformationMatrix();
            self.RMRCI.ShowARTags(game, robot);
        end

        function points = GeneratePathPoints(self, zDepthOverride)
            %GeneratePathPoints converts the camera image pixels into camera world coordinates
            %   zDepthOverride - Overrides the game distance from the camera.
            cameraMatrix = self.CVI.GetCameraMatrix();

            if exist('zDepthOverride', 'var')
                [normal, point] = self.CVI.GetGamePlane(zDepthOverride);
            else
                [normal, point] = self.CVI.GetGamePlane();
            end

            points = self.PEI.UpdatePathMask(self.imageMask, cameraMatrix, normal, point);
        end

        function DownsamplePathPoints(self, downsample)
            %DownsamplePathPoints downsamples the path points.
            %   downsample - the target point amount.
            self.PEI.DownsamplePoints(downsample);
        end

        function success = GeneratePath(self, maxDistance, averaging, smoothing)
            %GeneratePath Attempts to form a continuous path segment using the start
            % guess point and max distance threshold. Returns false if less than 5 points are
            % connected.
            %   maxDistance - Aax distance in meters between points for a connection to occur
            %   averaging - Applies movemean averaging to path segment.
            %   smoothing - Fits a smooth spline using smoothing value between 0 and 1.
            startGuess = self.pathStartGuess;

            if self.debug == true
                figure(105);
            end

            path = self.PEI.GeneratePath(startGuess, maxDistance);

            if length(path) < 5
                warning('No path found!');
                success = false;
                return
            end

            success = true;
            self.PEI.GenerateSpline(averaging, smoothing);
        end

        function UpdatePathStartGuess(self, manualGuess)
            %UpdatePathStartGuess updates the path start guess point using
            % the robots current position.
            %   manualGuess - Optional variable to override the robots current position.
            if exist('manualGuess', 'var')
                self.pathStartGuess = manualGuess;
            else
                self.pathStartGuess = transl(self.RMRCI.GetRobotPose())';
            end

        end

        function startGuess = GetPathStartGuess(self)
            %GetPathStartGuess returns the current start guess point.
            startGuess = self.pathStartGuess;
        end

        function path = GetTrajectory(self, samples)
            %GetTrajectory resamples the path using the smooth spline.
            % Returns the resampled trajectory.
            %   samples - Number of points in the trajectory.
            path = self.PEI.GetTrajectory(samples);
            self.pathPoints = path;
        end

        function LoadTrajectory(self)
            %LoadTrajectory passes the resampled trajectory to the RMRC algorithm.
            %   GetTrajectory must be run first.
            pathPoses = zeros(4, 4, size(self.pathPoints, 2));

            for i = 1:size(self.pathPoints, 2)
                pathPoses(:, :, i) = trotx(pi / 2) * transl(self.pathPoints(1, i), self.pathPoints(2, i), self.pathPoints(3, i));
            end

            self.RMRCI.UpdatePath(pathPoses);
        end

        function Run(self)
            %Run executes the trajectory using RMRC.
            %   Is thread blocking.
            self.RMRCI.FollowPath();
        end

        function EStop(self)
            %EStop immediately stops the robots current actions.
            %   Will terminate the Run method.
            self.RMRCI.Stop();
        end

        function ToggleCollisions(self, checkCollisions, renderCollisions)
            %ToggleCollisions enables or disables collision checking.
            %   checkCollisions - when true will stop the robot if a collision is predicted.
            %   renderCollisions - when true will render collision ellipsoids.
            self.RMRCI.ToggleCollisions(checkCollisions, renderCollisions);
        end

        function joints = GetDobotJoints(self)
            %GetDobotJoints returns the dobot current joint values.
            joints = self.RMRCI.GetRobotJoints();
        end

        function qlim = GetDobotJointLimits(self)
            %GetDobotJointLimits returns the dobot joint limits.
            qlim = self.RMRCI.GetRobotJointLimits();
        end

        function SetDobotJoints(self, q)
            %SetDobotJoints sets target joint positions for the dobot.
            self.RMRCI.SetRobotJoints(q);
        end

        function [x, y, z] = GetDobotPose(self)
            %GetDobotPose returns the dobot current end effector position.
            pose = transl(self.RMRCI.GetRobotPose());
            x = pose(1);
            y = pose(2);
            z = pose(3);
        end

        function [success, endPose] = JogDobot(self, x, y, z)
            %JogDobot creates a target end effector position and attempts to move
            % the dobot.
            pose = transl(x, y, z);
            [success, endPose] = self.RMRCI.MoveRobotToPose(pose);
        end

    end

    methods
        % Debug / Visual methods
        function ShowPathImage(self, ax)
            %ShowPathImage shows the Computer Vision image.
            imshow(self.imageMask, 'Parent', ax);
        end

        function ShowPathPointCloud(self, ax)
            %ShowPathPointCloud shows the Path Extraction world points.
            self.PEI.ShowPointCloud(ax);
            hold(ax, 'on');
            scatter3(ax, ...
                self.pathStartGuess(1), self.pathStartGuess(2), self.pathStartGuess(3), ...
                [], [0.4660 0.6740 0.1880], 'o', 'DisplayName', 'start guess');
            hold(ax, 'off');
            legend(ax, 'Location', 'northeast');
        end

        function ShowPathSpline(self, ax)
            %ShowPathSpline shows the Path Extraction smooth spline.
            self.PEI.PlotSpline(ax);
            legend(ax, 'Location', 'northeast');
        end

    end

end
