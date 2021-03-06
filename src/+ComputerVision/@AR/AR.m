classdef AR < handle
    properties (SetAccess = private)
        ARSub
        debug logical = false;
        arPoses
        expectedTagNumber = 2;
        timerObject timer
        timerPeriod = 5;
    end
    methods (Access = public)
        function self = AR(debug)
            self.ARSub = rossubscriber('/tags');
            self.TimerSetup()
            if ~isempty(debug)
                self.debug = debug;
            end
        end
        function delete(self)
            try
                % Makes sure the class will destruct regardless of timer
                % state
                stop(self.timerObject);
                delete(self.timerObject);
            end
        end
        function arTags = UpdateARTags(self)
            tagMsg = receive(self.ARSub, 5); %Set to 5 second wait currently ----
            tagData = tagMsg.Poses;
            tagSize = size(tagData , 1);
            if tagSize > 1
                for i = 1:tagSize
                    position = tagData(i).Position;
                    orientation = tagData(i).Orientation;
                    quat = Quaternion([orientation.W, orientation.X, orientation.Y, orientation.Z]);
                    self.arPoses(:,:,i) = transl(position.X, position.Y, position.Z) * quat.T;
                end
            end
        end
        function robotPose = GetRobotPose(self)
            if length(self.arTags) >= self.expectedTagNumber
                robotPose = self.arPoses(:,:,1);
            else
                robotPose = NaN(4,4);
            end
        end
        function gamePose = GetGamePose(self)
            if length(self.arPoses) >= self.expectedTagNumber
                gamePose = self.arPoses(:,:,2);
            else
                gamePose = NaN(4,4);
            end
        end
    end
    methods (Access = private)
        function TimerSetup(self)
            t = timer;
            t.Name = 'AR ROS Refresh Timer';
            t.TimerFcn = @(~, ~) self.TimerUpdate();
            t.Period = self.timerPeriod;
            t.ExecutionMode = 'fixedDelay';
            self.timerObject = t;
            start(self.timerObject);
        end
        function TimerUpdate(self)
            self.UpdateARTags();
        end
    end
end