classdef AR < handle
    properties (SetAccess = private)
        ARSub
        debug logical = false;
        ARPoses
        ExpectedTagNumber = 2;
    end
    methods (Access = public)
        function self = AR(debug)
            self.ARSub = rossubscriber('/tags');
            if ~isempty(debug)
                self.debug = debug;
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
                    self.ARPoses(:,:,i) = transl(position.X, position.Y, position.Z) * quat.T;
                end
            end
        end
        function robotPose = GetRobotPose(self)
            if length(self.ARTags) >= self.ExpectedTagNumber
                robotPose = self.ARPoses(:,:,1);
            else
                robotPose = transl(0,0,0);
            end
        end
        function gamePose = GetGamePose(self)
            if length(self.ARPoses) >= self.ExpectedTagNumber
                gamePose = self.ARPoses(:,:,2);
            else
                gamePose = transl(0,0,0);
            end
        end
    end
end