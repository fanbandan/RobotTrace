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
            tagSize = size(tagData , 2);
            if tagSize > 1
                arTags = tagMsg(1).Markers;
                for i = 1:length(arTags)
                    position = arTag(i).pose.position;
                    orientation = arTag(i).pose.orientation;
                    quat = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
                self.ARPoses(:,:,i) = transl(position.x, position.y, position.z) * quat.T;
                end
            end
        end
        function robotPose = GetRobotPose(self)
            if length(self.ARTags) == self.ExpectedTagNumber
                robotPose = self.ARPoses(:,:,1);
            end
        end
        function robotPose = GetGamePose(self)
            if length(self.ARTags) == self.ExpectedTagNumber
                robotPose = self.ARPoses(:,:,2);
            end
        end
    end
end