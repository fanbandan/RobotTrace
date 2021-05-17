classdef Robot < handle
    %Robot Generic robot class
    %   Provides a model and generic functionality for all robots.
    properties
        Model; %Robot model
        Item; %End Effector Object
        ItemOffset;
    end
    methods
        function self = Robot()
            %Robot Construct an instance of this class
            %   Generates the robot model
            self.GetRobot();
            self.Item = [];
            self.ItemOffset = transl(0,0,0);
        end
        function delete(self)
            %delete Destructor ensures deletion of model.
            delete(self.Model);
        end
        function Animate(self,q)
            %Animate plots the robot using joint states q.
            %   Will also animate EE objects if set.
            if ~isempty(self.Item)
                [q_l,~] = size(q);
                for i = 1:1:q_l
                    self.Model.animate(q(i,:));
                    % Need to multiply by trotx(-pi) to undo the EE frame.
                    self.Item.Plot(self.Model.fkine(q(i,:))*self.ItemOffset*trotx(-pi));
                end
            else
                self.Model.animate(q)
            end
        end
        function SetItem(self,item,offset)
            %SetItem Sets EE object with some offset.
            self.Item = item;
            self.ItemOffset = offset;
        end
        function ClearItem(self)
            %ClearItem clears EE item.
            %   Note, does not delete the item from the scene.
            self.Item = [];
        end
    end
    methods (Abstract)
        %Methods to be implemented by derived robots.
        GetRobot(self)
        PlotAndColourRobot(self, workspace, q)
    end
end