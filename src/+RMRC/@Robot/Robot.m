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
        function stats = GetStats(self)
            %GetStats provides stats about the robots current position.
            %   Mostly a helper function for the GUI interface.
            stats = self.Model.getpos();
        end
        function [volume,max_radius,min_radius] = CalculateWorkspace(self, q, steps, zLim)
            %CalculateWorkspace calculates the workable volume and radius
            %for the robot.
            %   Generates a point cloud and uses convhull (convex envelope) 
            %   to estimate volume
            if isempty(zLim)
                zLim = 0;
            end
            ptCloud = self.IterateJoint(q, 1, self.Model.n-1, steps);
            % Discard points below z limit (i.e. the floor)
            ptCloud = ptCloud(ptCloud(:,3)>zLim,:);
            % Inspired by: 
            % https://au.mathworks.com/matlabcentral/answers/145417-calculate-the-volume-of-shape-descibed-by-3d-points
            [k,volume] = convhull(ptCloud(:,1),ptCloud(:,2),ptCloud(:,3),'Simplify',true);
            distances = vecnorm((ptCloud'-transl(self.Model.base)))';
            max_radius = max(distances);
            min_radius = min(distances);
            figure;
            title({'Point Cloud and Volume Estimate',self.Model.name});
            hold on;
            plot3(ptCloud(:,1),ptCloud(:,2),ptCloud(:,3),'*');
            trisurf(k, ptCloud(:,1),ptCloud(:,2),ptCloud(:,3))
            view(60,30);
            hold off;            
        end
    end
    methods (Access = private)
        function pointCloud = IterateJoint(self, q_master, n, m, steps)
            %IterateJoint a recursive function to iterate through all
            %joints of the robot.
            
            qlim = self.Model.qlim;
            min = qlim(n,1);
            max = qlim(n,2);
            step = (max-min)/(steps-1);
            q_iteration = q_master;
            pointCloud = nan(steps.^(m-n+1),3);
            q_values = min:step:max;
            for i = 1:1:length(q_values)
                q_iteration(n) = q_values(i);                
                if (n >= m)
                    pt = transl(self.Model.fkine(q_iteration));
                    pointCloud(i,:) = pt';
                else
                    j = steps.^(m-n);
                    pointCloud(((i-1)*j+1):i*j,:) = self.IterateJoint(q_iteration, n+1, m, steps);
                end                 
            end
        end
    end
    methods (Abstract)
        %Methods to be implemented by derived robots.
        GetRobot(self)
        PlotAndColourRobot(self, workspace, q)
    end
end