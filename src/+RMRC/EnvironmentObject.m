classdef EnvironmentObject < handle
    %EnviromentObject Generic object for holding ply graphics.
    properties
        Pose
        h
    end
    properties (SetAccess = private)
        TriMatrix
        Verticies
        VerticiesTransformed
        Colours
    end
    
    methods
        % Constructor requires the path to a 3d model, desired transform
        % and colour
        function self = EnvironmentObject(plyPath,pose,colour)           
            [self.TriMatrix,self.Verticies,data] = plyread(plyPath,'tri');
            self.Verticies = [self.Verticies, ones(length(self.Verticies),1)];
            % If colour data is available, use it, otherwise use input
            % colour data
            if isfield(data.vertex, 'red')
                self.Colours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;                
            elseif isfield(data.face, 'red')
                self.Colours = [data.face.red, data.face.green, data.face.blue] / 255;
            else
                self.Colours = colour.*ones(size(self.TriMatrix));
            end
            
            self.SetPose(pose);
            
            self.h = self.Render();
        end
        function delete(self)
            %delete, ensures graphics object is delete from figure.
            delete(self.h);
        end
        
        function h = Plot(self, pose)
            %Plot draws the graphics object to the current figure.
            %   Optionally also updates the pose.
            if nargin > 1
                self.SetPose(pose);
            end
            
            delete(self.h);
            hold on;
            h = self.Render();
            hold off;
            self.h = h;
        end
        
        function SetPose(self,pose)
            %SetPose sets the pose of the graphics object.
            self.Pose = pose;
            % save transformed verticies to improve render performance.
            self.VerticiesTransformed = self.Pose*self.Verticies';
        end
    end
    methods (Access = private)
        function h = Render(self)
            %Render renders the graphics object using trisurf.
            h = trisurf(self.TriMatrix, ...
                self.VerticiesTransformed(1,:), self.VerticiesTransformed(2,:), self.VerticiesTransformed(3,:), ...
                'FaceVertexCData', self.Colours, 'FaceLighting', 'flat', 'LineStyle', 'none');
        end
    end
end

