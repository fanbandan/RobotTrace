%% Safety Light Curtain Restraints

% Intersection bool
% Number of planes
% Trajectory point count
intersect = false;
noOfPlane = 6;
noOfTrajPoints = 100;

% Create a plan at all 6 walls of the workspace cube by obtaining the
% workspace dimensions

% Get the points within the current trajectory and number of points
x = zeros(4,4,noOfTrajPoints);
for i=1:100
    x(:,:,i) = transl(i/100, 0.15 + 0.1*sin(i*2*pi*0.05), 0.15 +  0.1*cos(i*2*pi*0.05));
end

% Check for intersection of points with the planes
    % for i = (1: points)
        % for j = (1: planes)
            % if int = true
                % stop
                % break
        % end
    % end
