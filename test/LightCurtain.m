%% Safety Light Curtain Restraints

clf;

% Set/Create Parameters
intersect = false;
noOfPlane = 6;
noOfTrajPoints = 100;
intersectionPoint = [0 0 0];

%% Set up robot for sim
dobot = RMRC.Dobot();
workspace = [-1,1,-1,1,-1,1];
qn = [0, deg2rad([0, 5, 0, 0])];
dobot.PlotRobot(workspace,qn);
axis equal
q0 = qn;
hold on;

%% Create a plane
planePoint = [0.9; 0; 0];
planeNormal = [0.1; 0; 0];
normal = [0.1, 0, 0];

w = null(normal);
[P,Q] = meshgrid(-2:2);
x = planePoint(1,1) + w(1,1) * P + w(1,2) * Q;
y = planePoint(2,1) + w(2,1) * P + w(2,2) * Q;
z = planePoint(3,1) + w(3,1) * P + w(3,2) * Q;
hold on
surf(x,y,z);
drawnow;

%% Get the points within the current trajectory and number of points
x = zeros(4,4,noOfTrajPoints);
for i = 1:noOfTrajPoints
    x(:,:,i) = transl(i/100, 0.15 + 0.1*sin(i*2*pi*0.05), 0.15 +  0.1*cos(i*2*pi*0.05));
end

%% Check for intersection of points with the planes
for i = 1:noOfTrajPoints-1
    T = x(:,:,i);
    q = dobot.ikcon(T,q0);
    trajPoint1 = x(1:3,4,i);
    trajPoint2 = x(1:3,4,i+1);
    % LinePlaneIntersection
    % Given a plane (normal and point) and two points that make up another line, get the intersection
    u = trajPoint2 - trajPoint1;
    w = trajPoint1 - planePoint;
    D = dot(planeNormal,u);
    N = -dot(planeNormal,w);
    if abs(D) < 10^-7        % The segment is parallel to plane
        if N == 0           % The segment lies in plane
            check = 2;
            intersect = true;
            return
        else
            check = 0;       %no intersection
            intersect = false;
            return
        end
    end
    
    % compute the intersection parameter
    sI = N / D;
    intersectionPoint = trajPoint1 + sI.*u;
    
    if (sI < 0 || sI > 1)
        check = 3;          %The intersection point lies outside the segment, so there is no intersection
        intersect = false;
    else
        check = 1;
        intersect = true;
    end
    if intersect == true
        % stop dobot
        disp("Light Curtain Breached! Robot Operation Shut Down!");
        break;
    end
    dobot.Animate(q);
    pause(0.2);
    q0 = q;
end