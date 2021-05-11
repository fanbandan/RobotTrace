
clf
CVI = ComputerVision.Interface();
%% 1. Get image mask & AR Tag
imgMask = imread([workspace, '\data\lab_photos\5.jpg']);
% px = -size(imgMask,2)/2;
% py = 120;
u0 = size(imgMask,2)/2;
v0 = size(imgMask,1)/2;
f = 1;
AR_Q = Quaternion(0.0,[0,1,0]);
AR_P = [0,0,0.6];

pw = 1;
ph = 1;

C = [ ...
    665.578756,    0,          282.225564; ...
    0,              664.605455, 260.138094; ...
    0,              0,          1; ...
    ];

C = [f/pw,  0,      u0;
    0,      f/ph,   v0;
    0,      0,      1;];

%% 2. Convert to X Y Z cartesian
imgMask = imgMask./255;
[pixelY,pixelX,~] = find(imgMask);

p = C*AR_P';
zPlanePoint = p';
zPlaneNormal = Quaternion2Normal(AR_Q)';
%%
points = NaN(length(pixelX),3);

% [X,Y] = meshgrid(1:10:540,1:10:540);
% pixelX = X(:);
% pixelY = Y(:);

for i = 1:length(pixelX)
    points(i,:) = Pixel2World(pixelX(i),pixelY(i),C,zPlaneNormal,zPlanePoint)';
%     Z = GetZ(pixelX(i),pixelY(i),zPlaneNormal,zPlanePoint);
%     X = (Z/f) * (pixelX(i) - u0);
%     Y = (Z/f) * (pixelY(i) - v0);
%     points(i,:) = [X,Y,Z];
end
plot3(points(:,1),points(:,2),points(:,3),'*');

%% 3. Create Point Cloud

pcloud = pointCloud(points);
% pcloud = pcdenoise(pcloud);
pcloud = pcdownsample(pcloud,'nonuniformGridSample',200);
points2 = pcloud.Location;

pixel_guess = [915,629];
p_start_guess = Pixel2World(pixel_guess(1),pixel_guess(2),C,zPlaneNormal,zPlanePoint)';
% p_start_guess = zeros(1,3);
% p_start_guess(3) = GetZ(pixel_guess(1),pixel_guess(2),zPlaneNormal,zPlanePoint);
% p_start_guess(1) = (p_start_guess(3)/f) * (pixel_guess(1) - u0);
% p_start_guess(2) = (p_start_guess(3)/f) * (pixel_guess(2) - v0);
% [~,idx] = max(points2(:,1));
% p_start_guess = points2(idx,:);
% p_start_guess = [-8.8,-6.2,-18.2];

P = p_start_guess;
L = points2;
R = NaN(length(points2),3);
i = 1;
while ~isempty(L)
    [dist,I] = min(pdist2(P,L));
    if dist > 2
        i + 1;
        break
    end
    PN = L(I,:);
    R(i,:) = PN;
    i = i+1;
    L(I,:) = [];
    P = PN;
end
R = R(1:(i-1),:);

SX = R(:,1);
SY = R(:,2);
SZ = R(:,3);

figure(10)
plot(R(:,1),R(:,2))

figure(1);
clf;
subplot(2,1,1) 
imagesc(imgMask);
axis image;
colorbar;
% hold on;
% plot(movmean(SX,2),movmean(SY,2),'-r')

subplot(2,1,2) 
plot3(points(:,1),points(:,2),points(:,3),'*')
hold on;
plot3(points2(:,1),points2(:,2),points2(:,3),'*')
plot3(p_start_guess(:,1),p_start_guess(:,2),p_start_guess(:,3),'o')
% plot(points2(:,1),points2(:,2),'.r')

figure(2);
clf;
subplot(2,1,1) 
plot(1:length(R),R,'.');
set(gca, 'YDir','reverse')

subplot(2,1,2) 
plot(SX,SY)
set(gca, 'YDir','reverse')
%%
[Y,Z] = meshgrid(-5:1:5,-5:1:5);
X = zeros(size(Y)); 

plane = [X(:),Y(:),Z(:),ones(size(X(:)))];

% TC2G = CVI.GetCamera2GameTransformationMatrix();
TC2G = transl(3,0,0)*trotz(pi/6);%*troty(pi/10);
planeT = (TC2G*plane')';

% figure(8);
% surf_h = surf( ...
%     reshape(plane(:,1),size(X)), ...
%     reshape(plane(:,2),size(Y)), ...
%     reshape(plane(:,3),size(Z)) ...
%     );
% alpha(0.5);
% hold on;
% surf_h = surf( ...
%     reshape(planeT(:,1),size(X)), ...
%     reshape(planeT(:,2),size(Y)), ...
%     reshape(planeT(:,3),size(Z)) ...
%     );
% alpha(0.5);

% axis equal
% trplot(transl(0,0,0), 'frame', 'Base')
% trplot(TC2G, 'frame', 'AR')

%%
% clf
q = Quaternion(pi/4,[0,1,0]);
normal = Quaternion2Normal(q)';

% CreateSurface(normal,[0,0,3])
% Z = GetZ(0,0,normal,[0,0,3])
% axis equal
% hold on;
% % trplot(transl(0,0,0))
% q.plot();

function normal = Quaternion2Normal(quaternion)
    normal = quaternion*[0,0,1];
end

function world = Pixel2World(u,v,C,normal,point)
    w = GetZ(u,v,normal,point);
    pixel = [u;v;w];
    world = inv(C)*pixel;
end

function Z = GetZ(u,v,normal,point)
    x = u;
    y = v;
    Z = point(3) + ( ... 
        conj(normal(1))*(point(1) - x) + conj(normal(2))*(point(2) - y) ...
        ) / (conj(normal(3)));
end

function CreateSurface(normal,point)
w = null(normal); % Find two orthonormal vectors which are orthogonal to v
[P,Q] = meshgrid(-500:50:500); % Provide a gridwork (you choose the size)
X = point(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = point(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = point(3)+w(3,1)*P+w(3,2)*Q;
hold on
surf(X,Y,Z);
end