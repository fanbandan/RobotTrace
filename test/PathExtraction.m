
clf
CVI = ComputerVision.Interface();
%%
imgMask = imread([workspace, '\data\path_images\path_unproccessed (3).bmp']);

%%
% % img = 4*peaks(720,1080)+randn(720,1080);
% [U,V] = meshgrid(1:1080,1:720);
% % img = peaks(U);
% % img = sin(rand*U/10+rand/10)+sin(rand*V/10+rand/10) + ...
% %     sin(rand/5*V+rand)+sin(rand/5*U+rand);
% pk = peaks(1080);
% img = img + pk(1:720,:);
% % img = img(1:720,:);
% imgMask = logical(img>=0.5);
%%
% Ft = ones(3,3)/3^2;
a = 3;
Ft = ones(a,a)/a^2;
% Ft = [-1,-1,-1;-1,8,-1;-1,-1,-1];
imgFt = conv2(imgMask,Ft,'same'); 
imgFt = logical(imgFt >= 1);

[pty,ptx,~] = find(imgMask);
ptz = ones(size(ptx));
points = [ptx,pty,ptz];
pcloud = pointCloud(points);
pcloud = pcdenoise(pcloud);
pcloud = pcdownsample(pcloud,'gridAverage',20);
points2 = pcloud.Location;

p_start_guess = [915,650,1];

P = p_start_guess;
L = points2;
R = NaN(length(points2),3);
i = 1;
while ~isempty(L)
    [dist,I] = min(pdist2(P,L));
    if dist > 40
        break
    end
    PN = L(I,:);
    R(i,:) = PN;
    i = i+1;
    L(I,:) = [];
    P = PN;
end
R = R(1:i,:);

SX = R(:,1);
SY = R(:,2);
SZ = R(:,3);

figure(10)
plot(R(:,1),R(:,2))

path_idx = zeros(1,length(points2));

for i = 1:(length(points2)-1)
    p1 = points2(i,:);
    distances = vecnorm((p1-points2((i+1):end,:))');
    [~,p2_idx] = min(distances);
    path_idx(i+1) = p2_idx;
end

figure(1);
clf;
subplot(2,1,1) 
imagesc(imgMask);
axis image;
colorbar;
hold on;
plot(movmean(SX,2),movmean(SY,2),'-r')

subplot(2,1,2) 
imagesc(imgFt);
axis image;
colorbar;
hold on;
plot(points2(:,1),points2(:,2),'.r')

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
TC2G = transl(3,0,0)*trotz(pi/6)*troty(pi/10);
planeT = (TC2G*plane')';

figure(2);
surf_h = surf( ...
    reshape(planeT(:,1),size(X)), ...
    reshape(planeT(:,2),size(Y)), ...
    reshape(planeT(:,3),size(Z)) ...
    );
alpha(0.5);
hold on;

axis equal
trplot(transl(0,0,0), 'frame', 'Base')
trplot(TC2G, 'frame', 'AR')