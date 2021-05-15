% function project()
    % Main File for launching project 2.
    %%
    rosMode = false;
    debug = false;    
    GUI.GUI(rosMode, debug);
    
    %%
    controller = Controller(false, true);
    controller.AcquireImageMask();
    controller.GeneratePathPoints(1.8);
    controllder.DownsamplePathPoints(5000);
    
%     pause;
    figure(105);
    image = imread([pwd, '//data//demo.jpg']);
    imshow(image(:,1:400,:));
    controller.UpdatePathStartGuess([-0.204,-0.190,1.80]);
    controller.GeneratePath(0.025, 1, 0.6);
    h = figure(106);
    axis equal;
    set(gca, 'XDir','reverse')
    controller.ShowPathSpline(h);
% end