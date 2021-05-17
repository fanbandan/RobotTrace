% function project()
    % Main File for launching project 2.
    %%
    rosinit
    % arduino serial node
    sub = rossubscriber('/pushed');
    pause(1);

    rosMode = false;
    debug = false;    
    gui = GUI.GUI(rosMode, debug);
    gui.SetManualStartGuess([-0.204,-0.190,1.80]);
    gui.SetPathingParameters(8000, 0.025, 1, 0.6);
    
    %%
    controller = Controller(false, true);
    controller.Initialise();
    %%
    controller.AcquireImageMask();
    controller.GeneratePathPoints(1.8);
    controllder.DownsamplePathPoints(5000);
    %%
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