function project()
    % Main File for launching project 2.

    CV = ComputerVision.ComputerVisionInterface();
    Pathing = PathExtraction.PathExtractionInterface(CV);
    Move = RMRC.RMRCInterface(Pathing);
    GUI = GUI.GUIInterace(Move);

    % for image stuff: for sim use webcam, for real use usb_cam topic
    
    %% Arduino estop setup
    a = arduino;
    %% Arduino eStop
    data = readDigitalPin(a,'D3');
    if data == 1
%         L.mlog = {L.DEBUG,'project','EStop'};
        return
    end
    %%



    running = true;

    while (running)
        Move.DoSomething()
    end
end