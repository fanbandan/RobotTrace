 function project()
    % Main File for launching project 2.
    rosMode = false;
    debug = false;    
    gui = GUI.GUI(rosMode, debug);
    gui.SetManualStartGuess([-0.204,-0.190,1.80]);
    gui.SetPathingParameters(8000, 0.025, 1, 0.6);
end