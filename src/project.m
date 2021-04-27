% Main File for launching project 2.

CV = ComputerVision.ComputerVisionInterface();
Pathing = PathExtraction.PathExtractionInterface(CV);
Move = RMRC.RMRCInterface(Pathing);
GUI = GUI.GUIInterace(Move);

% for image stuff: for sim use webcam, for real use usb_cam topic


running = true;

while (running)
    Move.DoSomething()
end