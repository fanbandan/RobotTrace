% Main File for launching project 2.

CV = ComputerVision.ComputerVisionInterface();
Pathing = PathExtraction.PathExtractionInterface(CV);
Move = RMRC.RMRCInterface(Pathing);
GUI = GUI.GUIInterace(Move);

running = true;

while (running)
    Move.DoSomething()
end