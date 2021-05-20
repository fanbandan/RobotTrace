%%

TR = zeros(4,4,100);
for i = 1:100
    TR(:,:,i) = transl(-0.1,-0.25-(i/400),1.62);
end

controller = Controller(false,false);
controller.Initialise();
controller.RMRCI.UpdatePath(TR);
pause;
controller.Run();