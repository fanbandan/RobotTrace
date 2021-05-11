## Initialising Dobot Model

**Before initalising the robot** make sure to follow the instructions on the [Main Read Me](../../README.md)

### Dobot Sim Model

```MATLAB
% Initialise robot
%   RMRC is the namespace, it is derived from the +RMRC folder name.
%   Dobot is the class name, it is derived from the @Dobot folder name.
%   For more info read https://au.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html
dobot = RMRC.Dobot();
workspace = [-1,1,-1,1,-1,1];
q = [0, deg2rad([0, 5, 0, 0])];
% q is optional.
dobot.PlotRobot(workspace,q);

% Inverse Kinematics example.
pose_1 = transl(0.1,0.2,0.15);
q_1 = dobot.ikcon(pose_i,q);

% Once plotted the dobot model can be updated with Animate.
dobot.Animate(q_1)

% Forward Kinematics example.
pose_check = dobot.fkine(q_1);

% Get model current joint states.
q_s = dobot.GetPos();

% Jacobian.
J = dobot.jacob(q_s);

```

### RMRC Control Example

```MATLAB
dobot = RMRC.Dobot();
deltaT = 0.02;
% Initialise ResMotion
%   RMRC is the namespace, it is derived from the +RMRC folder name.
%   ResMotion is the class name, it is derived from the @ResMotion folder name.
%   This object will generate qMatrices using RMRC.
RM = RMRC.ResMotion(dobot,deltaT);

q0 = dobot.GetPos();
% This runs a script located in the test folder.
%   It generates a path for the dobot to follow and saves it
%   in the x variable.
GeneratePath;

% This methods applies RMRC
%   x is a 4x4xn matrix where n is the number of steps.
%       It is a set of 4x4 homogeneous transformation matrices.
%   q0 is the starting joint position of the robot.
[qMatrix,positionError,angleError] = RM.RateControl(x,q0);

% Animate the robot completing the path.
for i = 1:length(qMatrix)
    dobot.Animate(qMatrix(i,:));
    pause(deltaT);
end

```
