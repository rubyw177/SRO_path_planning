% build map
im_map = imread('map.png');
im_bin = im2bw(im_map);
myMaplogical = not(logical(im_bin));
map = binaryOccupancyMap(myMaplogical,100);
show(map); 

% create road map
PRM = mobileRobotPRM(map, 100);
show(PRM);

% find path
startPositionCoppelia = [0.175 4.25];
goalPositionCoppelia =  [-4.358 -4.469];

startPositionMatlab = posconvert("C2M", startPositionCoppelia);
goalPositionMatlab = posconvert("C2M", goalPositionCoppelia);

disp(startPositionMatlab)
disp(goalPositionMatlab)

pathMatlab = findpath(PRM, startPositionMatlab, goalPositionMatlab);
pathCoppelia = posconvert("M2C", pathMatlab);
show(PRM);

% Pure Pursuit
pp = controllerPurePursuit;
pp.Waypoints = pathMatlab;

% Connect to Coppelia
disp('program started');
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

% Control Loop
if (clientID>-1)
    disp('connected to remote API server');
    [~, right_motor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    [~, left_motor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [err_code, pioneer_p3dx] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
    goalReachedCount = 0;
    while goalReachedCount<5
        [~, pos]=sim.simxGetObjectPosition(clientID,pioneer_p3dx,-1,sim.simx_opmode_blocking);
        [~, ori]=sim.simxGetObjectOrientation(clientID,pioneer_p3dx,-1,sim.simx_opmode_blocking);
        currentPositionCoppelia = pos(1,1:2);
        currentOrientationCoppelia = ori(1,3);
        currentPositionMatlab = posconvert("C2M",currentPositionCoppelia);
        currentOrientationMatlab = oriconvert(currentOrientationCoppelia);
        currentPoseMatlab = [currentPositionMatlab currentOrientationMatlab]; 
        disp(currentPoseMatlab)
    
        [v,w]=pp(currentPoseMatlab);
        [phiR, phiL]= invkinem(v,w);
        distToGoal = sqrt(sum((currentPositionMatlab-goalPositionMatlab).^2));
        if distToGoal < 0.5
             reachedGoal = 1;
             goalReachedCount = goalReachedCount + 1;
        else
             reachedGoal = 0;
        end 
        phiR= (1-reachedGoal)*(phiR+3);
        phiL= (1-reachedGoal)*(phiL+3);

        ret_codeR = sim.simxSetJointTargetVelocity(clientID, right_motor, phiR, sim.simx_opmode_streaming);
        ret_codeL = sim.simxSetJointTargetVelocity(clientID, left_motor, phiL, sim.simx_opmode_streaming);
    end
    disp('goal has been reached')
    sim.simxFinish(clientID);
else
    disp('Failed Connecting  to remote API server');
end
sim.delete(); % call the destructor!
disp('Program ended'); 

% functions needed
function [posOut]= posconvert(trID,posIn)
% convert position between matlabh and coppelia
    transl = [1 0 4.95; 0 1 5.1; 0 0 1];
    rot = rotz(180);
    M2C = transl*rot;
    in = posIn';
    sz = size(in);
    k = ones(1,sz(2));
    inhomog = [in;k];
    if trID == "M2C"
        outhomog = M2C*inhomog;
    else 
        outhomog = M2C\inhomog;
    end
    posOut = outhomog(1:2,:)';
end
function [oriOut] = oriconvert(oriIn)
    oriOut = pi - oriIn;
    if oriIn > 0
        oriOut = -pi + oriIn;
    end
    if oriIn < 0
        oriOut = pi + oriIn;
    end
end

function [pR,pL] = invkinem(v,w)
    wheelradius= 0.195/2;
    bodywidht = 0.381;
    R2 = wheelradius/2;
    RB = wheelradius/bodywidht;
    vw = [v;w];
    kinemat = [R2 R2; RB -RB];
    pLR = kinemat\vw;
    pR = pLR(1);
    pL = pLR(2);
end
