
location_function_folder = pwd;
kuka_urdf = [location_function_folder,'/../../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_function_folder,'/../../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_function_folder,'/../../../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_function_folder,'/../../../URDFs/iCub_r_leg.urdf'];


%% input urdf file to acquire robot structure
robotModelURDF = kuka_kr210;

%% Extract robot from urdf 
smds = extractSystemModel(robotModelURDF);
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotModelURDF);
kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

model_iDyn = kinDynComp.model();
nrOfJoints = model_iDyn.getNrOfJoints();

q_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
grav = iDynTree.Vector3();

% Extract the robot model
smds = extractSystemModel(robotModelURDF);
% Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);


% Constants
gravityModulus = 9.80665;

jointPos = rand(nrOfJoints,1);
for i = 1:nrOfJoints
    
    % Compute jacobian with iDynTree
    q_idyn.fromMatlab(jointPos);
    dq_idyn.fromMatlab(zeros(nrOfJoints,1));
    grav.fromMatlab([0;0;-gravityModulus]);

    kinDynComp.setRobotState(q_idyn, dq_idyn, grav);
    from_frame = kinDynComp.getFrameName(i-1);
    to_frame = kinDynComp.getFrameName(i);
    x = kinDynComp.getRelativeTransform(to_frame,from_frame);
    px = x.getPosition();
    rx = x.getRotation();
    posIDyn{i} = px.toMatlab;
    rotIDyn{i} = rx.toMatlab;
    
    x = x.asAdjointTransform();
    x = x.toMatlab();
    % Featherstone's notation considers first the angular part
    xx(4:6,1:3) = x(1:3,4:6);
    xx(1:3,4:6) = x(4:6,1:3);
    xx(1:3,1:3) = x(4:6,4:6);
    xx(4:6,4:6) = x(1:3,1:3);
    XupIDyn{i} = xx;

    xWorld = kinDynComp.getWorldTransform(i); 
    xWorld = xWorld.asAdjointTransform();
    xWorld = xWorld.toMatlab();
    xxW(4:6,1:3) = xWorld(1:3,4:6);
    xxW(1:3,4:6) = xWorld(4:6,1:3);
    xxW(1:3,1:3) = xWorld(4:6,4:6);
    xxW(4:6,4:6) = xWorld(1:3,1:3);
    XWorldIDyn{i} = xxW;
    
    %% Compute the symbolic functions
    [X,~,S,Xup, ~, ~]  = computeKinematics (smds, jointPos, zeros(nrOfJoints,1), zeros(nrOfJoints,1), [0;0;-gravityModulus]);
    XWorld_symb{i} = (X{1}{1,i}*Xup{1})^(-1);
end
%%
for j =1:nrOfJoints+1
    l = model_iDyn.getLink(j-1);
    I = l.getInertia();
    I = I.asMatrix;
    I = I.toMatlab;
    % Featherstone's notation considers first the angular part
    ii(4:6,1:3) = I(1:3,4:6);
    ii(1:3,4:6) = I(4:6,1:3);
    ii(1:3,1:3) = I(4:6,4:6);
    ii(4:6,4:6) = I(1:3,1:3);
    IiDyn{j} = ii;
end
