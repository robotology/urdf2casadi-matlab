%% This is a script to compare the result of the symbolic expression with the result obtained using iDynTree(https://github.com/robotology/idyntree)
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2eom/URDFs/twoLinks.urdf';

location_urdf = kuka_urdf;
%% Prepare symbolic model
%% Now use the symbolic model
% Compute model from urdf
symbolicDynamicFunction = urdf2eom(location_urdf);
%% Prepare iDyntree model
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();

% input urdf file to acquire robot structure
robotModelURDF = location_urdf;

mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();
nrOfLinks = nrOfJoints + 1;

baseAcc_idyn = iDynTree.Vector6();
baseAcc_idyn.zero();

regr_idyn = iDynTree.MatrixDynSize();
grav_idyn = iDynTree.Vector3();

grav_idyn.zero();
gravityAccelerationModulus = 9.80665;
grav_idyn.setVal(2, -gravityAccelerationModulus);

linkExtForces = iDynTree.LinkWrenches(kinDynComp.model());
linkExtForces.zero();
output_FreeFloatingGeneralizedTorques = iDynTree.FreeFloatingGeneralizedTorques(kinDynComp.model());


%% Set the ID inputs for both iDyntree and symbolic function
jointVel = zeros(nrOfJoints, 1);
jointAcc = zeros(nrOfJoints, 1);
if nrOfJoints ==6
    jointPos = [pi/6 0 0 0 0 0]';
    jointVel = rand(6,1);
    jointAcc = rand(6,1);
elseif nrOfJoints ==1
    jointPos = rand;
    jointVel = rand;
    jointAcc = rand;
end

q_idyn = iDynTree.VectorDynSize(nrOfJoints);
q_idyn.fromMatlab(jointPos);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn.fromMatlab(jointVel);
ddq_idyn = iDynTree.VectorDynSize(nrOfJoints);
ddq_idyn.fromMatlab(jointAcc);

kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

% Compute joint torques
kinDynComp.inverseDynamics(baseAcc_idyn, ddq_idyn, linkExtForces, output_FreeFloatingGeneralizedTorques);

% Display the result
tau_iDynTree = output_FreeFloatingGeneralizedTorques.jointTorques().toMatlab;



%% Symbolic cmputations
g =[0;0;-gravityAccelerationModulus]; % Gravity column vector
% The external forces is a vector of (6,1). It has to be one per
% link, expect the base link(which for now is considered fixed)
% Test with the compiled function 
% If we do not supply the external forces they are automatically considered null in the compiled function
tau_symbolic_compiled = rnea('rnea',jointPos, jointVel, jointAcc, g);
% Test with symbolic function
% If we do not supply the external forces they are NOT automatically considered null in the symbolic function
extForce = zeros(6,1);
if nrOfJoints ==6
    tau_symbolic_function = symbolicDynamicFunction(jointPos, jointVel, jointAcc, g,0,0,0,0,0,0);
elseif nrOfJoints ==1
    tau_symbolic_function = symbolicDynamicFunction(jointPos, jointVel, jointAcc, g,0);
end
tau_symbolic_function = full(tau_symbolic_function);

%% Check results
plot(abs(tau_iDynTree-tau_symbolic_function));
tau_iDynTree
%tau_symbolic_compiled
tau_symbolic_function