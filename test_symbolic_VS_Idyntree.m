%% This is a script to compare the result of the symbolic expression with the result obtained using iDynTree(https://github.com/robotology/idyntree)
%% First compute ID with idyntree 
identified_urdf = '/home/iiticublap041/idjl-model-identification/robot_parameters/../results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();

% input urdf file to acquire robot structure
robotModelURDF = identified_urdf;

mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();
nrOfLinks = nrOfJoints + 1;

jointPos = [pi/2;-pi/2;0;0;-pi/2;pi/2];
jointVel = zeros(nrOfJoints, 1);
jointAcc = zeros(nrOfJoints, 1);


q_idyn = iDynTree.VectorDynSize(nrOfJoints);
q_idyn.fromMatlab(jointPos);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn.fromMatlab(jointVel);
ddq_idyn = iDynTree.VectorDynSize(nrOfJoints);
ddq_idyn.fromMatlab(jointAcc);

baseAcc_idyn = iDynTree.Vector6();
baseAcc_idyn.zero();

regr_idyn = iDynTree.MatrixDynSize();
grav_idyn = iDynTree.Vector3();

grav_idyn.zero();
grav_idyn.setVal(2, -9.80665);
% grav_idyn.setVal(2, 0);
kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

linkExtForces = iDynTree.LinkWrenches(kinDynComp.model());
linkExtForces.zero();

output_FreeFloatingGeneralizedTorques = iDynTree.FreeFloatingGeneralizedTorques(kinDynComp.model());
% Compute joint torques
kinDynComp.inverseDynamics(baseAcc_idyn, ddq_idyn, linkExtForces, output_FreeFloatingGeneralizedTorques);
% Display the result
tau_iDynTree = output_FreeFloatingGeneralizedTorques.jointTorques().toMatlab;

%% Now use the symbolic model
location_urdf2eom = '/home/iiticublap041/baljinder/urdf2eom';
cd(location_urdf2eom);
% Compute model from urdf
symbolicDynamicFunction = urdf2eom(identified_urdf);

% Test with the compiled function
g =[0;0;-9.80665]; % Gravity column vector
tau_symbolic_compiled = rnea('rnea',jointPos, jointVel, jointAcc, g);
% Test with symbolic function
tau_symbolic_function = symbolicDynamicFunction(jointPos, jointVel, jointAcc, g);

location_idjl = '/home/iiticublap041/idjl-model-identification';
cd(location_idjl);

%% Check results
plot(abs(tau_iDynTree-tau_symbolic_compiled));
tau_iDynTree
tau_symbolic_compiled