function [jointAccMatlab, jointAccSymbolic] = computeIDyntreeFD_VS_Symblic(modelRobotMatlab, q,dq, gravityModulus, tau, robotModelURDF)

%% Matlab
jointAccMatlab = forwardDynamics(modelRobotMatlab,q,dq,tau);

%% IDynTree: WIP, do not consider
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();

q_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);

q_idyn.fromMatlab(q);
dq_idyn.fromMatlab(dq);

grav_idyn = iDynTree.Vector3();
grav_idyn.zero();
grav_idyn.setVal(2, -gravityModulus);

kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

baseAcc_idyn = iDynTree.Vector6();
baseAcc_idyn.zero();

linkExtwrenches = iDynTree.LinkWrenches(kinDynComp.model());
linkExtwrenches.zero();

traversal = iDynTree.Traversal();
kinDynComp.model().computeFullTreeTraversal(traversal);

robotPos = iDynTree.FreeFloatingPos(kinDynComp.model());
robotPos.worldBasePos().Identity();
robotPos.jointPos.fromMatlab(q);

robotVel = iDynTree.FreeFloatingVel(kinDynComp.model());
robotVel.baseVel().zero();
robotVel.jointVel.fromMatlab(dq);

ABA_robotAcc = iDynTree.FreeFloatingAcc(kinDynComp.model());
ABA_jointTorques = iDynTree.JointDOFsDoubleArray(kinDynComp.model());
ABA_jointTorques.fromMatlab(tau);
ABA_bufs = iDynTree.ArticulatedBodyAlgorithmInternalBuffers(kinDynComp.model());

iDynTree.ArticulatedBodyAlgorithm(kinDynComp.model(), traversal, robotPos, robotVel, linkExtwrenches, ABA_jointTorques, ABA_bufs, ABA_robotAcc);

%% Symbolic
symbolicForwardDynamicFunction = urdf2casadi.Dynamics.symbolicForwardDynamics(robotModelURDF,0);
g =[0;0;-gravityModulus]; % Gravity column vector
% Test with symbolic function
jointAccSymbolic = symbolicForwardDynamicFunction(q, dq, g,tau);
jointAccSymbolic = full(jointAccSymbolic);
end