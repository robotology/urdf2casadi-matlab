function [tau_iDynTree, tau_symbolic_function] = compareIDyntreeVSSymbolic(jointPos,jointVel,jointAcc,gravityAccelerationModulus, robotModelURDF)
%% IDyntree
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();
nrOfLinks = nrOfJoints + 1;

baseAcc_idyn = iDynTree.Vector6();
baseAcc_idyn.zero();
linkExtForces = iDynTree.LinkWrenches(kinDynComp.model());
linkExtForces.zero();
output_FreeFloatingGeneralizedTorques = iDynTree.FreeFloatingGeneralizedTorques(kinDynComp.model());
q_idyn = iDynTree.VectorDynSize(nrOfJoints);
q_idyn.fromMatlab(jointPos);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn.fromMatlab(jointVel);
ddq_idyn = iDynTree.VectorDynSize(nrOfJoints);
ddq_idyn.fromMatlab(jointAcc);

grav_idyn = iDynTree.Vector3();

grav_idyn.zero();

grav_idyn.setVal(2, -gravityAccelerationModulus);
kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

% Compute joint torques
kinDynComp.inverseDynamics(baseAcc_idyn, ddq_idyn, linkExtForces, output_FreeFloatingGeneralizedTorques);

% Display the result
tau_iDynTree = output_FreeFloatingGeneralizedTorques.jointTorques().toMatlab;

%% Compute the symbolic model
% Compute model from urdf
% generate the c code only the first time
persistent firstTime
if isempty(firstTime)
    firstTime = 1;
else
    firstTime = 0;
end
symbolicDynamicFunction = urdf2eomID(robotModelURDF,firstTime);
g =[0;0;-gravityAccelerationModulus]; % Gravity column vector
% The external forces is a vector of (6,1). It has to be one per
% link, expect the base link(which for now is considered fixed)
% Test with the compiled function 
% If we do not supply the external forces they are automatically considered null in the compiled function
tau_symbolic_compiled = rnea('rnea',jointPos, jointVel, jointAcc, g);
% Test with symbolic function
% If we do not supply the external forces they are NOT automatically considered null in the symbolic function
extForce = zeros(6,nrOfJoints);

tau_symbolic_function = symbolicDynamicFunction(jointPos, jointVel, jointAcc, g,extForce);

tau_symbolic_function = full(tau_symbolic_function);
% %% Check results
% plot(abs(tau_iDynTree-tau_symbolic_function));
disp('IDyntree:')
disp(tau_iDynTree)
disp('Symbolic:')
disp(tau_symbolic_function)
end