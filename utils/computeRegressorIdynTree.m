function regressorInertiaSample = computeRegressorIdynTree(robotURDFModel,jointPos,jointVel,jointAcc,gravityModulus)
%% IDyntree
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotURDFModel);
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
grav_idyn.setVal(2, -gravityModulus);

kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);


q_idyn.fromMatlab(jointPos);
dq_idyn.fromMatlab(jointVel);
ddq_idyn.fromMatlab(jointAcc);

kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

regr_idyn = iDynTree.MatrixDynSize();
kinDynComp.inverseDynamicsInertialParametersRegressor(baseAcc_idyn, ddq_idyn, regr_idyn);

% Remove the first six rows that are related to the base dynamics
regrBuf =  regr_idyn.toMatlab();
regressorInertiaSample = regrBuf(7:end, :);
end