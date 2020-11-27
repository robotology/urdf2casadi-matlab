function tau = computeGeneralizedBiasForceIDynTree(robotURDFModel,jointPos,jointVel,gravityModulus)
%Compute the generalized bias forces using IDyntree
% f = C(q,qd)*qd + g(q)
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotURDFModel);
kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();
nrOfLinks = nrOfJoints + 1;

baseAcc_idyn = iDynTree.Vector6();
baseAcc_idyn.zero();


q_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
ddq_idyn = iDynTree.VectorDynSize(nrOfJoints);
q_idyn.fromMatlab(jointPos);
dq_idyn.fromMatlab(jointVel);
ddq_idyn.zero();

grav_idyn = iDynTree.Vector3();
grav_idyn.zero();
grav_idyn.setVal(2, -gravityModulus);

kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

f = iDynTree.FreeFloatingGeneralizedTorques();
f.resize(mdlLoader.model());
% Set BODY_FIXED_REPRESENTATION frame representation to use Featherstone's
% notation. See https://robotology.github.io/idyntree/master/namespaceiDynTree.html#a0089c6dac34bc6a4f623d9e9e565375a
kinDynComp.setFrameVelocityRepresentation(iDynTree.BODY_FIXED_REPRESENTATION);
kinDynComp.generalizedBiasForces(f);
% Convert to  MATLAB matrix
tau_IDyn = f.jointTorques();
tau = tau_IDyn.toMatlab();
end