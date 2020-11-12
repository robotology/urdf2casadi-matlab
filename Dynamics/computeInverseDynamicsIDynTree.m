function tau = computeInverseDynamicsIDynTree(robotModelURDF,jointPos,jointVel,jointAcc,gravityModulus)
%Compute the inverse dynamics using IDynTree. Takes as input trajectories
%of [nrOfSamples x nrOfJoints] size.

% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();

baseAcc_idyn = iDynTree.Vector6();
baseAcc_idyn.zero();

linkExtForces = iDynTree.LinkWrenches(kinDynComp.model());
linkExtForces.zero();

output_FreeFloatingGeneralizedTorques = iDynTree.FreeFloatingGeneralizedTorques(kinDynComp.model());

q_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
ddq_idyn = iDynTree.VectorDynSize(nrOfJoints);

grav_idyn = iDynTree.Vector3();
grav_idyn.zero();
grav_idyn.setVal(2, -gravityModulus);

nrOfSamples = size(jointVel,1);
tau = zeros(nrOfSamples,nrOfJoints);
for t = 1:nrOfSamples
    q_idyn.fromMatlab(jointPos(t,:)');
    dq_idyn.fromMatlab(jointVel(t,:)');
    ddq_idyn.fromMatlab(jointAcc(t,:)');
    kinDynComp.setRobotState(q_idyn, dq_idyn, grav_idyn);

    % Compute joint torques
    kinDynComp.inverseDynamics(baseAcc_idyn, ddq_idyn, linkExtForces, output_FreeFloatingGeneralizedTorques);

    % The result
    tau(t,:) = output_FreeFloatingGeneralizedTorques.jointTorques().toMatlab';
end
end