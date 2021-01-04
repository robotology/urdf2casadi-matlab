%% Choose a urdf model
location_tests_folder = pwd;
kuka_urdf = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];

%% Input urdf file to acquire robot structure
robotURDFModel = kuka_urdf;

%% Generate functions
% Fix location folder to store the generated c and .mex files
location_generated_functions = [location_tests_folder,'/../../automaticallyGeneratedFunctions'];

[HFunction,HDotFunction,CFunction]= createMassAndCoriolisMatrixFunction(robotURDFModel,1,location_generated_functions);
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,1,location_generated_functions);
opts.geneate_c_code = true;
opts.location_generated_fucntion = location_generated_functions;
opts.FrameVelocityRepresentation = "MIXED_REPRESENTATION";
[jacobian,X,XForce,S,O_X_ee] = createSpatialTransformsFunction(robotURDFModel,opts);

% IDynTree model loader
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotURDFModel);
kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());
base_frame = 0;
ee_frame = 7;
% Relative Jacobian of end-effector frame wrt base frame
J_idyn = iDynTree.MatrixDynSize();

%% Create trajectories for simulation
[smds,model] = extractSystemModel(robotURDFModel);
nrOfJoints = smds.NB;

K = eye(nrOfJoints);

q = efe.dataset.jointPos;
qd = efe.dataset.jointVel;
qdd = efe.dataset.jointAcc;

tau = efe.measuredJointTorques_list' - efe.estimatedFrictionTorques_list';

gravityModulus = 9.81;
g = [0;0;-gravityModulus];

nrOfSamples = max(size(q));
sampling_period = 0.004;

tau_rnea = zeros(nrOfSamples,nrOfJoints);
F_ext = zeros(nrOfSamples,nrOfJoints);
detJacobian_inTime = zeros(1,nrOfSamples);

% Some external force for the simulation
F_ext = zeros(6,nrOfJoints);

for t = 1:nrOfSamples
    tau_rnea(t,:) = rnea(q(t,:),qd(t,:),qdd(t,:),g,F_ext)';
    detJacobian_inTime(t) = det(full(computeNumericalJacobian(q(t,:))));
end


%% Plot results
timesInSeconds = sampling_period*(1:nrOfSamples);
plot_trajectories(q, timesInSeconds,'q');
plot_trajectories(qd, timesInSeconds,'dq');
plot_trajectories(qdd, timesInSeconds,'ddq');
plot_trajectories(tau_rnea, timesInSeconds,'tau_{rnea}');


% Store trajectories as timeseries for simulink
% Filter the data with an exponential filter
q_timeseries = timeseries(q);
qd_timeseries = timeseries(qd);
tauMotor_timeseries = timeseries(tau);
tauFriction_timeseries = timeseries(efe.estimatedFrictionTorques_list');

% After launching this script one can visualze the results also from matlab
% by setting `simulink_finished` to true
simulink_finished = false;
if simulink_finished
    IDEstimation  = expsmooth( efe.estimatedExternalForces_list', 1/sampling_period, 1000 );
    observerEstimation  = expsmooth( out.observerEstimatedForce.Data, 1/sampling_period , 1000 );
    subplot(2,1,1);
    plot(IDEstimation);title('Inverse dynamics estimation');
    grid on;
    legend('force x','force y','force z', 'tau x', 'tau y', 'tau z');
    ylim([-200, 150])
    subplot(2,1,2);
    plot(out.tout,observerEstimation);title('Observer based estimation');
    grid on;
    legend( 'tau x', 'tau y', 'tau z','force x','force y','force z');
    ylim([-200, 150])
end


