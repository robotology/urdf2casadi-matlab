%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/kuka_kr210.urdf';
robotURDFModel = kuka_urdf;

%% Generate functions
location_generated_fucntion = '/home/iiticublap041/baljinder/urdf2casadi-matlab/automaticallyGeneratedFunctions';
[HFunction,HDotFunction,CFunction]= createMassAndCoriolisMatrixFunction(robotURDFModel,1,location_generated_fucntion);
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,1,location_generated_fucntion);
[jacobian,X,XForce,S,O_X_ee] = createSpatialTransformsFunction(robotURDFModel,1,location_generated_fucntion);



%% Create trajectories for simulation
[smds,model] = extractSystemModel(robotURDFModel);
nrOfJoints = smds.NB;

K = eye(nrOfJoints);

q = efe.dataset.jointPos;
qd = efe.dataset.jointVel;
qdd = efe.dataset.jointAcc;

tau = efe.measuredJointTorques_list' - efe.estimatedFrictionTorques_list';

gravityModulus = 0;
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
plot_joint_trajectories(q, timesInSeconds,'q');
plot_joint_trajectories(qd, timesInSeconds,'dq');
plot_joint_trajectories(qdd, timesInSeconds,'ddq');
plot_joint_trajectories(tau_rnea, timesInSeconds,'tau_{rnea}');


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
    IDEstimation  = expsmooth( efe.estimatedExternalForces_list(:,1:29160)', 1/sampling_period, 1000 );
    observerEstimation  = expsmooth( out.observerEstimatedForce.Data, 1/sampling_period , 1000 );
    subplot(2,1,1);
    plot(IDEstimation);title('Inverse dynamics estimation');
    legend('force x','force y','force z', 'tau x', 'tau y', 'tau z');
    subplot(2,1,2);
    plot(observerEstimation);title('Observer based estimation');
    legend('force x','force y','force z', 'tau x', 'tau y', 'tau z');
end

