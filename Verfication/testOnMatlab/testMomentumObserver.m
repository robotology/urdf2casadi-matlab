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
[jacobian,X,XForce,S] = createSpatialTransformsFunction(robotURDFModel,1,location_generated_functions);
%% Create trajectories for simulation
[smds,model] = extractSystemModel(robotURDFModel);
nrOfJoints = smds.NB;
K = eye(nrOfJoints);
if nrOfJoints == 6
    initial_pos = [35.9124; -90.0776; 114.0132; -87.1129; 96.4907; 86.7602];
    final_pos = [50.9124; -90.0776; 114.0132; -87.1129; 96.4907; 86.7602];
end
if nrOfJoints==1
    initial_pos = [35.9124];
    final_pos = [335.9124];
end
trajectoryDuration = 10;
sampling_period = 0.01;
q = romualdi2020_generate_min_jerk_trajectories(initial_pos,...
                            zeros(nrOfJoints,1), zeros(nrOfJoints,1), final_pos, zeros(nrOfJoints,1), zeros(nrOfJoints,1),...
                            trajectoryDuration, sampling_period);
qd = [zeros(1,nrOfJoints);
      diff(q)];
qdd = [zeros(1,nrOfJoints);
      diff(qd)];

gravityModulus = 0;
g = [0;0;-gravityModulus];
nrOfSamples = trajectoryDuration/sampling_period +1;
tau_rnea = zeros(nrOfSamples,nrOfJoints);
tau_ext = zeros(nrOfSamples,nrOfJoints);
jacobian_inTime = zeros(6,nrOfJoints,nrOfSamples);
% Some external force for the simulation
F_ext = zeros(6,nrOfJoints);
% Test non null external force pushing against the end effector
% recall that we are using spatial forces as described in Featherstone(2008)
F_ext(:,end) = [0 0 0 10 10 0]';

for t = 1:nrOfSamples
    tau_rnea(t,:) = rnea(q(t,:),qd(t,:),qdd(t,:),g,F_ext)';
    tau_ext(t,:)  = (full(jacobian(q(t,:))).'*F_ext(:,end))';
    jacobian_inTime(:,:,t) = full(jacobian(q(t,:)));
end
% tau = computeInverseDynamicsIDynTree(robotURDFModel,q,qd,qdd,gravityModulus);

%% Simulate in MATLAB
% simulateObserverMatlab;

%% Plot results
timesInSeconds = sampling_period*(1:nrOfSamples);
plot_trajectories(q, timesInSeconds,'q');
plot_trajectories(qd, timesInSeconds,'dq');
plot_trajectories(qdd, timesInSeconds,'ddq');
% plot_trajectories(tau, timesInSeconds,'tau');
plot_trajectories(tau_rnea, timesInSeconds,'tau_{rnea}');
plot_trajectories(tau_ext, timesInSeconds,'tau_{ext}');


% Store trajectories as timeseries for simulink
% Filter the data with an exponential filter
q_data = timeseries(q);
qd_data = timeseries(qd);
qdd_data = timeseries(qdd);
tau_data = timeseries(tau_rnea);


% N = size(dataset.timestampInSeconds,1);
% for i = 1:N
%     measuredJointTorques(:,i) = Kt*K_gear_coup*dataset.trqSetMotorSide(i,:)';
% end
