%% Test on Simulink Algorithm 1 from  "Numerical Methods to Compute the Coriolis Matrix and Christoffel Symbols for Rigid-Body System" 
% by Sebastian Echeandia and Patrick M. Wensing
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/kuka_kr210.urdf';
robotURDFModel = kuka_kr210;

%% Generate functions
location_generated_fucntion = '/home/iiticublap041/baljinder/urdf2casadi-matlab/automaticallyGeneratedFunctions';
[HFunction,HDotFunction,CFunction]= createMassAndCoriolisMatrixFunction(robotURDFModel,1,location_generated_fucntion);
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,1,location_generated_fucntion);
[jacobian,X,XForce,S] = createSpatialTransformsFunction(robotURDFModel,1,location_generated_fucntion);
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

jacobian_inTime = zeros(6,nrOfJoints,nrOfSamples);
% Some external force for the simulation
F_ext = zeros(6,nrOfJoints);
for t = 1:nrOfSamples
    tau_rnea(t,:) = rnea(q(t,:),qd(t,:),qdd(t,:),g,F_ext)';
end
%% Plot results
timesInSeconds = sampling_period*(1:nrOfSamples);
plot_trajectories(q, timesInSeconds,'q');
plot_trajectories(qd, timesInSeconds,'dq');
plot_trajectories(qdd, timesInSeconds,'ddq');
plot_trajectories(tau_rnea, timesInSeconds,'tau_{rnea}');


% Store trajectories as timeseries for simulink
q_data = timeseries(q);
qd_data = timeseries(qd);
qdd_data = timeseries(qdd);
tau_data = timeseries(tau_rnea);

