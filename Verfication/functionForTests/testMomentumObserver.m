%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/kuka_kr210.urdf';
robotURDFModel = kuka_urdf;

%% Generate functions
location_generated_fucntion = '/home/iiticublap041/baljinder/urdf2casadi-matlab/automaticallyGeneratedFunctions';
[HFunction,HDotFunction,CFunction]= createMassAndCoriolisMatrixFunction(robotURDFModel,1,location_generated_fucntion);
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,1,location_generated_fucntion);
pFunction = createGeneralizedMomentumFunction(robotURDFModel);
pDotFunction = createGeneralizedMomentumDerivativeFunction(robotURDFModel);
pFunction = createGeneralizedMomentumFunction(robotURDFModel);
% Input to the integrator [q;qd;qdd;g;tau;r]
integralOfpHatDotFunction = createIntegralOfGeneralizedMomentumDotFunction(robotURDFModel);
[X,XForce,S] = createSpatialTransformsFunction(robotURDFModel,0,location_generated_fucntion);
%% Create trajectories for simulation
[smds,model] = extractSystemModel(robotURDFModel);
nrOfJoints = smds.NB;
K = eye(nrOfJoints);
if nrOfJoints == 6
    initial_pos = [35.9124; -90.0776; 114.0132; -87.1129; 96.4907; 86.7602];
    final_pos = [135.9124; -90.0776; 114.0132; -87.1129; 96.4907; 86.7602];
end
if nrOfJoints==1
    initial_pos = [35.9124];
    final_pos = [335.9124];
end
trajectoryDuration = 1;
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
% Some external force for the simulation
F_ext = zeros(6,nrOfJoints);
% Test non null external force pushing against the end effector
% recall that we are using spatial forces as described in Featherstone(2008)
% F_ext(:,end) = [0 0 0 0 0 100]';
J = [S{:}];
tau_ext = J.'*F_ext(:,end);
for t = 1:nrOfSamples
    tau_rnea(t,:) = rnea(q(t,:),qd(t,:),qdd(t,:),g,F_ext)';
end
tau = computeInverseDynamicsIDynTree(robotURDFModel,q,qd,qdd,gravityModulus);

%% Simulate with null points 
simple_test = false;
if simple_test
    N = 10;
    int = 0;
    q = zeros(nrOfJoints,N);
    qd = zeros(nrOfJoints,N);
    qdd = zeros(nrOfJoints,N);
    tau = zeros(nrOfJoints,N);
    r = zeros(nrOfJoints,N+1);
    r = mat2cell(r,[nrOfJoints], ones(1,N+1));
    for t =1:N
        res = integralOfpHatDotFunction('x0',0,...
                           'p',[q(:,t);qd(:,t);qdd(:,t);g;tau(:,t);r{:,t}]);
        int = int + res.xf;
        int_list(:,t)= full(int);
        r{:,t+1} = K*(pFunction(q(:,t),qd(:,t)) - int - pFunction(q(:,1),qd(:,1)));
    end
    rf = full([r{:}]);
else
    %% Load dataset of joint position, velocity and torques
    N = nrOfSamples;
    generalizedMomentum = zeros(N,nrOfJoints);
    r = zeros(nrOfJoints,N+1);
    r = mat2cell(r,[nrOfJoints], ones(1,N+1));
    int = 0;
    for t =1:N
        res = integralOfpHatDotFunction('x0',0,'p',[q(t,:)';qd(t,:)';qdd(t,:)';g;tau(t,:)';r{:,t}]);
        int = int + res.xf;
        r{:,t+1} = K*(pFunction(q(t,:),qd(t,:)) - int - pFunction(q(1,:),qd(1,:)));
        generalizedMomentum(t,:) = full(pFunction(q(t,:),qd(t,:)))';
    end
    rf = full([r{:}]);
    rf = rf(:,2:end)';
end
%% Plot results
timesInSeconds = sampling_period*(1:nrOfSamples);
plot_joint_trajectories(q, timesInSeconds,'q');
plot_joint_trajectories(qd, timesInSeconds,'dq');
plot_joint_trajectories(qdd, timesInSeconds,'ddq');
plot_joint_trajectories(tau, timesInSeconds,'tau');
plot_joint_trajectories(tau_rnea, timesInSeconds,'tau_{rnea}');
plot_joint_trajectories(rf, timesInSeconds,'r(t)');

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
