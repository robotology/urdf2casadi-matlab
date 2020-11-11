%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/kuka_kr210.urdf';
robotURDFModel = kuka_kr210;
pDotFunction = createGeneralizedMomentumDerivativeFunction(robotURDFModel);
pFunction = createGeneralizedMomentumFunction(robotURDFModel);

% Input to the integrator [q;qd;qdd;g;tau;r]
integralOfpHatDotFunction = createIntegralOfGeneralizedMomentumDotFunction(robotURDFModel);

% Generate functions
[HFunction,HDotFunction,CFunction]= createMassAndCoriolisMatrixFunction(robotURDFModel,1);
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,1);
%% Simulate with null points 
simple_test = false;
if simple_test
    N = 10;
    nrOfJoints = 6;
    K = eye(nrOfJoints);
    int = 0;
    q = zeros(nrOfJoints,N);
    qd = zeros(nrOfJoints,N);
    qdd = zeros(nrOfJoints,N);
    gravityModulus = 9.81;
    g = [0;0;-gravityModulus];
    tau = zeros(nrOfJoints,N);
    r = zeros(nrOfJoints,N+1);
    r = mat2cell(r,[nrOfJoints], ones(1,N+1));
    for t =1:N
        res = integralOfpHatDotFunction('x0',0,'p',[q(:,t);qd(:,t);qdd(:,t);g;tau(:,t);r{:,t}]);
        int = int + res.xf;
        r{:,t+1} = K*(pFunction(q(:,t),qd(:,t)) - int - pFunction(q(:,1),qd(:,1)));
    end
    rf = full([r{:}]);
else
    %% Load dataset of joint position, velocity and torques
    location_dataset = '/home/iiticublap041/idjl-model-identification/results/data_processing_results/2020_09_11_robotDataDumper.mat';
    load(location_dataset);
    % Retrive data
    q = dataset.jointPos';
    qd = dataset.jointVel';
    qdd = dataset.jointAcc';
    tau = dataset.trqSetMotorSide';
    N = size(dataset.timestampInSeconds,1);
    % Init parameters
    nrOfJoints = 6;
    K = 0.1*eye(nrOfJoints);
    int = 0;
    gravityModulus = 9.81;
    g = [0;0;-gravityModulus];
    r = zeros(nrOfJoints,N+1);
    r = mat2cell(r,[nrOfJoints], ones(1,N+1));
    for t =1:N
        res = integralOfpHatDotFunction('x0',pDotFunction(q(:,t),qd(:,t),g,tau(:,t),r{:,t}),...
                           'p',[q(:,t);qd(:,t);qdd(:,t);g;tau(:,t);r{:,t}]);
        int = int + res.xf;
        r{:,t+1} = K*(pFunction(q(:,t),qd(:,t)) - int - pFunction(q(:,1),qd(:,1)));
    end
    rf = full([r{:}]);
end

%% Load dataset for simulink
% Filter the data with an exponential filter
q_data = timeseries(dataset.jointPos(50:end,:));
qd_data = expsmooth(dataset.jointVel, 250, 1000);
qd_data = timeseries(qd_data);
tau_data = expsmooth(dataset.trqSetMotorSide, 250, 1000);
tau_data = timeseries(tau_data);

% fun = @(x,g) pDotFunction(x(1:nrOfJoints),x(nrOfJoints+1:2*nrOfJoints),g,x(2*nrOfJoints+1:3*nrOfJoints),x(3*nrOfJoints+1:4*nrOfJoints));
% t = 1;
% xmin = [q(:,t);qd(:,t);qdd(:,t);tau(:,t);r{:,t}];
% t = 5;
% xmax = [q(:,t);qd(:,t);qdd(:,t);tau(:,t);r{:,t}];
% xmax = full(xmax);
% q = integral(@(x)fun(x,g),1,2);
