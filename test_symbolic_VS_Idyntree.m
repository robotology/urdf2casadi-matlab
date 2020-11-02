%% This is a script to compare the result of the symbolic expression with the result obtained using iDynTree(https://github.com/robotology/idyntree)
% addpath('xml2struct')
% addpath('URDFs')
% addpath('Spatial')

%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2eom/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2eom/URDFs/kuka_kr210.urdf';

%% input urdf file to acquire robot structure
robotModelURDF = kuka_kr210;

%% Set the ID inputs for both iDyntree and symbolic function
nrOfTests = 20;
nrOfJoints = 6;
iDynResult = zeros(nrOfJoints,nrOfTests);
symbolcResult = zeros(nrOfJoints,nrOfTests);
jointAccMatlab_list = zeros(nrOfJoints,nrOfTests);
jointAccSymbolic_list = zeros(nrOfJoints,nrOfTests);
gravityAccelerationModulus = 9.80665;
% Use Matlab Robotic Toolbox
modelRobotMatlab = importrobot(robotModelURDF);
fext_matlab = zeros(nrOfJoints,6);
modelRobotMatlab.DataFormat = 'column';
modelRobotMatlab.Gravity = [0 0 -gravityAccelerationModulus];

id = true;
fd = false;
for i = 1:nrOfTests
    if id
        jointVel = zeros(nrOfJoints, 1);
        jointAcc = zeros(nrOfJoints, 1);
        if nrOfJoints ==6
            jointPos = [pi/6 0 0 0 0 0]';
            jointVel = rand(6,1);
            jointAcc = rand(6,1);
        elseif nrOfJoints ==1
            jointPos = rand;
            jointVel = rand;
            jointAcc = rand;
        end
        [tau_iDynTree, tau_symbolic_function] = compareIDyntreeVSSymbolic(jointPos,jointVel,jointAcc,gravityAccelerationModulus,robotModelURDF);
        iDynResult(:,i) = tau_iDynTree;
        symbolcResult(:,i)= tau_symbolic_function;
        % jointTorqMatlab(:,i) = inverseDynamics(model_matlab,jointPos,jointVel,jointAcc,fext_matlab);
    end
    if fd
        if nrOfJoints ==6
            jointPos = [0 0 0 0 0 0]';
            jointVel = rand(6,1);
            tau      = rand(nrOfJoints,1);
        elseif nrOfJoints ==1
            jointPos = 1;
            jointVel = 1;
            tau = 100;
        end
        
        [jointAccMatlab, jointAccSymbolic] = ...
        computeIDyntreeFD_VS_Symblic(modelRobotMatlab, jointPos,jointVel,gravityAccelerationModulus,tau,robotModelURDF);
        jointAccMatlab_list(:,i) = jointAccMatlab;
        jointAccSymbolic_list(:,i)= jointAccSymbolic;
        disp('Matlab:')
        disp(jointAccMatlab_list(:,i))
        disp('Symbolic:')
        disp(jointAccSymbolic_list(:,i))
    end

end
if id 
    clear firstTime 
    eps_t = abs(iDynResult'-symbolcResult');
    plot(eps_t);title('abs(iDynResult - symbolcResult)');legend;
end
if fd
    clear firstTime 
    eps_t = abs(jointAccMatlab_list'-jointAccSymbolic_list')'./jointAccMatlab_list;
    plot(eps_t);title('abs(jointAccMatlab - jointAccSymbolic)');legend;    
end

% epsT(:,:,count) = [eps_t];
% count = count +1;
%% Use Matlab RoboticToolbox
% eps_matlab = abs(jointTorqMatlab'-symbolcResult');
% eps_idynMatlab = abs(iDynResult'-jointTorqMatlab');
% plot(eps_matlab);title('abs(jointTorqMatlab - symbolcResult)');legend;
% plot(eps_idynMatlab);title('abs(iDynResult - jointTorqMatlab)');legend;