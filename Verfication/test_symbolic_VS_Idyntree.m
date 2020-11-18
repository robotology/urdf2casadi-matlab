%% This is a script to compare the result of the symbolic expression with the result obtained using iDynTree(https://github.com/robotology/idyntree)
% addpath('xml2struct')
% addpath('URDFs')
% addpath('Spatial')
% Add casadi to Matlab path
locationCasADi = '/home/iiticublap041/casadi-linux-matlabR2014b-v3.5.3';
addpath(locationCasADi);
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
location_tests_folder = pwd;
twoLink_urdf = [location_tests_folder,'/../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../URDFs/iCub_r_leg.urdf'];
location_generated_fucntion = [location_tests_folder,'/../automaticallyGeneratedFunctions'];
%% input urdf file to acquire robot structure
robotModelURDF = iCub_r_leg;

%% Get number of joints using iDynTree
mdlLoader = iDynTree.ModelLoader();
mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

nrOfJoints = kinDynComp.model().getNrOfDOFs();
%% Set the ID inputs for both iDyntree and symbolic function
nrOfTests = 5;
iDynResult = zeros(nrOfJoints,nrOfTests);
symbolcResult = zeros(nrOfJoints,nrOfTests);
jointAccMatlab_list = zeros(nrOfJoints,nrOfTests);
jointAccSymbolic_list = zeros(nrOfJoints,nrOfTests);
gravityModulus = 9.80665;
%Use Matlab Robotic Toolbox
% modelRobotMatlab = importrobot(robotModelURDF);
% fext_matlab = zeros(nrOfJoints,6);
% modelRobotMatlab.DataFormat = 'column';
% modelRobotMatlab.Gravity = [0 0 -gravityModulus];

id = true;
fd = false;
dynamicRegressor = false;
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
        [tau_iDynTree, tau_symbolic_function] = compareIDyntreeVSSymbolic(jointPos,jointVel,jointAcc,gravityModulus,robotModelURDF,location_generated_fucntion);
        iDynResult(:,i) = tau_iDynTree;
        symbolcResult(:,i)= tau_symbolic_function;
         %jointTorqMatlab(:,i) = inverseDynamics(model_matlab,jointPos,jointVel,jointAcc,fext_matlab);
        [tau_regressor, tau_RNEA] = computeIDWithDynamicRegressorAndRNEA(jointPos,jointVel,jointAcc, gravityModulus,robotModelURDF,location_generated_fucntion);
        tau_regressor_list(:,i) = tau_regressor;
        tau_RNEA_list(:,i) = tau_RNEA;
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
        computeIDyntreeFD_VS_Symblic(modelRobotMatlab, jointPos,jointVel,gravityModulus,tau,robotModelURDF);
        jointAccMatlab_list(:,i) = jointAccMatlab;
        jointAccSymbolic_list(:,i)= jointAccSymbolic;
        disp('Matlab:')
        disp(jointAccMatlab_list(:,i))
        disp('Symbolic:')
        disp(jointAccSymbolic_list(:,i))
    end

    if dynamicRegressor
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
    [tau_regressor, tau_RNEA] = computeIDWithDynamicRegressorAndRNEA(jointPos,jointVel,jointAcc, gravityModulus,robotModelURDF);
    tau_regressor_list(:,i) = tau_regressor;
    tau_RNEA_list(:,i) = tau_RNEA;
    end
end
if id 
    clear firstTime 
    figure;
    eps_t1 = abs(iDynResult'-symbolcResult');
    plot(eps_t1);title('abs(iDynResult - symbolcResult)');legend;
    eps_t3 = abs(tau_RNEA_list'-tau_regressor_list');
    figure;
    plot(eps_t3);title('abs(tau_{RNEA} - tau_{regressor})');legend;  
end
if fd
    clear firstTime 
    eps_t2 = abs(jointAccMatlab_list'-jointAccSymbolic_list');
    plot(eps_t2);title('abs(jointAccMatlab - jointAccSymbolic)');legend;    
end
if dynamicRegressor
    clear firstTime 
    eps_t3 = abs(tau_RNEA_list'-tau_regressor_list');
    figure;
    plot(eps_t3);title('abs(tau_{RNEA} - tau_{regressor})');legend;    
end
%% Use Matlab RoboticToolbox
% eps_matlab = abs(jointTorqMatlab'-symbolcResult');
% eps_idynMatlab = abs(iDynResult'-jointTorqMatlab');
% plot(eps_matlab);title('abs(jointTorqMatlab - symbolcResult)');legend;
% plot(eps_idynMatlab);title('abs(iDynResult - jointTorqMatlab)');legend;