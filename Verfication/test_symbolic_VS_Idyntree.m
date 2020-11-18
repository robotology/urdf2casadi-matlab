%% This is a script to compare the result of the symbolic expression with the result obtained using iDynTree(https://github.com/robotology/idyntree)

% Clear workspace
clear all;
% Add casadi to Matlab path
locationCasADi = '/home/iiticublap041/casadi-linux-matlabR2014b-v3.5.3';
addpath(locationCasADi);
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
location_tests_folder = pwd;
twoLink_urdf = [location_tests_folder,'/../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../URDFs/kuka_kr210.urdf'];

%% input urdf file to acquire robot structure
robotModelURDF = kuka_urdf;
location_generated_functions = '/home/iiticublap041/baljinder/urdf2casadi-matlab/automaticallyGeneratedFunctions';
%% Set the ID inputs for both iDyntree and symbolic function
smds = extractSystemModel(robotModelURDF);
nrOfJoints = smds.NB;
nrOfTests = 20;
iDynResult_list = zeros(nrOfJoints,nrOfTests);
symbolcResult_list = zeros(nrOfJoints,nrOfTests);
jointAccMatlab_list = zeros(nrOfJoints,nrOfTests);
jointAccSymbolic_list = zeros(nrOfJoints,nrOfTests);
tau_regressor_list = zeros(nrOfJoints,nrOfTests);
tau_RNEA_list = zeros(nrOfJoints,nrOfTests);
gravityModulus = 9.80665;
id = true;
fd = false;
dynamicRegressor = true;

if id
    %% Compute the symbolic model
    symbolicDynamicFunction = symbolicInverseDynamics(robotModelURDF,0, location_generated_functions);
    % The external forces is a vector of (6,1). It has to be one per
    % link, expect the base link(which for now is considered fixed)
    extForce = zeros(6,nrOfJoints);
    g = [0, 0, -gravityModulus]';
    for i = 1:nrOfTests
        jointPos = [pi/6 0 0 0 0 0]';
        jointPos = rand(nrOfJoints,1);
        jointVel = rand(nrOfJoints,1);
        jointAcc = rand(nrOfJoints,1);
        
        tau_symbolic_function = symbolicDynamicFunction(jointPos, jointVel, jointAcc, g,extForce);
        tau_symbolic_function = full(tau_symbolic_function);
        
        iDynResult_list(:,i) = computeInverseDynamicsIDynTree(robotModelURDF,jointPos',jointVel',jointAcc',gravityModulus);
        symbolcResult_list(:,i)= tau_symbolic_function;
    end
    eps_t1 = abs(iDynResult_list'-symbolcResult_list');
    figure;
    plot(eps_t1);title('Inverse Dynamics: abs(tau_{iDyn} - tau_{symb})');legend;   
end
if fd
     %Use Matlab Robotic Toolbox                                                                                                                               
    modelRobotMatlab = importrobot(robotModelURDF);                                                                                                          
    fext_matlab = zeros(nrOfJoints,6);                                                                                                                      
    modelRobotMatlab.DataFormat = 'column';                                                                              
    modelRobotMatlab.Gravity = [0 0 -gravityModulus]; 
    
    for i = 1:nrOfTests
        jointPos = rand(nrOfJoints,1);
        jointVel = rand(nrOfJoints,1);
        tau      = rand(nrOfJoints,1);

        [jointAccMatlab, jointAccSymbolic] = ...
        computeIDyntreeFD_VS_Symblic(modelRobotMatlab, jointPos,jointVel,gravityModulus,tau,robotModelURDF);
        jointAccMatlab_list(:,i) = jointAccMatlab;
        jointAccSymbolic_list(:,i)= jointAccSymbolic;
    end
    
    eps_t2 = abs(jointAccMatlab_list'-jointAccSymbolic_list');
    figure;
    plot(eps_t2);title('Forward Dynamics: abs(ddq_{matlab} - ddq_{symb})');legend;    
end
if dynamicRegressor
     
    % Sumbolic Inverse Dyncamics function
    symbolicIDFunction = symbolicInverseDynamics(robotModelURDF,0,location_generated_functions);
    % Gravity column vector
    g =[0;0;-gravityModulus];
    % Test with symbolic function
    % If we do not supply the external forces they are NOT automatically considered null in the symbolic function
    nrOfJoints = size(jointPos,1);
    extForce = zeros(6,nrOfJoints);
     
    % Rettrive Inertia parameters
    smds = extractSystemModel(robotModelURDF);
    for i = 1:nrOfJoints
        p(:,i) = [smds.mass{i}; smds.mass{i}*smds.com{i}; smds.I{i}(1,1); smds.I{i}(1,2); smds.I{i}(1,3);...
                               smds.I{i}(2,2); smds.I{i}(2,3); smds.I{i}(3,3)]; 
    end
    inertiaParameters = reshape(p,[],1);
    % Symbolic regressor
    Y = inverseDynamicsInertialParametersRegressor(robotModelURDF,0,location_generated_functions);

     for i = 1:nrOfTests
        jointPos = rand(nrOfJoints,1);
        jointVel = rand(nrOfJoints,1);
        jointAcc = rand(nrOfJoints,1);

        tau_RNEA = symbolicIDFunction(jointPos, jointVel, jointAcc, g,extForce);
        tau_RNEA = full(tau_RNEA);
        
        tau_regressor = Y(jointPos, jointVel, jointAcc, g)*inertiaParameters;
        tau_regressor = full(tau_regressor);
        
        tau_regressor_list(:,i) = tau_regressor;
        tau_RNEA_list(:,i) = tau_RNEA;
    end
    eps_t3 = abs(tau_RNEA_list'-tau_regressor_list');
    figure;
    plot(eps_t3);title('abs(tau_{RNEA} - tau_{regressor})');legend;   
end
