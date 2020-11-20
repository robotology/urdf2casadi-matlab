%% Test the regressor
locationCasADi = '/home/iiticublap041/casadi-linux-matlabR2014b-v3.5.3';
addpath(locationCasADi);
% URDF for test
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/kuka_kr210.urdf';
robotURDFModel = kuka_kr210;
Y = inverseDynamicsInertialParametersRegressor(robotURDFModel,1);

jointPos = [pi/6 0 0 0 0 0]';
jointVel = rand(6,1);
jointAcc = rand(6,1);
gravityModulus = -9.81;
g = [0;0;-gravityModulus];

Y_symb = Y(jointPos,jointVel,jointAcc,g);

Y_IDyn = computeRegressorIdynTree(robotURDFModel,jointPos,jointVel,jointAcc,gravityModulus);

e = abs(Y_IDyn(:,11:end)-full(Y_symb));