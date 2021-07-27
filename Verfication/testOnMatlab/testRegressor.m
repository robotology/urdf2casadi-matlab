% Test the regressor

location_tests_folder = pwd;
%% Choose a urdf model
kuka_urdf = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];

%% Import necessary functions
import urdf2casadi.Utils.inverseDynamicsInertialParametersRegressor
import urdf2casadi.Utils.iDynTreeDynamicsFunctions.computeRegressorIdynTree

%% Input urdf file to acquire robot structure
robotURDFModel = kuka_urdf;

%% Generate functions
% Fix location folder to store the generated c and .mex files
location_tests_folder = pwd;
location_generated_functions = [location_tests_folder,'/../../automaticallyGeneratedFunctions'];
Y = inverseDynamicsInertialParametersRegressor(robotURDFModel,1,location_generated_functions);

jointPos = [pi/6 0 0 0 0 0]';
jointVel = rand(6,1);
jointAcc = rand(6,1);
gravityModulus = -9.81;
g = [0;0;-gravityModulus];

Y_symb = Y(jointPos,jointVel,jointAcc,g);

Y_IDyn = computeRegressorIdynTree(robotURDFModel,jointPos,jointVel,jointAcc,gravityModulus);

e = abs(Y_IDyn(:,11:end)-full(Y_symb));