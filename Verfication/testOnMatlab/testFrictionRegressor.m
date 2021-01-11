% Test the regressor

location_tests_folder = pwd;
%% Choose a urdf model
kuka_urdf = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];

%% Input urdf file to acquire robot structure
robotURDFModel = kuka_urdf;

%% Generate functions
% Fix location folder to store the generated c and .mex files
location_tests_folder = pwd;
location_generated_functions = [location_tests_folder,'/../../automaticallyGeneratedFunctions'];
nrOfTrajectoryPoints = 10;
Y = computeSymbolicStackOfFrictionRegressorsTransposed (robotURDFModel,nrOfTrajectoryPoints,...
                             0,location_generated_functions);
jointPos = rand(6,nrOfTrajectoryPoints);
K_v = rand(6,1);
delta = rand(6,1);

Y_symb = Y(jointPos,K_v,delta);
