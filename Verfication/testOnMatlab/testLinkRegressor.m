%% Test Newton-Euler law on a single link

%% Fix location folder to store the generated c and .mex files
location_tests_folder = pwd;

%% Choose a urdf model
kuka_urdf = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];

%% Input urdf file to acquire robot structure
robotModelURDF = iCub_r_leg;
%Load urdf and convert to SMDS format
smds = extractSystemModel(robotModelURDF);
jointPos = [pi/6 0 0 0 0 0]';
jointVel = rand(6,1);
jointAcc = rand(6,1);
gravityModulus = -9.81;
g = [0;0;-gravityModulus];

% Compute frame transformations, joint motion subspace matrix, link velocity and accelerations
[X,XForce,S,Xup, v, a] = computeKinematics(smds, jointPos, jointVel, jointAcc, g);

% Compute for each link i the matrix A_j for all links j distal to link
% i. This matix arises from recasting the spatial Newton-Euler dynamic equation 
% of each link i in a form linear wrt the Inertial paramter vector of link i, expressed in the local frame i 
for l = 1:smds.NB
    % Gravity in local frame
    g_l = (X{1}{1,l}*Xup{1})*[0;0;0;g];
    % Velocity and acceleration must be expressed in the same frame as the
    % inertia and the center of mass: the body i local frame
    A{l} = computeLinkRegressor( v{l}, a{l});
    % Test the regressor 
    p = [smds.mass{l}; smds.mass{l}*smds.com{l}; smds.I{l}(1,1); smds.I{l}(1,2); smds.I{l}(1,3);...
                           smds.I{l}(2,2); smds.I{l}(2,3); smds.I{l}(3,3)];
    f_regressor = A{l}*p;
    f_NE = smds.I{l}*a{l} + crf(v{l})*smds.I{l}*v{l};   
end


