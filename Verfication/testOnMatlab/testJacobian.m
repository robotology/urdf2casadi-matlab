%% Choose a urdf model
location_tests_folder = pwd;
kuka_urdf = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];

%% Input urdf file to acquire robot structure
robotURDFModel = kuka_kr210;

%% Generate functions
% Fix location folder to store the generated c and .mex files
location_generated_functions = [location_tests_folder,'/../../automaticallyGeneratedFunctions'];
opts.geneate_c_code = true;
opts.location_generated_fucntion = location_generated_functions;
opts.FrameVelocityRepresentation = "MIXED_REPRESENTATION";
[J_symb,X,XForce,S,O_X_ee] = urdf2casadi.Dynamics.auxiliarySymbolicDynamicsFunctions.createSpatialTransformsFunction(robotURDFModel,opts);

% IDynTree variables
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotURDFModel);
kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());

% Set the body-fixed frame (left-trivialized velocity) representation
kinDynComp.setFrameVelocityRepresentation(iDynTree.MIXED_REPRESENTATION);
model_iDyn = kinDynComp.model();

nrOfJoints = model_iDyn.getNrOfJoints();
q_idyn = iDynTree.VectorDynSize(nrOfJoints);
dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
grav = iDynTree.Vector3();
J_idyn = iDynTree.MatrixDynSize();


% Constants
gravityModulus = 9.80665;
baseFrame = 0;
eeFrame = 6;
nrOfTests = 10;
J_iDyn_list = zeros(nrOfJoints,nrOfJoints,nrOfTests);
J_symb_list = zeros(nrOfJoints,nrOfJoints,nrOfTests);
e_jac = zeros(1,nrOfTests);
for i = 1:nrOfTests
    
    jointPos = rand(nrOfJoints,1);
    
    % Compute jacobian with iDynTree
    q_idyn.fromMatlab(jointPos);
    dq_idyn.fromMatlab(zeros(nrOfJoints,1));
    grav.fromMatlab([0;0;-gravityModulus]);

    kinDynComp.setRobotState(q_idyn, dq_idyn, grav);
    kinDynComp.getRelativeJacobian(baseFrame, eeFrame, J_idyn);
    J_tmp = J_idyn.toMatlab();
    
    % Featherstone's notation considers first the angular part
    J_iDyn_list(4:6,:,i) = J_tmp(1:3,:);
    J_iDyn_list(1:3,:,i) = J_tmp(4:6,:);

    % Compute Jacobian with symbolic function
    J_symb_list(:,:,i) = full(J_symb(jointPos));
    
    % Error in the norm of the difference of the Jacobians
    e_jac(:,i)  = norm(J_symb_list(:,:,i) - J_iDyn_list(:,:,i));

end
% Plot error
plot(e_jac'); title('Error: norm(J_{symb}  - J_{iDyn} )')