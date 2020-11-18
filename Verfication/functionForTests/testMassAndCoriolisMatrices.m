 
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/kuka_kr210.urdf';
iCub_r_leg = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/iCub_r_leg.urdf';
robotURDFModel=kuka_urdf;
nrOfJoints = 6;
jointPos = rand(nrOfJoints,1);
jointVel = rand(nrOfJoints,1);
jointAcc = rand(nrOfJoints,1);
gravityModulus = 9.80665;
tol = 1e-10;
% Compute mass matrix with IDynTree
M_IDyn=computeMassMatrixIDynTree(robotURDFModel,jointPos,jointVel,jointAcc,gravityModulus);
% Compute gravity torques IDynTree
tau_gravity_IDyn = computeGravityTorqueIDynTree(robotURDFModel,jointPos,gravityModulus);
% Compute the generalized bias force C(q,qd)*qd + g(q)
genealizedBias_IDyn = computeGeneralizedBiasForceIDynTree(robotURDFModel,jointPos,jointVel,gravityModulus);

%% Compute mass matrix with an efficient algorithm
smds = extractSystemModel(robotURDFModel);
g = [0;0;-gravityModulus];
[H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(jointPos,jointVel,jointAcc,smds);
H = cell2mat_casadi(H_cell);
HDot = cell2mat_casadi(HDot_cell);
C = cell2mat_casadi(C_cell);
%% Compare mass matrix 
e_massMatrix = abs(M_IDyn-H);
assert (norm(e_massMatrix,'fro')<tol);

%% Compute gravity torques using the symbolic Inverse Dynamics function
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,0);
tau_gravity = computeGravityTorque(jointPos,g,symbolicIDFunction);
tau_gravity = full(tau_gravity);

%% Compare gravity torques
e_gravity = abs(tau_gravity - tau_gravity_IDyn);
assert(any(e_gravity<tol));

%% Compare generalized bias forces
generalizedBias = C*jointVel + tau_gravity;
e_generalizedBias = abs(generalizedBias-genealizedBias_IDyn);

%% Compute the inverse dynamics with the matrices  
tau_efficient = H*jointAcc + C*jointVel + tau_gravity;
e_torqueID = abs(tau_efficient-symbolicIDFunction(jointPos,jointVel,jointAcc,g,0));