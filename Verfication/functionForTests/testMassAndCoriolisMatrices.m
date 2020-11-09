 
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2eom/URDFs/twoLinks.urdf';
kuka_kr210 = '/home/iiticublap041/baljinder/urdf2eom/URDFs/kuka_kr210.urdf';
robotURDFModel=kuka_kr210;
jointPos = rand(6,1);
jointVel = rand(6,1);
jointAcc = rand(6,1);
gravityModulus = 9.80665;
% Compute mass matrix with IDynTree
M_IDyn=computeMassMatrixIDynTree(robotURDFModel,jointPos,jointVel,jointAcc,gravityModulus);

% Compute symbolic mass matrix
smds = extractSystemModel(robotURDFModel);
g = [0;0;-gravityModulus];
[H,HDot,C] = computeSymbolicCoriolismatrix(jointPos,jointVel,jointAcc,g,smds);

