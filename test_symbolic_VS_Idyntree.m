%% This is a script to compare the result of the symbolic expression with the result obtained using iDynTree(https://github.com/robotology/idyntree)
%% First compute ID with idyntree 
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
twoLink_urdf = '/home/iiticublap041/baljinder/urdf2eom/URDFs/twoLinks.urdf';

location_urdf = kuka_urdf;

%% input urdf file to acquire robot structure
robotModelURDF = location_urdf;

%% Set the ID inputs for both iDyntree and symbolic function
nrOfTests = 20;
iDynResult = zeros(nrOfJoints,nrOfTests);
symbolcResult = zeros(nrOfJoints,nrOfTests);
nrOfJoints = 6;
for i = 1:nrOfTests
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
end
clear firstTime 
eps = abs(iDynResult'-symbolcResult');

plot(eps);title('abs(iDynResult - symbolcResult)');legend;