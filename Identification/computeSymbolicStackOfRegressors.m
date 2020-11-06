function stackOfRegressors = computeSymbolicStackOfRegressors(robotURDFModel,nrOfTrajectoryPoints)
%Load urdf and convert to SMDS format
smds = extractSystemModel(robotURDFModel);
%Initialize variables
import casadi.*;
q = SX.sym('q',[nrOfTrajectoryPoints,smds.NB]);
qd = SX.sym('qd',[nrOfTrajectoryPoints,smds.NB]);
qdd = SX.sym('qdd',[nrOfTrajectoryPoints,smds.NB]);
g = SX.sym('g',[3,1]);

for i = 1:nrOfTrajectoryPoints
    W_array{i,1}= computeSymbolicRegressor(q(i,:), qd(i,:), qdd(i,:), g, smds);
end
W = vertcat(W_array);
%% Define the symbolic function and set its input and output in poper order
% and with proper names
inputVarNames = {'q','qd','qdd','g'};
outputVarName = 'stackOfRegressors';
stackOfRegressors=Function('inertiaParametersRegressor',{q,qd,qdd,g},{W},inputVarNames,outputVarName);

end