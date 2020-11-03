function [tau_regressor, tau_RNEA] = computeIDWithDynamicRegressorAndRNEA(jointPos,jointVel,jointAcc, gravityModulus,robotModelURDF)

%% Symbolic
% Compute model from urdf
% generate the c code only the first time
persistent firstTime
if isempty(firstTime)
    firstTime = 1;
else
    firstTime = 0;
end
symbolicIDFunction = urdf2eomID(robotModelURDF,firstTime);
 % Gravity column vector
g =[0;0;-gravityModulus];
% The external forces is a vector of (6,1). It has to be one per
% link, expect the base link(which for now is considered fixed)
% If we do not supply the external forces they are automatically considered null in the compiled function
tau_symbolic_compiled = rnea('rnea',jointPos, jointVel, jointAcc, g);
% Test with symbolic function
% If we do not supply the external forces they are NOT automatically considered null in the symbolic function
nrOfJoints = size(jointPos,1);
extForce = zeros(6,nrOfJoints);
tau_RNEA = symbolicIDFunction(jointPos, jointVel, jointAcc, g,extForce);
tau_RNEA = full(tau_RNEA);

%% Use regressor
% Compute the Inertia parameters in body frame
%Load urdf and convert to SMDS format
smds = my_urdf2smds(robotModelURDF);
for i = 1:nrOfJoints
    p(:,i) = [smds.mass{i}; smds.mass{i}*smds.com{i}; smds.I{i}(1,1); smds.I{i}(1,2); smds.I{i}(1,3);...
                           smds.I{i}(2,2); smds.I{i}(2,3); smds.I{i}(3,3)]; 
end
inertiaParameters = reshape(p,[],1);
Y = inverseDynamicsInertialParametersRegressor(robotModelURDF,firstTime);
tau_regressor = Y(jointPos, jointVel, jointAcc, g)*inertiaParameters;
tau_regressor = full(tau_regressor);
end