function [jointAccMatlab, jointAccSymbolic] = computeIDyntreeFD_VS_Symblic(modelRobotMatlab, q,qd,gravityModulus,tau,robotModelURDF)

%% Matlab
jointAccMatlab = forwardDynamics(modelRobotMatlab,q,qd,tau);

%% IDynTree

%% Symbolic
symbolicDynamicFunction = symbolicForwardDynamics(robotModelURDF,0);
g =[0;0;-gravityModulus]; % Gravity column vector
% Test with symbolic function
jointAccSymbolic = symbolicDynamicFunction(q, qd, g,tau);
jointAccSymbolic = full(jointAccSymbolic);
end