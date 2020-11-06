function [jointAccMatlab, jointAccSymbolic] = computeIDyntreeFD_VS_Symblic(modelRobotMatlab, q,qd,gravityModulus,tau,robotModelURDF)

%% Matlab
jointAccMatlab = forwardDynamics(modelRobotMatlab,q,qd,tau);

%% IDynTree

%% Symbolic
% Compute model from urdf
% generate the c code only the first time
persistent firstTime
if isempty(firstTime)
    firstTime = 1;
else
    firstTime = 0;
end
symbolicDynamicFunction = symbolicForwardDynamics(robotModelURDF,firstTime);
g =[0;0;-gravityModulus]; % Gravity column vector
% Test with symbolic function
jointAccSymbolic = symbolicDynamicFunction(q, qd, g,tau);
jointAccSymbolic = full(jointAccSymbolic);
% %% Check results
% plot(abs(tau_iDynTree-tau_symbolic_function));
end