function symbolicFunction = computeSymbolicStackOfRegressorsTransposed (smds,nrOfTrajectoryPoints,geneate_c_code_fromSymbolicFunction,locationGeneratedCode)
%Compute the transpose of the regressor, this is necessary
% since in the map functionality from casadi aggregation will
% always happen horizontally.
% See https://web.casadi.org/docs/#for-loop-equivalents

%Initialize variables
import casadi.*;
q = SX.sym('q',[nrOfTrajectoryPoints,smds.NB]);
qd = SX.sym('qd',[nrOfTrajectoryPoints,smds.NB]);
qdd = SX.sym('qdd',[nrOfTrajectoryPoints,smds.NB]);
g = SX.sym('g',[3,1]);

regressor_opts.returnTransposedRegressor = true;
disp('Computing the regressor...')
tic
% Compute the symbolic regressor 
Y_transposed = computeSymbolicRegressor(q(1,:).', qd(1,:).', qdd(1,:).', g, smds,regressor_opts);
% Stack of regressors, one per trajectory point
stackOfRegressorsTransposed = Y_transposed.map(nrOfTrajectoryPoints);
toc

symbolicFunction = stackOfRegressorsTransposed;
%% Code generation option
if geneate_c_code_fromSymbolicFunction
    % Save the generated code in user defined location and then go back to
    % folder from which the function was called
    current_folder = pwd;
    cd(locationGeneratedCode);
    opts = struct('main', true,...
                  'mex', true);
    symbolicFunction.generate('gen_stackOfRegressorsTransposed.c',opts);
    mex gen_stackOfRegressorsTransposed.c -DMATLAB_MEX_FILE
    cd(current_folder);
end
end