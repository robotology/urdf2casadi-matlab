function symbolicFunction = computeSymbolicStackOfFrictionRegressorsTransposed (robotURDFModel,nrOfTrajectoryPoints, geneate_c_code_fromSymbolicFunction,locationGeneratedCode)
%Compute the transpose of the regressor, this is necessary
% since in the map functionality from casadi aggregation will
% always happen horizontally.
% See https://web.casadi.org/docs/#for-loop-equivalents

%Load urdf and convert to SMDS format
smds = extractSystemModel(robotURDFModel);

%Initialize variables
import casadi.*;
qd = SX.sym('qd',[nrOfTrajectoryPoints,smds.NB]);
K_v = SX.sym('K_v',smds.NB);
delta = SX.sym('delta',smds.NB);

regressor_opts.returnTransposedRegressor = true;
disp('Computing friction regressor...')
tic
% Compute the symbolic regressor 
Y_transposed = computeSymbolicFrictionRegressor(qd(1,:).',K_v,delta,smds,regressor_opts);
% Stack of regressors, one per trajectory point
stackOfFrictionRegressorsTransposed = Y_transposed.map(nrOfTrajectoryPoints);
toc

symbolicFunction = stackOfFrictionRegressorsTransposed;
%% Code generation option
if geneate_c_code_fromSymbolicFunction
    % Save the generated code in user defined location and then go back to
    % folder from which the function was called
    current_folder = pwd;
    cd(locationGeneratedCode);
    opts = struct('main', true,...
                  'mex', true);
    symbolicFunction.generate('gen_stackOfFrictionRegressorsTransposed.c',opts);
    mex gen_stackOfFrictionRegressorsTransposed.c -DMATLAB_MEX_FILE
    cd(current_folder);
end
end