function symbolicRegressorFunction = inverseDynamicsInertialParametersRegressor(robotURDFModel,geneate_c_code,location_generated_functions)
%Compute the dynamics linear wrt the Inertia paramteres. Specifically,
% compute Y(q,qd,qdd)*P = M(q)qdd + C(q,qd)qd + G(q)
% See Springer Handbook of Robotics(2016) in Chapter 6.3 for more details
% on the computations

%Load urdf and convert to SMDS format
smds = extractSystemModel(robotURDFModel);
%Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);

%% Compute the regressor function
symbolicRegressorFunction = computeSymbolicRegressor(q, qd, qdd, g, smds);
%% Code generation option
if geneate_c_code
    
    current_folder = pwd;
    cd(location_generated_functions);
    
    opts = struct('main', true,...
                  'mex', true);
    symbolicRegressorFunction.generate('inertiaParametersRegressor.c',opts);
    mex inertiaParametersRegressor.c -DMATLAB_MEX_FILE

    % Test the function with null inputs
    t=inertiaParametersRegressor('inertiaParametersRegressor');
    T=full(t);
    if (T~=zeros(smds.NB,1))
        error('The compiled regressor returns non null torques for all null inputs (gravity included)');
    end
    cd(current_folder);
end
end

