function regressor = computeSymbolicFrictionRegressor(qd, K_v, delta, smds, opts)
%Compute the regressor of the friction parameters
numberOfJoints = smds.NB; 
numberOfFrictionParameters = 4;
import casadi.*;
F = casadi.SX.sym('Y',Sparsity(numberOfJoints,numberOfFrictionParameters));

% For each joint set the coefficient of the friction parameters
pi = 3.1416;
F = [2/pi*atan(qd.*K_v), atan(qd.*delta), qd, (qd.^2)*(2/pi).*atan(qd.*K_v)];

%% Define the symbolic function and set its input and output in poper order
% and with proper names
inputVarNames = {'qd','K_v','delta'};
outputVarName = 'frictionRegressors';
if nargin > 4
    if opts.returnTransposedRegressor
        regressor=Function('frictionParametersRegressor',{qd, K_v, delta},{F.'},inputVarNames,outputVarName);
    else
        regressor=Function('frictionParametersRegressor',{qd, K_v, delta},{F},inputVarNames,outputVarName);
    end
else
    regressor=Function('frictionParametersRegressor',{qd, K_v, delta},{F},inputVarNames,outputVarName);
end
end