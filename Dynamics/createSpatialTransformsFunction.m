function [jacobian,X,XForce,S] = createSpatialTransformsFunction(robotURDFModel,geneate_c_code,location_generated_fucntion)
%Create symbolic and c code function for the spatial transform from a
%general link i to any link j in its subtree

% Extract the robot model
smds = extractSystemModel(robotURDFModel);
% Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);

%% Compute the symbolic functions
[X,XForce,S,Xup, ~, ~]  = computeKinematics (smds, q, qd, qdd, g);

% Jacobian 
k_X_N = (X{1}{1,end}*Xup{1})^(-1);
J = computeJacobian(k_X_N, Xup,S);
J = [J{:}];
jacobian = Function('computeNumericalJacobian',{q},{J},...
                    {'joints_position'},...
                    {'jacobian'});

%% Generete c code and mex files
if geneate_c_code
    current_folder = pwd;
    cd(location_generated_fucntion);
    
    % Create c code
    opts = struct('main', true,...
                  'mex', true);
    jacobian.generate('computeNumericalJacobian.c',opts);
    
    % Compile for matlab
    mex computeNumericalJacobian.c -DMATLAB_MEX_FILE

    cd(current_folder);
end
end