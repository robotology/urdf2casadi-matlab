function [jacobian,X,XForce,S,O_X_ee] = createSpatialTransformsFunction(robotURDFModel,options)
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

O_X_ee = Function('computesSpatialTransformFromBase',{q},{(X{1}{1,end})^(-1)},...
                    {'joints_position'},...
                    {'X'});
% Jacobian 
switch options.FrameVelocityRepresentation
    case "INERTIAL_FIXED_REPRESENTATION"
        % Compute transform from end-effector frame to base frame
        base_X_N = (X{1}{1,end}*Xup{1})^(-1);
        k_X_N = base_X_N;
    case "BODY_FIXED_REPRESENTATION"
        % Compute identity transform
        N_X_N = eye(6);
        k_X_N = N_X_N;
    case "MIXED_REPRESENTATION"
        % Compute transform from end-effector frame to a frame centered in
        % the origin of the end effector frame but with orientation that of
        % the base frame
        base_X_N = (X{1}{1,end}*Xup{1})^(-1);
        NIbaseI_X_N = SX.zeros(6,6);
        % Consider only the rotation part of the end-effector to base frame
        % transform
        NIbaseI_X_N(1:3,1:3) = base_X_N(1:3,1:3);
        NIbaseI_X_N(4:6,4:6) = base_X_N(4:6,4:6);
        k_X_N = NIbaseI_X_N;
end

J = computeJacobian(k_X_N, Xup,S);
J = [J{:}];
jacobian = Function('computeNumericalJacobian',{q},{J},...
                    {'joints_position'},...
                    {'jacobian'});

%% Generete c code and mex files
if options.geneate_c_code
    current_folder = pwd;
    cd(options.location_generated_fucntion);
    
    % Create c code
    opts = struct('main', true,...
                  'mex', true);
    jacobian.generate('computeNumericalJacobian.c',opts);
    
    % Compile for matlab
    mex computeNumericalJacobian.c -DMATLAB_MEX_FILE

    cd(current_folder);
end
end