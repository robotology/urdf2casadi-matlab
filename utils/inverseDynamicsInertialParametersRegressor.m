function symbolicRegressorFunction = inverseDynamicsInertialParametersRegressor(robotURDFModel,geneate_c_code)
%Compute the dynamics linear wrt the Inertia paramteres. Specifically,
% compute Y(q,qd,qdd)*P = M(q)qdd + C(q,qd)qd + G(q)
% See Springer Handbook of Robotics(2016) in Chapter 6.3 for more details
% on the computations

%Load urdf and convert to SMDS format
smds = my_urdf2smds(robotURDFModel);
%Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);

% Compute frame transformations, joint motion subspace matrix, link velocity and accelerations
[X,XForce,S,Xup, v, a] = computeKinematics(smds, q, qd, qdd, g);

% Compute for each link i the matrix A_j for all links j distal to link
% i. This matix arises from recasting the spatial Newton-Euler dynamic equation 
% of each link i in a form linear wrt the Inertial paramter vector of link i, expressed in the local frame i 
for l = 1:smds.NB
    % Velocity and acceleration must be expressed in the same frame as the
    % inertia and the center of mass: the body i local frame
    A{l} = computeLinkRegressor( v{l}, a{l});
end
% Compute the matrix Y (for a serial kinematic chain) as the upper triangular represenation of formula
% (6.46) in Springer Handbook of Robotics(2016), Chapter 6.3. 
% This matrix multiplied by the inertial parameters vector gives the
% spatial force at the NB joints
% Use Sparcvity class from CasADi
% Assuming one degree of freedom per joint
for i =1:smds.NB
    degreeOfFreedomJoint(i) = size(S{i},2);
end
totalDegreeOfFreedom = sum(degreeOfFreedomJoint);
numInertiaParametersPerLink = 10;
Y = SX.sym('Y',Sparsity(totalDegreeOfFreedom,numInertiaParametersPerLink*smds.NB));

jointOffset = 0;
for rowIndex= 1:smds.NB
    columnOffset = numInertiaParametersPerLink*(rowIndex-1);
    for columnIndex = rowIndex:smds.NB
        Y(jointOffset+1:jointOffset+degreeOfFreedomJoint(rowIndex),columnOffset+1:columnOffset+numInertiaParametersPerLink) = (S{rowIndex}.')*(XForce{columnIndex}{1,rowIndex})*A{columnIndex};
        columnOffset = columnOffset + numInertiaParametersPerLink;
    end
    jointOffset = jointOffset + degreeOfFreedomJoint(rowIndex);
end

%% Define the symbolic function and set its input and output in poper order
% and with proper names
inputVarNames = {'q','qd','qdd','g'};
outputVarName = 'regressor';
symbolicRegressorFunction=Function('inertiaParametersRegressor',{q,qd,qdd,g},{Y},inputVarNames,outputVarName);

%% Code generation option
if geneate_c_code
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
end
end

