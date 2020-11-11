function [HFunction,HDotFunction,CFunction]= createMassAndCoriolisMatrixFunction(robotURDFModel,geneate_c_code)
%Create a symbolic function returning the Mass and Coriolis matrices as a
%function of joint position, velocity. In addition it also returns the
%derivative of the mass matrix

% Extract the robot model
smds = extractSystemModel(robotURDFModel);

% Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);

[H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(q,qd,qdd,smds);
H = cell2mat_casadi(H_cell);
HDot = cell2mat_casadi(HDot_cell);
C = cell2mat_casadi(C_cell);
%% Symbolic matrix functions
HFunction = Function('computeMassMatrix',{q},{H},...
                    {'joints_position'},...
                    {'massMatrix'});
HDotFunction = Function('computeMassMatrixTimeDerivative',{q,qd},{HDot},...
                    {'joints_position','joints_velocity'},...
                    {'massMatrixDot'});
CFunction = Function('computeCoriolisMatrix',{q,qd},{C},...
                    {'joints_position','joints_velocity'},...
                    {'coriolisMatrix'});
                
%% Code generation option
if geneate_c_code
    % Create c code
    opts = struct('main', true,...
                  'mex', true);
    HFunction.generate('computeMassMatrix.c',opts);
    HDotFunction.generate('computeMassMatrixTimeDerivative.c',opts);
    CFunction.generate('computeCoriolisMatrix.c',opts);
    % Compile for matlab
    mex computeMassMatrix.c -DMATLAB_MEX_FILE
    mex computeMassMatrixTimeDerivative.c -DMATLAB_MEX_FILE
    mex computeCoriolisMatrix.c -DMATLAB_MEX_FILE
end

end