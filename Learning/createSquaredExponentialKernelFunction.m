function squaredExponentialKernel = createSquaredExponentialKernelFunction(sizeX1,sizeX2,geneate_c_code_fromSymbolicFunction,locationGeneratedCode)
%Compute the squared exponential Kernel function
% Input
% sizeX1 [n x d]    Size of the first input matrix containing n samples of
%                   the sample vectors of dimension d
% sizeX2 [m x d]    Size of the first input matrix containing m samples of
%                   the sample vectors of dimension d
% Output
% K [n x m]

% Check inputs
if sizeX1(2)~=sizeX2(2)
    error('Inputs should be of same dimension d. In particular, the inputs should have nrOfSamples rows and d (dimension of each input) columns. In general the number of samples of the first input vector can be different than the number of samples of the second input vectors')
end
%Initialize variables
import casadi.*;
X1 = SX.sym('X',sizeX1);
X2 = SX.sym('X',sizeX2);

Kernel = casadi.SX.sym('Y',Sparsity(sizeX1(1),sizeX2(1)));

for j = 1:size(X1,1)
    for k = 1:size(X2,1)
        Kernel(j,k) = exp(-0.5*norm(X1(j,:) - X2(k,:))^2);
    end
end
%% Define the symbolic function and set its input and output in poper order
% and with proper names
inputVarNames = {'X1','X2'};
outputVarName = 'squaredExponentialKernel';
squaredExponentialKernel = Function('computeSquaredExponentialKernel',{X1,X2},{Kernel},inputVarNames,outputVarName);
%% Code generation option
if geneate_c_code_fromSymbolicFunction
    % Save the generated code in user defined location and then go back to
    % folder from which the function was called
    current_folder = pwd;
    cd(locationGeneratedCode);
    opts = struct('main', true,...
                  'mex', true);
    squaredExponentialKernel.generate('computeSquaredExponentialKernel.c',opts);
    mex computeSquaredExponentialKernel.c -DMATLAB_MEX_FILE
    cd(current_folder);
end
end