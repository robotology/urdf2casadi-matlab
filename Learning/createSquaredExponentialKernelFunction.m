function [squaredExponentialKernel] = createSquaredExponentialKernelFunction(sizeX1,sizeX2,geneate_c_code_fromSymbolicFunction,locationGeneratedCode)
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

sigma1 = SX.sym('sigma1');
sigma2 = SX.sym('sigma2');
M  = SX.sym('X',[sizeX1(2),sizeX1(2)]);

Kernel = casadi.SX.sym('Y',Sparsity(sizeX1(1),sizeX2(1)));

r = SX.sym('r',sizeX1(2));
c = SX.sym('c',sizeX2(2));

f = sigma1*exp(-0.5*(r - c)' * M * (r - c));
f = Function('f',{r,c},{f},{'r','c'},'f');
F = f.map(size(X2,1),'thread',2);

inputVarNames = {'X1','X2','sigma1','sigma2','M'};
outputVarName = 'squaredExponentialKernel';
K =[];
tic
for j = 1:size(X1,1)
    kj2 = F(repmat(X1(j,:),size(X2,1))', X2');
    K = [K; kj2{:}];
end
K = K + sigma2*eye(size(X1,1),size(X2,1));
% Define the symbolic function and set its input and output in poper order and with proper names

squaredExponentialKernel = Function('computeSquaredExponentialKernel', {X1,X2,sigma1,sigma2,M},{K},inputVarNames,outputVarName);
toc


% tic
% % Create map function
% FF = F.map(size(X1,1));
% 
% % Manipulate input to match map function inputs
% X1_t = X1';
% X1_vec = [X1_t(:)]';
% X1_vecRep = repmat(X1_vec,[size(X2,1),1]);
% V = vertsplit(X1_vecRep',[0:size(X1,2):numel(X1)]);
% 
% X1_inputMap = [V{:}];
% X2_inputMap = repmat(X2', [1, size(X1,1)]);
% 
% % Compute the Kernel in vector form and reshape it in matrix form
% Kernel = FF(X1_inputMap,X2_inputMap);
% Kernel = reshape(Kernel', [size(X2,1), size(X1,1)])' + sigma2*eye(size(X1,1),size(X2,1));
% 
% % Create symbolic function
% squaredExponentialKernel = Function('computeSquaredExponentialKernel', {X1,X2,sigma1,sigma2,M},{Kernel},inputVarNames,outputVarName);
% toc


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