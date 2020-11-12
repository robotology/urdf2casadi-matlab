function [symbolicIDFunction] = symbolicInverseDynamics(file,geneate_c_code,location_generated_fucntion)
%Generates equation of motion in symbolic form from urdf file 
%Based on RNEA inverse dynamics code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%Load urdf and convert to SMDS format
smds = extractSystemModel(file);
%Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);
f_ext = SX.sym('f_ext', 6,1,smds.NB);
tau = SX.sym('tau', [smds.NB,1]);

%Gravity
a_grav = [0;0;0;g(1);g(2);g(3)];


%RNEA
%Forward recursion
for i = 1:smds.NB
  [ XJ, S{i} ] = jcalc( smds.jaxis{i}, smds.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  %% Compute i_X_pi;
  % the overall transformation from the local frame of body parent(i)
  % and the local frame of body i: i_X_pi = X_Ti * X_Ji
  Xup{i} = XJ * smds.Xtree{i};
  if smds.parent(i) == 0
    v{i} = vJ;
    a{i} = Xup{i}*(-a_grav) + S{i}*qdd(i);
  else
    v{i} = Xup{i}*v{smds.parent(i)} + vJ;
    a{i} = Xup{i}*a{smds.parent(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
  end
  f{i} = smds.I{i}*a{i} + crf(v{i})*smds.I{i}*v{i};
end

% f_ext:one (6,1) vector per each link excluding the base(considered fixed for now) 
f = apply_external_forces( smds.parent, Xup, f, f_ext );


for i = smds.NB:-1:1
  tau(i,1) = S{i}' * f{i};
  if smds.parent(i) ~= 0
    f{smds.parent(i)} = f{smds.parent(i)} + Xup{i}'*f{i};
  end
end
 

% Define the symbolic function and set its input and output in poper order
% and with proper names
% Require the linkExternlWrenches to be supplied as a single matrix of size
% [6,NB]
F_ext = SX.sym('f_ext', 6,smds.NB);
for ii = 1:smds.NB
    F_ext(:,ii)= f_ext{ii};
end
inputVarNames = {'q','qd','qdd','g','F_ext'};
outputVarName = 'jointToques';
symbolicIDFunction=Function('rnea',{q,qd,qdd,g,F_ext},{tau},inputVarNames,outputVarName);
% inputVarNames = {'q','qd','qdd','g'};
% outputVarName = 'jointToques';
% symbolicIDFunction=Function('rnea',[{q,qd,qdd,g}],{tau},inputVarNames,outputVarName);
%% Code generation option
if geneate_c_code
    current_folder = pwd;
    cd(location_generated_fucntion);
    opts = struct('main', true,...
                  'mex', true);
    symbolicIDFunction.generate('rnea.c',opts);
    mex rnea.c -DMATLAB_MEX_FILE

    % Test the function with null inputs
    t=rnea('rnea');
    T=full(t);
    if (T~=zeros(smds.NB,1))
        error('The compiled smds returns non null torques for all null inputs (gravity included)');
    end
    cd(current_folder);
end