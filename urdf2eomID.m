function [symbolicIDFunction] = urdf2eomID(file,geneate_c_code)
%Generates equation of motion in symbolic form from urdf file 
%Based on RNEA inverse dynamics code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%Load urdf and convert to SMDS format
smds = urdf2smds(file);

%Initialize variables
import casadi.*;
q = SX.sym('q',[1,smds.NB]);
qd = SX.sym('qd',[1,smds.NB]);
qdd = SX.sym('qdd',[1,smds.NB]);
g = SX.sym('g',[3,1]);
tau = SX.sym('tau', [smds.NB,1]);

%Gravity
a_grav = [0;0;0;g(1);g(2);g(3)];


%RNEA
%Forward recursion
for i = 1:smds.NB
    [ XJ, S{i} ] = jcalc( smds.jtype{i}, q(i) );
    vJ = S{i}*qd(i);
    Xup{i} = XJ * smds.Xtree{i};
    if smds.parent(i) == 0
        v{i} = vJ;
        a{i} = Xup{i}*(-a_grav) + S{i}*qdd(i);
    else
        v{i} = Xup{i}*v{smds.parent(i)} + vJ;
        a{i} = Xup{i}*a{smds.parent(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
    end    
    f{i} = smds.I{1,i}*a{i} + crf(v{i})*smds.I{1,i}*v{i};
end
%% MISSING EXTERNAL FORCES CONTRIBUTION

%Backwards recursion
for i = smds.NB:-1:1
    tau(i,1) = S{i}' * f{i};
    if smds.parent(i) ~= 0
        f{smds.parent(i)} = f{smds.parent(i)} + Xup{i}'*f{i};
    end
end
 

% Define the symbolic function and set its input and output in poper order
symbolicIDFunction=Function('rnea',{q,qd,qdd,g},{tau},{'q','qd','qdd','g'},{'tau'});

%% Code generation option
if geneate_c_code
    opts = struct('main', true,...
                  'mex', true);
    symbolicIDFunction.generate('rnea.c',opts);
    mex rnea.c -DMATLAB_MEX_FILE

    % Test the function
    q = zeros(6,1);
    qd = zeros(6,1);
    qdd = zeros(6,1);
    g = 0;
    t=rnea('rnea',q, qd, qdd, g);
    T=full(t);
    disp('Joint torques for all null inputs:')
    disp(T);
end