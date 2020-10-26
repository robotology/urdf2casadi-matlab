function [f] = urdf2eomFD(file,geneate_c_code)
%Generates equation of motion in symbolic form from urdf file
%Based on articulated body forward dynamics code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%Load urdf and convert to SMDS format
smds = urdf2smds(file);
import casadi.*;
%Initialize variables
q = SX.sym('q',[1,smds.NB])';
qd = SX.sym('qd',[1,smds.NB])';
qdd = SX.sym('qdd', [smds.NB,1]);
tau = SX.sym('tau',[1,smds.NB])';
g = SX.sym('g');
% q = sym('q',[1,smds.NB],'real')';
% qd = sym('qd',[1,smds.NB],'real')';
% tau = sym('tau',[1,smds.NB],'real')';
% syms g;

%Gravity
a_grav = [0;0;0;0;0;g];
I = smds.I;

%Articulated body algorithm
for i = 1:smds.NB
    [ XJ, S{i} ] = jcalc( smds.jtype{i}, q(i) );
    vJ = S{i}*qd(i);
    Xup{i} = XJ * smds.Xtree{i};
    if smds.parent(i) == 0
        v{i} = vJ;
        c{i} = zeros(size(a_grav));		
    else
        v{i} = Xup{i}*v{smds.parent(i)} + vJ;
        c{i} = crm(v{i}) * vJ;
    end
    if smds.use_urdf_inertia_param
        IA{i} = I(:,:,i);
        pA{i} = crf(v{i}) * I(:,:,i) * v{i};
    else
        IA{i} = I{1,i};
        pA{i} = crf(v{i}) * I{1,i} * v{i};
    end

end

for i = smds.NB:-1:1
    U{i} = IA{i} * S{i};
    d{i} = S{i}' * U{i};
    u{i} = tau(i) - S{i}'*pA{i};
    if smds.parent(i) ~= 0
        Ia = IA{i} - U{i}/d{i}*U{i}';
        pa = pA{i} + Ia*c{i} + U{i} * u{i}/d{i};
        IA{smds.parent(i)} = IA{smds.parent(i)} + Xup{i}' * Ia * Xup{i};
        pA{smds.parent(i)} = pA{smds.parent(i)} + Xup{i}' * pa;
    end
end

for i = 1:smds.NB
    if smds.parent(i) == 0
        a{i} = Xup{i} * -a_grav + c{i};
    else
        a{i} = Xup{i} * a{smds.parent(i)} + c{i};
    end
    qdd(i,1) = (u{i} - U{i}'*a{i})/d{i};
    a{i} = a{i} + S{i}*qdd(i);
end

%Write to file
% file = fopen('qdd.txt', 'w');
% for i = 1:smds.NB
%     fprintf(file, '%s\r\n\n', char(qdd(i)));
% end
% fclose(file);
% end

f=Function('forwardDynamics',{q,qd,tau},{qdd},{'q','qd','tau'},{'qdd'});

%% Code generation option
if geneate_c_code
    opts = struct('main', true,...
                  'mex', true);
    f.generate('forwardDynamics.c',opts);
    mex forwardDynamics.c -DMATLAB_MEX_FILE

    % Test the function
    q = zeros(6,1);
    qd = zeros(6,1);
    tau = zeros(6,1);
    t=forwardDynamics('forwardDynamics',q,qd,tau);
    T=full(t);
    disp(T);
end
