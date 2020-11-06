function [forwardDynamicsFunction] = urdf2eomFD(file,geneate_c_code)
%Generates equation of motion in symbolic form from urdf file
%Based on articulated body forward dynamics code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%Load urdf and convert to SMDS format
smds = my_urdf2smds(file);
import casadi.*;
%Initialize variables
q = SX.sym('q',[1,smds.NB])';
qd = SX.sym('qd',[1,smds.NB])';
qdd = SX.sym('qdd', [smds.NB,1]);
tau = SX.sym('tau',[1,smds.NB])';
g = SX.sym('g',[3,1]);

%Gravity
a_grav = [0;0;0;g(1);g(2);g(3)];


%Articulated body algorithm
for i = 1:smds.NB
    [ XJ, S{i} ] = jcalc( smds.jaxis{i},smds.jtype{i}, q(i) );
    vJ = S{i}*qd(i);
    Xup{i} = XJ * smds.Xtree{i};
    if smds.parent(i) == 0
        v{i} = vJ;
        c{i} = zeros(size(a_grav));		
    else
        v{i} = Xup{i}*v{smds.parent(i)} + vJ;
        c{i} = crm(v{i}) * vJ;
    end

    IA{i} = smds.I{1,i};
    pA{i} = crf(v{i}) * smds.I{1,i} * v{i};

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

% Define the symbolic function and set its input and output in poper order

forwardDynamicsFunction = Function('forwardDynamics',{q,qd,g,tau},{qdd},{'joints_position','joints_velocity','gravity','joints_torque'},{'joints_acceleration'});

  
%% Code generation option
if geneate_c_code
    opts = struct('main', true,...
                  'mex', true);
    forwardDynamicsFunction.generate('forwardDynamics.c',opts);
    mex forwardDynamics.c -DMATLAB_MEX_FILE

    % Test the function
    a=forwardDynamics('forwardDynamics');
    A=full(a);       
    if (A~=zeros(smds.NB,1))
        error('The compiled smds returns non null torques for all null inputs (gravity included)');
    end
end
