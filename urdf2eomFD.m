function [qdd] = urdf2eomFD(file)
%Generates equation of motion in symbolic form from urdf file
%Based on forward dynamics code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%Load urdf and convert to SMDS format
smds = urdf2smds(file);

%Initialize variables
q = sym('q',[1,smds.NB],'real')';
qd = sym('qd',[1,smds.NB],'real')';
tau = sym('tau',[1,smds.NB],'real')';
syms g;

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
    IA{i} = I(:,:,i);
    pA{i} = crf(v{i}) * I(:,:,i) * v{i};
end

if nargin == 5
    pA = apply_external_forces( smds.parent, Xup, pA, f_ext );
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

%Uncomment below line to simplify the equations. Can take very long.
% qdd = simplify(expand(qdd));

%Write to file
file = fopen('qdd.txt', 'w');
for i = 1:smds.NB
    fprintf(file, '%s\r\n\n', char(qdd(i)));
end
fclose(file);
end
