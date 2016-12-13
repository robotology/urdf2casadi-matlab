function [tau] = urdf2eomID(file,simplifyflag)
%Generates equation of motion in symbolic form from urdf file 
%Based on RNEA inverse dynamics code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%Load urdf and convert to SMDS format
smds = urdf2smds(file);

%Initialize variables
q = sym('q',[1,smds.NB],'real');
qd = sym('qd',[1,smds.NB],'real');
qdd = sym('qdd',[1,smds.NB],'real');
syms g;

%Gravity
a_grav = [0;0;0;0;0;g];
I = smds.I;

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
    f{i} = I(:,:,i)*a{i} + crf(v{i})*I(:,:,i)*v{i};
end

%Backwards recursion
for i = smds.NB:-1:1
    tau(i,1) = S{i}' * f{i};
    if smds.parent(i) ~= 0
        f{smds.parent(i)} = f{smds.parent(i)} + Xup{i}'*f{i};
    end
end

if simplifyflag == 1
    tau = simplify(expand(tau));
end

%Write to file
file = fopen('tau.txt', 'w');
for i = 1:smds.NB
    fprintf(file, '%s\r\n\n', char(tau(i)));
end
fclose(file);
end
