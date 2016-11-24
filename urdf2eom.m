function [tau] = urdf2eom(file)
%Generates equation of motion in symbolic form from urdf file 
%Based on ID code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

%load urdf and convert to SMDS format
smds = urdf2smds(file);

% Initialize variables
q = sym('q',[1,smds.NB]);
qd = sym('qd',[1,smds.NB]);
qdd = sym('qdd',[1,smds.NB]);
syms g;

a_grav = [0;0;0;0;0;g];
I = smds.I;

%RNEA
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
    f{i} = I(:,:,1)*a{i} + crf(v{i})*I(:,:,1)*v{i};
end

for i = smds.NB:-1:1
    tau(i,1) = S{i}' * f{i};
    if smds.parent(i) ~= 0
        f{smds.parent(i)} = f{smds.parent(i)} + Xup{i}'*f{i};
    end
end
end
