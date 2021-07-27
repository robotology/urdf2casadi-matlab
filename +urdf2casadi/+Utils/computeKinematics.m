function [X,XForce,S,Xup, v, a] = computeKinematics (smds, q, qd, qdd, g)
%Compute forward kinematics transformations, link velocities and accelerations
% Specificcaly compute:
%  *link i spatial velocity(the left trivialized velocity in Traversaro's notation), acceleation in LOCAL body i coordinates
%  *motion coordinate transform
%   from each link i parent lambda(i) and link i (i_X_(lambda(i))=
%   X_up{i});
%  *spatial motion transform from link i to any link j in its subtree: X{i}{1,j}. Notice that the
%  transform from base to link i is simply X{1}{1,i}*Xup{1}
%  *force coordinate transform from body i to its predecessor(XForce{i})
%  *Joint motion subspace matrix S (referred as K in Springer Handbook of Robotics(2016) in Chapter 6.3)

% Import necessary functions 
import urdf2casadi.Utils.Spatial.jcalc
import urdf2casadi.Utils.Spatial.crm

%Gravity
a_grav = [0;0;0;g(1);g(2);g(3)];

% smds.NB: The number of links excluding the base, which for now is considered
% fix(= number of Joints)
for i = 1:smds.NB
  [ XJ, S{i} ] = jcalc(smds.jaxis{i}, smds.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * smds.Xtree{i};
  if smds.parent(i) == 0
    v{i} = vJ;
    a{i} = Xup{i}*(-a_grav) + S{i}*qdd(i);
  else
    v{i} = Xup{i}*v{smds.parent(i)} + vJ;
    a{i} = Xup{i}*a{smds.parent(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
  end
end
for i = 1:smds.NB
    X{i}{1,i} = eye(6);
    XForce{i}{1,i} = eye(6);
    for j = i+1:smds.NB
        X{i}{1,j} = eye(6);
        for k = i+1:j
            X{i}{1,j} = Xup{k}*X{i}{1,j};
        end
        XForce{j}{1,i} =  X{i}{1,j}.';
    end
end
    
end