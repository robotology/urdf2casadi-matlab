function [X1,S,Xup, v, a] = computeKinematics (smds, q, qd, qdd, g)
%Compute forward kinematics transformations, link velocities and accelerations
% Specificcaly compute:
%  *links spatial velocities, acceleations and motion coordinate transform
%   between each link i and its parent lambda(i) (i_X_(lambda(i))=
%   X_up{i});
%  *spatial motion transform from link 1 to link i X1{i}. Notice that the
%  transform from base to link i is simply X1{i}*Xup{1}
%  *force coordinate transform from body i to its predecessor(XForce{i})
%  *Joint motion subspace matrix S (referred as K in Springer Handbook of Robotics(2016) in Chapter 6.3)

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
    X1{i} = eye(6);
  else
    v{i} = Xup{i}*v{smds.parent(i)} + vJ;
    a{i} = Xup{i}*a{smds.parent(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
    X1{i} = Xup{i}*X1{i-1};
  end
end
end