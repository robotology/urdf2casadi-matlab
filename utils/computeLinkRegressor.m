function A = computeLinkRegressor(linkSpatialVel, linkSpatialAcc)

% Link angular vel
linkAngularVel = linkSpatialVel(1:3);
% Derivative of the components of the agular vel
linkAngularVelDot = linkSpatialAcc(1:3);
% L(linkAngularVel)vec(Inertial_i) = Inertia_i*linkAngularVel
% Note that the Inertia must be expressed in the body i local frame
L_omega = computeRotationalMomentumRegressor(linkAngularVel);
L_omegaDot = computeRotationalMomentumRegressor(linkAngularVel);

% Compute the classical acceleration of the origin of body i frame(d_oi)
linkLinearVel =  linkSpatialVel(4:6);
d_oi_DDot = linkLinearVel + skew(linkAngularVel)*linkLinearVel;

A = [zeros(3,1),-skew(d_oi_DDot),                    L_omegaDot+skew(linkAngularVel)*L_omega;
     d_oi_DDot, skew(linkAngularVelDot)+skew(linkAngularVel)*skew(linkAngularVel), zeros(3,6)];
end