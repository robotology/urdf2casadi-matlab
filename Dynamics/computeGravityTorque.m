function tau_gravity = computeGravityTorque(jointPos,g,tau_symbolic)
%Compute the inverse dynamics by setting the acceleration and velocity to zero
% To set the vector/matrix arguments to all zeros in Casadi functions, it is
% sufficient to set them to a scalar zero
qd = 0;
qdd = 0;
F_ext = 0;
tau_gravity = tau_symbolic(jointPos,qd,qdd,g,F_ext);
end