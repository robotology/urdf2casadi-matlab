function computeMomentumObserver(robotURDFModel)
%Construct the symbolic representation of the Momentum observer, a
%technique used for force estimation and collision detection. See, for
%example, "Robot Collisions: A Survey on Detection,Isolation, and
%Identification" by Sami Haddadin et al.

% Extract the robot model
smds = extractSystemModel(robotURDFModel);

%Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);
f_ext = SX.sym('f_ext', 6,1,smds.NB);
tau = SX.sym('tau', [smds.NB,1]);
r = SX.sym('r', [smds.NB,1]);
p = SX.sym('p', [smds.NB,1]);
% Compue the Mass and Coriolis matrix
[H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(q,qd,qdd,g,smds);
H = cell2mat_casadi(H_cell);
HDot = cell2mat_casadi(HDot_cell);
C = cell2mat_casadi(C_cell);
%% Compute gravity torques using the symbolic Inverse Dynamics function
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,0);
tau_gravity = computeGravityTorque(q,g,symbolicIDFunction);
K = eye(smds.NB);
beta = tau_gravity - (C.')*qd;

pHatDot = tau-beta+r;

%%
dae = struct('x',p,'p',[q;qd;tau;g;r],'ode',tau-beta+r);
F = integrator('F', 'idas', dae);

%%
int = F;
observerOutput = K*(p(q,qd) - int - p(0,0));
end