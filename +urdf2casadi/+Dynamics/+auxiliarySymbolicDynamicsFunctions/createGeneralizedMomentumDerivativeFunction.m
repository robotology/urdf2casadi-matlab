function pHatDotFunction = createGeneralizedMomentumDerivativeFunction(robotURDFModel)

% Extract the robot model
smds = extractSystemModel(robotURDFModel);

% Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);
tau = SX.sym('tau', [smds.NB,1]);
r = SX.sym('r', [smds.NB,1]);

%% Compue the Mass and Coriolis matrix
[H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(q,qd,qdd,smds);
H = cell2mat_casadi(H_cell);
HDot = cell2mat_casadi(HDot_cell);
C = cell2mat_casadi(C_cell);

%% Compute gravity torques using the symbolic Inverse Dynamics function
symbolicIDFunction = symbolicInverseDynamics(robotURDFModel,0);
tau_gravity = computeGravityTorque(q,g,symbolicIDFunction);

beta = tau_gravity - (C.')*qd;

pHatDot = tau-beta+r;

pHatDotFunction = Function('computeGeneralizedMomentumDerivative',{q,qd,g,tau,r},{pHatDot},...
                    {'joints_position','joints_velocity','gravity','joints_torque','observerOutput'},...
                    {'generalizedMomentumDot'});

end