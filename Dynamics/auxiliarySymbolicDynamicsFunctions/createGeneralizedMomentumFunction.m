function pFunction = createGeneralizedMomentumFunction(robotURDFModel)

% Extract the robot model
smds = extractSystemModel(robotURDFModel);

% Initialize variables
import casadi.*;
q = SX.sym('q',[smds.NB,1]);
qd = SX.sym('qd',[smds.NB,1]);
qdd = SX.sym('qdd',[smds.NB,1]);
g = SX.sym('g',[3,1]);

%% Compue the Mass and Coriolis matrix
[H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(q,qd,qdd,smds);
H = cell2mat_casadi(H_cell);
HDot = cell2mat_casadi(HDot_cell);
C = cell2mat_casadi(C_cell);

%% Create generalized momentum function
p = H*qd;
pFunction = Function('computeGeneralizedMomentum',{q,qd},{p},...
                    {'joints_position','joints_velocity'},...
                    {'generalizedMomentum'});

end