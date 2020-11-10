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

%% Formulate an integrator
params = {q,qd,g,r};
% Form an ode function
ode = Function('ode',{params},{pHatDot});

%%%%%%%%%%%% Creating a simulator %%%%%%%%%%
N_steps_per_sample = 10;
fs = 250; % Sampling frequency [hz]
dt = 1/fs/N_steps_per_sample;

% Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(params);
k2 = ode(params+dt/2.0*k1);
k3 = ode(params+dt/2.0*k2);
k4 = ode(params+dt*k3);

states_final = params+dt/6.0*(k1+2*k2+2*k3+k4);

% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{params},{states_final});

X = params;
for i=1:N_steps_per_sample
    X = one_step(params);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{params}, {X});

% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

N = 10000;  % Number of samples
all_samples = one_sample.mapaccum('all_samples', N);
%%
observerOutput = K*(p(q,qd) - int - p(0,0));
end