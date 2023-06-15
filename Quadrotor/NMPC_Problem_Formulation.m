clear all
addpath('../ParNMPC/')
%% Formulate an OCP using Class OptimalControlProblem

% Create an OptimalControlProblem object
OCP = OptimalControlProblem(3,... % dim of inputs 
                           12,... % dim of states 
                            0,... % dim of parameters 
                            24);  % N: num of discritization grids

% Give names to x, u, p
[omega_x , omega_y , omega_z , ta_x , eta_y , eta_z , etadot_x , etadot_y , etadot_z , phi , theta ,psi] = ...
    OCP.setStateName({'omega_x','omega_y','omega_z','eta_x','eta_y','eta_z','etadot_x','etadot_y','etadot_z','phi','theta','psi'});
[u1,u2,u3] = ...
    OCP.setInputName({'u1','u2','u3'});

% Set the prediction horizon T
OCP.setT(1);

% Set the dynamic function f
g = 9.81;
f = [ 2.68e-4*eta_y - 0.00933*eta_x - 6.61e-4*eta_z - 1.36e-4*eta_dot_x + 1.52e-5*eta_dot_y - 3.53e-5*eta_dot_z + 0.00255*u1 - 1.06e-4*u2 - 2.41e-4*u3 + 7.89e-5*eta_dot_x*omega_x - 0.00705*eta_dot_x*omega_y + 3.98e-4*eta_dot_y*omega_x + 0.00394*eta_dot_x*omega_z + 0.00457*eta_dot_y*omega_y + 6.88e-4*eta_dot_z*omega_x + 0.00221*eta_dot_y*omega_z + 0.00186*eta_dot_z*omega_y + 0.00647*eta_dot_z*omega_z - 0.0281*omega_x*omega_y + 0.027*omega_x*omega_z + 0.229*omega_y*omega_z + 2.98e-4*omega_x^2 - 0.0262*omega_y^2 + 0.0259*omega_z^2;
0.00347*u2 - 3.3e-4*eta_y - 0.00205*eta_z - 2.95e-5*eta_dot_x - 1.87e-5*eta_dot_y - 1.1e-4*eta_dot_z - 1.06e-4*u1 - 0.00202*eta_x - 1.5e-4*u3 + 0.00768*eta_dot_x*omega_x - 7.39e-4*eta_dot_x*omega_y - 0.00567*eta_dot_y*omega_x - 0.0226*eta_dot_x*omega_z + 1.12e-5*eta_dot_y*omega_y - 0.00253*eta_dot_z*omega_x + 0.00427*eta_dot_y*omega_z - 2.56e-4*eta_dot_z*omega_y - 0.00414*eta_dot_z*omega_z + 0.0247*omega_x*omega_y - 0.555*omega_x*omega_z - 0.0206*omega_y*omega_z + 0.0143*omega_x^2 + 6.11e-4*omega_y^2 - 0.015*omega_z^2; 
7.68e-4*eta_y - 0.00543*eta_x + 0.00116*eta_z - 7.92e-5*eta_dot_x + 4.35e-5*eta_dot_y + 6.2e-5*eta_dot_z - 2.41e-4*u1 - 1.5e-4*u2 + 0.00508*u3 - 0.00681*eta_dot_x*omega_x + 0.0333*eta_dot_x*omega_y - 0.00441*eta_dot_y*omega_x + 6.61e-4*eta_dot_x*omega_z - 0.00679*eta_dot_y*omega_y - 0.0125*eta_dot_z*omega_x - 4.1e-4*eta_dot_y*omega_z + 0.00547*eta_dot_z*omega_y - 4.32e-4*eta_dot_z*omega_z + 0.355*omega_x*omega_y - 0.0275*omega_x*omega_z - 9.14e-4*omega_y*omega_z - 0.0158*omega_x^2 + 0.0176*omega_y^2 - 0.00181*omega_z^2;
eta_dot_x;
eta_dot_y;
eta_dot_z;
0.00439*eta_z - 0.00297*eta_y - 0.515*eta_x - 0.00752*eta_dot_x - 1.68e-4*eta_dot_y + 2.34e-4*eta_dot_z - 0.0158*u1 - 0.00343*u2 - 0.0092*u3 + 0.00436*eta_dot_x*omega_x - 0.0253*eta_dot_x*omega_y + 0.0142*eta_dot_y*omega_x + 0.00195*eta_dot_x*omega_z - 0.0149*eta_dot_y*omega_y + 0.0258*eta_dot_z*omega_x - 0.0188*eta_dot_y*omega_z - 0.0235*eta_dot_z*omega_y - 0.0355*eta_dot_z*omega_z - 0.615*omega_x*omega_y + 0.594*omega_x*omega_z - 1.45*omega_y*omega_z + 0.0139*omega_x^2 + 0.13*omega_y^2 - 0.144*omega_z^2;
0.003*eta_z - 0.0904*eta_y - 0.019*eta_x - 2.76e-4*eta_dot_x - 0.00512*eta_dot_y + 1.6e-4*eta_dot_z + 0.0029*u1 - 0.00357*u2 + 0.00833*u3 - 0.0183*eta_dot_x*omega_x + 0.0475*eta_dot_x*omega_y - 0.00167*eta_dot_y*omega_x + 0.0268*eta_dot_x*omega_z - 0.00562*eta_dot_y*omega_y - 0.0177*eta_dot_z*omega_x - 0.00183*eta_dot_y*omega_z + 0.0117*eta_dot_z*omega_y + 0.0112*eta_dot_z*omega_z + 0.536*omega_x*omega_y + 0.497*omega_x*omega_z + 0.305*omega_y*omega_z - 0.0393*omega_x^2 - 0.00405*omega_y^2 + 0.0433*omega_z^2;
0.0109*eta_x + 0.00116*eta_y - 0.231*eta_z + 1.59e-4*eta_dot_x + 6.59e-5*eta_dot_y - 0.0123*eta_dot_z - 0.00279*u1 - 0.00865*u2 + 0.00489*u3 - 0.0249*eta_dot_x*omega_x + 0.0376*eta_dot_x*omega_y + 0.00999*eta_dot_y*omega_x + 0.0523*eta_dot_x*omega_z - 0.0108*eta_dot_y*omega_y - 0.00493*eta_dot_z*omega_x - 0.0134*eta_dot_y*omega_z + 0.00313*eta_dot_z*omega_y + 0.00273*eta_dot_z*omega_z + 0.267*omega_x*omega_y + 1.33*omega_x*omega_z - 0.205*omega_y*omega_z - 0.0493*omega_x^2 + 0.0425*omega_y^2 + 0.00676*omega_z^2
omega_x + omega_z*cos(phi)*tan(theta) + omega_y*sin(phi)*tan(theta);
omega_y*cos(phi) - 1.0*omega_z*sin(phi);
(omega_z*cos(phi) + omega_y*sin(phi))/cos(theta)];
OCP.setf(f);
OCP.setDiscretizationMethod('RK4');

% Set the cost function L
Q = diag([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
R = diag([0, 0, 0]);
q_terminal = [2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000];
xRef = [0, 0, 0, 0, 0, 0, 0, 0, 0,0.1, 0, 0];

uRef = [0;0;0];
L =    0.5*(OCP.x-xRef).'*Q*(OCP.x-xRef)...
     + 0.5*(OCP.u(1:3)-uRef).'*R*(OCP.u(1:3)-uRef);
phi = sum((q_terminal.*(x-x_ref).^2)/2);
L = L + phi;

OCP.setL(L);

% Set the linear constraints G(u,x,p)>=0
G =-[OCP.u(1:3) - [10;10;10];...
    -OCP.u(1:3) - [-10;-10;-10]];
OCP.setG(G);

% Generate necessary files
OCP.codeGen();
%% Configrate the solver using Class NMPCSolver

% Create a NMPCSolver object
nmpcSolver = NMPCSolver(OCP);

% Configurate the Hessian approximation method
nmpcSolver.setHessianApproximation('Newton');

% Generate necessary files
nmpcSolver.codeGen();
%% Solve the very first OCP for a given initial state and given parameters using Class OCPSolver

% Set the initial state
x0 =   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

% Set the parameters
dim      = OCP.dim;
N        = OCP.N;
p      = zeros(dim.p,N);

% Solve the very first OCP 
solutionInitGuess.lambda = [randn(dim.lambda,1),zeros(dim.lambda,1)];
solutionInitGuess.mu     = randn(dim.mu,1);
solutionInitGuess.u      = [uRef;1];
solutionInitGuess.x      = [x0,[0;0;0;0;0;0;0;0;0]];
solutionInitGuess.z      = ones(dim.z,N);
solution = NMPC_SolveOffline(x0,p,solutionInitGuess,0.01,1000);

plot(solution.x([1 3 5],:).');

% Save to file
save GEN_initData.mat dim x0 p N

% Set initial guess
global ParNMPCGlobalVariable
ParNMPCGlobalVariable.solutionInitGuess = solution;
%% Define the controlled plant using Class DynamicSystem

% M(u,x,p) \dot(x) = f(u,x,p)
% Create a DynamicSystem object
plant = DynamicSystem(4,9,0);

% Give names to x, u
[omega_x , omega_y , omega_z , ta_x , eta_y , eta_z , etadot_x , etadot_y , etadot_z , phi , theta ,psi] = ...
    plant.setStateName({'omega_x','omega_y','omega_z','eta_x','eta_y','eta_z','etadot_x','etadot_y','etadot_z','phi','theta','psi'});
[u1,u2,u3] = ...
    plant.setInputName({'u1','u2','u3'});
g = 9.81;
fPlant = [  2.68e-4*eta_y - 0.00933*eta_x - 6.61e-4*eta_z - 1.36e-4*eta_dot_x + 1.52e-5*eta_dot_y - 3.53e-5*eta_dot_z + 0.00255*u1 - 1.06e-4*u2 - 2.41e-4*u3 + 7.89e-5*eta_dot_x*omega_x - 0.00705*eta_dot_x*omega_y + 3.98e-4*eta_dot_y*omega_x + 0.00394*eta_dot_x*omega_z + 0.00457*eta_dot_y*omega_y + 6.88e-4*eta_dot_z*omega_x + 0.00221*eta_dot_y*omega_z + 0.00186*eta_dot_z*omega_y + 0.00647*eta_dot_z*omega_z - 0.0281*omega_x*omega_y + 0.027*omega_x*omega_z + 0.229*omega_y*omega_z + 2.98e-4*omega_x^2 - 0.0262*omega_y^2 + 0.0259*omega_z^2;
0.00347*u2 - 3.3e-4*eta_y - 0.00205*eta_z - 2.95e-5*eta_dot_x - 1.87e-5*eta_dot_y - 1.1e-4*eta_dot_z - 1.06e-4*u1 - 0.00202*eta_x - 1.5e-4*u3 + 0.00768*eta_dot_x*omega_x - 7.39e-4*eta_dot_x*omega_y - 0.00567*eta_dot_y*omega_x - 0.0226*eta_dot_x*omega_z + 1.12e-5*eta_dot_y*omega_y - 0.00253*eta_dot_z*omega_x + 0.00427*eta_dot_y*omega_z - 2.56e-4*eta_dot_z*omega_y - 0.00414*eta_dot_z*omega_z + 0.0247*omega_x*omega_y - 0.555*omega_x*omega_z - 0.0206*omega_y*omega_z + 0.0143*omega_x^2 + 6.11e-4*omega_y^2 - 0.015*omega_z^2; 
7.68e-4*eta_y - 0.00543*eta_x + 0.00116*eta_z - 7.92e-5*eta_dot_x + 4.35e-5*eta_dot_y + 6.2e-5*eta_dot_z - 2.41e-4*u1 - 1.5e-4*u2 + 0.00508*u3 - 0.00681*eta_dot_x*omega_x + 0.0333*eta_dot_x*omega_y - 0.00441*eta_dot_y*omega_x + 6.61e-4*eta_dot_x*omega_z - 0.00679*eta_dot_y*omega_y - 0.0125*eta_dot_z*omega_x - 4.1e-4*eta_dot_y*omega_z + 0.00547*eta_dot_z*omega_y - 4.32e-4*eta_dot_z*omega_z + 0.355*omega_x*omega_y - 0.0275*omega_x*omega_z - 9.14e-4*omega_y*omega_z - 0.0158*omega_x^2 + 0.0176*omega_y^2 - 0.00181*omega_z^2;
eta_dot_x;
eta_dot_y;
eta_dot_z;
0.00439*eta_z - 0.00297*eta_y - 0.515*eta_x - 0.00752*eta_dot_x - 1.68e-4*eta_dot_y + 2.34e-4*eta_dot_z - 0.0158*u1 - 0.00343*u2 - 0.0092*u3 + 0.00436*eta_dot_x*omega_x - 0.0253*eta_dot_x*omega_y + 0.0142*eta_dot_y*omega_x + 0.00195*eta_dot_x*omega_z - 0.0149*eta_dot_y*omega_y + 0.0258*eta_dot_z*omega_x - 0.0188*eta_dot_y*omega_z - 0.0235*eta_dot_z*omega_y - 0.0355*eta_dot_z*omega_z - 0.615*omega_x*omega_y + 0.594*omega_x*omega_z - 1.45*omega_y*omega_z + 0.0139*omega_x^2 + 0.13*omega_y^2 - 0.144*omega_z^2;
0.003*eta_z - 0.0904*eta_y - 0.019*eta_x - 2.76e-4*eta_dot_x - 0.00512*eta_dot_y + 1.6e-4*eta_dot_z + 0.0029*u1 - 0.00357*u2 + 0.00833*u3 - 0.0183*eta_dot_x*omega_x + 0.0475*eta_dot_x*omega_y - 0.00167*eta_dot_y*omega_x + 0.0268*eta_dot_x*omega_z - 0.00562*eta_dot_y*omega_y - 0.0177*eta_dot_z*omega_x - 0.00183*eta_dot_y*omega_z + 0.0117*eta_dot_z*omega_y + 0.0112*eta_dot_z*omega_z + 0.536*omega_x*omega_y + 0.497*omega_x*omega_z + 0.305*omega_y*omega_z - 0.0393*omega_x^2 - 0.00405*omega_y^2 + 0.0433*omega_z^2;
0.0109*eta_x + 0.00116*eta_y - 0.231*eta_z + 1.59e-4*eta_dot_x + 6.59e-5*eta_dot_y - 0.0123*eta_dot_z - 0.00279*u1 - 0.00865*u2 + 0.00489*u3 - 0.0249*eta_dot_x*omega_x + 0.0376*eta_dot_x*omega_y + 0.00999*eta_dot_y*omega_x + 0.0523*eta_dot_x*omega_z - 0.0108*eta_dot_y*omega_y - 0.00493*eta_dot_z*omega_x - 0.0134*eta_dot_y*omega_z + 0.00313*eta_dot_z*omega_y + 0.00273*eta_dot_z*omega_z + 0.267*omega_x*omega_y + 1.33*omega_x*omega_z - 0.205*omega_y*omega_z - 0.0493*omega_x^2 + 0.0425*omega_y^2 + 0.00676*omega_z^2
omega_x + omega_z*cos(phi)*tan(theta) + omega_y*sin(phi)*tan(theta);
omega_y*cos(phi) - 1.0*omega_z*sin(phi);
(omega_z*cos(phi) + omega_y*sin(phi))/cos(theta)];

% Set the dynamic function f
plant.setf(fPlant); % same model 

% Generate necessary files
plant.codeGen();
