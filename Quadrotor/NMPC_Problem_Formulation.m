clear all
addpath('../ParNMPC/')
%% Formulate an OCP using Class OptimalControlProblem

% Create an OptimalControlProblem object
OCP = OptimalControlProblem(12,... % dim of inputs 
                           12,... % dim of states 
                            3,... % dim of parameters 
                            48);  % N: num of discritization grids

% Give names to x, u, p
[X,dX,Y,dY,Z,dZ,Gamma,Beta,Alpha] = ...
    OCP.setStateName({'omega_x','omega_y','omega_z','eta_x','eta_y','eta_z','etadot_x','etadot_y','etadot_z','phi','theta','psi'});
[a,omegaX,omegaY,omegaZ,slack] = ...
    OCP.setInputName({'a','omegaX','omegaY','omegaZ','slack'});
[XSP,YSP,ZSP] = ...
    OCP.setParameterName({'XSP','YSP','ZSP'},[1 2 3]);

% Set the prediction horizon T
OCP.setT(1);

% Set the dynamic function f
g = 9.81;
f = [  2.74e-4 * x[4] - 0.0102 * x[3] - 6.84e-4 * x[5] - 1.53e-4 * x[6] + 1.65e-5 * x[7] - 2.74e-5 * x[8] + 0.00254 * u[0] - 8.76e-5 * u[1] - 2.01e-4 * u[2] + 6.79e-5 * x[6] * x[0] - 0.00689 * x[6] * x[1] + 3.5e-4 * x[7] * x[0] + 0.00387 * x[6] * x[2] + 0.00458 * x[7] * x[1] + 5.72e-4 * x[8] * x[0] + 0.00243 * x[7] * x[2] + 0.00181 * x[8] * x[1] + 0.00645 * x[8] * x[2] - 0.014 * x[0] * x[1] + 0.014 * x[0] * x[2] + 0.229 * x[1] * x[2];
0.00346 * u[1] - 3.19e-4 * x[4] - 0.00214 * x[5] - 3.82e-5 * x[6] - 1.91e-5 * x[7] - 8.55e-5 * x[8] - 8.76e-5 * u[0] - 0.00255 * x[3] + 2.16e-5 * u[2] + 0.00759 * x[6] * x[0] + 3.33e-4 * x[6] * x[1] - 0.00591 * x[7] * x[0] - 0.0226 * x[6] * x[2] - 1.77e-4 * x[7] * x[1] - 0.00283 * x[8] * x[0] + 0.00442 * x[7] * x[2] - 4.64e-5 * x[8] * x[1] - 0.00403 * x[8] * x[2] + 0.00151 * x[0] * x[1] - 0.554 * x[0] * x[2] - 0.00789 * x[1] * x[2];
7.48e-4 * x[4] - 0.0063 * x[3] + 0.00105 * x[5] - 9.46e-5 * x[6] + 4.49e-5 * x[7] + 4.21e-5 * x[8] - 2.01e-4 * u[0] + 2.16e-5 * u[1] + 0.00506 * u[2] - 0.00653 * x[6] * x[0] + 0.0333 * x[6] * x[1] - 0.00509 * x[7] * x[0] - 4.01e-4 * x[6] * x[2] - 0.00692 * x[7] * x[1] - 0.0127 * x[8] * x[0] - 1.73e-4 * x[7] * x[2] + 0.0054 * x[8] * x[1] - 5.25e-4 * x[8] * x[2] + 0.354 * x[0] * x[1] - 0.00345 * x[0] * x[2] - 0.0181 * x[1] * x[2];
x[6];
x[7];
x[8];
0.00491 * x[5] - 0.00302 * x[4] - 0.556 * x[3] - 0.00835 * x[6] - 1.81e-4 * x[7] + 1.96e-4 * x[8] - 0.016 * u[0] - 0.00398 * u[1] - 0.00985 * u[2] + 0.00405 * x[6] * x[0] - 0.0289 * x[6] * x[1] + 0.0166 * x[7] * x[0] + 0.00513 * x[6] * x[2] - 0.0143 * x[7] * x[1] + 0.0278 * x[8] * x[0] - 0.0211 * x[7] * x[2] - 0.0236 * x[8] * x[1] - 0.0355 * x[8] * x[2] - 0.689 * x[0] * x[1] + 0.637 * x[0] * x[2] - 1.44 * x[1] * x[2];
0.00304 * x[5] - 0.0881 * x[4] - 0.0214 * x[3] - 3.22e-4 * x[6] - 0.00528 * x[7] + 1.22e-4 * x[8] + 0.00305 * u[0] - 0.00354 * u[1] + 0.00831 * u[2] - 0.0186 * x[6] * x[0] + 0.0473 * x[6] * x[1] - 0.00229 * x[7] * x[0] + 0.027 * x[6] * x[2] - 0.00562 * x[7] * x[1] - 0.018 * x[8] * x[0] - 0.00155 * x[7] * x[2] + 0.0116 * x[8] * x[1] + 0.0115 * x[8] * x[2] + 0.582 * x[0] * x[1] + 0.567 * x[0] * x[2] + 0.274 * x[1] * x[2];
0.0126 * x[3] + 0.00109 * x[4] - 0.243 * x[5] + 1.88e-4 * x[6] + 6.56e-5 * x[7] - 0.00972 * x[8] - 0.00274 * u[0] - 0.00855 * u[1] + 0.00421 * u[2] - 0.0243 * x[6] * x[0] + 0.0334 * x[6] * x[1] + 0.0103 * x[7] * x[0] + 0.052 * x[6] * x[2] - 0.0101 * x[7] * x[1] - 0.00369 * x[8] * x[0] - 0.0138 * x[7] * x[2] + 0.00244 * x[8] * x[1] + 0.00256 * x[8] * x[2] + 0.295 * x[0] * x[1] + 1.37 * x[0] * x[2] - 0.246 * x[1] * x[2];
x[0] + x[2] * cos(x[9]) * tan(x[10]) + x[1] * sin(x[9]) * tan(x[10]);
x[1] * cos(x[9]) - 1.0 * x[2] * sin(x[9]);
(x[2] * cos(x[9]) + x[1] * sin(x[9])) / cos(x[10])];
OCP.setf(f);
OCP.setDiscretizationMethod('Euler');

% Set the cost function L
Q = diag([10, 1, 10, 1, 10, 1, 1, 1, 1]);
R = diag([0.1, 0.1, 0.1, 0.1]);
xRef = [0, 0, 0, 0, 0, 0, 0, 0, 0,0.1, 0, 0];

uRef = [0;0;0];
L =    0.5*(OCP.x-xRef).'*Q*(OCP.x-xRef)...
     + 0.5*(OCP.u(1:3)-uRef).'*R*(OCP.u(1:3)-uRef)...
     + 1e5*slack^2;
OCP.setL(L);

% Set the linear constraints G(u,x,p)>=0
G =-[OCP.u(1:4) - [11;1;1; 1];...
    -OCP.u(1:4) - [0;1;1;1];...
    -slack;...
     Gamma - slack - 0.2;...
    -Gamma - slack - 0.2;...
     Beta  - slack - 0.2;...
    -Beta  - slack - 0.2;...
     Alpha - slack - 0.2;...
    -Alpha - slack - 0.2];
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
x0 =   [1;0;1;0;1;0;0;0;0];

% Set the parameters
dim      = OCP.dim;
N        = OCP.N;
p      = zeros(dim.p,N);
p(1,:) = 0;   % X setpoint
p(2,:) = 0;   % Y setpoint
p(3,:) = 0;   % Z setpoint 

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
[X,dX,Y,dY,Z,dZ,Gamma,Beta,Alpha] = ...
    plant.setStateName({'X','dX','Y','dY','Z','dZ','Gamma','Beta','Alpha'});
[a,omegaX,omegaY,omegaZ] = ...
    plant.setInputName({'a','omegaX','omegaY','omegaZ'});
g = 9.81;
fPlant = [  dX;...
            a*(cos(Gamma)*sin(Beta)*cos(Alpha) + sin(Gamma)*sin(Alpha));...
            dY;...
            a*(cos(Gamma)*sin(Beta)*sin(Alpha) - sin(Gamma)*cos(Alpha));... 
            dZ;...
            a*cos(Gamma)*cos(Beta) - g;... 
           (omegaX*cos(Gamma) + omegaY*sin(Gamma))/cos(Beta);... 
           -omegaX*sin(Gamma) + omegaY*cos(Gamma);... 
            omegaX*cos(Gamma)*tan(Beta) + omegaY*sin(Gamma)*tan(Beta) + omegaZ];

% Set the dynamic function f
plant.setf(fPlant); % same model 

% Generate necessary files
plant.codeGen();
