%% Add paths
clear;clc;close all;

addpath('/home/fetty/FORCES_client/');
filePath = fileparts(mfilename('fullpath'));
cd(filePath);
addpath(filePath);

%% Generate FORCESPRO solver

% Create a new folder for all solver related files
solverDir = [filePath, filesep, 'codeGen'];
if ~exist(solverDir,'dir')
    mkdir(solverDir)
end
addpath(solverDir);

%% Problem info
t = 25e-3; % Runge Kutta integrator time
N = 40; % Horizon Length

% Call function that generates the solver
cd(solverDir);
[model, codeoptions] = generate_solver(solverDir, t, N);
[stages, options, formulation] = FORCES_NLP(model, codeoptions);
cd(filePath);

%% SIMULATION LOOP



