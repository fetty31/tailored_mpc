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
N = 20; % Horizon Length
n_states = 5;
n_controls = 3;

% Call function that generates the solver
cd(solverDir);
[model, codeoptions] = generate_solver(solverDir, N, n_states, n_controls);
output1= newOutput('U',1:N,1:4);
output2= newOutput('X',1:N,5:n_states+n_controls);
[stages, options, formulation] = FORCES_NLP(model, codeoptions, [output1, output2]);
cd(filePath);

