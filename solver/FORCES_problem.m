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
N = 80; % Horizon Length
n_states = 4; % [y, vy, heading (or yaw), r] 
n_controls = 1; % [delta]

% Call function that generates the solver
cd(solverDir);
[model, codeoptions] = generate_solver_cartesian(solverDir, N, n_states, n_controls);
output1= newOutput('U',1:N,1:1);
output2= newOutput('X',1:N,2:n_states+n_controls);
[stages, options, formulation] = FORCES_NLP(model, codeoptions, [output1, output2]);
cd(filePath);

