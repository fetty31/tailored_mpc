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
N = 40; % Horizon Length
n_states = 5; % [delta, y, vy, heading (or yaw), r] 
n_controls = 2; % [slack_track, diff_delta]

% Call function that generates the solver
cd(solverDir);
[model, codeoptions] = generate_solver_cartesian(solverDir, N, n_states, n_controls);
output1= newOutput('U',1:N,1:3);
output2= newOutput('X',1:N,4:n_states+n_controls);
[stages, options, formulation] = FORCES_NLP(model, codeoptions, [output1, output2]);
cd(filePath);

