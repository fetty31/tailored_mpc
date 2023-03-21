% TailoredSolver : A fast customized optimization solver.
% 
% Copyright (C) 2013-2022 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 
% [OUTPUTS] = TailoredSolver(INPUTS) solves an optimization problem where:
% Inputs:
% - lb - matrix of size [400x1]
% - ub - matrix of size [400x1]
% - xinit - matrix of size [5x1]
% - x0 - matrix of size [400x1]
% - all_parameters - matrix of size [1920x1]
% - num_of_threads - scalar
% Outputs:
% - outputs - column vector of length 400
function [outputs] = TailoredSolver(lb, ub, xinit, x0, all_parameters, num_of_threads)
    
    [output, ~, ~] = TailoredSolverBuildable.forcesCall(lb, ub, xinit, x0, all_parameters, num_of_threads);
    outputs = coder.nullcopy(zeros(400,1));
    outputs(1:80) = output.U;
    outputs(81:400) = output.X;
end
