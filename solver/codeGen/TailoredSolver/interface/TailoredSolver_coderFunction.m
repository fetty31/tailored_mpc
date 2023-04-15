% TailoredSolver : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
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
% - lb - matrix of size [320x1]
% - ub - matrix of size [280x1]
% - hu - matrix of size [80x1]
% - xinit - matrix of size [7x1]
% - x0 - matrix of size [320x1]
% - all_parameters - matrix of size [1000x1]
% - num_of_threads - scalar
% Outputs:
% - U - column vector of length 160
% - X - column vector of length 160
function [U, X] = TailoredSolver(lb, ub, hu, xinit, x0, all_parameters, num_of_threads)
    
    [output, ~, ~] = TailoredSolverBuildable.forcesCall(lb, ub, hu, xinit, x0, all_parameters, num_of_threads);
    U = output.U;
    X = output.X;
end
