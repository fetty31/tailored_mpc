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
% - lb - matrix of size [480x1]
% - ub - matrix of size [400x1]
% - hu - matrix of size [200x1]
% - xinit - matrix of size [10x1]
% - x0 - matrix of size [480x1]
% - all_parameters - matrix of size [1240x1]
% - num_of_threads - scalar
% Outputs:
% - U - column vector of length 280
% - X - column vector of length 200
function [U, X] = TailoredSolver(lb, ub, hu, xinit, x0, all_parameters, num_of_threads)
    
    [output, ~, ~] = TailoredSolverBuildable.forcesCall(lb, ub, hu, xinit, x0, all_parameters, num_of_threads);
    U = output.U;
    X = output.X;
end
