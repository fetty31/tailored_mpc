/*
 Copyright (c) 2023 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef forcespro_MPC_HH
#define forcespro_MPC_HH

#include "../solver/codeGen/TailoredSolver/include/TailoredSolver_memory.h"


extern "C" void TailoredSolver_adtool2forces(TailoredSolver_float* x,           /* primal vars                                          */
                                            TailoredSolver_float* y,            /* eq. constraint multiplers                            */
                                            TailoredSolver_float* lambda,       /* ineq. constraint multipliers                         */
                                            TailoredSolver_float* params,       /* parameters                                           */
                                            TailoredSolver_float* pobj,         /* objective function (scalar)                          */
                                            TailoredSolver_float* g,            /* gradient of objective function                       */
                                            TailoredSolver_float* c,            /* dynamics                                             */
                                            TailoredSolver_float* Jeq,          /* Jacobian of the dynamics (column major)              */
                                            TailoredSolver_float* h,            /* inequality constraints                               */
                                            TailoredSolver_float* Jineq,        /* Jacobian of inequality constraints (column major)    */
                                            TailoredSolver_float* H,            /* Hessian (column major)                               */
                                            solver_int32_default stage,         /* stage number (0 indexed)                             */
                                            solver_int32_default iterations,    /* iteration number of solver                           */
                                            solver_int32_default threadID       /* Id of caller thread                                  */
                                                                            );

struct ForcesproSolver{

    ForcesproSolver(){
        exit_flag = -1;
    }
    
    TailoredSolver_params params;                                       /* Real time parameters vectors */

    TailoredSolver_output solution;                                     /* Output --> U vec for control vars, X vec for state vars*/

    TailoredSolver_info info;                                           /* Solver info */

    TailoredSolver_extfunc ext_func = &TailoredSolver_adtool2forces;    /* Algorithmic differentation function call */
        
    TailoredSolver_mem * mem_handle;                                    /* Handle to the solver memory */

    int exit_flag;                                                      /* Exit flag */
                                                                        /* == 1     OPTIMAL SOLUTION FOUND */
                                                                        /* == 0     MAXIMUM NUMBER OF ITERATIONS REACHED */
                                                                        /* == -7    SOLVER COULD NOT PROCEED (probably the problem is infeasible) */
                                                                        /* == -100  LICENSE ERROR */
                                                                        /* for more info see: https://forces.embotech.com/Documentation/exitflags/index.html#tab-exitflag-values */
};

#endif
