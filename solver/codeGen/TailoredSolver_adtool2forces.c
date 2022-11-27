/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2022. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "TailoredSolver/include/TailoredSolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "TailoredSolver_casadi.h"



/* copies data from sparse matrix into a dense one */
static void TailoredSolver_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, TailoredSolver_callback_float *data, TailoredSolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((TailoredSolver_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default TailoredSolver_adtool2forces(TailoredSolver_float *x,        /* primal vars                                         */
                                 TailoredSolver_float *y,        /* eq. constraint multiplers                           */
                                 TailoredSolver_float *l,        /* ineq. constraint multipliers                        */
                                 TailoredSolver_float *p,        /* parameters                                          */
                                 TailoredSolver_float *f,        /* objective function (scalar)                         */
                                 TailoredSolver_float *nabla_f,  /* gradient of objective function                      */
                                 TailoredSolver_float *c,        /* dynamics                                            */
                                 TailoredSolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 TailoredSolver_float *h,        /* inequality constraints                              */
                                 TailoredSolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 TailoredSolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const TailoredSolver_callback_float *in[4];
    TailoredSolver_callback_float *out[7];
	

	/* Allocate working arrays for AD tool */
	TailoredSolver_callback_float w[218];
	
    /* temporary storage for AD tool sparse output */
    TailoredSolver_callback_float this_f;
    TailoredSolver_callback_float nabla_f_sparse[10];
    TailoredSolver_callback_float h_sparse[5];
    TailoredSolver_callback_float nabla_h_sparse[17];
    TailoredSolver_callback_float c_sparse[7];
    TailoredSolver_callback_float nabla_c_sparse[54];
            
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 39))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		TailoredSolver_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = TailoredSolver_objective_1_sparsity_out(1)[0];
			ncol = TailoredSolver_objective_1_sparsity_out(1)[1];
			colind = TailoredSolver_objective_1_sparsity_out(1) + 2;
			row = TailoredSolver_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		TailoredSolver_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = TailoredSolver_dynamics_1_sparsity_out(0)[0];
			ncol = TailoredSolver_dynamics_1_sparsity_out(0)[1];
			colind = TailoredSolver_dynamics_1_sparsity_out(0) + 2;
			row = TailoredSolver_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = TailoredSolver_dynamics_1_sparsity_out(1)[0];
			ncol = TailoredSolver_dynamics_1_sparsity_out(1)[1];
			colind = TailoredSolver_dynamics_1_sparsity_out(1) + 2;
			row = TailoredSolver_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		TailoredSolver_inequalities_1(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = TailoredSolver_inequalities_1_sparsity_out(0)[0];
			ncol = TailoredSolver_inequalities_1_sparsity_out(0)[1];
			colind = TailoredSolver_inequalities_1_sparsity_out(0) + 2;
			row = TailoredSolver_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = TailoredSolver_inequalities_1_sparsity_out(1)[0];
			ncol = TailoredSolver_inequalities_1_sparsity_out(1)[1];
			colind = TailoredSolver_inequalities_1_sparsity_out(1) + 2;
			row = TailoredSolver_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 39) && (stage < 40))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		TailoredSolver_objective_40(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = TailoredSolver_objective_40_sparsity_out(1)[0];
			ncol = TailoredSolver_objective_40_sparsity_out(1)[1];
			colind = TailoredSolver_objective_40_sparsity_out(1) + 2;
			row = TailoredSolver_objective_40_sparsity_out(1) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		TailoredSolver_inequalities_40(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = TailoredSolver_inequalities_40_sparsity_out(0)[0];
			ncol = TailoredSolver_inequalities_40_sparsity_out(0)[1];
			colind = TailoredSolver_inequalities_40_sparsity_out(0) + 2;
			row = TailoredSolver_inequalities_40_sparsity_out(0) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = TailoredSolver_inequalities_40_sparsity_out(1)[0];
			ncol = TailoredSolver_inequalities_40_sparsity_out(1)[1];
			colind = TailoredSolver_inequalities_40_sparsity_out(1) + 2;
			row = TailoredSolver_inequalities_40_sparsity_out(1) + 2 + (ncol + 1);
			TailoredSolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((TailoredSolver_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
