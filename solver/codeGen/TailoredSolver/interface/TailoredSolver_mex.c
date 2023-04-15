/*
TailoredSolver : A fast customized optimization solver.

Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.


This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#include "mex.h"
#include "math.h"
#include "../include/TailoredSolver.h"
#include "../include/TailoredSolver_memory.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}

/* copy functions */

void copyCArrayToM_solver_int32_unsigned(solver_int32_unsigned *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_solver_int32_unsigned(double *src, solver_int32_unsigned *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (solver_int32_unsigned) (*src++) ;
    }
}

void copyMValueToC_solver_int32_unsigned(double * src, solver_int32_unsigned * dest)
{
	*dest = (solver_int32_unsigned) *src;
}

/* copy functions */

void copyCArrayToM_solver_int32_default(solver_int32_default *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_solver_int32_default(double *src, solver_int32_default *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (solver_int32_default) (*src++) ;
    }
}

void copyMValueToC_solver_int32_default(double * src, solver_int32_default * dest)
{
	*dest = (solver_int32_default) *src;
}

/* copy functions */

void copyCArrayToM_TailoredSolver_float(TailoredSolver_float *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_TailoredSolver_float(double *src, TailoredSolver_float *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (TailoredSolver_float) (*src++) ;
    }
}

void copyMValueToC_TailoredSolver_float(double * src, TailoredSolver_float * dest)
{
	*dest = (TailoredSolver_float) *src;
}



extern solver_int32_default TailoredSolver_adtool2forces(TailoredSolver_float *x, TailoredSolver_float *y, TailoredSolver_float *l, TailoredSolver_float *p, TailoredSolver_float *f, TailoredSolver_float *nabla_f, TailoredSolver_float *c, TailoredSolver_float *nabla_c, TailoredSolver_float *h, TailoredSolver_float *nabla_h, TailoredSolver_float *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
TailoredSolver_extfunc pt2function_TailoredSolver = &TailoredSolver_adtool2forces;


/* Some memory for mex-function */
static TailoredSolver_params params;
static TailoredSolver_output output;
static TailoredSolver_info info;
static TailoredSolver_mem * mem;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[2] = {"U", "X"};
	const solver_int8_default *infofields[20] = { "it", "it2opt", "res_eq", "res_ineq", "rsnorm", "rcompnorm", "pobj", "dobj", "dgap", "rdgap", "mu", "mu_aff", "sigma", "lsit_aff", "lsit_cc", "step_aff", "step_cc", "solvetime", "fevalstime", "solver_id"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help TailoredSolver_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help TailoredSolver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

    /* initialize memory */
    if (mem == NULL)
    {
        mem = TailoredSolver_internal_mem(0);
    }

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "lb");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.lb not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb must be a double.");
    }
    if( mxGetM(par) != 320 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.lb must be of size [320 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.lb,320);

	}
	par = mxGetField(PARAMS, 0, "ub");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.ub not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub must be a double.");
    }
    if( mxGetM(par) != 280 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.ub must be of size [280 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.ub,280);

	}
	par = mxGetField(PARAMS, 0, "hu");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.hu not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.hu must be a double.");
    }
    if( mxGetM(par) != 80 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.hu must be of size [80 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.hu,80);

	}
	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 7 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.xinit must be of size [7 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.xinit,7);

	}
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 320 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.x0 must be of size [320 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.x0,320);

	}
	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 1000 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.all_parameters must be of size [1000 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.all_parameters,1000);

	}
	par = mxGetField(PARAMS, 0, "num_of_threads");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_solver_int32_unsigned(mxGetPr(par), &params.num_of_threads);

	}




	#if SET_PRINTLEVEL_TailoredSolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */

	exitflag = TailoredSolver_solve(&params, &output, &info, mem, fp, pt2function_TailoredSolver);
	
	#if SET_PRINTLEVEL_TailoredSolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 2, outputnames);
		/* column vector of length 160 */
	outvar = mxCreateDoubleMatrix(160, 1, mxREAL);
	copyCArrayToM_double((&(output.U[0])), mxGetPr(outvar), 160);
	mxSetField(plhs[0], 0, "U", outvar);


	/* column vector of length 160 */
	outvar = mxCreateDoubleMatrix(160, 1, mxREAL);
	copyCArrayToM_double((&(output.X[0])), mxGetPr(outvar), 160);
	mxSetField(plhs[0], 0, "X", outvar);


	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	plhs[2] = mxCreateStructMatrix(1, 1, 20, infofields);
				/* scalar: iteration number */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.it)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "it", outvar);


		/* scalar: number of iterations needed to optimality (branch-and-bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.it2opt)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "it2opt", outvar);


		/* scalar: inf-norm of equality constraint residuals */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.res_eq)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "res_eq", outvar);


		/* scalar: inf-norm of inequality constraint residuals */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.res_ineq)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "res_ineq", outvar);


		/* scalar: norm of stationarity condition */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.rsnorm)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "rsnorm", outvar);


		/* scalar: max of all complementarity violations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.rcompnorm)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "rcompnorm", outvar);


		/* scalar: primal objective */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.pobj)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "pobj", outvar);


		/* scalar: dual objective */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.dobj)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "dobj", outvar);


		/* scalar: duality gap := pobj - dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.dgap)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "dgap", outvar);


		/* scalar: relative duality gap := |dgap / pobj | */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.rdgap)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "rdgap", outvar);


		/* scalar: duality measure */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.mu)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "mu", outvar);


		/* scalar: duality measure (after affine step) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.mu_aff)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "mu_aff", outvar);


		/* scalar: centering parameter */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.sigma)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "sigma", outvar);


		/* scalar: number of backtracking line search steps (affine direction) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.lsit_aff)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "lsit_aff", outvar);


		/* scalar: number of backtracking line search steps (combined direction) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.lsit_cc)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "lsit_cc", outvar);


		/* scalar: step size (affine direction) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.step_aff)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "step_aff", outvar);


		/* scalar: step size (combined direction) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.step_cc)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "step_cc", outvar);


		/* scalar: total solve time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.solvetime)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "solvetime", outvar);


		/* scalar: time spent in function evaluations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_TailoredSolver_float((&(info.fevalstime)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "fevalstime", outvar);


		/* column vector of length 8: solver ID of FORCESPRO solver */
		outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.solver_id[0])), mxGetPr(outvar), 8);
		mxSetField(plhs[2], 0, "solver_id", outvar);

	}
}
