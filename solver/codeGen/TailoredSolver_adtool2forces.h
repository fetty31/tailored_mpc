#ifdef __cplusplus
extern "C" {
#endif
    
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
										 solver_int32_default stage,     /* stage number (0 indexed)                            */
										 solver_int32_default iteration, /* iteration number of solver                          */
										 solver_int32_default threadID  /* Id of caller thread */);

#ifdef __cplusplus
} /* extern "C" */
#endif
