#TailoredSolver : A fast customized optimization solver.
#
#Copyright (C) 2013-2022 EMBOTECH AG [info@embotech.com]. All rights reserved.
#
#
#This program is distributed in the hope that it will be useful.
#EMBOTECH makes NO WARRANTIES with respect to the use of the software 
#without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
#PARTICULAR PURPOSE. 
#
#EMBOTECH shall not have any liability for any damage arising from the use
#of the software.
#
#This Agreement shall exclusively be governed by and interpreted in 
#accordance with the laws of Switzerland, excluding its principles
#of conflict of laws. The Courts of Zurich-City shall have exclusive 
#jurisdiction in case of any dispute.
#
#def __init__():
'''
a Python wrapper for a fast solver generated by FORCESPRO v6.0.1

   OUTPUT = TailoredSolver_py.TailoredSolver_solve(PARAMS) solves a multistage problem
   subject to the parameters supplied in the following dictionary:
       PARAMS['lb'] - matrix of size [480x1]
       PARAMS['ub'] - matrix of size [480x1]
       PARAMS['xinit'] - matrix of size [6x1]
       PARAMS['x0'] - matrix of size [480x1]
       PARAMS['all_parameters'] - matrix of size [1920x1]
       PARAMS['num_of_threads'] - scalar

   OUTPUT returns the values of the last iteration of the solver where
       OUTPUT['U'] - column vector of length 160
       OUTPUT['X'] - column vector of length 320

   [OUTPUT, EXITFLAG] = TailoredSolver_py.TailoredSolver_solve(PARAMS) returns additionally
   the integer EXITFLAG indicating the state of the solution with 
       1 - Optimal solution has been found (subject to desired accuracy)
       2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
       0 - Timeout - maximum number of iterations reached
      -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
      -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
     -10 - The convex solver could not proceed due to an internal error
     -98 - Thread error
     -99 - Locking mechanism error
    -100 - License error
    -101 - Insufficient number of internal memory instances
    -102 - Number of threads larger than specified

   [OUTPUT, EXITFLAG, INFO] = TailoredSolver_py.TailoredSolver_solve(PARAMS) returns 
   additional information about the last iterate:
       INFO['it'] - scalar: iteration number
       INFO['it2opt'] - scalar: number of iterations needed to optimality (branch-and-bound)
       INFO['res_eq'] - scalar: inf-norm of equality constraint residuals
       INFO['res_ineq'] - scalar: inf-norm of inequality constraint residuals
       INFO['rsnorm'] - scalar: norm of stationarity condition
       INFO['rcompnorm'] - scalar: max of all complementarity violations
       INFO['pobj'] - scalar: primal objective
       INFO['dobj'] - scalar: dual objective
       INFO['dgap'] - scalar: duality gap := pobj - dobj
       INFO['rdgap'] - scalar: relative duality gap := |dgap / pobj |
       INFO['mu'] - scalar: duality measure
       INFO['mu_aff'] - scalar: duality measure (after affine step)
       INFO['sigma'] - scalar: centering parameter
       INFO['lsit_aff'] - scalar: number of backtracking line search steps (affine direction)
       INFO['lsit_cc'] - scalar: number of backtracking line search steps (combined direction)
       INFO['step_aff'] - scalar: step size (affine direction)
       INFO['step_cc'] - scalar: step size (combined direction)
       INFO['solvetime'] - scalar: total solve time
       INFO['fevalstime'] - scalar: time spent in function evaluations
       INFO['solver_id'] - column vector of length 8: solver ID of FORCESPRO solver

 See also COPYING

'''

import ctypes
import os
import numpy as np
import numpy.ctypeslib as npct
import sys

par_lib_name = 'libgomp.so.1'
error_message = 'OpenMP library required for solver execution could not be found. Make sure OpenMP is installed on your system and the path to the OpenMP library "libgomp.so.1" has been included in the "LD_LIBRARY_PATH" environment variable'
try:
    ctypes.CDLL(par_lib_name)
except:
    raise RuntimeError(error_message)


try:
    _lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'TailoredSolver/lib/TailoredSolver_withModel.so'))
    csolver = getattr(_lib,'TailoredSolver_solve')
except:
    _lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'TailoredSolver/lib/libTailoredSolver_withModel.so'))
    csolver = getattr(_lib,'TailoredSolver_solve')

internal_mem = getattr(_lib,'TailoredSolver_internal_mem')

class TailoredSolver_params_ctypes(ctypes.Structure):
#    @classmethod
#    def from_param(self):
#        return self
    _fields_ = [('lb', ctypes.c_double * 480),
('ub', ctypes.c_double * 480),
('xinit', ctypes.c_double * 6),
('x0', ctypes.c_double * 480),
('all_parameters', ctypes.c_double * 1920),
('num_of_threads', ctypes.c_uint),
]

TailoredSolver_params = {'lb' : np.array([]),
'ub' : np.array([]),
'xinit' : np.array([]),
'x0' : np.array([]),
'all_parameters' : np.array([]),
'num_of_threads' : np.array([]),
}
params = {'lb' : np.array([]),
'ub' : np.array([]),
'xinit' : np.array([]),
'x0' : np.array([]),
'all_parameters' : np.array([]),
'num_of_threads' : np.array([]),
}
TailoredSolver_params_types = {'lb' : np.float64,
'ub' : np.float64,
'xinit' : np.float64,
'x0' : np.float64,
'all_parameters' : np.float64,
'num_of_threads' : np.uint32,
}

class TailoredSolver_outputs_ctypes(ctypes.Structure):
#    @classmethod
#    def from_param(self):
#        return self
    _fields_ = [('U', ctypes.c_double * 160),
('X', ctypes.c_double * 320),
]

TailoredSolver_outputs = {'U' : np.array([]),
'X' : np.array([]),
}


class TailoredSolver_info(ctypes.Structure):
#    @classmethod
#    def from_param(self):
#        return self
    _fields_ = [('it', ctypes.c_int),
 ('it2opt', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('res_ineq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('rcompnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('dobj', ctypes.c_double),
 ('dgap', ctypes.c_double),
 ('rdgap', ctypes.c_double),
 ('mu', ctypes.c_double),
 ('mu_aff', ctypes.c_double),
 ('sigma', ctypes.c_double),
 ('lsit_aff', ctypes.c_int),
 ('lsit_cc', ctypes.c_int),
 ('step_aff', ctypes.c_double),
 ('step_cc', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('solver_id', ctypes.c_int * 8)
]

class FILE(ctypes.Structure):
    pass

class TailoredSolver_mem(ctypes.Structure):
    pass

if sys.version_info.major == 2:
    PyFile_AsFile = ctypes.pythonapi.PyFile_AsFile # problem here with python 3 http://stackoverflow.com/questions/16130268/python-3-replacement-for-pyfile-asfile
    PyFile_AsFile.argtypes = [ctypes.py_object]
    PyFile_AsFile.restype = ctypes.POINTER(FILE)

# determine data types for solver function prototype 
csolver.argtypes = ( ctypes.POINTER(TailoredSolver_params_ctypes), ctypes.POINTER(TailoredSolver_outputs_ctypes), ctypes.POINTER(TailoredSolver_info), ctypes.POINTER(TailoredSolver_mem), ctypes.POINTER(FILE))
csolver.restype = ctypes.c_int

internal_mem.argtypes = [ctypes.c_uint]
internal_mem.restype = ctypes.POINTER(TailoredSolver_mem)

def TailoredSolver_solve(params_arg):
    '''
a Python wrapper for a fast solver generated by FORCESPRO v6.0.1

   OUTPUT = TailoredSolver_py.TailoredSolver_solve(PARAMS) solves a multistage problem
   subject to the parameters supplied in the following dictionary:
       PARAMS['lb'] - matrix of size [480x1]
       PARAMS['ub'] - matrix of size [480x1]
       PARAMS['xinit'] - matrix of size [6x1]
       PARAMS['x0'] - matrix of size [480x1]
       PARAMS['all_parameters'] - matrix of size [1920x1]
       PARAMS['num_of_threads'] - scalar

   OUTPUT returns the values of the last iteration of the solver where
       OUTPUT['U'] - column vector of length 160
       OUTPUT['X'] - column vector of length 320

   [OUTPUT, EXITFLAG] = TailoredSolver_py.TailoredSolver_solve(PARAMS) returns additionally
   the integer EXITFLAG indicating the state of the solution with 
       1 - Optimal solution has been found (subject to desired accuracy)
       2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
       0 - Timeout - maximum number of iterations reached
      -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
      -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
     -10 - The convex solver could not proceed due to an internal error
     -98 - Thread error
     -99 - Locking mechanism error
    -100 - License error
    -101 - Insufficient number of internal memory instances
    -102 - Number of threads larger than specified

   [OUTPUT, EXITFLAG, INFO] = TailoredSolver_py.TailoredSolver_solve(PARAMS) returns 
   additional information about the last iterate:
       INFO['it'] - scalar: iteration number
       INFO['it2opt'] - scalar: number of iterations needed to optimality (branch-and-bound)
       INFO['res_eq'] - scalar: inf-norm of equality constraint residuals
       INFO['res_ineq'] - scalar: inf-norm of inequality constraint residuals
       INFO['rsnorm'] - scalar: norm of stationarity condition
       INFO['rcompnorm'] - scalar: max of all complementarity violations
       INFO['pobj'] - scalar: primal objective
       INFO['dobj'] - scalar: dual objective
       INFO['dgap'] - scalar: duality gap := pobj - dobj
       INFO['rdgap'] - scalar: relative duality gap := |dgap / pobj |
       INFO['mu'] - scalar: duality measure
       INFO['mu_aff'] - scalar: duality measure (after affine step)
       INFO['sigma'] - scalar: centering parameter
       INFO['lsit_aff'] - scalar: number of backtracking line search steps (affine direction)
       INFO['lsit_cc'] - scalar: number of backtracking line search steps (combined direction)
       INFO['step_aff'] - scalar: step size (affine direction)
       INFO['step_cc'] - scalar: step size (combined direction)
       INFO['solvetime'] - scalar: total solve time
       INFO['fevalstime'] - scalar: time spent in function evaluations
       INFO['solver_id'] - column vector of length 8: solver ID of FORCESPRO solver

 See also COPYING

    '''
    global _lib

    # convert parameters
    params_py = TailoredSolver_params_ctypes()
    for par in params_arg:
        if isinstance(params_arg[par], np.ndarray) and params_arg[par].size == 0:
            raise ValueError('Parameter ' + par + ' is unset. Please make sure to initialize the parameter before calling the solver.')
        try:
            #setattr(params_py, par, npct.as_ctypes(np.reshape(params_arg[par],np.size(params_arg[par]),order='A')))
            if isinstance(getattr(params_py, par), ctypes.Array):
                params_arg[par] = np.require(params_arg[par], dtype=TailoredSolver_params_types[par], requirements='F')
                setattr(params_py, par, npct.as_ctypes(np.reshape(params_arg[par],np.size(params_arg[par]),order='F')))
            else:
                setattr(params_py, par, params_arg[par])
        except:
            raise ValueError('Parameter ' + par + ' does not have the appropriate dimensions or data type. Please use numpy arrays for parameters.')
    
    outputs_py = TailoredSolver_outputs_ctypes()
    info_py = TailoredSolver_info()
    if sys.version_info.major == 2:
        if sys.platform.startswith('win'):
            fp = None # if set to none, the solver prints to stdout by default - necessary because we have an access violation otherwise under windows
        else:
            #fp = open('stdout_temp.txt','w')
            fp = sys.stdout
        try:
            PyFile_AsFile.restype = ctypes.POINTER(FILE)
            exitflag = _lib.TailoredSolver_solve( ctypes.byref(params_py), ctypes.byref(outputs_py), ctypes.byref(info_py), internal_mem(0), PyFile_AsFile(fp) , _lib.TailoredSolver_adtool2forces )
            #fp = open('stdout_temp.txt','r')
            #print (fp.read())
            #fp.close()
        except:
            #print 'Problem with solver'
            raise
    elif sys.version_info.major == 3:
        if sys.platform.startswith('win'):
            libc = ctypes.cdll.msvcrt
        elif sys.platform.startswith('darwin'):
            libc = ctypes.CDLL('libc.dylib')
        else:
            libc = ctypes.CDLL('libc.so.6')       # Open libc
        cfopen = getattr(libc,'fopen')        # Get its fopen
        cfopen.restype = ctypes.POINTER(FILE) # Yes, fopen gives a file pointer
        cfopen.argtypes = [ctypes.c_char_p, ctypes.c_char_p] # Yes, fopen gives a file pointer 
        fp = cfopen('stdout_temp.txt'.encode('utf-8'),'w'.encode('utf-8'))    # Use that fopen 

        try:
            if sys.platform.startswith('win'):
                exitflag = _lib.TailoredSolver_solve( ctypes.byref(params_py), ctypes.byref(outputs_py), ctypes.byref(info_py), internal_mem(0), None , _lib.TailoredSolver_adtool2forces)
            else:
                exitflag = _lib.TailoredSolver_solve( ctypes.byref(params_py), ctypes.byref(outputs_py), ctypes.byref(info_py), internal_mem(0), fp , _lib.TailoredSolver_adtool2forces)
            libc.fclose(fp)
            fptemp = open('stdout_temp.txt','r')
            print (fptemp.read())
            fptemp.close()            
        except:
            #print 'Problem with solver'
            raise

    # convert outputs
    for out in TailoredSolver_outputs:
        TailoredSolver_outputs[out] = npct.as_array(getattr(outputs_py,out))

    return TailoredSolver_outputs,int(exitflag),info_py

solve = TailoredSolver_solve


