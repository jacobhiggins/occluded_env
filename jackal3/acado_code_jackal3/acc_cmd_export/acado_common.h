/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4
#define ACADO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES

#include "acado_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 40
/** Number of online data values. */
#define ACADO_NOD 0
/** Number of path constraints. */
#define ACADO_NPAC 3
/** Number of control variables. */
#define ACADO_NU 3
/** Number of differential variables. */
#define ACADO_NX 14
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 7
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 4
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 120
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 5
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 2
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 2


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Matrix of size: 41 x 14 (row major format)
 * 
 *  Matrix containing 41 differential variable vectors.
 */
real_t x[ 574 ];

/** Matrix of size: 40 x 3 (row major format)
 * 
 *  Matrix containing 40 control variable vectors.
 */
real_t u[ 120 ];

/** Column vector of size: 280
 * 
 *  Matrix containing 40 reference/measurement vectors of size 7 for first 40 nodes.
 */
real_t y[ 280 ];

/** Column vector of size: 4
 * 
 *  Reference/measurement vector for the 41. node.
 */
real_t yN[ 4 ];

/** Matrix of size: 280 x 7 (row major format) */
real_t W[ 1960 ];

/** Matrix of size: 4 x 4 (row major format) */
real_t WN[ 16 ];

/** Column vector of size: 14
 * 
 *  Current state feedback vector.
 */
real_t x0[ 14 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim28_swap;

/** Column vector of size: 28 */
real_t rk_dim28_bPerm[ 28 ];

/** Column vector of size: 58 */
real_t rhs_aux[ 58 ];

real_t rk_ttt;

/** Row vector of size: 17 */
real_t rk_xxx[ 17 ];

/** Matrix of size: 14 x 2 (row major format) */
real_t rk_kkk[ 28 ];

/** Matrix of size: 28 x 28 (row major format) */
real_t rk_A[ 784 ];

/** Column vector of size: 28 */
real_t rk_b[ 28 ];

/** Row vector of size: 28 */
int rk_dim28_perm[ 28 ];

/** Column vector of size: 14 */
real_t rk_rhsTemp[ 14 ];

/** Matrix of size: 2 x 238 (row major format) */
real_t rk_diffsTemp2[ 476 ];

/** Matrix of size: 14 x 2 (row major format) */
real_t rk_diffK[ 28 ];

/** Matrix of size: 14 x 17 (row major format) */
real_t rk_diffsPrev2[ 238 ];

/** Matrix of size: 14 x 17 (row major format) */
real_t rk_diffsNew2[ 238 ];

/** Row vector of size: 255 */
real_t state[ 255 ];

/** Column vector of size: 560 */
real_t d[ 560 ];

/** Column vector of size: 280 */
real_t Dy[ 280 ];

/** Column vector of size: 4 */
real_t DyN[ 4 ];

/** Matrix of size: 560 x 14 (row major format) */
real_t evGx[ 7840 ];

/** Matrix of size: 560 x 3 (row major format) */
real_t evGu[ 1680 ];

/** Row vector of size: 17 */
real_t objValueIn[ 17 ];

/** Row vector of size: 7 */
real_t objValueOut[ 7 ];

/** Matrix of size: 560 x 14 (row major format) */
real_t Q1[ 7840 ];

/** Matrix of size: 560 x 7 (row major format) */
real_t Q2[ 3920 ];

/** Matrix of size: 120 x 3 (row major format) */
real_t R1[ 360 ];

/** Matrix of size: 120 x 7 (row major format) */
real_t R2[ 840 ];

/** Matrix of size: 14 x 14 (row major format) */
real_t QN1[ 196 ];

/** Matrix of size: 14 x 4 (row major format) */
real_t QN2[ 56 ];

/** Column vector of size: 52 */
real_t conAuxVar[ 52 ];

/** Row vector of size: 17 */
real_t conValueIn[ 17 ];

/** Row vector of size: 54 */
real_t conValueOut[ 54 ];

/** Column vector of size: 120 */
real_t evH[ 120 ];

/** Matrix of size: 120 x 14 (row major format) */
real_t evHx[ 1680 ];

/** Matrix of size: 120 x 3 (row major format) */
real_t evHu[ 360 ];

/** Column vector of size: 3 */
real_t evHxd[ 3 ];

/** Column vector of size: 14 */
real_t Dx0[ 14 ];

/** Matrix of size: 14 x 14 (row major format) */
real_t T[ 196 ];

/** Matrix of size: 11480 x 3 (row major format) */
real_t E[ 34440 ];

/** Matrix of size: 11480 x 3 (row major format) */
real_t QE[ 34440 ];

/** Column vector of size: 560 */
real_t Qd[ 560 ];

/** Column vector of size: 574 */
real_t QDy[ 574 ];

/** Matrix of size: 120 x 14 (row major format) */
real_t H10[ 1680 ];

/** Matrix of size: 120 x 14 (row major format) */
real_t A01[ 1680 ];

/** Column vector of size: 120 */
real_t pacA01Dx0[ 120 ];

/** Matrix of size: 120 x 120 (row major format) */
real_t H[ 14400 ];

/** Matrix of size: 200 x 120 (row major format) */
real_t A[ 24000 ];

/** Column vector of size: 120 */
real_t g[ 120 ];

/** Column vector of size: 120 */
real_t lb[ 120 ];

/** Column vector of size: 120 */
real_t ub[ 120 ];

/** Column vector of size: 200 */
real_t lbA[ 200 ];

/** Column vector of size: 200 */
real_t ubA[ 200 ];

/** Column vector of size: 120 */
real_t x[ 120 ];

/** Column vector of size: 320 */
real_t y[ 320 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array of size 17 to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_rhs(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_diffs(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 41 with xEnd. 2. Initialize node 41 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t acado_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t acado_getObjective(  );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
