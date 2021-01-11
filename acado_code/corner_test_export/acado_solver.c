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


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[27] = acadoVariables.u[lRun1 * 5];
acadoWorkspace.state[28] = acadoVariables.u[lRun1 * 5 + 1];
acadoWorkspace.state[29] = acadoVariables.u[lRun1 * 5 + 2];
acadoWorkspace.state[30] = acadoVariables.u[lRun1 * 5 + 3];
acadoWorkspace.state[31] = acadoVariables.u[lRun1 * 5 + 4];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 15] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 15 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 15 + 2] = acadoWorkspace.state[14];
acadoWorkspace.evGu[lRun1 * 15 + 3] = acadoWorkspace.state[15];
acadoWorkspace.evGu[lRun1 * 15 + 4] = acadoWorkspace.state[16];
acadoWorkspace.evGu[lRun1 * 15 + 5] = acadoWorkspace.state[17];
acadoWorkspace.evGu[lRun1 * 15 + 6] = acadoWorkspace.state[18];
acadoWorkspace.evGu[lRun1 * 15 + 7] = acadoWorkspace.state[19];
acadoWorkspace.evGu[lRun1 * 15 + 8] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 15 + 9] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 15 + 10] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 15 + 11] = acadoWorkspace.state[23];
acadoWorkspace.evGu[lRun1 * 15 + 12] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 15 + 13] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 15 + 14] = acadoWorkspace.state[26];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = u[0];
out[3] = u[1];
out[4] = u[2];
out[5] = u[3];
out[6] = u[4];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = 0.0;
;
tmpQ2[15] = 0.0;
;
tmpQ2[16] = 0.0;
;
tmpQ2[17] = 0.0;
;
tmpQ2[18] = 0.0;
;
tmpQ2[19] = 0.0;
;
tmpQ2[20] = 0.0;
;
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = 0.0;
;
tmpQ1[3] = + tmpQ2[7];
tmpQ1[4] = + tmpQ2[8];
tmpQ1[5] = 0.0;
;
tmpQ1[6] = + tmpQ2[14];
tmpQ1[7] = + tmpQ2[15];
tmpQ1[8] = 0.0;
;
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[14];
tmpR2[1] = +tmpObjS[15];
tmpR2[2] = +tmpObjS[16];
tmpR2[3] = +tmpObjS[17];
tmpR2[4] = +tmpObjS[18];
tmpR2[5] = +tmpObjS[19];
tmpR2[6] = +tmpObjS[20];
tmpR2[7] = +tmpObjS[21];
tmpR2[8] = +tmpObjS[22];
tmpR2[9] = +tmpObjS[23];
tmpR2[10] = +tmpObjS[24];
tmpR2[11] = +tmpObjS[25];
tmpR2[12] = +tmpObjS[26];
tmpR2[13] = +tmpObjS[27];
tmpR2[14] = +tmpObjS[28];
tmpR2[15] = +tmpObjS[29];
tmpR2[16] = +tmpObjS[30];
tmpR2[17] = +tmpObjS[31];
tmpR2[18] = +tmpObjS[32];
tmpR2[19] = +tmpObjS[33];
tmpR2[20] = +tmpObjS[34];
tmpR2[21] = +tmpObjS[35];
tmpR2[22] = +tmpObjS[36];
tmpR2[23] = +tmpObjS[37];
tmpR2[24] = +tmpObjS[38];
tmpR2[25] = +tmpObjS[39];
tmpR2[26] = +tmpObjS[40];
tmpR2[27] = +tmpObjS[41];
tmpR2[28] = +tmpObjS[42];
tmpR2[29] = +tmpObjS[43];
tmpR2[30] = +tmpObjS[44];
tmpR2[31] = +tmpObjS[45];
tmpR2[32] = +tmpObjS[46];
tmpR2[33] = +tmpObjS[47];
tmpR2[34] = +tmpObjS[48];
tmpR1[0] = + tmpR2[2];
tmpR1[1] = + tmpR2[3];
tmpR1[2] = + tmpR2[4];
tmpR1[3] = + tmpR2[5];
tmpR1[4] = + tmpR2[6];
tmpR1[5] = + tmpR2[9];
tmpR1[6] = + tmpR2[10];
tmpR1[7] = + tmpR2[11];
tmpR1[8] = + tmpR2[12];
tmpR1[9] = + tmpR2[13];
tmpR1[10] = + tmpR2[16];
tmpR1[11] = + tmpR2[17];
tmpR1[12] = + tmpR2[18];
tmpR1[13] = + tmpR2[19];
tmpR1[14] = + tmpR2[20];
tmpR1[15] = + tmpR2[23];
tmpR1[16] = + tmpR2[24];
tmpR1[17] = + tmpR2[25];
tmpR1[18] = + tmpR2[26];
tmpR1[19] = + tmpR2[27];
tmpR1[20] = + tmpR2[30];
tmpR1[21] = + tmpR2[31];
tmpR1[22] = + tmpR2[32];
tmpR1[23] = + tmpR2[33];
tmpR1[24] = + tmpR2[34];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = 0.0;
;
tmpQN2[5] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = 0.0;
;
tmpQN1[3] = + tmpQN2[2];
tmpQN1[4] = + tmpQN2[3];
tmpQN1[5] = 0.0;
;
tmpQN1[6] = + tmpQN2[4];
tmpQN1[7] = + tmpQN2[5];
tmpQN1[8] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 50; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 5];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 5 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 5 + 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 5 + 3];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 5 + 4];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 7] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 7 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 7 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 7 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 7 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 7 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 7 + 6] = acadoWorkspace.objValueOut[6];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 21 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.R1[ runObj * 25 ]), &(acadoWorkspace.R2[ runObj * 35 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] += + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[3] + Gx1[2]*Gx2[6];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[7];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[8];
Gx3[3] = + Gx1[3]*Gx2[0] + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[6];
Gx3[4] = + Gx1[3]*Gx2[1] + Gx1[4]*Gx2[4] + Gx1[5]*Gx2[7];
Gx3[5] = + Gx1[3]*Gx2[2] + Gx1[4]*Gx2[5] + Gx1[5]*Gx2[8];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[6];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[7];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[8];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[10];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[11];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[12];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[8] + Gx1[2]*Gu1[13];
Gu2[4] = + Gx1[0]*Gu1[4] + Gx1[1]*Gu1[9] + Gx1[2]*Gu1[14];
Gu2[5] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[5] + Gx1[5]*Gu1[10];
Gu2[6] = + Gx1[3]*Gu1[1] + Gx1[4]*Gu1[6] + Gx1[5]*Gu1[11];
Gu2[7] = + Gx1[3]*Gu1[2] + Gx1[4]*Gu1[7] + Gx1[5]*Gu1[12];
Gu2[8] = + Gx1[3]*Gu1[3] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[13];
Gu2[9] = + Gx1[3]*Gu1[4] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[14];
Gu2[10] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[10];
Gu2[11] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[6] + Gx1[8]*Gu1[11];
Gu2[12] = + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[7] + Gx1[8]*Gu1[12];
Gu2[13] = + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[8] + Gx1[8]*Gu1[13];
Gu2[14] = + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[9] + Gx1[8]*Gu1[14];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 1250) + (iCol * 5)] += + Gu1[0]*Gu2[0] + Gu1[5]*Gu2[5] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 1)] += + Gu1[0]*Gu2[1] + Gu1[5]*Gu2[6] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 2)] += + Gu1[0]*Gu2[2] + Gu1[5]*Gu2[7] + Gu1[10]*Gu2[12];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 3)] += + Gu1[0]*Gu2[3] + Gu1[5]*Gu2[8] + Gu1[10]*Gu2[13];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 4)] += + Gu1[0]*Gu2[4] + Gu1[5]*Gu2[9] + Gu1[10]*Gu2[14];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5)] += + Gu1[1]*Gu2[0] + Gu1[6]*Gu2[5] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 1)] += + Gu1[1]*Gu2[1] + Gu1[6]*Gu2[6] + Gu1[11]*Gu2[11];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 2)] += + Gu1[1]*Gu2[2] + Gu1[6]*Gu2[7] + Gu1[11]*Gu2[12];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 3)] += + Gu1[1]*Gu2[3] + Gu1[6]*Gu2[8] + Gu1[11]*Gu2[13];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 4)] += + Gu1[1]*Gu2[4] + Gu1[6]*Gu2[9] + Gu1[11]*Gu2[14];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5)] += + Gu1[2]*Gu2[0] + Gu1[7]*Gu2[5] + Gu1[12]*Gu2[10];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 1)] += + Gu1[2]*Gu2[1] + Gu1[7]*Gu2[6] + Gu1[12]*Gu2[11];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 2)] += + Gu1[2]*Gu2[2] + Gu1[7]*Gu2[7] + Gu1[12]*Gu2[12];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 3)] += + Gu1[2]*Gu2[3] + Gu1[7]*Gu2[8] + Gu1[12]*Gu2[13];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 4)] += + Gu1[2]*Gu2[4] + Gu1[7]*Gu2[9] + Gu1[12]*Gu2[14];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5)] += + Gu1[3]*Gu2[0] + Gu1[8]*Gu2[5] + Gu1[13]*Gu2[10];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 1)] += + Gu1[3]*Gu2[1] + Gu1[8]*Gu2[6] + Gu1[13]*Gu2[11];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 2)] += + Gu1[3]*Gu2[2] + Gu1[8]*Gu2[7] + Gu1[13]*Gu2[12];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 3)] += + Gu1[3]*Gu2[3] + Gu1[8]*Gu2[8] + Gu1[13]*Gu2[13];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 4)] += + Gu1[3]*Gu2[4] + Gu1[8]*Gu2[9] + Gu1[13]*Gu2[14];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5)] += + Gu1[4]*Gu2[0] + Gu1[9]*Gu2[5] + Gu1[14]*Gu2[10];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 1)] += + Gu1[4]*Gu2[1] + Gu1[9]*Gu2[6] + Gu1[14]*Gu2[11];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 2)] += + Gu1[4]*Gu2[2] + Gu1[9]*Gu2[7] + Gu1[14]*Gu2[12];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 3)] += + Gu1[4]*Gu2[3] + Gu1[9]*Gu2[8] + Gu1[14]*Gu2[13];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 4)] += + Gu1[4]*Gu2[4] + Gu1[9]*Gu2[9] + Gu1[14]*Gu2[14];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 1250) + (iCol * 5)] = R11[0];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 3)] = R11[3];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 4)] = R11[4];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5)] = R11[5];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 1)] = R11[6];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 2)] = R11[7];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 3)] = R11[8];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 4)] = R11[9];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5)] = R11[10];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 1)] = R11[11];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 2)] = R11[12];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 3)] = R11[13];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 4)] = R11[14];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5)] = R11[15];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 1)] = R11[16];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 2)] = R11[17];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 3)] = R11[18];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 4)] = R11[19];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5)] = R11[20];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 1)] = R11[21];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 2)] = R11[22];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 3)] = R11[23];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 4)] = R11[24];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 1250) + (iCol * 5)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 4)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 1250) + (iCol * 5)] = acadoWorkspace.H[(iCol * 1250) + (iRow * 5)];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 1)] = acadoWorkspace.H[(iCol * 1250 + 250) + (iRow * 5)];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 2)] = acadoWorkspace.H[(iCol * 1250 + 500) + (iRow * 5)];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 3)] = acadoWorkspace.H[(iCol * 1250 + 750) + (iRow * 5)];
acadoWorkspace.H[(iRow * 1250) + (iCol * 5 + 4)] = acadoWorkspace.H[(iCol * 1250 + 1000) + (iRow * 5)];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5)] = acadoWorkspace.H[(iCol * 1250) + (iRow * 5 + 1)];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 1)] = acadoWorkspace.H[(iCol * 1250 + 250) + (iRow * 5 + 1)];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 2)] = acadoWorkspace.H[(iCol * 1250 + 500) + (iRow * 5 + 1)];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 3)] = acadoWorkspace.H[(iCol * 1250 + 750) + (iRow * 5 + 1)];
acadoWorkspace.H[(iRow * 1250 + 250) + (iCol * 5 + 4)] = acadoWorkspace.H[(iCol * 1250 + 1000) + (iRow * 5 + 1)];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5)] = acadoWorkspace.H[(iCol * 1250) + (iRow * 5 + 2)];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 1)] = acadoWorkspace.H[(iCol * 1250 + 250) + (iRow * 5 + 2)];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 2)] = acadoWorkspace.H[(iCol * 1250 + 500) + (iRow * 5 + 2)];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 3)] = acadoWorkspace.H[(iCol * 1250 + 750) + (iRow * 5 + 2)];
acadoWorkspace.H[(iRow * 1250 + 500) + (iCol * 5 + 4)] = acadoWorkspace.H[(iCol * 1250 + 1000) + (iRow * 5 + 2)];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5)] = acadoWorkspace.H[(iCol * 1250) + (iRow * 5 + 3)];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 1)] = acadoWorkspace.H[(iCol * 1250 + 250) + (iRow * 5 + 3)];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 2)] = acadoWorkspace.H[(iCol * 1250 + 500) + (iRow * 5 + 3)];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 3)] = acadoWorkspace.H[(iCol * 1250 + 750) + (iRow * 5 + 3)];
acadoWorkspace.H[(iRow * 1250 + 750) + (iCol * 5 + 4)] = acadoWorkspace.H[(iCol * 1250 + 1000) + (iRow * 5 + 3)];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5)] = acadoWorkspace.H[(iCol * 1250) + (iRow * 5 + 4)];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 1)] = acadoWorkspace.H[(iCol * 1250 + 250) + (iRow * 5 + 4)];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 2)] = acadoWorkspace.H[(iCol * 1250 + 500) + (iRow * 5 + 4)];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 3)] = acadoWorkspace.H[(iCol * 1250 + 750) + (iRow * 5 + 4)];
acadoWorkspace.H[(iRow * 1250 + 1000) + (iCol * 5 + 4)] = acadoWorkspace.H[(iCol * 1250 + 1000) + (iRow * 5 + 4)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] = + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] = + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2];
dNew[1] = + acadoWorkspace.QN1[3]*dOld[0] + acadoWorkspace.QN1[4]*dOld[1] + acadoWorkspace.QN1[5]*dOld[2];
dNew[2] = + acadoWorkspace.QN1[6]*dOld[0] + acadoWorkspace.QN1[7]*dOld[1] + acadoWorkspace.QN1[8]*dOld[2];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6];
RDy1[1] = + R2[7]*Dy1[0] + R2[8]*Dy1[1] + R2[9]*Dy1[2] + R2[10]*Dy1[3] + R2[11]*Dy1[4] + R2[12]*Dy1[5] + R2[13]*Dy1[6];
RDy1[2] = + R2[14]*Dy1[0] + R2[15]*Dy1[1] + R2[16]*Dy1[2] + R2[17]*Dy1[3] + R2[18]*Dy1[4] + R2[19]*Dy1[5] + R2[20]*Dy1[6];
RDy1[3] = + R2[21]*Dy1[0] + R2[22]*Dy1[1] + R2[23]*Dy1[2] + R2[24]*Dy1[3] + R2[25]*Dy1[4] + R2[26]*Dy1[5] + R2[27]*Dy1[6];
RDy1[4] = + R2[28]*Dy1[0] + R2[29]*Dy1[1] + R2[30]*Dy1[2] + R2[31]*Dy1[3] + R2[32]*Dy1[4] + R2[33]*Dy1[5] + R2[34]*Dy1[6];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6];
QDy1[1] = + Q2[7]*Dy1[0] + Q2[8]*Dy1[1] + Q2[9]*Dy1[2] + Q2[10]*Dy1[3] + Q2[11]*Dy1[4] + Q2[12]*Dy1[5] + Q2[13]*Dy1[6];
QDy1[2] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[5]*QDy1[1] + E1[10]*QDy1[2];
U1[1] += + E1[1]*QDy1[0] + E1[6]*QDy1[1] + E1[11]*QDy1[2];
U1[2] += + E1[2]*QDy1[0] + E1[7]*QDy1[1] + E1[12]*QDy1[2];
U1[3] += + E1[3]*QDy1[0] + E1[8]*QDy1[1] + E1[13]*QDy1[2];
U1[4] += + E1[4]*QDy1[0] + E1[9]*QDy1[1] + E1[14]*QDy1[2];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[5]*Gx1[3] + E1[10]*Gx1[6];
H101[1] += + E1[0]*Gx1[1] + E1[5]*Gx1[4] + E1[10]*Gx1[7];
H101[2] += + E1[0]*Gx1[2] + E1[5]*Gx1[5] + E1[10]*Gx1[8];
H101[3] += + E1[1]*Gx1[0] + E1[6]*Gx1[3] + E1[11]*Gx1[6];
H101[4] += + E1[1]*Gx1[1] + E1[6]*Gx1[4] + E1[11]*Gx1[7];
H101[5] += + E1[1]*Gx1[2] + E1[6]*Gx1[5] + E1[11]*Gx1[8];
H101[6] += + E1[2]*Gx1[0] + E1[7]*Gx1[3] + E1[12]*Gx1[6];
H101[7] += + E1[2]*Gx1[1] + E1[7]*Gx1[4] + E1[12]*Gx1[7];
H101[8] += + E1[2]*Gx1[2] + E1[7]*Gx1[5] + E1[12]*Gx1[8];
H101[9] += + E1[3]*Gx1[0] + E1[8]*Gx1[3] + E1[13]*Gx1[6];
H101[10] += + E1[3]*Gx1[1] + E1[8]*Gx1[4] + E1[13]*Gx1[7];
H101[11] += + E1[3]*Gx1[2] + E1[8]*Gx1[5] + E1[13]*Gx1[8];
H101[12] += + E1[4]*Gx1[0] + E1[9]*Gx1[3] + E1[14]*Gx1[6];
H101[13] += + E1[4]*Gx1[1] + E1[9]*Gx1[4] + E1[14]*Gx1[7];
H101[14] += + E1[4]*Gx1[2] + E1[9]*Gx1[5] + E1[14]*Gx1[8];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 15; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3] + E1[4]*U1[4];
dNew[1] += + E1[5]*U1[0] + E1[6]*U1[1] + E1[7]*U1[2] + E1[8]*U1[3] + E1[9]*U1[4];
dNew[2] += + E1[10]*U1[0] + E1[11]*U1[1] + E1[12]*U1[2] + E1[13]*U1[3] + E1[14]*U1[4];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[3] + Hx[2]*Gx[6];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[4] + Hx[2]*Gx[7];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[5] + Hx[2]*Gx[8];
A01[3] = + Hx[3]*Gx[0] + Hx[4]*Gx[3] + Hx[5]*Gx[6];
A01[4] = + Hx[3]*Gx[1] + Hx[4]*Gx[4] + Hx[5]*Gx[7];
A01[5] = + Hx[3]*Gx[2] + Hx[4]*Gx[5] + Hx[5]*Gx[8];
A01[6] = + Hx[6]*Gx[0] + Hx[7]*Gx[3] + Hx[8]*Gx[6];
A01[7] = + Hx[6]*Gx[1] + Hx[7]*Gx[4] + Hx[8]*Gx[7];
A01[8] = + Hx[6]*Gx[2] + Hx[7]*Gx[5] + Hx[8]*Gx[8];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 750) + (col * 5)] = + Hx[0]*E[0] + Hx[1]*E[5] + Hx[2]*E[10];
acadoWorkspace.A[(row * 750) + (col * 5 + 1)] = + Hx[0]*E[1] + Hx[1]*E[6] + Hx[2]*E[11];
acadoWorkspace.A[(row * 750) + (col * 5 + 2)] = + Hx[0]*E[2] + Hx[1]*E[7] + Hx[2]*E[12];
acadoWorkspace.A[(row * 750) + (col * 5 + 3)] = + Hx[0]*E[3] + Hx[1]*E[8] + Hx[2]*E[13];
acadoWorkspace.A[(row * 750) + (col * 5 + 4)] = + Hx[0]*E[4] + Hx[1]*E[9] + Hx[2]*E[14];
acadoWorkspace.A[(row * 750 + 250) + (col * 5)] = + Hx[3]*E[0] + Hx[4]*E[5] + Hx[5]*E[10];
acadoWorkspace.A[(row * 750 + 250) + (col * 5 + 1)] = + Hx[3]*E[1] + Hx[4]*E[6] + Hx[5]*E[11];
acadoWorkspace.A[(row * 750 + 250) + (col * 5 + 2)] = + Hx[3]*E[2] + Hx[4]*E[7] + Hx[5]*E[12];
acadoWorkspace.A[(row * 750 + 250) + (col * 5 + 3)] = + Hx[3]*E[3] + Hx[4]*E[8] + Hx[5]*E[13];
acadoWorkspace.A[(row * 750 + 250) + (col * 5 + 4)] = + Hx[3]*E[4] + Hx[4]*E[9] + Hx[5]*E[14];
acadoWorkspace.A[(row * 750 + 500) + (col * 5)] = + Hx[6]*E[0] + Hx[7]*E[5] + Hx[8]*E[10];
acadoWorkspace.A[(row * 750 + 500) + (col * 5 + 1)] = + Hx[6]*E[1] + Hx[7]*E[6] + Hx[8]*E[11];
acadoWorkspace.A[(row * 750 + 500) + (col * 5 + 2)] = + Hx[6]*E[2] + Hx[7]*E[7] + Hx[8]*E[12];
acadoWorkspace.A[(row * 750 + 500) + (col * 5 + 3)] = + Hx[6]*E[3] + Hx[7]*E[8] + Hx[8]*E[13];
acadoWorkspace.A[(row * 750 + 500) + (col * 5 + 4)] = + Hx[6]*E[4] + Hx[7]*E[9] + Hx[8]*E[14];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2];
acadoWorkspace.evHxd[1] = + Hx[3]*tmpd[0] + Hx[4]*tmpd[1] + Hx[5]*tmpd[2];
acadoWorkspace.evHxd[2] = + Hx[6]*tmpd[0] + Hx[7]*tmpd[1] + Hx[8]*tmpd[2];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
/* Vector of auxiliary variables; number of elements: 24. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(1.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(-1.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(-1.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(-1.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = u[3];
a[22] = u[2];
a[23] = (real_t)(-1.0000000000000000e+00);

/* Compute outputs: */
out[0] = (xd[0]-u[2]);
out[1] = (((real_t)(0.0000000000000000e+00)-xd[1])-u[3]);
out[2] = ((u[2]*u[3])-u[4]);
out[3] = a[0];
out[4] = a[1];
out[5] = a[2];
out[6] = a[3];
out[7] = a[4];
out[8] = a[5];
out[9] = a[6];
out[10] = a[7];
out[11] = a[8];
out[12] = a[9];
out[13] = a[10];
out[14] = a[11];
out[15] = a[12];
out[16] = a[13];
out[17] = a[14];
out[18] = a[15];
out[19] = a[16];
out[20] = a[17];
out[21] = a[18];
out[22] = a[19];
out[23] = a[20];
out[24] = a[21];
out[25] = a[22];
out[26] = a[23];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
g1[4] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 50; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 3-3 ]), &(acadoWorkspace.evGx[ lRun1 * 9 ]), &(acadoWorkspace.d[ lRun1 * 3 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 9-9 ]), &(acadoWorkspace.evGx[ lRun1 * 9 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 15 ]), &(acadoWorkspace.E[ lRun3 * 15 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 15 ]), &(acadoWorkspace.E[ lRun3 * 15 ]) );
}

for (lRun1 = 0; lRun1 < 49; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 9 + 9 ]), &(acadoWorkspace.E[ lRun3 * 15 ]), &(acadoWorkspace.QE[ lRun3 * 15 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 15 ]), &(acadoWorkspace.QE[ lRun3 * 15 ]) );
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 15 ]) );
for (lRun2 = lRun1; lRun2 < 50; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 15 ]), &(acadoWorkspace.evGx[ lRun2 * 9 ]), &(acadoWorkspace.H10[ lRun1 * 15 ]) );
}
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 25 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 50; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 15 ]), &(acadoWorkspace.QE[ lRun5 * 15 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 50; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 50; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 15 ]), &(acadoWorkspace.QE[ lRun5 * 15 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 9 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.d[ 3 ]), &(acadoWorkspace.Qd[ 3 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.Qd[ 6 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.Qd[ 9 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.Qd[ 21 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.Qd[ 27 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.d[ 33 ]), &(acadoWorkspace.Qd[ 33 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.d[ 39 ]), &(acadoWorkspace.Qd[ 39 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.Qd[ 42 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.d[ 51 ]), &(acadoWorkspace.Qd[ 51 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.Qd[ 54 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.d[ 57 ]), &(acadoWorkspace.Qd[ 57 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.Qd[ 63 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.Qd[ 66 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.d[ 69 ]), &(acadoWorkspace.Qd[ 69 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 234 ]), &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.Qd[ 75 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.d[ 78 ]), &(acadoWorkspace.Qd[ 78 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.d[ 81 ]), &(acadoWorkspace.Qd[ 81 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 261 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 270 ]), &(acadoWorkspace.d[ 87 ]), &(acadoWorkspace.Qd[ 87 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 279 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 93 ]), &(acadoWorkspace.Qd[ 93 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 297 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 306 ]), &(acadoWorkspace.d[ 99 ]), &(acadoWorkspace.Qd[ 99 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 315 ]), &(acadoWorkspace.d[ 102 ]), &(acadoWorkspace.Qd[ 102 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.Qd[ 105 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 333 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.Qd[ 108 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 342 ]), &(acadoWorkspace.d[ 111 ]), &(acadoWorkspace.Qd[ 111 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 351 ]), &(acadoWorkspace.d[ 114 ]), &(acadoWorkspace.Qd[ 114 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.d[ 117 ]), &(acadoWorkspace.Qd[ 117 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 369 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 378 ]), &(acadoWorkspace.d[ 123 ]), &(acadoWorkspace.Qd[ 123 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 387 ]), &(acadoWorkspace.d[ 126 ]), &(acadoWorkspace.Qd[ 126 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.d[ 129 ]), &(acadoWorkspace.Qd[ 129 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.d[ 132 ]), &(acadoWorkspace.Qd[ 132 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 414 ]), &(acadoWorkspace.d[ 135 ]), &(acadoWorkspace.Qd[ 135 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 423 ]), &(acadoWorkspace.d[ 138 ]), &(acadoWorkspace.Qd[ 138 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.d[ 141 ]), &(acadoWorkspace.Qd[ 141 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.d[ 144 ]), &(acadoWorkspace.Qd[ 144 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 147 ]), &(acadoWorkspace.Qd[ 147 ]) );

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 50; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 15 ]), &(acadoWorkspace.g[ lRun1 * 5 ]) );
}
}
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.lb[2] = - acadoVariables.u[2];
acadoWorkspace.lb[3] = - acadoVariables.u[3];
acadoWorkspace.lb[4] = - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = - acadoVariables.u[7];
acadoWorkspace.lb[8] = - acadoVariables.u[8];
acadoWorkspace.lb[9] = - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.lb[12] = - acadoVariables.u[12];
acadoWorkspace.lb[13] = - acadoVariables.u[13];
acadoWorkspace.lb[14] = - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.lb[17] = - acadoVariables.u[17];
acadoWorkspace.lb[18] = - acadoVariables.u[18];
acadoWorkspace.lb[19] = - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[21];
acadoWorkspace.lb[22] = - acadoVariables.u[22];
acadoWorkspace.lb[23] = - acadoVariables.u[23];
acadoWorkspace.lb[24] = - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.lb[27] = - acadoVariables.u[27];
acadoWorkspace.lb[28] = - acadoVariables.u[28];
acadoWorkspace.lb[29] = - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[31];
acadoWorkspace.lb[32] = - acadoVariables.u[32];
acadoWorkspace.lb[33] = - acadoVariables.u[33];
acadoWorkspace.lb[34] = - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[36];
acadoWorkspace.lb[37] = - acadoVariables.u[37];
acadoWorkspace.lb[38] = - acadoVariables.u[38];
acadoWorkspace.lb[39] = - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[41];
acadoWorkspace.lb[42] = - acadoVariables.u[42];
acadoWorkspace.lb[43] = - acadoVariables.u[43];
acadoWorkspace.lb[44] = - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.lb[47] = - acadoVariables.u[47];
acadoWorkspace.lb[48] = - acadoVariables.u[48];
acadoWorkspace.lb[49] = - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[51];
acadoWorkspace.lb[52] = - acadoVariables.u[52];
acadoWorkspace.lb[53] = - acadoVariables.u[53];
acadoWorkspace.lb[54] = - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[56];
acadoWorkspace.lb[57] = - acadoVariables.u[57];
acadoWorkspace.lb[58] = - acadoVariables.u[58];
acadoWorkspace.lb[59] = - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[61];
acadoWorkspace.lb[62] = - acadoVariables.u[62];
acadoWorkspace.lb[63] = - acadoVariables.u[63];
acadoWorkspace.lb[64] = - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[66];
acadoWorkspace.lb[67] = - acadoVariables.u[67];
acadoWorkspace.lb[68] = - acadoVariables.u[68];
acadoWorkspace.lb[69] = - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[71];
acadoWorkspace.lb[72] = - acadoVariables.u[72];
acadoWorkspace.lb[73] = - acadoVariables.u[73];
acadoWorkspace.lb[74] = - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[76];
acadoWorkspace.lb[77] = - acadoVariables.u[77];
acadoWorkspace.lb[78] = - acadoVariables.u[78];
acadoWorkspace.lb[79] = - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[81];
acadoWorkspace.lb[82] = - acadoVariables.u[82];
acadoWorkspace.lb[83] = - acadoVariables.u[83];
acadoWorkspace.lb[84] = - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[86];
acadoWorkspace.lb[87] = - acadoVariables.u[87];
acadoWorkspace.lb[88] = - acadoVariables.u[88];
acadoWorkspace.lb[89] = - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[91];
acadoWorkspace.lb[92] = - acadoVariables.u[92];
acadoWorkspace.lb[93] = - acadoVariables.u[93];
acadoWorkspace.lb[94] = - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[96];
acadoWorkspace.lb[97] = - acadoVariables.u[97];
acadoWorkspace.lb[98] = - acadoVariables.u[98];
acadoWorkspace.lb[99] = - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[101];
acadoWorkspace.lb[102] = - acadoVariables.u[102];
acadoWorkspace.lb[103] = - acadoVariables.u[103];
acadoWorkspace.lb[104] = - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[106];
acadoWorkspace.lb[107] = - acadoVariables.u[107];
acadoWorkspace.lb[108] = - acadoVariables.u[108];
acadoWorkspace.lb[109] = - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[111];
acadoWorkspace.lb[112] = - acadoVariables.u[112];
acadoWorkspace.lb[113] = - acadoVariables.u[113];
acadoWorkspace.lb[114] = - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[116];
acadoWorkspace.lb[117] = - acadoVariables.u[117];
acadoWorkspace.lb[118] = - acadoVariables.u[118];
acadoWorkspace.lb[119] = - acadoVariables.u[119];
acadoWorkspace.lb[120] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[120];
acadoWorkspace.lb[121] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[121];
acadoWorkspace.lb[122] = - acadoVariables.u[122];
acadoWorkspace.lb[123] = - acadoVariables.u[123];
acadoWorkspace.lb[124] = - acadoVariables.u[124];
acadoWorkspace.lb[125] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[125];
acadoWorkspace.lb[126] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[126];
acadoWorkspace.lb[127] = - acadoVariables.u[127];
acadoWorkspace.lb[128] = - acadoVariables.u[128];
acadoWorkspace.lb[129] = - acadoVariables.u[129];
acadoWorkspace.lb[130] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[130];
acadoWorkspace.lb[131] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[131];
acadoWorkspace.lb[132] = - acadoVariables.u[132];
acadoWorkspace.lb[133] = - acadoVariables.u[133];
acadoWorkspace.lb[134] = - acadoVariables.u[134];
acadoWorkspace.lb[135] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[135];
acadoWorkspace.lb[136] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[136];
acadoWorkspace.lb[137] = - acadoVariables.u[137];
acadoWorkspace.lb[138] = - acadoVariables.u[138];
acadoWorkspace.lb[139] = - acadoVariables.u[139];
acadoWorkspace.lb[140] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[140];
acadoWorkspace.lb[141] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[141];
acadoWorkspace.lb[142] = - acadoVariables.u[142];
acadoWorkspace.lb[143] = - acadoVariables.u[143];
acadoWorkspace.lb[144] = - acadoVariables.u[144];
acadoWorkspace.lb[145] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[145];
acadoWorkspace.lb[146] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[146];
acadoWorkspace.lb[147] = - acadoVariables.u[147];
acadoWorkspace.lb[148] = - acadoVariables.u[148];
acadoWorkspace.lb[149] = - acadoVariables.u[149];
acadoWorkspace.lb[150] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[150];
acadoWorkspace.lb[151] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[151];
acadoWorkspace.lb[152] = - acadoVariables.u[152];
acadoWorkspace.lb[153] = - acadoVariables.u[153];
acadoWorkspace.lb[154] = - acadoVariables.u[154];
acadoWorkspace.lb[155] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[155];
acadoWorkspace.lb[156] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[156];
acadoWorkspace.lb[157] = - acadoVariables.u[157];
acadoWorkspace.lb[158] = - acadoVariables.u[158];
acadoWorkspace.lb[159] = - acadoVariables.u[159];
acadoWorkspace.lb[160] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[160];
acadoWorkspace.lb[161] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[161];
acadoWorkspace.lb[162] = - acadoVariables.u[162];
acadoWorkspace.lb[163] = - acadoVariables.u[163];
acadoWorkspace.lb[164] = - acadoVariables.u[164];
acadoWorkspace.lb[165] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[165];
acadoWorkspace.lb[166] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[166];
acadoWorkspace.lb[167] = - acadoVariables.u[167];
acadoWorkspace.lb[168] = - acadoVariables.u[168];
acadoWorkspace.lb[169] = - acadoVariables.u[169];
acadoWorkspace.lb[170] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[170];
acadoWorkspace.lb[171] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[171];
acadoWorkspace.lb[172] = - acadoVariables.u[172];
acadoWorkspace.lb[173] = - acadoVariables.u[173];
acadoWorkspace.lb[174] = - acadoVariables.u[174];
acadoWorkspace.lb[175] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[175];
acadoWorkspace.lb[176] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[176];
acadoWorkspace.lb[177] = - acadoVariables.u[177];
acadoWorkspace.lb[178] = - acadoVariables.u[178];
acadoWorkspace.lb[179] = - acadoVariables.u[179];
acadoWorkspace.lb[180] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[180];
acadoWorkspace.lb[181] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[181];
acadoWorkspace.lb[182] = - acadoVariables.u[182];
acadoWorkspace.lb[183] = - acadoVariables.u[183];
acadoWorkspace.lb[184] = - acadoVariables.u[184];
acadoWorkspace.lb[185] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[185];
acadoWorkspace.lb[186] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[186];
acadoWorkspace.lb[187] = - acadoVariables.u[187];
acadoWorkspace.lb[188] = - acadoVariables.u[188];
acadoWorkspace.lb[189] = - acadoVariables.u[189];
acadoWorkspace.lb[190] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[190];
acadoWorkspace.lb[191] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[191];
acadoWorkspace.lb[192] = - acadoVariables.u[192];
acadoWorkspace.lb[193] = - acadoVariables.u[193];
acadoWorkspace.lb[194] = - acadoVariables.u[194];
acadoWorkspace.lb[195] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[195];
acadoWorkspace.lb[196] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[196];
acadoWorkspace.lb[197] = - acadoVariables.u[197];
acadoWorkspace.lb[198] = - acadoVariables.u[198];
acadoWorkspace.lb[199] = - acadoVariables.u[199];
acadoWorkspace.lb[200] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[200];
acadoWorkspace.lb[201] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[201];
acadoWorkspace.lb[202] = - acadoVariables.u[202];
acadoWorkspace.lb[203] = - acadoVariables.u[203];
acadoWorkspace.lb[204] = - acadoVariables.u[204];
acadoWorkspace.lb[205] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[205];
acadoWorkspace.lb[206] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[206];
acadoWorkspace.lb[207] = - acadoVariables.u[207];
acadoWorkspace.lb[208] = - acadoVariables.u[208];
acadoWorkspace.lb[209] = - acadoVariables.u[209];
acadoWorkspace.lb[210] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[210];
acadoWorkspace.lb[211] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[211];
acadoWorkspace.lb[212] = - acadoVariables.u[212];
acadoWorkspace.lb[213] = - acadoVariables.u[213];
acadoWorkspace.lb[214] = - acadoVariables.u[214];
acadoWorkspace.lb[215] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[215];
acadoWorkspace.lb[216] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[216];
acadoWorkspace.lb[217] = - acadoVariables.u[217];
acadoWorkspace.lb[218] = - acadoVariables.u[218];
acadoWorkspace.lb[219] = - acadoVariables.u[219];
acadoWorkspace.lb[220] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[220];
acadoWorkspace.lb[221] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[221];
acadoWorkspace.lb[222] = - acadoVariables.u[222];
acadoWorkspace.lb[223] = - acadoVariables.u[223];
acadoWorkspace.lb[224] = - acadoVariables.u[224];
acadoWorkspace.lb[225] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[225];
acadoWorkspace.lb[226] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[226];
acadoWorkspace.lb[227] = - acadoVariables.u[227];
acadoWorkspace.lb[228] = - acadoVariables.u[228];
acadoWorkspace.lb[229] = - acadoVariables.u[229];
acadoWorkspace.lb[230] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[230];
acadoWorkspace.lb[231] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[231];
acadoWorkspace.lb[232] = - acadoVariables.u[232];
acadoWorkspace.lb[233] = - acadoVariables.u[233];
acadoWorkspace.lb[234] = - acadoVariables.u[234];
acadoWorkspace.lb[235] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[235];
acadoWorkspace.lb[236] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[236];
acadoWorkspace.lb[237] = - acadoVariables.u[237];
acadoWorkspace.lb[238] = - acadoVariables.u[238];
acadoWorkspace.lb[239] = - acadoVariables.u[239];
acadoWorkspace.lb[240] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[240];
acadoWorkspace.lb[241] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[241];
acadoWorkspace.lb[242] = - acadoVariables.u[242];
acadoWorkspace.lb[243] = - acadoVariables.u[243];
acadoWorkspace.lb[244] = - acadoVariables.u[244];
acadoWorkspace.lb[245] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[245];
acadoWorkspace.lb[246] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[246];
acadoWorkspace.lb[247] = - acadoVariables.u[247];
acadoWorkspace.lb[248] = - acadoVariables.u[248];
acadoWorkspace.lb[249] = - acadoVariables.u[249];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+00 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+00 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+00 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+12 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+00 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+00 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+12 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+00 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+00 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.0000000000000000e+00 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+12 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)1.0000000000000000e+12 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)1.0000000000000000e+12 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.0000000000000000e+00 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)1.0000000000000000e+12 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+00 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)1.0000000000000000e+00 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)1.0000000000000000e+12 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+12 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)1.0000000000000000e+00 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)1.0000000000000000e+00 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.0000000000000000e+12 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)1.0000000000000000e+12 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)1.0000000000000000e+00 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.0000000000000000e+00 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.0000000000000000e+12 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)1.0000000000000000e+12 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)1.0000000000000000e+12 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+00 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.0000000000000000e+00 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)1.0000000000000000e+12 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)1.0000000000000000e+12 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+12 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)1.0000000000000000e+00 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)1.0000000000000000e+00 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)1.0000000000000000e+12 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)1.0000000000000000e+12 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)1.0000000000000000e+12 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)1.0000000000000000e+00 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)1.0000000000000000e+00 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)1.0000000000000000e+12 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)1.0000000000000000e+12 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)1.0000000000000000e+12 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)1.0000000000000000e+00 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)1.0000000000000000e+00 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)1.0000000000000000e+12 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)1.0000000000000000e+12 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)1.0000000000000000e+12 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)1.0000000000000000e+00 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)1.0000000000000000e+00 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)1.0000000000000000e+12 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)1.0000000000000000e+12 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)1.0000000000000000e+12 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)1.0000000000000000e+00 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)1.0000000000000000e+00 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)1.0000000000000000e+12 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)1.0000000000000000e+12 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)1.0000000000000000e+12 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)1.0000000000000000e+00 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)1.0000000000000000e+00 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)1.0000000000000000e+12 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)1.0000000000000000e+12 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)1.0000000000000000e+12 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)1.0000000000000000e+00 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)1.0000000000000000e+00 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)1.0000000000000000e+12 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)1.0000000000000000e+12 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)1.0000000000000000e+12 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)1.0000000000000000e+00 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)1.0000000000000000e+00 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)1.0000000000000000e+12 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)1.0000000000000000e+12 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)1.0000000000000000e+12 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)1.0000000000000000e+00 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)1.0000000000000000e+00 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)1.0000000000000000e+12 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)1.0000000000000000e+12 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)1.0000000000000000e+12 - acadoVariables.u[119];
acadoWorkspace.ub[120] = (real_t)1.0000000000000000e+00 - acadoVariables.u[120];
acadoWorkspace.ub[121] = (real_t)1.0000000000000000e+00 - acadoVariables.u[121];
acadoWorkspace.ub[122] = (real_t)1.0000000000000000e+12 - acadoVariables.u[122];
acadoWorkspace.ub[123] = (real_t)1.0000000000000000e+12 - acadoVariables.u[123];
acadoWorkspace.ub[124] = (real_t)1.0000000000000000e+12 - acadoVariables.u[124];
acadoWorkspace.ub[125] = (real_t)1.0000000000000000e+00 - acadoVariables.u[125];
acadoWorkspace.ub[126] = (real_t)1.0000000000000000e+00 - acadoVariables.u[126];
acadoWorkspace.ub[127] = (real_t)1.0000000000000000e+12 - acadoVariables.u[127];
acadoWorkspace.ub[128] = (real_t)1.0000000000000000e+12 - acadoVariables.u[128];
acadoWorkspace.ub[129] = (real_t)1.0000000000000000e+12 - acadoVariables.u[129];
acadoWorkspace.ub[130] = (real_t)1.0000000000000000e+00 - acadoVariables.u[130];
acadoWorkspace.ub[131] = (real_t)1.0000000000000000e+00 - acadoVariables.u[131];
acadoWorkspace.ub[132] = (real_t)1.0000000000000000e+12 - acadoVariables.u[132];
acadoWorkspace.ub[133] = (real_t)1.0000000000000000e+12 - acadoVariables.u[133];
acadoWorkspace.ub[134] = (real_t)1.0000000000000000e+12 - acadoVariables.u[134];
acadoWorkspace.ub[135] = (real_t)1.0000000000000000e+00 - acadoVariables.u[135];
acadoWorkspace.ub[136] = (real_t)1.0000000000000000e+00 - acadoVariables.u[136];
acadoWorkspace.ub[137] = (real_t)1.0000000000000000e+12 - acadoVariables.u[137];
acadoWorkspace.ub[138] = (real_t)1.0000000000000000e+12 - acadoVariables.u[138];
acadoWorkspace.ub[139] = (real_t)1.0000000000000000e+12 - acadoVariables.u[139];
acadoWorkspace.ub[140] = (real_t)1.0000000000000000e+00 - acadoVariables.u[140];
acadoWorkspace.ub[141] = (real_t)1.0000000000000000e+00 - acadoVariables.u[141];
acadoWorkspace.ub[142] = (real_t)1.0000000000000000e+12 - acadoVariables.u[142];
acadoWorkspace.ub[143] = (real_t)1.0000000000000000e+12 - acadoVariables.u[143];
acadoWorkspace.ub[144] = (real_t)1.0000000000000000e+12 - acadoVariables.u[144];
acadoWorkspace.ub[145] = (real_t)1.0000000000000000e+00 - acadoVariables.u[145];
acadoWorkspace.ub[146] = (real_t)1.0000000000000000e+00 - acadoVariables.u[146];
acadoWorkspace.ub[147] = (real_t)1.0000000000000000e+12 - acadoVariables.u[147];
acadoWorkspace.ub[148] = (real_t)1.0000000000000000e+12 - acadoVariables.u[148];
acadoWorkspace.ub[149] = (real_t)1.0000000000000000e+12 - acadoVariables.u[149];
acadoWorkspace.ub[150] = (real_t)1.0000000000000000e+00 - acadoVariables.u[150];
acadoWorkspace.ub[151] = (real_t)1.0000000000000000e+00 - acadoVariables.u[151];
acadoWorkspace.ub[152] = (real_t)1.0000000000000000e+12 - acadoVariables.u[152];
acadoWorkspace.ub[153] = (real_t)1.0000000000000000e+12 - acadoVariables.u[153];
acadoWorkspace.ub[154] = (real_t)1.0000000000000000e+12 - acadoVariables.u[154];
acadoWorkspace.ub[155] = (real_t)1.0000000000000000e+00 - acadoVariables.u[155];
acadoWorkspace.ub[156] = (real_t)1.0000000000000000e+00 - acadoVariables.u[156];
acadoWorkspace.ub[157] = (real_t)1.0000000000000000e+12 - acadoVariables.u[157];
acadoWorkspace.ub[158] = (real_t)1.0000000000000000e+12 - acadoVariables.u[158];
acadoWorkspace.ub[159] = (real_t)1.0000000000000000e+12 - acadoVariables.u[159];
acadoWorkspace.ub[160] = (real_t)1.0000000000000000e+00 - acadoVariables.u[160];
acadoWorkspace.ub[161] = (real_t)1.0000000000000000e+00 - acadoVariables.u[161];
acadoWorkspace.ub[162] = (real_t)1.0000000000000000e+12 - acadoVariables.u[162];
acadoWorkspace.ub[163] = (real_t)1.0000000000000000e+12 - acadoVariables.u[163];
acadoWorkspace.ub[164] = (real_t)1.0000000000000000e+12 - acadoVariables.u[164];
acadoWorkspace.ub[165] = (real_t)1.0000000000000000e+00 - acadoVariables.u[165];
acadoWorkspace.ub[166] = (real_t)1.0000000000000000e+00 - acadoVariables.u[166];
acadoWorkspace.ub[167] = (real_t)1.0000000000000000e+12 - acadoVariables.u[167];
acadoWorkspace.ub[168] = (real_t)1.0000000000000000e+12 - acadoVariables.u[168];
acadoWorkspace.ub[169] = (real_t)1.0000000000000000e+12 - acadoVariables.u[169];
acadoWorkspace.ub[170] = (real_t)1.0000000000000000e+00 - acadoVariables.u[170];
acadoWorkspace.ub[171] = (real_t)1.0000000000000000e+00 - acadoVariables.u[171];
acadoWorkspace.ub[172] = (real_t)1.0000000000000000e+12 - acadoVariables.u[172];
acadoWorkspace.ub[173] = (real_t)1.0000000000000000e+12 - acadoVariables.u[173];
acadoWorkspace.ub[174] = (real_t)1.0000000000000000e+12 - acadoVariables.u[174];
acadoWorkspace.ub[175] = (real_t)1.0000000000000000e+00 - acadoVariables.u[175];
acadoWorkspace.ub[176] = (real_t)1.0000000000000000e+00 - acadoVariables.u[176];
acadoWorkspace.ub[177] = (real_t)1.0000000000000000e+12 - acadoVariables.u[177];
acadoWorkspace.ub[178] = (real_t)1.0000000000000000e+12 - acadoVariables.u[178];
acadoWorkspace.ub[179] = (real_t)1.0000000000000000e+12 - acadoVariables.u[179];
acadoWorkspace.ub[180] = (real_t)1.0000000000000000e+00 - acadoVariables.u[180];
acadoWorkspace.ub[181] = (real_t)1.0000000000000000e+00 - acadoVariables.u[181];
acadoWorkspace.ub[182] = (real_t)1.0000000000000000e+12 - acadoVariables.u[182];
acadoWorkspace.ub[183] = (real_t)1.0000000000000000e+12 - acadoVariables.u[183];
acadoWorkspace.ub[184] = (real_t)1.0000000000000000e+12 - acadoVariables.u[184];
acadoWorkspace.ub[185] = (real_t)1.0000000000000000e+00 - acadoVariables.u[185];
acadoWorkspace.ub[186] = (real_t)1.0000000000000000e+00 - acadoVariables.u[186];
acadoWorkspace.ub[187] = (real_t)1.0000000000000000e+12 - acadoVariables.u[187];
acadoWorkspace.ub[188] = (real_t)1.0000000000000000e+12 - acadoVariables.u[188];
acadoWorkspace.ub[189] = (real_t)1.0000000000000000e+12 - acadoVariables.u[189];
acadoWorkspace.ub[190] = (real_t)1.0000000000000000e+00 - acadoVariables.u[190];
acadoWorkspace.ub[191] = (real_t)1.0000000000000000e+00 - acadoVariables.u[191];
acadoWorkspace.ub[192] = (real_t)1.0000000000000000e+12 - acadoVariables.u[192];
acadoWorkspace.ub[193] = (real_t)1.0000000000000000e+12 - acadoVariables.u[193];
acadoWorkspace.ub[194] = (real_t)1.0000000000000000e+12 - acadoVariables.u[194];
acadoWorkspace.ub[195] = (real_t)1.0000000000000000e+00 - acadoVariables.u[195];
acadoWorkspace.ub[196] = (real_t)1.0000000000000000e+00 - acadoVariables.u[196];
acadoWorkspace.ub[197] = (real_t)1.0000000000000000e+12 - acadoVariables.u[197];
acadoWorkspace.ub[198] = (real_t)1.0000000000000000e+12 - acadoVariables.u[198];
acadoWorkspace.ub[199] = (real_t)1.0000000000000000e+12 - acadoVariables.u[199];
acadoWorkspace.ub[200] = (real_t)1.0000000000000000e+00 - acadoVariables.u[200];
acadoWorkspace.ub[201] = (real_t)1.0000000000000000e+00 - acadoVariables.u[201];
acadoWorkspace.ub[202] = (real_t)1.0000000000000000e+12 - acadoVariables.u[202];
acadoWorkspace.ub[203] = (real_t)1.0000000000000000e+12 - acadoVariables.u[203];
acadoWorkspace.ub[204] = (real_t)1.0000000000000000e+12 - acadoVariables.u[204];
acadoWorkspace.ub[205] = (real_t)1.0000000000000000e+00 - acadoVariables.u[205];
acadoWorkspace.ub[206] = (real_t)1.0000000000000000e+00 - acadoVariables.u[206];
acadoWorkspace.ub[207] = (real_t)1.0000000000000000e+12 - acadoVariables.u[207];
acadoWorkspace.ub[208] = (real_t)1.0000000000000000e+12 - acadoVariables.u[208];
acadoWorkspace.ub[209] = (real_t)1.0000000000000000e+12 - acadoVariables.u[209];
acadoWorkspace.ub[210] = (real_t)1.0000000000000000e+00 - acadoVariables.u[210];
acadoWorkspace.ub[211] = (real_t)1.0000000000000000e+00 - acadoVariables.u[211];
acadoWorkspace.ub[212] = (real_t)1.0000000000000000e+12 - acadoVariables.u[212];
acadoWorkspace.ub[213] = (real_t)1.0000000000000000e+12 - acadoVariables.u[213];
acadoWorkspace.ub[214] = (real_t)1.0000000000000000e+12 - acadoVariables.u[214];
acadoWorkspace.ub[215] = (real_t)1.0000000000000000e+00 - acadoVariables.u[215];
acadoWorkspace.ub[216] = (real_t)1.0000000000000000e+00 - acadoVariables.u[216];
acadoWorkspace.ub[217] = (real_t)1.0000000000000000e+12 - acadoVariables.u[217];
acadoWorkspace.ub[218] = (real_t)1.0000000000000000e+12 - acadoVariables.u[218];
acadoWorkspace.ub[219] = (real_t)1.0000000000000000e+12 - acadoVariables.u[219];
acadoWorkspace.ub[220] = (real_t)1.0000000000000000e+00 - acadoVariables.u[220];
acadoWorkspace.ub[221] = (real_t)1.0000000000000000e+00 - acadoVariables.u[221];
acadoWorkspace.ub[222] = (real_t)1.0000000000000000e+12 - acadoVariables.u[222];
acadoWorkspace.ub[223] = (real_t)1.0000000000000000e+12 - acadoVariables.u[223];
acadoWorkspace.ub[224] = (real_t)1.0000000000000000e+12 - acadoVariables.u[224];
acadoWorkspace.ub[225] = (real_t)1.0000000000000000e+00 - acadoVariables.u[225];
acadoWorkspace.ub[226] = (real_t)1.0000000000000000e+00 - acadoVariables.u[226];
acadoWorkspace.ub[227] = (real_t)1.0000000000000000e+12 - acadoVariables.u[227];
acadoWorkspace.ub[228] = (real_t)1.0000000000000000e+12 - acadoVariables.u[228];
acadoWorkspace.ub[229] = (real_t)1.0000000000000000e+12 - acadoVariables.u[229];
acadoWorkspace.ub[230] = (real_t)1.0000000000000000e+00 - acadoVariables.u[230];
acadoWorkspace.ub[231] = (real_t)1.0000000000000000e+00 - acadoVariables.u[231];
acadoWorkspace.ub[232] = (real_t)1.0000000000000000e+12 - acadoVariables.u[232];
acadoWorkspace.ub[233] = (real_t)1.0000000000000000e+12 - acadoVariables.u[233];
acadoWorkspace.ub[234] = (real_t)1.0000000000000000e+12 - acadoVariables.u[234];
acadoWorkspace.ub[235] = (real_t)1.0000000000000000e+00 - acadoVariables.u[235];
acadoWorkspace.ub[236] = (real_t)1.0000000000000000e+00 - acadoVariables.u[236];
acadoWorkspace.ub[237] = (real_t)1.0000000000000000e+12 - acadoVariables.u[237];
acadoWorkspace.ub[238] = (real_t)1.0000000000000000e+12 - acadoVariables.u[238];
acadoWorkspace.ub[239] = (real_t)1.0000000000000000e+12 - acadoVariables.u[239];
acadoWorkspace.ub[240] = (real_t)1.0000000000000000e+00 - acadoVariables.u[240];
acadoWorkspace.ub[241] = (real_t)1.0000000000000000e+00 - acadoVariables.u[241];
acadoWorkspace.ub[242] = (real_t)1.0000000000000000e+12 - acadoVariables.u[242];
acadoWorkspace.ub[243] = (real_t)1.0000000000000000e+12 - acadoVariables.u[243];
acadoWorkspace.ub[244] = (real_t)1.0000000000000000e+12 - acadoVariables.u[244];
acadoWorkspace.ub[245] = (real_t)1.0000000000000000e+00 - acadoVariables.u[245];
acadoWorkspace.ub[246] = (real_t)1.0000000000000000e+00 - acadoVariables.u[246];
acadoWorkspace.ub[247] = (real_t)1.0000000000000000e+12 - acadoVariables.u[247];
acadoWorkspace.ub[248] = (real_t)1.0000000000000000e+12 - acadoVariables.u[248];
acadoWorkspace.ub[249] = (real_t)1.0000000000000000e+12 - acadoVariables.u[249];

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.u[lRun1 * 5];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 5 + 1];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 5 + 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 5 + 3];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun1 * 5 + 4];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 3] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 3 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 3 + 2] = acadoWorkspace.conValueOut[2];

acadoWorkspace.evHx[lRun1 * 9] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 9 + 1] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 9 + 2] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 9 + 3] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 9 + 4] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 9 + 5] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 9 + 6] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 9 + 7] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 9 + 8] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHu[lRun1 * 15] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHu[lRun1 * 15 + 1] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHu[lRun1 * 15 + 2] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHu[lRun1 * 15 + 3] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHu[lRun1 * 15 + 4] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHu[lRun1 * 15 + 5] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHu[lRun1 * 15 + 6] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHu[lRun1 * 15 + 7] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHu[lRun1 * 15 + 8] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHu[lRun1 * 15 + 9] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHu[lRun1 * 15 + 10] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHu[lRun1 * 15 + 11] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHu[lRun1 * 15 + 12] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHu[lRun1 * 15 + 13] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHu[lRun1 * 15 + 14] = acadoWorkspace.conValueOut[26];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];

acado_multHxC( &(acadoWorkspace.evHx[ 9 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 9 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.A01[ 18 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 27 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.A01[ 27 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 45 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.A01[ 45 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.A01[ 54 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 63 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.A01[ 63 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 81 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.A01[ 81 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.A01[ 90 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 99 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.A01[ 99 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 117 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.A01[ 117 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.A01[ 126 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 135 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.A01[ 135 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.A01[ 144 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 153 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 153 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 162 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.A01[ 162 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 171 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.A01[ 171 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 189 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.A01[ 189 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 198 ]), &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.A01[ 198 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 207 ]), &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.A01[ 207 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.A01[ 216 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 225 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.A01[ 225 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 234 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.A01[ 234 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 243 ]), &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.A01[ 243 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.A01[ 252 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 261 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.A01[ 261 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.A01[ 270 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 279 ]), &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.A01[ 279 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.A01[ 288 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 297 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A01[ 297 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 306 ]), &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.A01[ 306 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 315 ]), &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.A01[ 315 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 324 ]), &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.A01[ 324 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 333 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.A01[ 333 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 342 ]), &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.A01[ 342 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 351 ]), &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.A01[ 351 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.A01[ 360 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 369 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.A01[ 369 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 378 ]), &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.A01[ 378 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 387 ]), &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.A01[ 387 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 396 ]), &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.A01[ 396 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 405 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.A01[ 405 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 414 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.A01[ 414 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 423 ]), &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.A01[ 423 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 432 ]), &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.A01[ 432 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 441 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.A01[ 441 ]) );

for (lRun2 = 0; lRun2 < 49; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 9 + 9 ]), &(acadoWorkspace.E[ lRun4 * 15 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3] = acadoWorkspace.evHu[3];
acadoWorkspace.A[4] = acadoWorkspace.evHu[4];
acadoWorkspace.A[250] = acadoWorkspace.evHu[5];
acadoWorkspace.A[251] = acadoWorkspace.evHu[6];
acadoWorkspace.A[252] = acadoWorkspace.evHu[7];
acadoWorkspace.A[253] = acadoWorkspace.evHu[8];
acadoWorkspace.A[254] = acadoWorkspace.evHu[9];
acadoWorkspace.A[500] = acadoWorkspace.evHu[10];
acadoWorkspace.A[501] = acadoWorkspace.evHu[11];
acadoWorkspace.A[502] = acadoWorkspace.evHu[12];
acadoWorkspace.A[503] = acadoWorkspace.evHu[13];
acadoWorkspace.A[504] = acadoWorkspace.evHu[14];
acadoWorkspace.A[755] = acadoWorkspace.evHu[15];
acadoWorkspace.A[756] = acadoWorkspace.evHu[16];
acadoWorkspace.A[757] = acadoWorkspace.evHu[17];
acadoWorkspace.A[758] = acadoWorkspace.evHu[18];
acadoWorkspace.A[759] = acadoWorkspace.evHu[19];
acadoWorkspace.A[1005] = acadoWorkspace.evHu[20];
acadoWorkspace.A[1006] = acadoWorkspace.evHu[21];
acadoWorkspace.A[1007] = acadoWorkspace.evHu[22];
acadoWorkspace.A[1008] = acadoWorkspace.evHu[23];
acadoWorkspace.A[1009] = acadoWorkspace.evHu[24];
acadoWorkspace.A[1255] = acadoWorkspace.evHu[25];
acadoWorkspace.A[1256] = acadoWorkspace.evHu[26];
acadoWorkspace.A[1257] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1258] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1259] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1510] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1511] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1512] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1513] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1514] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1760] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1761] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1762] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1763] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1764] = acadoWorkspace.evHu[39];
acadoWorkspace.A[2010] = acadoWorkspace.evHu[40];
acadoWorkspace.A[2011] = acadoWorkspace.evHu[41];
acadoWorkspace.A[2012] = acadoWorkspace.evHu[42];
acadoWorkspace.A[2013] = acadoWorkspace.evHu[43];
acadoWorkspace.A[2014] = acadoWorkspace.evHu[44];
acadoWorkspace.A[2265] = acadoWorkspace.evHu[45];
acadoWorkspace.A[2266] = acadoWorkspace.evHu[46];
acadoWorkspace.A[2267] = acadoWorkspace.evHu[47];
acadoWorkspace.A[2268] = acadoWorkspace.evHu[48];
acadoWorkspace.A[2269] = acadoWorkspace.evHu[49];
acadoWorkspace.A[2515] = acadoWorkspace.evHu[50];
acadoWorkspace.A[2516] = acadoWorkspace.evHu[51];
acadoWorkspace.A[2517] = acadoWorkspace.evHu[52];
acadoWorkspace.A[2518] = acadoWorkspace.evHu[53];
acadoWorkspace.A[2519] = acadoWorkspace.evHu[54];
acadoWorkspace.A[2765] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2766] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2767] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2768] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2769] = acadoWorkspace.evHu[59];
acadoWorkspace.A[3020] = acadoWorkspace.evHu[60];
acadoWorkspace.A[3021] = acadoWorkspace.evHu[61];
acadoWorkspace.A[3022] = acadoWorkspace.evHu[62];
acadoWorkspace.A[3023] = acadoWorkspace.evHu[63];
acadoWorkspace.A[3024] = acadoWorkspace.evHu[64];
acadoWorkspace.A[3270] = acadoWorkspace.evHu[65];
acadoWorkspace.A[3271] = acadoWorkspace.evHu[66];
acadoWorkspace.A[3272] = acadoWorkspace.evHu[67];
acadoWorkspace.A[3273] = acadoWorkspace.evHu[68];
acadoWorkspace.A[3274] = acadoWorkspace.evHu[69];
acadoWorkspace.A[3520] = acadoWorkspace.evHu[70];
acadoWorkspace.A[3521] = acadoWorkspace.evHu[71];
acadoWorkspace.A[3522] = acadoWorkspace.evHu[72];
acadoWorkspace.A[3523] = acadoWorkspace.evHu[73];
acadoWorkspace.A[3524] = acadoWorkspace.evHu[74];
acadoWorkspace.A[3775] = acadoWorkspace.evHu[75];
acadoWorkspace.A[3776] = acadoWorkspace.evHu[76];
acadoWorkspace.A[3777] = acadoWorkspace.evHu[77];
acadoWorkspace.A[3778] = acadoWorkspace.evHu[78];
acadoWorkspace.A[3779] = acadoWorkspace.evHu[79];
acadoWorkspace.A[4025] = acadoWorkspace.evHu[80];
acadoWorkspace.A[4026] = acadoWorkspace.evHu[81];
acadoWorkspace.A[4027] = acadoWorkspace.evHu[82];
acadoWorkspace.A[4028] = acadoWorkspace.evHu[83];
acadoWorkspace.A[4029] = acadoWorkspace.evHu[84];
acadoWorkspace.A[4275] = acadoWorkspace.evHu[85];
acadoWorkspace.A[4276] = acadoWorkspace.evHu[86];
acadoWorkspace.A[4277] = acadoWorkspace.evHu[87];
acadoWorkspace.A[4278] = acadoWorkspace.evHu[88];
acadoWorkspace.A[4279] = acadoWorkspace.evHu[89];
acadoWorkspace.A[4530] = acadoWorkspace.evHu[90];
acadoWorkspace.A[4531] = acadoWorkspace.evHu[91];
acadoWorkspace.A[4532] = acadoWorkspace.evHu[92];
acadoWorkspace.A[4533] = acadoWorkspace.evHu[93];
acadoWorkspace.A[4534] = acadoWorkspace.evHu[94];
acadoWorkspace.A[4780] = acadoWorkspace.evHu[95];
acadoWorkspace.A[4781] = acadoWorkspace.evHu[96];
acadoWorkspace.A[4782] = acadoWorkspace.evHu[97];
acadoWorkspace.A[4783] = acadoWorkspace.evHu[98];
acadoWorkspace.A[4784] = acadoWorkspace.evHu[99];
acadoWorkspace.A[5030] = acadoWorkspace.evHu[100];
acadoWorkspace.A[5031] = acadoWorkspace.evHu[101];
acadoWorkspace.A[5032] = acadoWorkspace.evHu[102];
acadoWorkspace.A[5033] = acadoWorkspace.evHu[103];
acadoWorkspace.A[5034] = acadoWorkspace.evHu[104];
acadoWorkspace.A[5285] = acadoWorkspace.evHu[105];
acadoWorkspace.A[5286] = acadoWorkspace.evHu[106];
acadoWorkspace.A[5287] = acadoWorkspace.evHu[107];
acadoWorkspace.A[5288] = acadoWorkspace.evHu[108];
acadoWorkspace.A[5289] = acadoWorkspace.evHu[109];
acadoWorkspace.A[5535] = acadoWorkspace.evHu[110];
acadoWorkspace.A[5536] = acadoWorkspace.evHu[111];
acadoWorkspace.A[5537] = acadoWorkspace.evHu[112];
acadoWorkspace.A[5538] = acadoWorkspace.evHu[113];
acadoWorkspace.A[5539] = acadoWorkspace.evHu[114];
acadoWorkspace.A[5785] = acadoWorkspace.evHu[115];
acadoWorkspace.A[5786] = acadoWorkspace.evHu[116];
acadoWorkspace.A[5787] = acadoWorkspace.evHu[117];
acadoWorkspace.A[5788] = acadoWorkspace.evHu[118];
acadoWorkspace.A[5789] = acadoWorkspace.evHu[119];
acadoWorkspace.A[6040] = acadoWorkspace.evHu[120];
acadoWorkspace.A[6041] = acadoWorkspace.evHu[121];
acadoWorkspace.A[6042] = acadoWorkspace.evHu[122];
acadoWorkspace.A[6043] = acadoWorkspace.evHu[123];
acadoWorkspace.A[6044] = acadoWorkspace.evHu[124];
acadoWorkspace.A[6290] = acadoWorkspace.evHu[125];
acadoWorkspace.A[6291] = acadoWorkspace.evHu[126];
acadoWorkspace.A[6292] = acadoWorkspace.evHu[127];
acadoWorkspace.A[6293] = acadoWorkspace.evHu[128];
acadoWorkspace.A[6294] = acadoWorkspace.evHu[129];
acadoWorkspace.A[6540] = acadoWorkspace.evHu[130];
acadoWorkspace.A[6541] = acadoWorkspace.evHu[131];
acadoWorkspace.A[6542] = acadoWorkspace.evHu[132];
acadoWorkspace.A[6543] = acadoWorkspace.evHu[133];
acadoWorkspace.A[6544] = acadoWorkspace.evHu[134];
acadoWorkspace.A[6795] = acadoWorkspace.evHu[135];
acadoWorkspace.A[6796] = acadoWorkspace.evHu[136];
acadoWorkspace.A[6797] = acadoWorkspace.evHu[137];
acadoWorkspace.A[6798] = acadoWorkspace.evHu[138];
acadoWorkspace.A[6799] = acadoWorkspace.evHu[139];
acadoWorkspace.A[7045] = acadoWorkspace.evHu[140];
acadoWorkspace.A[7046] = acadoWorkspace.evHu[141];
acadoWorkspace.A[7047] = acadoWorkspace.evHu[142];
acadoWorkspace.A[7048] = acadoWorkspace.evHu[143];
acadoWorkspace.A[7049] = acadoWorkspace.evHu[144];
acadoWorkspace.A[7295] = acadoWorkspace.evHu[145];
acadoWorkspace.A[7296] = acadoWorkspace.evHu[146];
acadoWorkspace.A[7297] = acadoWorkspace.evHu[147];
acadoWorkspace.A[7298] = acadoWorkspace.evHu[148];
acadoWorkspace.A[7299] = acadoWorkspace.evHu[149];
acadoWorkspace.A[7550] = acadoWorkspace.evHu[150];
acadoWorkspace.A[7551] = acadoWorkspace.evHu[151];
acadoWorkspace.A[7552] = acadoWorkspace.evHu[152];
acadoWorkspace.A[7553] = acadoWorkspace.evHu[153];
acadoWorkspace.A[7554] = acadoWorkspace.evHu[154];
acadoWorkspace.A[7800] = acadoWorkspace.evHu[155];
acadoWorkspace.A[7801] = acadoWorkspace.evHu[156];
acadoWorkspace.A[7802] = acadoWorkspace.evHu[157];
acadoWorkspace.A[7803] = acadoWorkspace.evHu[158];
acadoWorkspace.A[7804] = acadoWorkspace.evHu[159];
acadoWorkspace.A[8050] = acadoWorkspace.evHu[160];
acadoWorkspace.A[8051] = acadoWorkspace.evHu[161];
acadoWorkspace.A[8052] = acadoWorkspace.evHu[162];
acadoWorkspace.A[8053] = acadoWorkspace.evHu[163];
acadoWorkspace.A[8054] = acadoWorkspace.evHu[164];
acadoWorkspace.A[8305] = acadoWorkspace.evHu[165];
acadoWorkspace.A[8306] = acadoWorkspace.evHu[166];
acadoWorkspace.A[8307] = acadoWorkspace.evHu[167];
acadoWorkspace.A[8308] = acadoWorkspace.evHu[168];
acadoWorkspace.A[8309] = acadoWorkspace.evHu[169];
acadoWorkspace.A[8555] = acadoWorkspace.evHu[170];
acadoWorkspace.A[8556] = acadoWorkspace.evHu[171];
acadoWorkspace.A[8557] = acadoWorkspace.evHu[172];
acadoWorkspace.A[8558] = acadoWorkspace.evHu[173];
acadoWorkspace.A[8559] = acadoWorkspace.evHu[174];
acadoWorkspace.A[8805] = acadoWorkspace.evHu[175];
acadoWorkspace.A[8806] = acadoWorkspace.evHu[176];
acadoWorkspace.A[8807] = acadoWorkspace.evHu[177];
acadoWorkspace.A[8808] = acadoWorkspace.evHu[178];
acadoWorkspace.A[8809] = acadoWorkspace.evHu[179];
acadoWorkspace.A[9060] = acadoWorkspace.evHu[180];
acadoWorkspace.A[9061] = acadoWorkspace.evHu[181];
acadoWorkspace.A[9062] = acadoWorkspace.evHu[182];
acadoWorkspace.A[9063] = acadoWorkspace.evHu[183];
acadoWorkspace.A[9064] = acadoWorkspace.evHu[184];
acadoWorkspace.A[9310] = acadoWorkspace.evHu[185];
acadoWorkspace.A[9311] = acadoWorkspace.evHu[186];
acadoWorkspace.A[9312] = acadoWorkspace.evHu[187];
acadoWorkspace.A[9313] = acadoWorkspace.evHu[188];
acadoWorkspace.A[9314] = acadoWorkspace.evHu[189];
acadoWorkspace.A[9560] = acadoWorkspace.evHu[190];
acadoWorkspace.A[9561] = acadoWorkspace.evHu[191];
acadoWorkspace.A[9562] = acadoWorkspace.evHu[192];
acadoWorkspace.A[9563] = acadoWorkspace.evHu[193];
acadoWorkspace.A[9564] = acadoWorkspace.evHu[194];
acadoWorkspace.A[9815] = acadoWorkspace.evHu[195];
acadoWorkspace.A[9816] = acadoWorkspace.evHu[196];
acadoWorkspace.A[9817] = acadoWorkspace.evHu[197];
acadoWorkspace.A[9818] = acadoWorkspace.evHu[198];
acadoWorkspace.A[9819] = acadoWorkspace.evHu[199];
acadoWorkspace.A[10065] = acadoWorkspace.evHu[200];
acadoWorkspace.A[10066] = acadoWorkspace.evHu[201];
acadoWorkspace.A[10067] = acadoWorkspace.evHu[202];
acadoWorkspace.A[10068] = acadoWorkspace.evHu[203];
acadoWorkspace.A[10069] = acadoWorkspace.evHu[204];
acadoWorkspace.A[10315] = acadoWorkspace.evHu[205];
acadoWorkspace.A[10316] = acadoWorkspace.evHu[206];
acadoWorkspace.A[10317] = acadoWorkspace.evHu[207];
acadoWorkspace.A[10318] = acadoWorkspace.evHu[208];
acadoWorkspace.A[10319] = acadoWorkspace.evHu[209];
acadoWorkspace.A[10570] = acadoWorkspace.evHu[210];
acadoWorkspace.A[10571] = acadoWorkspace.evHu[211];
acadoWorkspace.A[10572] = acadoWorkspace.evHu[212];
acadoWorkspace.A[10573] = acadoWorkspace.evHu[213];
acadoWorkspace.A[10574] = acadoWorkspace.evHu[214];
acadoWorkspace.A[10820] = acadoWorkspace.evHu[215];
acadoWorkspace.A[10821] = acadoWorkspace.evHu[216];
acadoWorkspace.A[10822] = acadoWorkspace.evHu[217];
acadoWorkspace.A[10823] = acadoWorkspace.evHu[218];
acadoWorkspace.A[10824] = acadoWorkspace.evHu[219];
acadoWorkspace.A[11070] = acadoWorkspace.evHu[220];
acadoWorkspace.A[11071] = acadoWorkspace.evHu[221];
acadoWorkspace.A[11072] = acadoWorkspace.evHu[222];
acadoWorkspace.A[11073] = acadoWorkspace.evHu[223];
acadoWorkspace.A[11074] = acadoWorkspace.evHu[224];
acadoWorkspace.A[11325] = acadoWorkspace.evHu[225];
acadoWorkspace.A[11326] = acadoWorkspace.evHu[226];
acadoWorkspace.A[11327] = acadoWorkspace.evHu[227];
acadoWorkspace.A[11328] = acadoWorkspace.evHu[228];
acadoWorkspace.A[11329] = acadoWorkspace.evHu[229];
acadoWorkspace.A[11575] = acadoWorkspace.evHu[230];
acadoWorkspace.A[11576] = acadoWorkspace.evHu[231];
acadoWorkspace.A[11577] = acadoWorkspace.evHu[232];
acadoWorkspace.A[11578] = acadoWorkspace.evHu[233];
acadoWorkspace.A[11579] = acadoWorkspace.evHu[234];
acadoWorkspace.A[11825] = acadoWorkspace.evHu[235];
acadoWorkspace.A[11826] = acadoWorkspace.evHu[236];
acadoWorkspace.A[11827] = acadoWorkspace.evHu[237];
acadoWorkspace.A[11828] = acadoWorkspace.evHu[238];
acadoWorkspace.A[11829] = acadoWorkspace.evHu[239];
acadoWorkspace.A[12080] = acadoWorkspace.evHu[240];
acadoWorkspace.A[12081] = acadoWorkspace.evHu[241];
acadoWorkspace.A[12082] = acadoWorkspace.evHu[242];
acadoWorkspace.A[12083] = acadoWorkspace.evHu[243];
acadoWorkspace.A[12084] = acadoWorkspace.evHu[244];
acadoWorkspace.A[12330] = acadoWorkspace.evHu[245];
acadoWorkspace.A[12331] = acadoWorkspace.evHu[246];
acadoWorkspace.A[12332] = acadoWorkspace.evHu[247];
acadoWorkspace.A[12333] = acadoWorkspace.evHu[248];
acadoWorkspace.A[12334] = acadoWorkspace.evHu[249];
acadoWorkspace.A[12580] = acadoWorkspace.evHu[250];
acadoWorkspace.A[12581] = acadoWorkspace.evHu[251];
acadoWorkspace.A[12582] = acadoWorkspace.evHu[252];
acadoWorkspace.A[12583] = acadoWorkspace.evHu[253];
acadoWorkspace.A[12584] = acadoWorkspace.evHu[254];
acadoWorkspace.A[12835] = acadoWorkspace.evHu[255];
acadoWorkspace.A[12836] = acadoWorkspace.evHu[256];
acadoWorkspace.A[12837] = acadoWorkspace.evHu[257];
acadoWorkspace.A[12838] = acadoWorkspace.evHu[258];
acadoWorkspace.A[12839] = acadoWorkspace.evHu[259];
acadoWorkspace.A[13085] = acadoWorkspace.evHu[260];
acadoWorkspace.A[13086] = acadoWorkspace.evHu[261];
acadoWorkspace.A[13087] = acadoWorkspace.evHu[262];
acadoWorkspace.A[13088] = acadoWorkspace.evHu[263];
acadoWorkspace.A[13089] = acadoWorkspace.evHu[264];
acadoWorkspace.A[13335] = acadoWorkspace.evHu[265];
acadoWorkspace.A[13336] = acadoWorkspace.evHu[266];
acadoWorkspace.A[13337] = acadoWorkspace.evHu[267];
acadoWorkspace.A[13338] = acadoWorkspace.evHu[268];
acadoWorkspace.A[13339] = acadoWorkspace.evHu[269];
acadoWorkspace.A[13590] = acadoWorkspace.evHu[270];
acadoWorkspace.A[13591] = acadoWorkspace.evHu[271];
acadoWorkspace.A[13592] = acadoWorkspace.evHu[272];
acadoWorkspace.A[13593] = acadoWorkspace.evHu[273];
acadoWorkspace.A[13594] = acadoWorkspace.evHu[274];
acadoWorkspace.A[13840] = acadoWorkspace.evHu[275];
acadoWorkspace.A[13841] = acadoWorkspace.evHu[276];
acadoWorkspace.A[13842] = acadoWorkspace.evHu[277];
acadoWorkspace.A[13843] = acadoWorkspace.evHu[278];
acadoWorkspace.A[13844] = acadoWorkspace.evHu[279];
acadoWorkspace.A[14090] = acadoWorkspace.evHu[280];
acadoWorkspace.A[14091] = acadoWorkspace.evHu[281];
acadoWorkspace.A[14092] = acadoWorkspace.evHu[282];
acadoWorkspace.A[14093] = acadoWorkspace.evHu[283];
acadoWorkspace.A[14094] = acadoWorkspace.evHu[284];
acadoWorkspace.A[14345] = acadoWorkspace.evHu[285];
acadoWorkspace.A[14346] = acadoWorkspace.evHu[286];
acadoWorkspace.A[14347] = acadoWorkspace.evHu[287];
acadoWorkspace.A[14348] = acadoWorkspace.evHu[288];
acadoWorkspace.A[14349] = acadoWorkspace.evHu[289];
acadoWorkspace.A[14595] = acadoWorkspace.evHu[290];
acadoWorkspace.A[14596] = acadoWorkspace.evHu[291];
acadoWorkspace.A[14597] = acadoWorkspace.evHu[292];
acadoWorkspace.A[14598] = acadoWorkspace.evHu[293];
acadoWorkspace.A[14599] = acadoWorkspace.evHu[294];
acadoWorkspace.A[14845] = acadoWorkspace.evHu[295];
acadoWorkspace.A[14846] = acadoWorkspace.evHu[296];
acadoWorkspace.A[14847] = acadoWorkspace.evHu[297];
acadoWorkspace.A[14848] = acadoWorkspace.evHu[298];
acadoWorkspace.A[14849] = acadoWorkspace.evHu[299];
acadoWorkspace.A[15100] = acadoWorkspace.evHu[300];
acadoWorkspace.A[15101] = acadoWorkspace.evHu[301];
acadoWorkspace.A[15102] = acadoWorkspace.evHu[302];
acadoWorkspace.A[15103] = acadoWorkspace.evHu[303];
acadoWorkspace.A[15104] = acadoWorkspace.evHu[304];
acadoWorkspace.A[15350] = acadoWorkspace.evHu[305];
acadoWorkspace.A[15351] = acadoWorkspace.evHu[306];
acadoWorkspace.A[15352] = acadoWorkspace.evHu[307];
acadoWorkspace.A[15353] = acadoWorkspace.evHu[308];
acadoWorkspace.A[15354] = acadoWorkspace.evHu[309];
acadoWorkspace.A[15600] = acadoWorkspace.evHu[310];
acadoWorkspace.A[15601] = acadoWorkspace.evHu[311];
acadoWorkspace.A[15602] = acadoWorkspace.evHu[312];
acadoWorkspace.A[15603] = acadoWorkspace.evHu[313];
acadoWorkspace.A[15604] = acadoWorkspace.evHu[314];
acadoWorkspace.A[15855] = acadoWorkspace.evHu[315];
acadoWorkspace.A[15856] = acadoWorkspace.evHu[316];
acadoWorkspace.A[15857] = acadoWorkspace.evHu[317];
acadoWorkspace.A[15858] = acadoWorkspace.evHu[318];
acadoWorkspace.A[15859] = acadoWorkspace.evHu[319];
acadoWorkspace.A[16105] = acadoWorkspace.evHu[320];
acadoWorkspace.A[16106] = acadoWorkspace.evHu[321];
acadoWorkspace.A[16107] = acadoWorkspace.evHu[322];
acadoWorkspace.A[16108] = acadoWorkspace.evHu[323];
acadoWorkspace.A[16109] = acadoWorkspace.evHu[324];
acadoWorkspace.A[16355] = acadoWorkspace.evHu[325];
acadoWorkspace.A[16356] = acadoWorkspace.evHu[326];
acadoWorkspace.A[16357] = acadoWorkspace.evHu[327];
acadoWorkspace.A[16358] = acadoWorkspace.evHu[328];
acadoWorkspace.A[16359] = acadoWorkspace.evHu[329];
acadoWorkspace.A[16610] = acadoWorkspace.evHu[330];
acadoWorkspace.A[16611] = acadoWorkspace.evHu[331];
acadoWorkspace.A[16612] = acadoWorkspace.evHu[332];
acadoWorkspace.A[16613] = acadoWorkspace.evHu[333];
acadoWorkspace.A[16614] = acadoWorkspace.evHu[334];
acadoWorkspace.A[16860] = acadoWorkspace.evHu[335];
acadoWorkspace.A[16861] = acadoWorkspace.evHu[336];
acadoWorkspace.A[16862] = acadoWorkspace.evHu[337];
acadoWorkspace.A[16863] = acadoWorkspace.evHu[338];
acadoWorkspace.A[16864] = acadoWorkspace.evHu[339];
acadoWorkspace.A[17110] = acadoWorkspace.evHu[340];
acadoWorkspace.A[17111] = acadoWorkspace.evHu[341];
acadoWorkspace.A[17112] = acadoWorkspace.evHu[342];
acadoWorkspace.A[17113] = acadoWorkspace.evHu[343];
acadoWorkspace.A[17114] = acadoWorkspace.evHu[344];
acadoWorkspace.A[17365] = acadoWorkspace.evHu[345];
acadoWorkspace.A[17366] = acadoWorkspace.evHu[346];
acadoWorkspace.A[17367] = acadoWorkspace.evHu[347];
acadoWorkspace.A[17368] = acadoWorkspace.evHu[348];
acadoWorkspace.A[17369] = acadoWorkspace.evHu[349];
acadoWorkspace.A[17615] = acadoWorkspace.evHu[350];
acadoWorkspace.A[17616] = acadoWorkspace.evHu[351];
acadoWorkspace.A[17617] = acadoWorkspace.evHu[352];
acadoWorkspace.A[17618] = acadoWorkspace.evHu[353];
acadoWorkspace.A[17619] = acadoWorkspace.evHu[354];
acadoWorkspace.A[17865] = acadoWorkspace.evHu[355];
acadoWorkspace.A[17866] = acadoWorkspace.evHu[356];
acadoWorkspace.A[17867] = acadoWorkspace.evHu[357];
acadoWorkspace.A[17868] = acadoWorkspace.evHu[358];
acadoWorkspace.A[17869] = acadoWorkspace.evHu[359];
acadoWorkspace.A[18120] = acadoWorkspace.evHu[360];
acadoWorkspace.A[18121] = acadoWorkspace.evHu[361];
acadoWorkspace.A[18122] = acadoWorkspace.evHu[362];
acadoWorkspace.A[18123] = acadoWorkspace.evHu[363];
acadoWorkspace.A[18124] = acadoWorkspace.evHu[364];
acadoWorkspace.A[18370] = acadoWorkspace.evHu[365];
acadoWorkspace.A[18371] = acadoWorkspace.evHu[366];
acadoWorkspace.A[18372] = acadoWorkspace.evHu[367];
acadoWorkspace.A[18373] = acadoWorkspace.evHu[368];
acadoWorkspace.A[18374] = acadoWorkspace.evHu[369];
acadoWorkspace.A[18620] = acadoWorkspace.evHu[370];
acadoWorkspace.A[18621] = acadoWorkspace.evHu[371];
acadoWorkspace.A[18622] = acadoWorkspace.evHu[372];
acadoWorkspace.A[18623] = acadoWorkspace.evHu[373];
acadoWorkspace.A[18624] = acadoWorkspace.evHu[374];
acadoWorkspace.A[18875] = acadoWorkspace.evHu[375];
acadoWorkspace.A[18876] = acadoWorkspace.evHu[376];
acadoWorkspace.A[18877] = acadoWorkspace.evHu[377];
acadoWorkspace.A[18878] = acadoWorkspace.evHu[378];
acadoWorkspace.A[18879] = acadoWorkspace.evHu[379];
acadoWorkspace.A[19125] = acadoWorkspace.evHu[380];
acadoWorkspace.A[19126] = acadoWorkspace.evHu[381];
acadoWorkspace.A[19127] = acadoWorkspace.evHu[382];
acadoWorkspace.A[19128] = acadoWorkspace.evHu[383];
acadoWorkspace.A[19129] = acadoWorkspace.evHu[384];
acadoWorkspace.A[19375] = acadoWorkspace.evHu[385];
acadoWorkspace.A[19376] = acadoWorkspace.evHu[386];
acadoWorkspace.A[19377] = acadoWorkspace.evHu[387];
acadoWorkspace.A[19378] = acadoWorkspace.evHu[388];
acadoWorkspace.A[19379] = acadoWorkspace.evHu[389];
acadoWorkspace.A[19630] = acadoWorkspace.evHu[390];
acadoWorkspace.A[19631] = acadoWorkspace.evHu[391];
acadoWorkspace.A[19632] = acadoWorkspace.evHu[392];
acadoWorkspace.A[19633] = acadoWorkspace.evHu[393];
acadoWorkspace.A[19634] = acadoWorkspace.evHu[394];
acadoWorkspace.A[19880] = acadoWorkspace.evHu[395];
acadoWorkspace.A[19881] = acadoWorkspace.evHu[396];
acadoWorkspace.A[19882] = acadoWorkspace.evHu[397];
acadoWorkspace.A[19883] = acadoWorkspace.evHu[398];
acadoWorkspace.A[19884] = acadoWorkspace.evHu[399];
acadoWorkspace.A[20130] = acadoWorkspace.evHu[400];
acadoWorkspace.A[20131] = acadoWorkspace.evHu[401];
acadoWorkspace.A[20132] = acadoWorkspace.evHu[402];
acadoWorkspace.A[20133] = acadoWorkspace.evHu[403];
acadoWorkspace.A[20134] = acadoWorkspace.evHu[404];
acadoWorkspace.A[20385] = acadoWorkspace.evHu[405];
acadoWorkspace.A[20386] = acadoWorkspace.evHu[406];
acadoWorkspace.A[20387] = acadoWorkspace.evHu[407];
acadoWorkspace.A[20388] = acadoWorkspace.evHu[408];
acadoWorkspace.A[20389] = acadoWorkspace.evHu[409];
acadoWorkspace.A[20635] = acadoWorkspace.evHu[410];
acadoWorkspace.A[20636] = acadoWorkspace.evHu[411];
acadoWorkspace.A[20637] = acadoWorkspace.evHu[412];
acadoWorkspace.A[20638] = acadoWorkspace.evHu[413];
acadoWorkspace.A[20639] = acadoWorkspace.evHu[414];
acadoWorkspace.A[20885] = acadoWorkspace.evHu[415];
acadoWorkspace.A[20886] = acadoWorkspace.evHu[416];
acadoWorkspace.A[20887] = acadoWorkspace.evHu[417];
acadoWorkspace.A[20888] = acadoWorkspace.evHu[418];
acadoWorkspace.A[20889] = acadoWorkspace.evHu[419];
acadoWorkspace.A[21140] = acadoWorkspace.evHu[420];
acadoWorkspace.A[21141] = acadoWorkspace.evHu[421];
acadoWorkspace.A[21142] = acadoWorkspace.evHu[422];
acadoWorkspace.A[21143] = acadoWorkspace.evHu[423];
acadoWorkspace.A[21144] = acadoWorkspace.evHu[424];
acadoWorkspace.A[21390] = acadoWorkspace.evHu[425];
acadoWorkspace.A[21391] = acadoWorkspace.evHu[426];
acadoWorkspace.A[21392] = acadoWorkspace.evHu[427];
acadoWorkspace.A[21393] = acadoWorkspace.evHu[428];
acadoWorkspace.A[21394] = acadoWorkspace.evHu[429];
acadoWorkspace.A[21640] = acadoWorkspace.evHu[430];
acadoWorkspace.A[21641] = acadoWorkspace.evHu[431];
acadoWorkspace.A[21642] = acadoWorkspace.evHu[432];
acadoWorkspace.A[21643] = acadoWorkspace.evHu[433];
acadoWorkspace.A[21644] = acadoWorkspace.evHu[434];
acadoWorkspace.A[21895] = acadoWorkspace.evHu[435];
acadoWorkspace.A[21896] = acadoWorkspace.evHu[436];
acadoWorkspace.A[21897] = acadoWorkspace.evHu[437];
acadoWorkspace.A[21898] = acadoWorkspace.evHu[438];
acadoWorkspace.A[21899] = acadoWorkspace.evHu[439];
acadoWorkspace.A[22145] = acadoWorkspace.evHu[440];
acadoWorkspace.A[22146] = acadoWorkspace.evHu[441];
acadoWorkspace.A[22147] = acadoWorkspace.evHu[442];
acadoWorkspace.A[22148] = acadoWorkspace.evHu[443];
acadoWorkspace.A[22149] = acadoWorkspace.evHu[444];
acadoWorkspace.A[22395] = acadoWorkspace.evHu[445];
acadoWorkspace.A[22396] = acadoWorkspace.evHu[446];
acadoWorkspace.A[22397] = acadoWorkspace.evHu[447];
acadoWorkspace.A[22398] = acadoWorkspace.evHu[448];
acadoWorkspace.A[22399] = acadoWorkspace.evHu[449];
acadoWorkspace.A[22650] = acadoWorkspace.evHu[450];
acadoWorkspace.A[22651] = acadoWorkspace.evHu[451];
acadoWorkspace.A[22652] = acadoWorkspace.evHu[452];
acadoWorkspace.A[22653] = acadoWorkspace.evHu[453];
acadoWorkspace.A[22654] = acadoWorkspace.evHu[454];
acadoWorkspace.A[22900] = acadoWorkspace.evHu[455];
acadoWorkspace.A[22901] = acadoWorkspace.evHu[456];
acadoWorkspace.A[22902] = acadoWorkspace.evHu[457];
acadoWorkspace.A[22903] = acadoWorkspace.evHu[458];
acadoWorkspace.A[22904] = acadoWorkspace.evHu[459];
acadoWorkspace.A[23150] = acadoWorkspace.evHu[460];
acadoWorkspace.A[23151] = acadoWorkspace.evHu[461];
acadoWorkspace.A[23152] = acadoWorkspace.evHu[462];
acadoWorkspace.A[23153] = acadoWorkspace.evHu[463];
acadoWorkspace.A[23154] = acadoWorkspace.evHu[464];
acadoWorkspace.A[23405] = acadoWorkspace.evHu[465];
acadoWorkspace.A[23406] = acadoWorkspace.evHu[466];
acadoWorkspace.A[23407] = acadoWorkspace.evHu[467];
acadoWorkspace.A[23408] = acadoWorkspace.evHu[468];
acadoWorkspace.A[23409] = acadoWorkspace.evHu[469];
acadoWorkspace.A[23655] = acadoWorkspace.evHu[470];
acadoWorkspace.A[23656] = acadoWorkspace.evHu[471];
acadoWorkspace.A[23657] = acadoWorkspace.evHu[472];
acadoWorkspace.A[23658] = acadoWorkspace.evHu[473];
acadoWorkspace.A[23659] = acadoWorkspace.evHu[474];
acadoWorkspace.A[23905] = acadoWorkspace.evHu[475];
acadoWorkspace.A[23906] = acadoWorkspace.evHu[476];
acadoWorkspace.A[23907] = acadoWorkspace.evHu[477];
acadoWorkspace.A[23908] = acadoWorkspace.evHu[478];
acadoWorkspace.A[23909] = acadoWorkspace.evHu[479];
acadoWorkspace.A[24160] = acadoWorkspace.evHu[480];
acadoWorkspace.A[24161] = acadoWorkspace.evHu[481];
acadoWorkspace.A[24162] = acadoWorkspace.evHu[482];
acadoWorkspace.A[24163] = acadoWorkspace.evHu[483];
acadoWorkspace.A[24164] = acadoWorkspace.evHu[484];
acadoWorkspace.A[24410] = acadoWorkspace.evHu[485];
acadoWorkspace.A[24411] = acadoWorkspace.evHu[486];
acadoWorkspace.A[24412] = acadoWorkspace.evHu[487];
acadoWorkspace.A[24413] = acadoWorkspace.evHu[488];
acadoWorkspace.A[24414] = acadoWorkspace.evHu[489];
acadoWorkspace.A[24660] = acadoWorkspace.evHu[490];
acadoWorkspace.A[24661] = acadoWorkspace.evHu[491];
acadoWorkspace.A[24662] = acadoWorkspace.evHu[492];
acadoWorkspace.A[24663] = acadoWorkspace.evHu[493];
acadoWorkspace.A[24664] = acadoWorkspace.evHu[494];
acadoWorkspace.A[24915] = acadoWorkspace.evHu[495];
acadoWorkspace.A[24916] = acadoWorkspace.evHu[496];
acadoWorkspace.A[24917] = acadoWorkspace.evHu[497];
acadoWorkspace.A[24918] = acadoWorkspace.evHu[498];
acadoWorkspace.A[24919] = acadoWorkspace.evHu[499];
acadoWorkspace.A[25165] = acadoWorkspace.evHu[500];
acadoWorkspace.A[25166] = acadoWorkspace.evHu[501];
acadoWorkspace.A[25167] = acadoWorkspace.evHu[502];
acadoWorkspace.A[25168] = acadoWorkspace.evHu[503];
acadoWorkspace.A[25169] = acadoWorkspace.evHu[504];
acadoWorkspace.A[25415] = acadoWorkspace.evHu[505];
acadoWorkspace.A[25416] = acadoWorkspace.evHu[506];
acadoWorkspace.A[25417] = acadoWorkspace.evHu[507];
acadoWorkspace.A[25418] = acadoWorkspace.evHu[508];
acadoWorkspace.A[25419] = acadoWorkspace.evHu[509];
acadoWorkspace.A[25670] = acadoWorkspace.evHu[510];
acadoWorkspace.A[25671] = acadoWorkspace.evHu[511];
acadoWorkspace.A[25672] = acadoWorkspace.evHu[512];
acadoWorkspace.A[25673] = acadoWorkspace.evHu[513];
acadoWorkspace.A[25674] = acadoWorkspace.evHu[514];
acadoWorkspace.A[25920] = acadoWorkspace.evHu[515];
acadoWorkspace.A[25921] = acadoWorkspace.evHu[516];
acadoWorkspace.A[25922] = acadoWorkspace.evHu[517];
acadoWorkspace.A[25923] = acadoWorkspace.evHu[518];
acadoWorkspace.A[25924] = acadoWorkspace.evHu[519];
acadoWorkspace.A[26170] = acadoWorkspace.evHu[520];
acadoWorkspace.A[26171] = acadoWorkspace.evHu[521];
acadoWorkspace.A[26172] = acadoWorkspace.evHu[522];
acadoWorkspace.A[26173] = acadoWorkspace.evHu[523];
acadoWorkspace.A[26174] = acadoWorkspace.evHu[524];
acadoWorkspace.A[26425] = acadoWorkspace.evHu[525];
acadoWorkspace.A[26426] = acadoWorkspace.evHu[526];
acadoWorkspace.A[26427] = acadoWorkspace.evHu[527];
acadoWorkspace.A[26428] = acadoWorkspace.evHu[528];
acadoWorkspace.A[26429] = acadoWorkspace.evHu[529];
acadoWorkspace.A[26675] = acadoWorkspace.evHu[530];
acadoWorkspace.A[26676] = acadoWorkspace.evHu[531];
acadoWorkspace.A[26677] = acadoWorkspace.evHu[532];
acadoWorkspace.A[26678] = acadoWorkspace.evHu[533];
acadoWorkspace.A[26679] = acadoWorkspace.evHu[534];
acadoWorkspace.A[26925] = acadoWorkspace.evHu[535];
acadoWorkspace.A[26926] = acadoWorkspace.evHu[536];
acadoWorkspace.A[26927] = acadoWorkspace.evHu[537];
acadoWorkspace.A[26928] = acadoWorkspace.evHu[538];
acadoWorkspace.A[26929] = acadoWorkspace.evHu[539];
acadoWorkspace.A[27180] = acadoWorkspace.evHu[540];
acadoWorkspace.A[27181] = acadoWorkspace.evHu[541];
acadoWorkspace.A[27182] = acadoWorkspace.evHu[542];
acadoWorkspace.A[27183] = acadoWorkspace.evHu[543];
acadoWorkspace.A[27184] = acadoWorkspace.evHu[544];
acadoWorkspace.A[27430] = acadoWorkspace.evHu[545];
acadoWorkspace.A[27431] = acadoWorkspace.evHu[546];
acadoWorkspace.A[27432] = acadoWorkspace.evHu[547];
acadoWorkspace.A[27433] = acadoWorkspace.evHu[548];
acadoWorkspace.A[27434] = acadoWorkspace.evHu[549];
acadoWorkspace.A[27680] = acadoWorkspace.evHu[550];
acadoWorkspace.A[27681] = acadoWorkspace.evHu[551];
acadoWorkspace.A[27682] = acadoWorkspace.evHu[552];
acadoWorkspace.A[27683] = acadoWorkspace.evHu[553];
acadoWorkspace.A[27684] = acadoWorkspace.evHu[554];
acadoWorkspace.A[27935] = acadoWorkspace.evHu[555];
acadoWorkspace.A[27936] = acadoWorkspace.evHu[556];
acadoWorkspace.A[27937] = acadoWorkspace.evHu[557];
acadoWorkspace.A[27938] = acadoWorkspace.evHu[558];
acadoWorkspace.A[27939] = acadoWorkspace.evHu[559];
acadoWorkspace.A[28185] = acadoWorkspace.evHu[560];
acadoWorkspace.A[28186] = acadoWorkspace.evHu[561];
acadoWorkspace.A[28187] = acadoWorkspace.evHu[562];
acadoWorkspace.A[28188] = acadoWorkspace.evHu[563];
acadoWorkspace.A[28189] = acadoWorkspace.evHu[564];
acadoWorkspace.A[28435] = acadoWorkspace.evHu[565];
acadoWorkspace.A[28436] = acadoWorkspace.evHu[566];
acadoWorkspace.A[28437] = acadoWorkspace.evHu[567];
acadoWorkspace.A[28438] = acadoWorkspace.evHu[568];
acadoWorkspace.A[28439] = acadoWorkspace.evHu[569];
acadoWorkspace.A[28690] = acadoWorkspace.evHu[570];
acadoWorkspace.A[28691] = acadoWorkspace.evHu[571];
acadoWorkspace.A[28692] = acadoWorkspace.evHu[572];
acadoWorkspace.A[28693] = acadoWorkspace.evHu[573];
acadoWorkspace.A[28694] = acadoWorkspace.evHu[574];
acadoWorkspace.A[28940] = acadoWorkspace.evHu[575];
acadoWorkspace.A[28941] = acadoWorkspace.evHu[576];
acadoWorkspace.A[28942] = acadoWorkspace.evHu[577];
acadoWorkspace.A[28943] = acadoWorkspace.evHu[578];
acadoWorkspace.A[28944] = acadoWorkspace.evHu[579];
acadoWorkspace.A[29190] = acadoWorkspace.evHu[580];
acadoWorkspace.A[29191] = acadoWorkspace.evHu[581];
acadoWorkspace.A[29192] = acadoWorkspace.evHu[582];
acadoWorkspace.A[29193] = acadoWorkspace.evHu[583];
acadoWorkspace.A[29194] = acadoWorkspace.evHu[584];
acadoWorkspace.A[29445] = acadoWorkspace.evHu[585];
acadoWorkspace.A[29446] = acadoWorkspace.evHu[586];
acadoWorkspace.A[29447] = acadoWorkspace.evHu[587];
acadoWorkspace.A[29448] = acadoWorkspace.evHu[588];
acadoWorkspace.A[29449] = acadoWorkspace.evHu[589];
acadoWorkspace.A[29695] = acadoWorkspace.evHu[590];
acadoWorkspace.A[29696] = acadoWorkspace.evHu[591];
acadoWorkspace.A[29697] = acadoWorkspace.evHu[592];
acadoWorkspace.A[29698] = acadoWorkspace.evHu[593];
acadoWorkspace.A[29699] = acadoWorkspace.evHu[594];
acadoWorkspace.A[29945] = acadoWorkspace.evHu[595];
acadoWorkspace.A[29946] = acadoWorkspace.evHu[596];
acadoWorkspace.A[29947] = acadoWorkspace.evHu[597];
acadoWorkspace.A[29948] = acadoWorkspace.evHu[598];
acadoWorkspace.A[29949] = acadoWorkspace.evHu[599];
acadoWorkspace.A[30200] = acadoWorkspace.evHu[600];
acadoWorkspace.A[30201] = acadoWorkspace.evHu[601];
acadoWorkspace.A[30202] = acadoWorkspace.evHu[602];
acadoWorkspace.A[30203] = acadoWorkspace.evHu[603];
acadoWorkspace.A[30204] = acadoWorkspace.evHu[604];
acadoWorkspace.A[30450] = acadoWorkspace.evHu[605];
acadoWorkspace.A[30451] = acadoWorkspace.evHu[606];
acadoWorkspace.A[30452] = acadoWorkspace.evHu[607];
acadoWorkspace.A[30453] = acadoWorkspace.evHu[608];
acadoWorkspace.A[30454] = acadoWorkspace.evHu[609];
acadoWorkspace.A[30700] = acadoWorkspace.evHu[610];
acadoWorkspace.A[30701] = acadoWorkspace.evHu[611];
acadoWorkspace.A[30702] = acadoWorkspace.evHu[612];
acadoWorkspace.A[30703] = acadoWorkspace.evHu[613];
acadoWorkspace.A[30704] = acadoWorkspace.evHu[614];
acadoWorkspace.A[30955] = acadoWorkspace.evHu[615];
acadoWorkspace.A[30956] = acadoWorkspace.evHu[616];
acadoWorkspace.A[30957] = acadoWorkspace.evHu[617];
acadoWorkspace.A[30958] = acadoWorkspace.evHu[618];
acadoWorkspace.A[30959] = acadoWorkspace.evHu[619];
acadoWorkspace.A[31205] = acadoWorkspace.evHu[620];
acadoWorkspace.A[31206] = acadoWorkspace.evHu[621];
acadoWorkspace.A[31207] = acadoWorkspace.evHu[622];
acadoWorkspace.A[31208] = acadoWorkspace.evHu[623];
acadoWorkspace.A[31209] = acadoWorkspace.evHu[624];
acadoWorkspace.A[31455] = acadoWorkspace.evHu[625];
acadoWorkspace.A[31456] = acadoWorkspace.evHu[626];
acadoWorkspace.A[31457] = acadoWorkspace.evHu[627];
acadoWorkspace.A[31458] = acadoWorkspace.evHu[628];
acadoWorkspace.A[31459] = acadoWorkspace.evHu[629];
acadoWorkspace.A[31710] = acadoWorkspace.evHu[630];
acadoWorkspace.A[31711] = acadoWorkspace.evHu[631];
acadoWorkspace.A[31712] = acadoWorkspace.evHu[632];
acadoWorkspace.A[31713] = acadoWorkspace.evHu[633];
acadoWorkspace.A[31714] = acadoWorkspace.evHu[634];
acadoWorkspace.A[31960] = acadoWorkspace.evHu[635];
acadoWorkspace.A[31961] = acadoWorkspace.evHu[636];
acadoWorkspace.A[31962] = acadoWorkspace.evHu[637];
acadoWorkspace.A[31963] = acadoWorkspace.evHu[638];
acadoWorkspace.A[31964] = acadoWorkspace.evHu[639];
acadoWorkspace.A[32210] = acadoWorkspace.evHu[640];
acadoWorkspace.A[32211] = acadoWorkspace.evHu[641];
acadoWorkspace.A[32212] = acadoWorkspace.evHu[642];
acadoWorkspace.A[32213] = acadoWorkspace.evHu[643];
acadoWorkspace.A[32214] = acadoWorkspace.evHu[644];
acadoWorkspace.A[32465] = acadoWorkspace.evHu[645];
acadoWorkspace.A[32466] = acadoWorkspace.evHu[646];
acadoWorkspace.A[32467] = acadoWorkspace.evHu[647];
acadoWorkspace.A[32468] = acadoWorkspace.evHu[648];
acadoWorkspace.A[32469] = acadoWorkspace.evHu[649];
acadoWorkspace.A[32715] = acadoWorkspace.evHu[650];
acadoWorkspace.A[32716] = acadoWorkspace.evHu[651];
acadoWorkspace.A[32717] = acadoWorkspace.evHu[652];
acadoWorkspace.A[32718] = acadoWorkspace.evHu[653];
acadoWorkspace.A[32719] = acadoWorkspace.evHu[654];
acadoWorkspace.A[32965] = acadoWorkspace.evHu[655];
acadoWorkspace.A[32966] = acadoWorkspace.evHu[656];
acadoWorkspace.A[32967] = acadoWorkspace.evHu[657];
acadoWorkspace.A[32968] = acadoWorkspace.evHu[658];
acadoWorkspace.A[32969] = acadoWorkspace.evHu[659];
acadoWorkspace.A[33220] = acadoWorkspace.evHu[660];
acadoWorkspace.A[33221] = acadoWorkspace.evHu[661];
acadoWorkspace.A[33222] = acadoWorkspace.evHu[662];
acadoWorkspace.A[33223] = acadoWorkspace.evHu[663];
acadoWorkspace.A[33224] = acadoWorkspace.evHu[664];
acadoWorkspace.A[33470] = acadoWorkspace.evHu[665];
acadoWorkspace.A[33471] = acadoWorkspace.evHu[666];
acadoWorkspace.A[33472] = acadoWorkspace.evHu[667];
acadoWorkspace.A[33473] = acadoWorkspace.evHu[668];
acadoWorkspace.A[33474] = acadoWorkspace.evHu[669];
acadoWorkspace.A[33720] = acadoWorkspace.evHu[670];
acadoWorkspace.A[33721] = acadoWorkspace.evHu[671];
acadoWorkspace.A[33722] = acadoWorkspace.evHu[672];
acadoWorkspace.A[33723] = acadoWorkspace.evHu[673];
acadoWorkspace.A[33724] = acadoWorkspace.evHu[674];
acadoWorkspace.A[33975] = acadoWorkspace.evHu[675];
acadoWorkspace.A[33976] = acadoWorkspace.evHu[676];
acadoWorkspace.A[33977] = acadoWorkspace.evHu[677];
acadoWorkspace.A[33978] = acadoWorkspace.evHu[678];
acadoWorkspace.A[33979] = acadoWorkspace.evHu[679];
acadoWorkspace.A[34225] = acadoWorkspace.evHu[680];
acadoWorkspace.A[34226] = acadoWorkspace.evHu[681];
acadoWorkspace.A[34227] = acadoWorkspace.evHu[682];
acadoWorkspace.A[34228] = acadoWorkspace.evHu[683];
acadoWorkspace.A[34229] = acadoWorkspace.evHu[684];
acadoWorkspace.A[34475] = acadoWorkspace.evHu[685];
acadoWorkspace.A[34476] = acadoWorkspace.evHu[686];
acadoWorkspace.A[34477] = acadoWorkspace.evHu[687];
acadoWorkspace.A[34478] = acadoWorkspace.evHu[688];
acadoWorkspace.A[34479] = acadoWorkspace.evHu[689];
acadoWorkspace.A[34730] = acadoWorkspace.evHu[690];
acadoWorkspace.A[34731] = acadoWorkspace.evHu[691];
acadoWorkspace.A[34732] = acadoWorkspace.evHu[692];
acadoWorkspace.A[34733] = acadoWorkspace.evHu[693];
acadoWorkspace.A[34734] = acadoWorkspace.evHu[694];
acadoWorkspace.A[34980] = acadoWorkspace.evHu[695];
acadoWorkspace.A[34981] = acadoWorkspace.evHu[696];
acadoWorkspace.A[34982] = acadoWorkspace.evHu[697];
acadoWorkspace.A[34983] = acadoWorkspace.evHu[698];
acadoWorkspace.A[34984] = acadoWorkspace.evHu[699];
acadoWorkspace.A[35230] = acadoWorkspace.evHu[700];
acadoWorkspace.A[35231] = acadoWorkspace.evHu[701];
acadoWorkspace.A[35232] = acadoWorkspace.evHu[702];
acadoWorkspace.A[35233] = acadoWorkspace.evHu[703];
acadoWorkspace.A[35234] = acadoWorkspace.evHu[704];
acadoWorkspace.A[35485] = acadoWorkspace.evHu[705];
acadoWorkspace.A[35486] = acadoWorkspace.evHu[706];
acadoWorkspace.A[35487] = acadoWorkspace.evHu[707];
acadoWorkspace.A[35488] = acadoWorkspace.evHu[708];
acadoWorkspace.A[35489] = acadoWorkspace.evHu[709];
acadoWorkspace.A[35735] = acadoWorkspace.evHu[710];
acadoWorkspace.A[35736] = acadoWorkspace.evHu[711];
acadoWorkspace.A[35737] = acadoWorkspace.evHu[712];
acadoWorkspace.A[35738] = acadoWorkspace.evHu[713];
acadoWorkspace.A[35739] = acadoWorkspace.evHu[714];
acadoWorkspace.A[35985] = acadoWorkspace.evHu[715];
acadoWorkspace.A[35986] = acadoWorkspace.evHu[716];
acadoWorkspace.A[35987] = acadoWorkspace.evHu[717];
acadoWorkspace.A[35988] = acadoWorkspace.evHu[718];
acadoWorkspace.A[35989] = acadoWorkspace.evHu[719];
acadoWorkspace.A[36240] = acadoWorkspace.evHu[720];
acadoWorkspace.A[36241] = acadoWorkspace.evHu[721];
acadoWorkspace.A[36242] = acadoWorkspace.evHu[722];
acadoWorkspace.A[36243] = acadoWorkspace.evHu[723];
acadoWorkspace.A[36244] = acadoWorkspace.evHu[724];
acadoWorkspace.A[36490] = acadoWorkspace.evHu[725];
acadoWorkspace.A[36491] = acadoWorkspace.evHu[726];
acadoWorkspace.A[36492] = acadoWorkspace.evHu[727];
acadoWorkspace.A[36493] = acadoWorkspace.evHu[728];
acadoWorkspace.A[36494] = acadoWorkspace.evHu[729];
acadoWorkspace.A[36740] = acadoWorkspace.evHu[730];
acadoWorkspace.A[36741] = acadoWorkspace.evHu[731];
acadoWorkspace.A[36742] = acadoWorkspace.evHu[732];
acadoWorkspace.A[36743] = acadoWorkspace.evHu[733];
acadoWorkspace.A[36744] = acadoWorkspace.evHu[734];
acadoWorkspace.A[36995] = acadoWorkspace.evHu[735];
acadoWorkspace.A[36996] = acadoWorkspace.evHu[736];
acadoWorkspace.A[36997] = acadoWorkspace.evHu[737];
acadoWorkspace.A[36998] = acadoWorkspace.evHu[738];
acadoWorkspace.A[36999] = acadoWorkspace.evHu[739];
acadoWorkspace.A[37245] = acadoWorkspace.evHu[740];
acadoWorkspace.A[37246] = acadoWorkspace.evHu[741];
acadoWorkspace.A[37247] = acadoWorkspace.evHu[742];
acadoWorkspace.A[37248] = acadoWorkspace.evHu[743];
acadoWorkspace.A[37249] = acadoWorkspace.evHu[744];
acadoWorkspace.A[37495] = acadoWorkspace.evHu[745];
acadoWorkspace.A[37496] = acadoWorkspace.evHu[746];
acadoWorkspace.A[37497] = acadoWorkspace.evHu[747];
acadoWorkspace.A[37498] = acadoWorkspace.evHu[748];
acadoWorkspace.A[37499] = acadoWorkspace.evHu[749];
acadoWorkspace.lbA[0] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[79];
acadoWorkspace.lbA[80] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[80];
acadoWorkspace.lbA[81] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[81];
acadoWorkspace.lbA[82] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[82];
acadoWorkspace.lbA[83] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[83];
acadoWorkspace.lbA[84] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[84];
acadoWorkspace.lbA[85] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[85];
acadoWorkspace.lbA[86] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[86];
acadoWorkspace.lbA[87] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[87];
acadoWorkspace.lbA[88] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[88];
acadoWorkspace.lbA[89] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[89];
acadoWorkspace.lbA[90] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[90];
acadoWorkspace.lbA[91] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[91];
acadoWorkspace.lbA[92] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[92];
acadoWorkspace.lbA[93] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[93];
acadoWorkspace.lbA[94] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[94];
acadoWorkspace.lbA[95] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[95];
acadoWorkspace.lbA[96] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[96];
acadoWorkspace.lbA[97] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[97];
acadoWorkspace.lbA[98] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[98];
acadoWorkspace.lbA[99] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[99];
acadoWorkspace.lbA[100] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[100];
acadoWorkspace.lbA[101] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[101];
acadoWorkspace.lbA[102] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[102];
acadoWorkspace.lbA[103] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[103];
acadoWorkspace.lbA[104] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[104];
acadoWorkspace.lbA[105] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[105];
acadoWorkspace.lbA[106] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[106];
acadoWorkspace.lbA[107] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[107];
acadoWorkspace.lbA[108] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[108];
acadoWorkspace.lbA[109] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[109];
acadoWorkspace.lbA[110] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[110];
acadoWorkspace.lbA[111] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[111];
acadoWorkspace.lbA[112] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[112];
acadoWorkspace.lbA[113] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[113];
acadoWorkspace.lbA[114] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[114];
acadoWorkspace.lbA[115] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[115];
acadoWorkspace.lbA[116] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[116];
acadoWorkspace.lbA[117] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[117];
acadoWorkspace.lbA[118] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[118];
acadoWorkspace.lbA[119] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[119];
acadoWorkspace.lbA[120] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[120];
acadoWorkspace.lbA[121] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[121];
acadoWorkspace.lbA[122] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[122];
acadoWorkspace.lbA[123] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[123];
acadoWorkspace.lbA[124] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[124];
acadoWorkspace.lbA[125] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[125];
acadoWorkspace.lbA[126] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[126];
acadoWorkspace.lbA[127] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[127];
acadoWorkspace.lbA[128] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[128];
acadoWorkspace.lbA[129] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[129];
acadoWorkspace.lbA[130] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[130];
acadoWorkspace.lbA[131] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[131];
acadoWorkspace.lbA[132] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[132];
acadoWorkspace.lbA[133] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[133];
acadoWorkspace.lbA[134] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[134];
acadoWorkspace.lbA[135] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[135];
acadoWorkspace.lbA[136] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[136];
acadoWorkspace.lbA[137] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[137];
acadoWorkspace.lbA[138] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[138];
acadoWorkspace.lbA[139] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[139];
acadoWorkspace.lbA[140] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[140];
acadoWorkspace.lbA[141] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[141];
acadoWorkspace.lbA[142] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[142];
acadoWorkspace.lbA[143] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[143];
acadoWorkspace.lbA[144] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[144];
acadoWorkspace.lbA[145] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[145];
acadoWorkspace.lbA[146] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[146];
acadoWorkspace.lbA[147] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[147];
acadoWorkspace.lbA[148] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[148];
acadoWorkspace.lbA[149] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[149];

acadoWorkspace.ubA[0] = - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = - acadoWorkspace.evH[79];
acadoWorkspace.ubA[80] = - acadoWorkspace.evH[80];
acadoWorkspace.ubA[81] = - acadoWorkspace.evH[81];
acadoWorkspace.ubA[82] = - acadoWorkspace.evH[82];
acadoWorkspace.ubA[83] = - acadoWorkspace.evH[83];
acadoWorkspace.ubA[84] = - acadoWorkspace.evH[84];
acadoWorkspace.ubA[85] = - acadoWorkspace.evH[85];
acadoWorkspace.ubA[86] = - acadoWorkspace.evH[86];
acadoWorkspace.ubA[87] = - acadoWorkspace.evH[87];
acadoWorkspace.ubA[88] = - acadoWorkspace.evH[88];
acadoWorkspace.ubA[89] = - acadoWorkspace.evH[89];
acadoWorkspace.ubA[90] = - acadoWorkspace.evH[90];
acadoWorkspace.ubA[91] = - acadoWorkspace.evH[91];
acadoWorkspace.ubA[92] = - acadoWorkspace.evH[92];
acadoWorkspace.ubA[93] = - acadoWorkspace.evH[93];
acadoWorkspace.ubA[94] = - acadoWorkspace.evH[94];
acadoWorkspace.ubA[95] = - acadoWorkspace.evH[95];
acadoWorkspace.ubA[96] = - acadoWorkspace.evH[96];
acadoWorkspace.ubA[97] = - acadoWorkspace.evH[97];
acadoWorkspace.ubA[98] = - acadoWorkspace.evH[98];
acadoWorkspace.ubA[99] = - acadoWorkspace.evH[99];
acadoWorkspace.ubA[100] = - acadoWorkspace.evH[100];
acadoWorkspace.ubA[101] = - acadoWorkspace.evH[101];
acadoWorkspace.ubA[102] = - acadoWorkspace.evH[102];
acadoWorkspace.ubA[103] = - acadoWorkspace.evH[103];
acadoWorkspace.ubA[104] = - acadoWorkspace.evH[104];
acadoWorkspace.ubA[105] = - acadoWorkspace.evH[105];
acadoWorkspace.ubA[106] = - acadoWorkspace.evH[106];
acadoWorkspace.ubA[107] = - acadoWorkspace.evH[107];
acadoWorkspace.ubA[108] = - acadoWorkspace.evH[108];
acadoWorkspace.ubA[109] = - acadoWorkspace.evH[109];
acadoWorkspace.ubA[110] = - acadoWorkspace.evH[110];
acadoWorkspace.ubA[111] = - acadoWorkspace.evH[111];
acadoWorkspace.ubA[112] = - acadoWorkspace.evH[112];
acadoWorkspace.ubA[113] = - acadoWorkspace.evH[113];
acadoWorkspace.ubA[114] = - acadoWorkspace.evH[114];
acadoWorkspace.ubA[115] = - acadoWorkspace.evH[115];
acadoWorkspace.ubA[116] = - acadoWorkspace.evH[116];
acadoWorkspace.ubA[117] = - acadoWorkspace.evH[117];
acadoWorkspace.ubA[118] = - acadoWorkspace.evH[118];
acadoWorkspace.ubA[119] = - acadoWorkspace.evH[119];
acadoWorkspace.ubA[120] = - acadoWorkspace.evH[120];
acadoWorkspace.ubA[121] = - acadoWorkspace.evH[121];
acadoWorkspace.ubA[122] = - acadoWorkspace.evH[122];
acadoWorkspace.ubA[123] = - acadoWorkspace.evH[123];
acadoWorkspace.ubA[124] = - acadoWorkspace.evH[124];
acadoWorkspace.ubA[125] = - acadoWorkspace.evH[125];
acadoWorkspace.ubA[126] = - acadoWorkspace.evH[126];
acadoWorkspace.ubA[127] = - acadoWorkspace.evH[127];
acadoWorkspace.ubA[128] = - acadoWorkspace.evH[128];
acadoWorkspace.ubA[129] = - acadoWorkspace.evH[129];
acadoWorkspace.ubA[130] = - acadoWorkspace.evH[130];
acadoWorkspace.ubA[131] = - acadoWorkspace.evH[131];
acadoWorkspace.ubA[132] = - acadoWorkspace.evH[132];
acadoWorkspace.ubA[133] = - acadoWorkspace.evH[133];
acadoWorkspace.ubA[134] = - acadoWorkspace.evH[134];
acadoWorkspace.ubA[135] = - acadoWorkspace.evH[135];
acadoWorkspace.ubA[136] = - acadoWorkspace.evH[136];
acadoWorkspace.ubA[137] = - acadoWorkspace.evH[137];
acadoWorkspace.ubA[138] = - acadoWorkspace.evH[138];
acadoWorkspace.ubA[139] = - acadoWorkspace.evH[139];
acadoWorkspace.ubA[140] = - acadoWorkspace.evH[140];
acadoWorkspace.ubA[141] = - acadoWorkspace.evH[141];
acadoWorkspace.ubA[142] = - acadoWorkspace.evH[142];
acadoWorkspace.ubA[143] = - acadoWorkspace.evH[143];
acadoWorkspace.ubA[144] = - acadoWorkspace.evH[144];
acadoWorkspace.ubA[145] = - acadoWorkspace.evH[145];
acadoWorkspace.ubA[146] = - acadoWorkspace.evH[146];
acadoWorkspace.ubA[147] = - acadoWorkspace.evH[147];
acadoWorkspace.ubA[148] = - acadoWorkspace.evH[148];
acadoWorkspace.ubA[149] = - acadoWorkspace.evH[149];

acado_macHxd( &(acadoWorkspace.evHx[ 9 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.d[ 3 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 27 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 45 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 63 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 81 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 99 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.d[ 33 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 117 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 39 ]), &(acadoWorkspace.ubA[ 39 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.d[ 39 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 135 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.lbA[ 45 ]), &(acadoWorkspace.ubA[ 45 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 153 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 51 ]), &(acadoWorkspace.ubA[ 51 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 162 ]), &(acadoWorkspace.d[ 51 ]), &(acadoWorkspace.lbA[ 54 ]), &(acadoWorkspace.ubA[ 54 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 171 ]), &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.lbA[ 57 ]), &(acadoWorkspace.ubA[ 57 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 57 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 189 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 63 ]), &(acadoWorkspace.ubA[ 63 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 198 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.lbA[ 66 ]), &(acadoWorkspace.ubA[ 66 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 207 ]), &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.lbA[ 69 ]), &(acadoWorkspace.ubA[ 69 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.d[ 69 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 225 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.lbA[ 75 ]), &(acadoWorkspace.ubA[ 75 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 234 ]), &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 243 ]), &(acadoWorkspace.d[ 78 ]), &(acadoWorkspace.lbA[ 81 ]), &(acadoWorkspace.ubA[ 81 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.d[ 81 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 261 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.lbA[ 87 ]), &(acadoWorkspace.ubA[ 87 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.d[ 87 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 279 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.lbA[ 93 ]), &(acadoWorkspace.ubA[ 93 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.d[ 93 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 297 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.lbA[ 99 ]), &(acadoWorkspace.ubA[ 99 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 306 ]), &(acadoWorkspace.d[ 99 ]), &(acadoWorkspace.lbA[ 102 ]), &(acadoWorkspace.ubA[ 102 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 315 ]), &(acadoWorkspace.d[ 102 ]), &(acadoWorkspace.lbA[ 105 ]), &(acadoWorkspace.ubA[ 105 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 324 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.lbA[ 108 ]), &(acadoWorkspace.ubA[ 108 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 333 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.lbA[ 111 ]), &(acadoWorkspace.ubA[ 111 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 342 ]), &(acadoWorkspace.d[ 111 ]), &(acadoWorkspace.lbA[ 114 ]), &(acadoWorkspace.ubA[ 114 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 351 ]), &(acadoWorkspace.d[ 114 ]), &(acadoWorkspace.lbA[ 117 ]), &(acadoWorkspace.ubA[ 117 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.d[ 117 ]), &(acadoWorkspace.lbA[ 120 ]), &(acadoWorkspace.ubA[ 120 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 369 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.lbA[ 123 ]), &(acadoWorkspace.ubA[ 123 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 378 ]), &(acadoWorkspace.d[ 123 ]), &(acadoWorkspace.lbA[ 126 ]), &(acadoWorkspace.ubA[ 126 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 387 ]), &(acadoWorkspace.d[ 126 ]), &(acadoWorkspace.lbA[ 129 ]), &(acadoWorkspace.ubA[ 129 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 396 ]), &(acadoWorkspace.d[ 129 ]), &(acadoWorkspace.lbA[ 132 ]), &(acadoWorkspace.ubA[ 132 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 405 ]), &(acadoWorkspace.d[ 132 ]), &(acadoWorkspace.lbA[ 135 ]), &(acadoWorkspace.ubA[ 135 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 414 ]), &(acadoWorkspace.d[ 135 ]), &(acadoWorkspace.lbA[ 138 ]), &(acadoWorkspace.ubA[ 138 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 423 ]), &(acadoWorkspace.d[ 138 ]), &(acadoWorkspace.lbA[ 141 ]), &(acadoWorkspace.ubA[ 141 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 432 ]), &(acadoWorkspace.d[ 141 ]), &(acadoWorkspace.lbA[ 144 ]), &(acadoWorkspace.ubA[ 144 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 441 ]), &(acadoWorkspace.d[ 144 ]), &(acadoWorkspace.lbA[ 147 ]), &(acadoWorkspace.ubA[ 147 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];

for (lRun2 = 0; lRun2 < 350; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 35 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 105 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 175 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 245 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.g[ 35 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 315 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 350 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 385 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 55 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 420 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 455 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.g[ 65 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 490 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 70 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 525 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 560 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 595 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.g[ 85 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 630 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 665 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.g[ 95 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 700 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 100 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 735 ]), &(acadoWorkspace.Dy[ 147 ]), &(acadoWorkspace.g[ 105 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 770 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 110 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 805 ]), &(acadoWorkspace.Dy[ 161 ]), &(acadoWorkspace.g[ 115 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 840 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 120 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 875 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.g[ 125 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 910 ]), &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.g[ 130 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 945 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.g[ 135 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 980 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.g[ 140 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1015 ]), &(acadoWorkspace.Dy[ 203 ]), &(acadoWorkspace.g[ 145 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1050 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 150 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1085 ]), &(acadoWorkspace.Dy[ 217 ]), &(acadoWorkspace.g[ 155 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1120 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 160 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1155 ]), &(acadoWorkspace.Dy[ 231 ]), &(acadoWorkspace.g[ 165 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1190 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.g[ 170 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1225 ]), &(acadoWorkspace.Dy[ 245 ]), &(acadoWorkspace.g[ 175 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1260 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 180 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1295 ]), &(acadoWorkspace.Dy[ 259 ]), &(acadoWorkspace.g[ 185 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1330 ]), &(acadoWorkspace.Dy[ 266 ]), &(acadoWorkspace.g[ 190 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1365 ]), &(acadoWorkspace.Dy[ 273 ]), &(acadoWorkspace.g[ 195 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1400 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.g[ 200 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1435 ]), &(acadoWorkspace.Dy[ 287 ]), &(acadoWorkspace.g[ 205 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1470 ]), &(acadoWorkspace.Dy[ 294 ]), &(acadoWorkspace.g[ 210 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1505 ]), &(acadoWorkspace.Dy[ 301 ]), &(acadoWorkspace.g[ 215 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1540 ]), &(acadoWorkspace.Dy[ 308 ]), &(acadoWorkspace.g[ 220 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1575 ]), &(acadoWorkspace.Dy[ 315 ]), &(acadoWorkspace.g[ 225 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1610 ]), &(acadoWorkspace.Dy[ 322 ]), &(acadoWorkspace.g[ 230 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1645 ]), &(acadoWorkspace.Dy[ 329 ]), &(acadoWorkspace.g[ 235 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1680 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.g[ 240 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1715 ]), &(acadoWorkspace.Dy[ 343 ]), &(acadoWorkspace.g[ 245 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 21 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 42 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 63 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 105 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 126 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 147 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 189 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 231 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 273 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 294 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 315 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 357 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.QDy[ 51 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 378 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 399 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.QDy[ 57 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 441 ]), &(acadoWorkspace.Dy[ 147 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 462 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 483 ]), &(acadoWorkspace.Dy[ 161 ]), &(acadoWorkspace.QDy[ 69 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 525 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 546 ]), &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 567 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 588 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 609 ]), &(acadoWorkspace.Dy[ 203 ]), &(acadoWorkspace.QDy[ 87 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 630 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 651 ]), &(acadoWorkspace.Dy[ 217 ]), &(acadoWorkspace.QDy[ 93 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 693 ]), &(acadoWorkspace.Dy[ 231 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 714 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 735 ]), &(acadoWorkspace.Dy[ 245 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 777 ]), &(acadoWorkspace.Dy[ 259 ]), &(acadoWorkspace.QDy[ 111 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 798 ]), &(acadoWorkspace.Dy[ 266 ]), &(acadoWorkspace.QDy[ 114 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 819 ]), &(acadoWorkspace.Dy[ 273 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 861 ]), &(acadoWorkspace.Dy[ 287 ]), &(acadoWorkspace.QDy[ 123 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 882 ]), &(acadoWorkspace.Dy[ 294 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 903 ]), &(acadoWorkspace.Dy[ 301 ]), &(acadoWorkspace.QDy[ 129 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 924 ]), &(acadoWorkspace.Dy[ 308 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 945 ]), &(acadoWorkspace.Dy[ 315 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 966 ]), &(acadoWorkspace.Dy[ 322 ]), &(acadoWorkspace.QDy[ 138 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 987 ]), &(acadoWorkspace.Dy[ 329 ]), &(acadoWorkspace.QDy[ 141 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1008 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1029 ]), &(acadoWorkspace.Dy[ 343 ]), &(acadoWorkspace.QDy[ 147 ]) );

acadoWorkspace.QDy[150] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[151] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[152] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1];

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.QDy[lRun2 + 3] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 50; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 15 ]), &(acadoWorkspace.QDy[ lRun2 * 3 + 3 ]), &(acadoWorkspace.g[ lRun1 * 5 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[1] += + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[2] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[3] += + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[4] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[5] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[6] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[7] += + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[8] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[9] += + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[10] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[11] += + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[12] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[13] += + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[14] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[15] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[16] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[17] += + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[18] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[19] += + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[20] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[21] += + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[22] += + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[23] += + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[24] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[25] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[26] += + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[27] += + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[28] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[29] += + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[30] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[31] += + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[32] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[33] += + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[34] += + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[35] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[36] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[37] += + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[38] += + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[39] += + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[40] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[41] += + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[42] += + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[43] += + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[44] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[45] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[46] += + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[47] += + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[48] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[49] += + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[50] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[51] += + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[52] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[53] += + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[54] += + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[55] += + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[56] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[57] += + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[58] += + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[59] += + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[60] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[61] += + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[62] += + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[63] += + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[64] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[65] += + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[66] += + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[67] += + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[68] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[69] += + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[70] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[71] += + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[72] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[73] += + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[74] += + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[75] += + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[76] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[77] += + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[78] += + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[79] += + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[80] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[81] += + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[82] += + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[83] += + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[84] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[85] += + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[86] += + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[87] += + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[88] += + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[89] += + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[90] += + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[91] += + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[92] += + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[93] += + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[94] += + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[95] += + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[96] += + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[97] += + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[98] += + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[99] += + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[100] += + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[101] += + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[102] += + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[103] += + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[104] += + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[105] += + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[106] += + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[320]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[107] += + acadoWorkspace.H10[321]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[322]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[323]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[108] += + acadoWorkspace.H10[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[326]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[109] += + acadoWorkspace.H10[327]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[328]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[329]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[110] += + acadoWorkspace.H10[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[332]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[111] += + acadoWorkspace.H10[333]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[334]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[335]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[112] += + acadoWorkspace.H10[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[338]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[113] += + acadoWorkspace.H10[339]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[340]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[341]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[114] += + acadoWorkspace.H10[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[344]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[115] += + acadoWorkspace.H10[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[347]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[116] += + acadoWorkspace.H10[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[350]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[117] += + acadoWorkspace.H10[351]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[352]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[353]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[118] += + acadoWorkspace.H10[354]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[355]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[356]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[119] += + acadoWorkspace.H10[357]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[358]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[359]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[120] += + acadoWorkspace.H10[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[362]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[121] += + acadoWorkspace.H10[363]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[364]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[365]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[122] += + acadoWorkspace.H10[366]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[367]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[368]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[123] += + acadoWorkspace.H10[369]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[370]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[371]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[124] += + acadoWorkspace.H10[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[374]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[125] += + acadoWorkspace.H10[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[377]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[126] += + acadoWorkspace.H10[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[380]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[127] += + acadoWorkspace.H10[381]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[382]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[383]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[128] += + acadoWorkspace.H10[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[386]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[129] += + acadoWorkspace.H10[387]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[388]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[389]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[130] += + acadoWorkspace.H10[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[392]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[131] += + acadoWorkspace.H10[393]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[394]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[395]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[132] += + acadoWorkspace.H10[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[398]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[133] += + acadoWorkspace.H10[399]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[400]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[401]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[134] += + acadoWorkspace.H10[402]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[403]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[404]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[135] += + acadoWorkspace.H10[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[407]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[136] += + acadoWorkspace.H10[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[410]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[137] += + acadoWorkspace.H10[411]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[412]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[413]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[138] += + acadoWorkspace.H10[414]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[415]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[416]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[139] += + acadoWorkspace.H10[417]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[418]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[419]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[140] += + acadoWorkspace.H10[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[422]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[141] += + acadoWorkspace.H10[423]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[424]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[425]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[142] += + acadoWorkspace.H10[426]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[427]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[428]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[143] += + acadoWorkspace.H10[429]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[430]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[431]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[144] += + acadoWorkspace.H10[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[434]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[145] += + acadoWorkspace.H10[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[437]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[146] += + acadoWorkspace.H10[438]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[439]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[440]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[147] += + acadoWorkspace.H10[441]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[442]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[443]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[148] += + acadoWorkspace.H10[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[446]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[149] += + acadoWorkspace.H10[447]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[448]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[449]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[150] += + acadoWorkspace.H10[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[452]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[151] += + acadoWorkspace.H10[453]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[454]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[455]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[152] += + acadoWorkspace.H10[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[458]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[153] += + acadoWorkspace.H10[459]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[460]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[461]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[154] += + acadoWorkspace.H10[462]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[463]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[464]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[155] += + acadoWorkspace.H10[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[467]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[156] += + acadoWorkspace.H10[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[470]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[157] += + acadoWorkspace.H10[471]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[472]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[473]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[158] += + acadoWorkspace.H10[474]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[475]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[476]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[159] += + acadoWorkspace.H10[477]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[478]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[479]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[160] += + acadoWorkspace.H10[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[482]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[161] += + acadoWorkspace.H10[483]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[484]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[485]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[162] += + acadoWorkspace.H10[486]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[487]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[488]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[163] += + acadoWorkspace.H10[489]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[490]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[491]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[164] += + acadoWorkspace.H10[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[494]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[165] += + acadoWorkspace.H10[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[497]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[166] += + acadoWorkspace.H10[498]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[499]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[500]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[167] += + acadoWorkspace.H10[501]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[502]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[503]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[168] += + acadoWorkspace.H10[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[506]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[169] += + acadoWorkspace.H10[507]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[508]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[509]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[170] += + acadoWorkspace.H10[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[512]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[171] += + acadoWorkspace.H10[513]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[514]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[515]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[172] += + acadoWorkspace.H10[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[518]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[173] += + acadoWorkspace.H10[519]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[520]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[521]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[174] += + acadoWorkspace.H10[522]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[523]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[524]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[175] += + acadoWorkspace.H10[525]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[526]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[527]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[176] += + acadoWorkspace.H10[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[530]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[177] += + acadoWorkspace.H10[531]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[532]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[533]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[178] += + acadoWorkspace.H10[534]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[535]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[536]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[179] += + acadoWorkspace.H10[537]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[538]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[539]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[180] += + acadoWorkspace.H10[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[542]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[181] += + acadoWorkspace.H10[543]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[544]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[545]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[182] += + acadoWorkspace.H10[546]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[547]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[548]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[183] += + acadoWorkspace.H10[549]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[550]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[551]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[184] += + acadoWorkspace.H10[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[554]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[185] += + acadoWorkspace.H10[555]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[556]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[557]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[186] += + acadoWorkspace.H10[558]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[559]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[560]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[187] += + acadoWorkspace.H10[561]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[562]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[563]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[188] += + acadoWorkspace.H10[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[566]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[189] += + acadoWorkspace.H10[567]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[568]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[569]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[190] += + acadoWorkspace.H10[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[572]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[191] += + acadoWorkspace.H10[573]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[574]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[575]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[192] += + acadoWorkspace.H10[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[578]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[193] += + acadoWorkspace.H10[579]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[580]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[581]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[194] += + acadoWorkspace.H10[582]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[583]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[584]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[195] += + acadoWorkspace.H10[585]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[586]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[587]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[196] += + acadoWorkspace.H10[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[590]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[197] += + acadoWorkspace.H10[591]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[592]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[593]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[198] += + acadoWorkspace.H10[594]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[595]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[596]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[199] += + acadoWorkspace.H10[597]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[598]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[599]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[200] += + acadoWorkspace.H10[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[602]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[201] += + acadoWorkspace.H10[603]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[604]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[605]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[202] += + acadoWorkspace.H10[606]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[607]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[608]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[203] += + acadoWorkspace.H10[609]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[610]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[611]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[204] += + acadoWorkspace.H10[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[614]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[205] += + acadoWorkspace.H10[615]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[616]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[617]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[206] += + acadoWorkspace.H10[618]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[619]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[620]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[207] += + acadoWorkspace.H10[621]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[622]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[623]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[208] += + acadoWorkspace.H10[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[626]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[209] += + acadoWorkspace.H10[627]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[628]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[629]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[210] += + acadoWorkspace.H10[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[632]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[211] += + acadoWorkspace.H10[633]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[634]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[635]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[212] += + acadoWorkspace.H10[636]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[637]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[638]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[213] += + acadoWorkspace.H10[639]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[640]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[641]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[214] += + acadoWorkspace.H10[642]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[643]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[644]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[215] += + acadoWorkspace.H10[645]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[646]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[647]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[216] += + acadoWorkspace.H10[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[650]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[217] += + acadoWorkspace.H10[651]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[652]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[653]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[218] += + acadoWorkspace.H10[654]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[655]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[656]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[219] += + acadoWorkspace.H10[657]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[658]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[659]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[220] += + acadoWorkspace.H10[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[662]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[221] += + acadoWorkspace.H10[663]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[664]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[665]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[222] += + acadoWorkspace.H10[666]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[667]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[668]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[223] += + acadoWorkspace.H10[669]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[670]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[671]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[224] += + acadoWorkspace.H10[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[674]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[225] += + acadoWorkspace.H10[675]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[676]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[677]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[226] += + acadoWorkspace.H10[678]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[679]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[680]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[227] += + acadoWorkspace.H10[681]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[682]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[683]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[228] += + acadoWorkspace.H10[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[686]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[229] += + acadoWorkspace.H10[687]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[688]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[689]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[230] += + acadoWorkspace.H10[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[692]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[231] += + acadoWorkspace.H10[693]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[694]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[695]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[232] += + acadoWorkspace.H10[696]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[697]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[698]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[233] += + acadoWorkspace.H10[699]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[700]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[701]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[234] += + acadoWorkspace.H10[702]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[703]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[704]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[235] += + acadoWorkspace.H10[705]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[706]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[707]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[236] += + acadoWorkspace.H10[708]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[709]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[710]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[237] += + acadoWorkspace.H10[711]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[712]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[713]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[238] += + acadoWorkspace.H10[714]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[715]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[716]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[239] += + acadoWorkspace.H10[717]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[718]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[719]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[240] += + acadoWorkspace.H10[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[722]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[241] += + acadoWorkspace.H10[723]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[724]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[725]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[242] += + acadoWorkspace.H10[726]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[727]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[728]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[243] += + acadoWorkspace.H10[729]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[730]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[731]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[244] += + acadoWorkspace.H10[732]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[733]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[734]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[245] += + acadoWorkspace.H10[735]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[736]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[737]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[246] += + acadoWorkspace.H10[738]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[739]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[740]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[247] += + acadoWorkspace.H10[741]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[742]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[743]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[248] += + acadoWorkspace.H10[744]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[745]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[746]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[249] += + acadoWorkspace.H10[747]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[748]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[749]*acadoWorkspace.Dx0[2];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[88] = + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[89] = + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[90] = + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[91] = + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[92] = + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[93] = + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[94] = + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[95] = + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[96] = + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[97] = + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[98] = + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[99] = + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[100] = + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[101] = + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[102] = + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[103] = + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[104] = + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[105] = + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[106] = + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[107] = + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[108] = + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[109] = + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[110] = + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[111] = + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[112] = + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[113] = + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[114] = + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[115] = + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[116] = + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[117] = + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[118] = + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[119] = + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[120] = + acadoWorkspace.A01[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[362]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[121] = + acadoWorkspace.A01[363]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[364]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[365]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[122] = + acadoWorkspace.A01[366]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[367]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[368]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[123] = + acadoWorkspace.A01[369]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[370]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[371]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[124] = + acadoWorkspace.A01[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[374]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[125] = + acadoWorkspace.A01[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[377]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[126] = + acadoWorkspace.A01[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[380]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[127] = + acadoWorkspace.A01[381]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[382]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[383]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[128] = + acadoWorkspace.A01[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[386]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[129] = + acadoWorkspace.A01[387]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[388]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[389]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[130] = + acadoWorkspace.A01[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[392]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[131] = + acadoWorkspace.A01[393]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[394]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[395]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[132] = + acadoWorkspace.A01[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[398]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[133] = + acadoWorkspace.A01[399]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[400]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[401]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[134] = + acadoWorkspace.A01[402]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[403]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[404]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[135] = + acadoWorkspace.A01[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[407]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[136] = + acadoWorkspace.A01[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[410]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[137] = + acadoWorkspace.A01[411]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[412]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[413]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[138] = + acadoWorkspace.A01[414]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[415]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[416]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[139] = + acadoWorkspace.A01[417]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[418]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[419]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[140] = + acadoWorkspace.A01[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[422]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[141] = + acadoWorkspace.A01[423]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[424]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[425]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[142] = + acadoWorkspace.A01[426]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[427]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[428]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[143] = + acadoWorkspace.A01[429]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[430]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[431]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[144] = + acadoWorkspace.A01[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[434]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[145] = + acadoWorkspace.A01[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[437]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[146] = + acadoWorkspace.A01[438]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[439]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[440]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[147] = + acadoWorkspace.A01[441]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[442]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[443]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[148] = + acadoWorkspace.A01[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[446]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[149] = + acadoWorkspace.A01[447]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[448]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[449]*acadoWorkspace.Dx0[2];
for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.lbA[lRun2] -= acadoWorkspace.pacA01Dx0[lRun2];


for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.ubA[lRun2] -= acadoWorkspace.pacA01Dx0[lRun2];


}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 250; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];


acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];

acadoVariables.x[3] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[0];
acadoVariables.x[4] += + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[1];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[2];
acadoVariables.x[6] += + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[3];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[4];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[5];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[6];
acadoVariables.x[10] += + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[7];
acadoVariables.x[11] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[8];
acadoVariables.x[12] += + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[9];
acadoVariables.x[13] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[10];
acadoVariables.x[14] += + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[11];
acadoVariables.x[15] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[12];
acadoVariables.x[16] += + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[13];
acadoVariables.x[17] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[14];
acadoVariables.x[18] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[15];
acadoVariables.x[19] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[16];
acadoVariables.x[20] += + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[17];
acadoVariables.x[21] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[18];
acadoVariables.x[22] += + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[19];
acadoVariables.x[23] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[20];
acadoVariables.x[24] += + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[21];
acadoVariables.x[25] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[22];
acadoVariables.x[26] += + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[23];
acadoVariables.x[27] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[24];
acadoVariables.x[28] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[25];
acadoVariables.x[29] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[26];
acadoVariables.x[30] += + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[27];
acadoVariables.x[31] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[28];
acadoVariables.x[32] += + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[29];
acadoVariables.x[33] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[30];
acadoVariables.x[34] += + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[31];
acadoVariables.x[35] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[32];
acadoVariables.x[36] += + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[33];
acadoVariables.x[37] += + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[34];
acadoVariables.x[38] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[35];
acadoVariables.x[39] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[36];
acadoVariables.x[40] += + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[37];
acadoVariables.x[41] += + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[38];
acadoVariables.x[42] += + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[39];
acadoVariables.x[43] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[40];
acadoVariables.x[44] += + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[41];
acadoVariables.x[45] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[42];
acadoVariables.x[46] += + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[43];
acadoVariables.x[47] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[44];
acadoVariables.x[48] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[45];
acadoVariables.x[49] += + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[46];
acadoVariables.x[50] += + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[47];
acadoVariables.x[51] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[48];
acadoVariables.x[52] += + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[49];
acadoVariables.x[53] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[50];
acadoVariables.x[54] += + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[51];
acadoVariables.x[55] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[52];
acadoVariables.x[56] += + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[53];
acadoVariables.x[57] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[54];
acadoVariables.x[58] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[55];
acadoVariables.x[59] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[56];
acadoVariables.x[60] += + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[57];
acadoVariables.x[61] += + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[58];
acadoVariables.x[62] += + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[59];
acadoVariables.x[63] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[60];
acadoVariables.x[64] += + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[61];
acadoVariables.x[65] += + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[62];
acadoVariables.x[66] += + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[63];
acadoVariables.x[67] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[64];
acadoVariables.x[68] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[65];
acadoVariables.x[69] += + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[66];
acadoVariables.x[70] += + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[67];
acadoVariables.x[71] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[68];
acadoVariables.x[72] += + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[69];
acadoVariables.x[73] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[70];
acadoVariables.x[74] += + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[71];
acadoVariables.x[75] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[72];
acadoVariables.x[76] += + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[73];
acadoVariables.x[77] += + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[74];
acadoVariables.x[78] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[75];
acadoVariables.x[79] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[76];
acadoVariables.x[80] += + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[77];
acadoVariables.x[81] += + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[78];
acadoVariables.x[82] += + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[79];
acadoVariables.x[83] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[80];
acadoVariables.x[84] += + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[81];
acadoVariables.x[85] += + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[82];
acadoVariables.x[86] += + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[83];
acadoVariables.x[87] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[84];
acadoVariables.x[88] += + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[85];
acadoVariables.x[89] += + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[86];
acadoVariables.x[90] += + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[87];
acadoVariables.x[91] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[88];
acadoVariables.x[92] += + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[89];
acadoVariables.x[93] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[90];
acadoVariables.x[94] += + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[91];
acadoVariables.x[95] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[92];
acadoVariables.x[96] += + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[93];
acadoVariables.x[97] += + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[94];
acadoVariables.x[98] += + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[95];
acadoVariables.x[99] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[96];
acadoVariables.x[100] += + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[97];
acadoVariables.x[101] += + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[98];
acadoVariables.x[102] += + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[99];
acadoVariables.x[103] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[100];
acadoVariables.x[104] += + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[101];
acadoVariables.x[105] += + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[102];
acadoVariables.x[106] += + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[103];
acadoVariables.x[107] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[104];
acadoVariables.x[108] += + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[105];
acadoVariables.x[109] += + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[106];
acadoVariables.x[110] += + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[107];
acadoVariables.x[111] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[108];
acadoVariables.x[112] += + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[109];
acadoVariables.x[113] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[110];
acadoVariables.x[114] += + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[111];
acadoVariables.x[115] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[112];
acadoVariables.x[116] += + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[113];
acadoVariables.x[117] += + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[114];
acadoVariables.x[118] += + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[115];
acadoVariables.x[119] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[116];
acadoVariables.x[120] += + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[117];
acadoVariables.x[121] += + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[118];
acadoVariables.x[122] += + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[119];
acadoVariables.x[123] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[120];
acadoVariables.x[124] += + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[121];
acadoVariables.x[125] += + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[122];
acadoVariables.x[126] += + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[123];
acadoVariables.x[127] += + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[124];
acadoVariables.x[128] += + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[125];
acadoVariables.x[129] += + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[126];
acadoVariables.x[130] += + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[127];
acadoVariables.x[131] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[128];
acadoVariables.x[132] += + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[129];
acadoVariables.x[133] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[130];
acadoVariables.x[134] += + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[131];
acadoVariables.x[135] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[132];
acadoVariables.x[136] += + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[133];
acadoVariables.x[137] += + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[134];
acadoVariables.x[138] += + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[135];
acadoVariables.x[139] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[136];
acadoVariables.x[140] += + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[137];
acadoVariables.x[141] += + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[138];
acadoVariables.x[142] += + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[139];
acadoVariables.x[143] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[140];
acadoVariables.x[144] += + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[141];
acadoVariables.x[145] += + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[142];
acadoVariables.x[146] += + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[143];
acadoVariables.x[147] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[144];
acadoVariables.x[148] += + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[145];
acadoVariables.x[149] += + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[146];
acadoVariables.x[150] += + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[147];
acadoVariables.x[151] += + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[148];
acadoVariables.x[152] += + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[149];

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 15 ]), &(acadoWorkspace.x[ lRun2 * 5 ]), &(acadoVariables.x[ lRun1 * 3 + 3 ]) );
}
}
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 50; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[27] = acadoVariables.u[index * 5];
acadoWorkspace.state[28] = acadoVariables.u[index * 5 + 1];
acadoWorkspace.state[29] = acadoVariables.u[index * 5 + 2];
acadoWorkspace.state[30] = acadoVariables.u[index * 5 + 3];
acadoWorkspace.state[31] = acadoVariables.u[index * 5 + 4];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 50; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
if (uEnd != 0)
{
acadoWorkspace.state[27] = uEnd[0];
acadoWorkspace.state[28] = uEnd[1];
acadoWorkspace.state[29] = uEnd[2];
acadoWorkspace.state[30] = uEnd[3];
acadoWorkspace.state[31] = uEnd[4];
}
else
{
acadoWorkspace.state[27] = acadoVariables.u[245];
acadoWorkspace.state[28] = acadoVariables.u[246];
acadoWorkspace.state[29] = acadoVariables.u[247];
acadoWorkspace.state[30] = acadoVariables.u[248];
acadoWorkspace.state[31] = acadoVariables.u[249];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 49; ++index)
{
acadoVariables.u[index * 5] = acadoVariables.u[index * 5 + 5];
acadoVariables.u[index * 5 + 1] = acadoVariables.u[index * 5 + 6];
acadoVariables.u[index * 5 + 2] = acadoVariables.u[index * 5 + 7];
acadoVariables.u[index * 5 + 3] = acadoVariables.u[index * 5 + 8];
acadoVariables.u[index * 5 + 4] = acadoVariables.u[index * 5 + 9];
}

if (uEnd != 0)
{
acadoVariables.u[245] = uEnd[0];
acadoVariables.u[246] = uEnd[1];
acadoVariables.u[247] = uEnd[2];
acadoVariables.u[248] = uEnd[3];
acadoVariables.u[249] = uEnd[4];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147] + acadoWorkspace.g[148]*acadoWorkspace.x[148] + acadoWorkspace.g[149]*acadoWorkspace.x[149] + acadoWorkspace.g[150]*acadoWorkspace.x[150] + acadoWorkspace.g[151]*acadoWorkspace.x[151] + acadoWorkspace.g[152]*acadoWorkspace.x[152] + acadoWorkspace.g[153]*acadoWorkspace.x[153] + acadoWorkspace.g[154]*acadoWorkspace.x[154] + acadoWorkspace.g[155]*acadoWorkspace.x[155] + acadoWorkspace.g[156]*acadoWorkspace.x[156] + acadoWorkspace.g[157]*acadoWorkspace.x[157] + acadoWorkspace.g[158]*acadoWorkspace.x[158] + acadoWorkspace.g[159]*acadoWorkspace.x[159] + acadoWorkspace.g[160]*acadoWorkspace.x[160] + acadoWorkspace.g[161]*acadoWorkspace.x[161] + acadoWorkspace.g[162]*acadoWorkspace.x[162] + acadoWorkspace.g[163]*acadoWorkspace.x[163] + acadoWorkspace.g[164]*acadoWorkspace.x[164] + acadoWorkspace.g[165]*acadoWorkspace.x[165] + acadoWorkspace.g[166]*acadoWorkspace.x[166] + acadoWorkspace.g[167]*acadoWorkspace.x[167] + acadoWorkspace.g[168]*acadoWorkspace.x[168] + acadoWorkspace.g[169]*acadoWorkspace.x[169] + acadoWorkspace.g[170]*acadoWorkspace.x[170] + acadoWorkspace.g[171]*acadoWorkspace.x[171] + acadoWorkspace.g[172]*acadoWorkspace.x[172] + acadoWorkspace.g[173]*acadoWorkspace.x[173] + acadoWorkspace.g[174]*acadoWorkspace.x[174] + acadoWorkspace.g[175]*acadoWorkspace.x[175] + acadoWorkspace.g[176]*acadoWorkspace.x[176] + acadoWorkspace.g[177]*acadoWorkspace.x[177] + acadoWorkspace.g[178]*acadoWorkspace.x[178] + acadoWorkspace.g[179]*acadoWorkspace.x[179] + acadoWorkspace.g[180]*acadoWorkspace.x[180] + acadoWorkspace.g[181]*acadoWorkspace.x[181] + acadoWorkspace.g[182]*acadoWorkspace.x[182] + acadoWorkspace.g[183]*acadoWorkspace.x[183] + acadoWorkspace.g[184]*acadoWorkspace.x[184] + acadoWorkspace.g[185]*acadoWorkspace.x[185] + acadoWorkspace.g[186]*acadoWorkspace.x[186] + acadoWorkspace.g[187]*acadoWorkspace.x[187] + acadoWorkspace.g[188]*acadoWorkspace.x[188] + acadoWorkspace.g[189]*acadoWorkspace.x[189] + acadoWorkspace.g[190]*acadoWorkspace.x[190] + acadoWorkspace.g[191]*acadoWorkspace.x[191] + acadoWorkspace.g[192]*acadoWorkspace.x[192] + acadoWorkspace.g[193]*acadoWorkspace.x[193] + acadoWorkspace.g[194]*acadoWorkspace.x[194] + acadoWorkspace.g[195]*acadoWorkspace.x[195] + acadoWorkspace.g[196]*acadoWorkspace.x[196] + acadoWorkspace.g[197]*acadoWorkspace.x[197] + acadoWorkspace.g[198]*acadoWorkspace.x[198] + acadoWorkspace.g[199]*acadoWorkspace.x[199] + acadoWorkspace.g[200]*acadoWorkspace.x[200] + acadoWorkspace.g[201]*acadoWorkspace.x[201] + acadoWorkspace.g[202]*acadoWorkspace.x[202] + acadoWorkspace.g[203]*acadoWorkspace.x[203] + acadoWorkspace.g[204]*acadoWorkspace.x[204] + acadoWorkspace.g[205]*acadoWorkspace.x[205] + acadoWorkspace.g[206]*acadoWorkspace.x[206] + acadoWorkspace.g[207]*acadoWorkspace.x[207] + acadoWorkspace.g[208]*acadoWorkspace.x[208] + acadoWorkspace.g[209]*acadoWorkspace.x[209] + acadoWorkspace.g[210]*acadoWorkspace.x[210] + acadoWorkspace.g[211]*acadoWorkspace.x[211] + acadoWorkspace.g[212]*acadoWorkspace.x[212] + acadoWorkspace.g[213]*acadoWorkspace.x[213] + acadoWorkspace.g[214]*acadoWorkspace.x[214] + acadoWorkspace.g[215]*acadoWorkspace.x[215] + acadoWorkspace.g[216]*acadoWorkspace.x[216] + acadoWorkspace.g[217]*acadoWorkspace.x[217] + acadoWorkspace.g[218]*acadoWorkspace.x[218] + acadoWorkspace.g[219]*acadoWorkspace.x[219] + acadoWorkspace.g[220]*acadoWorkspace.x[220] + acadoWorkspace.g[221]*acadoWorkspace.x[221] + acadoWorkspace.g[222]*acadoWorkspace.x[222] + acadoWorkspace.g[223]*acadoWorkspace.x[223] + acadoWorkspace.g[224]*acadoWorkspace.x[224] + acadoWorkspace.g[225]*acadoWorkspace.x[225] + acadoWorkspace.g[226]*acadoWorkspace.x[226] + acadoWorkspace.g[227]*acadoWorkspace.x[227] + acadoWorkspace.g[228]*acadoWorkspace.x[228] + acadoWorkspace.g[229]*acadoWorkspace.x[229] + acadoWorkspace.g[230]*acadoWorkspace.x[230] + acadoWorkspace.g[231]*acadoWorkspace.x[231] + acadoWorkspace.g[232]*acadoWorkspace.x[232] + acadoWorkspace.g[233]*acadoWorkspace.x[233] + acadoWorkspace.g[234]*acadoWorkspace.x[234] + acadoWorkspace.g[235]*acadoWorkspace.x[235] + acadoWorkspace.g[236]*acadoWorkspace.x[236] + acadoWorkspace.g[237]*acadoWorkspace.x[237] + acadoWorkspace.g[238]*acadoWorkspace.x[238] + acadoWorkspace.g[239]*acadoWorkspace.x[239] + acadoWorkspace.g[240]*acadoWorkspace.x[240] + acadoWorkspace.g[241]*acadoWorkspace.x[241] + acadoWorkspace.g[242]*acadoWorkspace.x[242] + acadoWorkspace.g[243]*acadoWorkspace.x[243] + acadoWorkspace.g[244]*acadoWorkspace.x[244] + acadoWorkspace.g[245]*acadoWorkspace.x[245] + acadoWorkspace.g[246]*acadoWorkspace.x[246] + acadoWorkspace.g[247]*acadoWorkspace.x[247] + acadoWorkspace.g[248]*acadoWorkspace.x[248] + acadoWorkspace.g[249]*acadoWorkspace.x[249];
kkt = fabs( kkt );
for (index = 0; index < 250; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 150; ++index)
{
prd = acadoWorkspace.y[index + 250];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 5];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 5 + 4];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 7] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 7];
acadoWorkspace.Dy[lRun1 * 7 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 7 + 1];
acadoWorkspace.Dy[lRun1 * 7 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 7 + 2];
acadoWorkspace.Dy[lRun1 * 7 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 7 + 3];
acadoWorkspace.Dy[lRun1 * 7 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 7 + 4];
acadoWorkspace.Dy[lRun1 * 7 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 7 + 5];
acadoWorkspace.Dy[lRun1 * 7 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 7 + 6];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 7] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 14] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 21] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 28] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 35] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 42];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 1] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 8] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 15] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 22] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 29] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 36] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 43];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 2] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 9] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 16] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 23] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 30] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 37] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 44];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 3] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 10] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 17] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 24] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 31] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 38] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 45];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 4] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 11] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 18] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 25] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 32] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 39] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 46];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 5] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 12] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 19] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 26] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 33] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 40] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 47];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 6] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 13] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 20] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 27] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 34] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 41] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 48];
objVal += + acadoWorkspace.Dy[lRun1 * 7]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 7 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 7 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 7 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 7 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 7 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 7 + 6]*tmpDy[6];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

