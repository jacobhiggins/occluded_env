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
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 10 + 9];

acadoWorkspace.state[140] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[141] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[142] = acadoVariables.u[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 10] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 10 + 10];
acadoWorkspace.d[lRun1 * 10 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 10 + 11];
acadoWorkspace.d[lRun1 * 10 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 10 + 12];
acadoWorkspace.d[lRun1 * 10 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 10 + 13];
acadoWorkspace.d[lRun1 * 10 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 10 + 14];
acadoWorkspace.d[lRun1 * 10 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 10 + 15];
acadoWorkspace.d[lRun1 * 10 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 10 + 16];
acadoWorkspace.d[lRun1 * 10 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 10 + 17];
acadoWorkspace.d[lRun1 * 10 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 10 + 18];
acadoWorkspace.d[lRun1 * 10 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 10 + 19];

acadoWorkspace.evGx[lRun1 * 100] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 100 + 1] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 100 + 2] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 100 + 3] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 100 + 4] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 100 + 5] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 100 + 6] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 100 + 7] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 100 + 8] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 100 + 9] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 100 + 10] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 100 + 11] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 100 + 12] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 100 + 13] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 100 + 14] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 100 + 15] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 100 + 16] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 100 + 17] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 100 + 18] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 100 + 19] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 100 + 20] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 100 + 21] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 100 + 22] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 100 + 23] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 100 + 24] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 100 + 25] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 100 + 26] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 100 + 27] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 100 + 28] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 100 + 29] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 100 + 30] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 100 + 31] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 100 + 32] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 100 + 33] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 100 + 34] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 100 + 35] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 100 + 36] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 100 + 37] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 100 + 38] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 100 + 39] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 100 + 40] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 100 + 41] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 100 + 42] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 100 + 43] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 100 + 44] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 100 + 45] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 100 + 46] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 100 + 47] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 100 + 48] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 100 + 49] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 100 + 50] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 100 + 51] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 100 + 52] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 100 + 53] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 100 + 54] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 100 + 55] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 100 + 56] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 100 + 57] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 100 + 58] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 100 + 59] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 100 + 60] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 100 + 61] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 100 + 62] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 100 + 63] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 100 + 64] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 100 + 65] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 100 + 66] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 100 + 67] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 100 + 68] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 100 + 69] = acadoWorkspace.state[79];
acadoWorkspace.evGx[lRun1 * 100 + 70] = acadoWorkspace.state[80];
acadoWorkspace.evGx[lRun1 * 100 + 71] = acadoWorkspace.state[81];
acadoWorkspace.evGx[lRun1 * 100 + 72] = acadoWorkspace.state[82];
acadoWorkspace.evGx[lRun1 * 100 + 73] = acadoWorkspace.state[83];
acadoWorkspace.evGx[lRun1 * 100 + 74] = acadoWorkspace.state[84];
acadoWorkspace.evGx[lRun1 * 100 + 75] = acadoWorkspace.state[85];
acadoWorkspace.evGx[lRun1 * 100 + 76] = acadoWorkspace.state[86];
acadoWorkspace.evGx[lRun1 * 100 + 77] = acadoWorkspace.state[87];
acadoWorkspace.evGx[lRun1 * 100 + 78] = acadoWorkspace.state[88];
acadoWorkspace.evGx[lRun1 * 100 + 79] = acadoWorkspace.state[89];
acadoWorkspace.evGx[lRun1 * 100 + 80] = acadoWorkspace.state[90];
acadoWorkspace.evGx[lRun1 * 100 + 81] = acadoWorkspace.state[91];
acadoWorkspace.evGx[lRun1 * 100 + 82] = acadoWorkspace.state[92];
acadoWorkspace.evGx[lRun1 * 100 + 83] = acadoWorkspace.state[93];
acadoWorkspace.evGx[lRun1 * 100 + 84] = acadoWorkspace.state[94];
acadoWorkspace.evGx[lRun1 * 100 + 85] = acadoWorkspace.state[95];
acadoWorkspace.evGx[lRun1 * 100 + 86] = acadoWorkspace.state[96];
acadoWorkspace.evGx[lRun1 * 100 + 87] = acadoWorkspace.state[97];
acadoWorkspace.evGx[lRun1 * 100 + 88] = acadoWorkspace.state[98];
acadoWorkspace.evGx[lRun1 * 100 + 89] = acadoWorkspace.state[99];
acadoWorkspace.evGx[lRun1 * 100 + 90] = acadoWorkspace.state[100];
acadoWorkspace.evGx[lRun1 * 100 + 91] = acadoWorkspace.state[101];
acadoWorkspace.evGx[lRun1 * 100 + 92] = acadoWorkspace.state[102];
acadoWorkspace.evGx[lRun1 * 100 + 93] = acadoWorkspace.state[103];
acadoWorkspace.evGx[lRun1 * 100 + 94] = acadoWorkspace.state[104];
acadoWorkspace.evGx[lRun1 * 100 + 95] = acadoWorkspace.state[105];
acadoWorkspace.evGx[lRun1 * 100 + 96] = acadoWorkspace.state[106];
acadoWorkspace.evGx[lRun1 * 100 + 97] = acadoWorkspace.state[107];
acadoWorkspace.evGx[lRun1 * 100 + 98] = acadoWorkspace.state[108];
acadoWorkspace.evGx[lRun1 * 100 + 99] = acadoWorkspace.state[109];

acadoWorkspace.evGu[lRun1 * 30] = acadoWorkspace.state[110];
acadoWorkspace.evGu[lRun1 * 30 + 1] = acadoWorkspace.state[111];
acadoWorkspace.evGu[lRun1 * 30 + 2] = acadoWorkspace.state[112];
acadoWorkspace.evGu[lRun1 * 30 + 3] = acadoWorkspace.state[113];
acadoWorkspace.evGu[lRun1 * 30 + 4] = acadoWorkspace.state[114];
acadoWorkspace.evGu[lRun1 * 30 + 5] = acadoWorkspace.state[115];
acadoWorkspace.evGu[lRun1 * 30 + 6] = acadoWorkspace.state[116];
acadoWorkspace.evGu[lRun1 * 30 + 7] = acadoWorkspace.state[117];
acadoWorkspace.evGu[lRun1 * 30 + 8] = acadoWorkspace.state[118];
acadoWorkspace.evGu[lRun1 * 30 + 9] = acadoWorkspace.state[119];
acadoWorkspace.evGu[lRun1 * 30 + 10] = acadoWorkspace.state[120];
acadoWorkspace.evGu[lRun1 * 30 + 11] = acadoWorkspace.state[121];
acadoWorkspace.evGu[lRun1 * 30 + 12] = acadoWorkspace.state[122];
acadoWorkspace.evGu[lRun1 * 30 + 13] = acadoWorkspace.state[123];
acadoWorkspace.evGu[lRun1 * 30 + 14] = acadoWorkspace.state[124];
acadoWorkspace.evGu[lRun1 * 30 + 15] = acadoWorkspace.state[125];
acadoWorkspace.evGu[lRun1 * 30 + 16] = acadoWorkspace.state[126];
acadoWorkspace.evGu[lRun1 * 30 + 17] = acadoWorkspace.state[127];
acadoWorkspace.evGu[lRun1 * 30 + 18] = acadoWorkspace.state[128];
acadoWorkspace.evGu[lRun1 * 30 + 19] = acadoWorkspace.state[129];
acadoWorkspace.evGu[lRun1 * 30 + 20] = acadoWorkspace.state[130];
acadoWorkspace.evGu[lRun1 * 30 + 21] = acadoWorkspace.state[131];
acadoWorkspace.evGu[lRun1 * 30 + 22] = acadoWorkspace.state[132];
acadoWorkspace.evGu[lRun1 * 30 + 23] = acadoWorkspace.state[133];
acadoWorkspace.evGu[lRun1 * 30 + 24] = acadoWorkspace.state[134];
acadoWorkspace.evGu[lRun1 * 30 + 25] = acadoWorkspace.state[135];
acadoWorkspace.evGu[lRun1 * 30 + 26] = acadoWorkspace.state[136];
acadoWorkspace.evGu[lRun1 * 30 + 27] = acadoWorkspace.state[137];
acadoWorkspace.evGu[lRun1 * 30 + 28] = acadoWorkspace.state[138];
acadoWorkspace.evGu[lRun1 * 30 + 29] = acadoWorkspace.state[139];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = u[0];
out[4] = u[1];
out[5] = u[2];
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
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = 0.0;
;
tmpQ2[19] = 0.0;
;
tmpQ2[20] = 0.0;
;
tmpQ2[21] = 0.0;
;
tmpQ2[22] = 0.0;
;
tmpQ2[23] = 0.0;
;
tmpQ2[24] = 0.0;
;
tmpQ2[25] = 0.0;
;
tmpQ2[26] = 0.0;
;
tmpQ2[27] = 0.0;
;
tmpQ2[28] = 0.0;
;
tmpQ2[29] = 0.0;
;
tmpQ2[30] = 0.0;
;
tmpQ2[31] = 0.0;
;
tmpQ2[32] = 0.0;
;
tmpQ2[33] = 0.0;
;
tmpQ2[34] = 0.0;
;
tmpQ2[35] = 0.0;
;
tmpQ2[36] = 0.0;
;
tmpQ2[37] = 0.0;
;
tmpQ2[38] = 0.0;
;
tmpQ2[39] = 0.0;
;
tmpQ2[40] = 0.0;
;
tmpQ2[41] = 0.0;
;
tmpQ2[42] = 0.0;
;
tmpQ2[43] = 0.0;
;
tmpQ2[44] = 0.0;
;
tmpQ2[45] = 0.0;
;
tmpQ2[46] = 0.0;
;
tmpQ2[47] = 0.0;
;
tmpQ2[48] = 0.0;
;
tmpQ2[49] = 0.0;
;
tmpQ2[50] = 0.0;
;
tmpQ2[51] = 0.0;
;
tmpQ2[52] = 0.0;
;
tmpQ2[53] = 0.0;
;
tmpQ2[54] = 0.0;
;
tmpQ2[55] = 0.0;
;
tmpQ2[56] = 0.0;
;
tmpQ2[57] = 0.0;
;
tmpQ2[58] = 0.0;
;
tmpQ2[59] = 0.0;
;
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = 0.0;
;
tmpQ1[4] = 0.0;
;
tmpQ1[5] = 0.0;
;
tmpQ1[6] = 0.0;
;
tmpQ1[7] = 0.0;
;
tmpQ1[8] = 0.0;
;
tmpQ1[9] = 0.0;
;
tmpQ1[10] = + tmpQ2[6];
tmpQ1[11] = + tmpQ2[7];
tmpQ1[12] = + tmpQ2[8];
tmpQ1[13] = 0.0;
;
tmpQ1[14] = 0.0;
;
tmpQ1[15] = 0.0;
;
tmpQ1[16] = 0.0;
;
tmpQ1[17] = 0.0;
;
tmpQ1[18] = 0.0;
;
tmpQ1[19] = 0.0;
;
tmpQ1[20] = + tmpQ2[12];
tmpQ1[21] = + tmpQ2[13];
tmpQ1[22] = + tmpQ2[14];
tmpQ1[23] = 0.0;
;
tmpQ1[24] = 0.0;
;
tmpQ1[25] = 0.0;
;
tmpQ1[26] = 0.0;
;
tmpQ1[27] = 0.0;
;
tmpQ1[28] = 0.0;
;
tmpQ1[29] = 0.0;
;
tmpQ1[30] = + tmpQ2[18];
tmpQ1[31] = + tmpQ2[19];
tmpQ1[32] = + tmpQ2[20];
tmpQ1[33] = 0.0;
;
tmpQ1[34] = 0.0;
;
tmpQ1[35] = 0.0;
;
tmpQ1[36] = 0.0;
;
tmpQ1[37] = 0.0;
;
tmpQ1[38] = 0.0;
;
tmpQ1[39] = 0.0;
;
tmpQ1[40] = + tmpQ2[24];
tmpQ1[41] = + tmpQ2[25];
tmpQ1[42] = + tmpQ2[26];
tmpQ1[43] = 0.0;
;
tmpQ1[44] = 0.0;
;
tmpQ1[45] = 0.0;
;
tmpQ1[46] = 0.0;
;
tmpQ1[47] = 0.0;
;
tmpQ1[48] = 0.0;
;
tmpQ1[49] = 0.0;
;
tmpQ1[50] = + tmpQ2[30];
tmpQ1[51] = + tmpQ2[31];
tmpQ1[52] = + tmpQ2[32];
tmpQ1[53] = 0.0;
;
tmpQ1[54] = 0.0;
;
tmpQ1[55] = 0.0;
;
tmpQ1[56] = 0.0;
;
tmpQ1[57] = 0.0;
;
tmpQ1[58] = 0.0;
;
tmpQ1[59] = 0.0;
;
tmpQ1[60] = + tmpQ2[36];
tmpQ1[61] = + tmpQ2[37];
tmpQ1[62] = + tmpQ2[38];
tmpQ1[63] = 0.0;
;
tmpQ1[64] = 0.0;
;
tmpQ1[65] = 0.0;
;
tmpQ1[66] = 0.0;
;
tmpQ1[67] = 0.0;
;
tmpQ1[68] = 0.0;
;
tmpQ1[69] = 0.0;
;
tmpQ1[70] = + tmpQ2[42];
tmpQ1[71] = + tmpQ2[43];
tmpQ1[72] = + tmpQ2[44];
tmpQ1[73] = 0.0;
;
tmpQ1[74] = 0.0;
;
tmpQ1[75] = 0.0;
;
tmpQ1[76] = 0.0;
;
tmpQ1[77] = 0.0;
;
tmpQ1[78] = 0.0;
;
tmpQ1[79] = 0.0;
;
tmpQ1[80] = + tmpQ2[48];
tmpQ1[81] = + tmpQ2[49];
tmpQ1[82] = + tmpQ2[50];
tmpQ1[83] = 0.0;
;
tmpQ1[84] = 0.0;
;
tmpQ1[85] = 0.0;
;
tmpQ1[86] = 0.0;
;
tmpQ1[87] = 0.0;
;
tmpQ1[88] = 0.0;
;
tmpQ1[89] = 0.0;
;
tmpQ1[90] = + tmpQ2[54];
tmpQ1[91] = + tmpQ2[55];
tmpQ1[92] = + tmpQ2[56];
tmpQ1[93] = 0.0;
;
tmpQ1[94] = 0.0;
;
tmpQ1[95] = 0.0;
;
tmpQ1[96] = 0.0;
;
tmpQ1[97] = 0.0;
;
tmpQ1[98] = 0.0;
;
tmpQ1[99] = 0.0;
;
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[18];
tmpR2[1] = +tmpObjS[19];
tmpR2[2] = +tmpObjS[20];
tmpR2[3] = +tmpObjS[21];
tmpR2[4] = +tmpObjS[22];
tmpR2[5] = +tmpObjS[23];
tmpR2[6] = +tmpObjS[24];
tmpR2[7] = +tmpObjS[25];
tmpR2[8] = +tmpObjS[26];
tmpR2[9] = +tmpObjS[27];
tmpR2[10] = +tmpObjS[28];
tmpR2[11] = +tmpObjS[29];
tmpR2[12] = +tmpObjS[30];
tmpR2[13] = +tmpObjS[31];
tmpR2[14] = +tmpObjS[32];
tmpR2[15] = +tmpObjS[33];
tmpR2[16] = +tmpObjS[34];
tmpR2[17] = +tmpObjS[35];
tmpR1[0] = + tmpR2[3];
tmpR1[1] = + tmpR2[4];
tmpR1[2] = + tmpR2[5];
tmpR1[3] = + tmpR2[9];
tmpR1[4] = + tmpR2[10];
tmpR1[5] = + tmpR2[11];
tmpR1[6] = + tmpR2[15];
tmpR1[7] = + tmpR2[16];
tmpR1[8] = + tmpR2[17];
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
tmpQN2[6] = 0.0;
;
tmpQN2[7] = 0.0;
;
tmpQN2[8] = 0.0;
;
tmpQN2[9] = 0.0;
;
tmpQN2[10] = 0.0;
;
tmpQN2[11] = 0.0;
;
tmpQN2[12] = 0.0;
;
tmpQN2[13] = 0.0;
;
tmpQN2[14] = 0.0;
;
tmpQN2[15] = 0.0;
;
tmpQN2[16] = 0.0;
;
tmpQN2[17] = 0.0;
;
tmpQN2[18] = 0.0;
;
tmpQN2[19] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = 0.0;
;
tmpQN1[3] = 0.0;
;
tmpQN1[4] = 0.0;
;
tmpQN1[5] = 0.0;
;
tmpQN1[6] = 0.0;
;
tmpQN1[7] = 0.0;
;
tmpQN1[8] = 0.0;
;
tmpQN1[9] = 0.0;
;
tmpQN1[10] = + tmpQN2[2];
tmpQN1[11] = + tmpQN2[3];
tmpQN1[12] = 0.0;
;
tmpQN1[13] = 0.0;
;
tmpQN1[14] = 0.0;
;
tmpQN1[15] = 0.0;
;
tmpQN1[16] = 0.0;
;
tmpQN1[17] = 0.0;
;
tmpQN1[18] = 0.0;
;
tmpQN1[19] = 0.0;
;
tmpQN1[20] = + tmpQN2[4];
tmpQN1[21] = + tmpQN2[5];
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = 0.0;
;
tmpQN1[25] = 0.0;
;
tmpQN1[26] = 0.0;
;
tmpQN1[27] = 0.0;
;
tmpQN1[28] = 0.0;
;
tmpQN1[29] = 0.0;
;
tmpQN1[30] = + tmpQN2[6];
tmpQN1[31] = + tmpQN2[7];
tmpQN1[32] = 0.0;
;
tmpQN1[33] = 0.0;
;
tmpQN1[34] = 0.0;
;
tmpQN1[35] = 0.0;
;
tmpQN1[36] = 0.0;
;
tmpQN1[37] = 0.0;
;
tmpQN1[38] = 0.0;
;
tmpQN1[39] = 0.0;
;
tmpQN1[40] = + tmpQN2[8];
tmpQN1[41] = + tmpQN2[9];
tmpQN1[42] = 0.0;
;
tmpQN1[43] = 0.0;
;
tmpQN1[44] = 0.0;
;
tmpQN1[45] = 0.0;
;
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = 0.0;
;
tmpQN1[49] = 0.0;
;
tmpQN1[50] = + tmpQN2[10];
tmpQN1[51] = + tmpQN2[11];
tmpQN1[52] = 0.0;
;
tmpQN1[53] = 0.0;
;
tmpQN1[54] = 0.0;
;
tmpQN1[55] = 0.0;
;
tmpQN1[56] = 0.0;
;
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[12];
tmpQN1[61] = + tmpQN2[13];
tmpQN1[62] = 0.0;
;
tmpQN1[63] = 0.0;
;
tmpQN1[64] = 0.0;
;
tmpQN1[65] = 0.0;
;
tmpQN1[66] = 0.0;
;
tmpQN1[67] = 0.0;
;
tmpQN1[68] = 0.0;
;
tmpQN1[69] = 0.0;
;
tmpQN1[70] = + tmpQN2[14];
tmpQN1[71] = + tmpQN2[15];
tmpQN1[72] = 0.0;
;
tmpQN1[73] = 0.0;
;
tmpQN1[74] = 0.0;
;
tmpQN1[75] = 0.0;
;
tmpQN1[76] = 0.0;
;
tmpQN1[77] = 0.0;
;
tmpQN1[78] = 0.0;
;
tmpQN1[79] = 0.0;
;
tmpQN1[80] = + tmpQN2[16];
tmpQN1[81] = + tmpQN2[17];
tmpQN1[82] = 0.0;
;
tmpQN1[83] = 0.0;
;
tmpQN1[84] = 0.0;
;
tmpQN1[85] = 0.0;
;
tmpQN1[86] = 0.0;
;
tmpQN1[87] = 0.0;
;
tmpQN1[88] = 0.0;
;
tmpQN1[89] = 0.0;
;
tmpQN1[90] = + tmpQN2[18];
tmpQN1[91] = + tmpQN2[19];
tmpQN1[92] = 0.0;
;
tmpQN1[93] = 0.0;
;
tmpQN1[94] = 0.0;
;
tmpQN1[95] = 0.0;
;
tmpQN1[96] = 0.0;
;
tmpQN1[97] = 0.0;
;
tmpQN1[98] = 0.0;
;
tmpQN1[99] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 6] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 6 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 6 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 6 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 6 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 6 + 5] = acadoWorkspace.objValueOut[5];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 36 ]), &(acadoWorkspace.Q1[ runObj * 100 ]), &(acadoWorkspace.Q2[ runObj * 60 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 36 ]), &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 18 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[100];
acadoWorkspace.objValueIn[1] = acadoVariables.x[101];
acadoWorkspace.objValueIn[2] = acadoVariables.x[102];
acadoWorkspace.objValueIn[3] = acadoVariables.x[103];
acadoWorkspace.objValueIn[4] = acadoVariables.x[104];
acadoWorkspace.objValueIn[5] = acadoVariables.x[105];
acadoWorkspace.objValueIn[6] = acadoVariables.x[106];
acadoWorkspace.objValueIn[7] = acadoVariables.x[107];
acadoWorkspace.objValueIn[8] = acadoVariables.x[108];
acadoWorkspace.objValueIn[9] = acadoVariables.x[109];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9];
dNew[1] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4] + Gx1[15]*dOld[5] + Gx1[16]*dOld[6] + Gx1[17]*dOld[7] + Gx1[18]*dOld[8] + Gx1[19]*dOld[9];
dNew[2] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4] + Gx1[25]*dOld[5] + Gx1[26]*dOld[6] + Gx1[27]*dOld[7] + Gx1[28]*dOld[8] + Gx1[29]*dOld[9];
dNew[3] += + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5] + Gx1[36]*dOld[6] + Gx1[37]*dOld[7] + Gx1[38]*dOld[8] + Gx1[39]*dOld[9];
dNew[4] += + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7] + Gx1[48]*dOld[8] + Gx1[49]*dOld[9];
dNew[5] += + Gx1[50]*dOld[0] + Gx1[51]*dOld[1] + Gx1[52]*dOld[2] + Gx1[53]*dOld[3] + Gx1[54]*dOld[4] + Gx1[55]*dOld[5] + Gx1[56]*dOld[6] + Gx1[57]*dOld[7] + Gx1[58]*dOld[8] + Gx1[59]*dOld[9];
dNew[6] += + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9];
dNew[7] += + Gx1[70]*dOld[0] + Gx1[71]*dOld[1] + Gx1[72]*dOld[2] + Gx1[73]*dOld[3] + Gx1[74]*dOld[4] + Gx1[75]*dOld[5] + Gx1[76]*dOld[6] + Gx1[77]*dOld[7] + Gx1[78]*dOld[8] + Gx1[79]*dOld[9];
dNew[8] += + Gx1[80]*dOld[0] + Gx1[81]*dOld[1] + Gx1[82]*dOld[2] + Gx1[83]*dOld[3] + Gx1[84]*dOld[4] + Gx1[85]*dOld[5] + Gx1[86]*dOld[6] + Gx1[87]*dOld[7] + Gx1[88]*dOld[8] + Gx1[89]*dOld[9];
dNew[9] += + Gx1[90]*dOld[0] + Gx1[91]*dOld[1] + Gx1[92]*dOld[2] + Gx1[93]*dOld[3] + Gx1[94]*dOld[4] + Gx1[95]*dOld[5] + Gx1[96]*dOld[6] + Gx1[97]*dOld[7] + Gx1[98]*dOld[8] + Gx1[99]*dOld[9];
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
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
Gx2[64] = Gx1[64];
Gx2[65] = Gx1[65];
Gx2[66] = Gx1[66];
Gx2[67] = Gx1[67];
Gx2[68] = Gx1[68];
Gx2[69] = Gx1[69];
Gx2[70] = Gx1[70];
Gx2[71] = Gx1[71];
Gx2[72] = Gx1[72];
Gx2[73] = Gx1[73];
Gx2[74] = Gx1[74];
Gx2[75] = Gx1[75];
Gx2[76] = Gx1[76];
Gx2[77] = Gx1[77];
Gx2[78] = Gx1[78];
Gx2[79] = Gx1[79];
Gx2[80] = Gx1[80];
Gx2[81] = Gx1[81];
Gx2[82] = Gx1[82];
Gx2[83] = Gx1[83];
Gx2[84] = Gx1[84];
Gx2[85] = Gx1[85];
Gx2[86] = Gx1[86];
Gx2[87] = Gx1[87];
Gx2[88] = Gx1[88];
Gx2[89] = Gx1[89];
Gx2[90] = Gx1[90];
Gx2[91] = Gx1[91];
Gx2[92] = Gx1[92];
Gx2[93] = Gx1[93];
Gx2[94] = Gx1[94];
Gx2[95] = Gx1[95];
Gx2[96] = Gx1[96];
Gx2[97] = Gx1[97];
Gx2[98] = Gx1[98];
Gx2[99] = Gx1[99];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[40] + Gx1[5]*Gx2[50] + Gx1[6]*Gx2[60] + Gx1[7]*Gx2[70] + Gx1[8]*Gx2[80] + Gx1[9]*Gx2[90];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[41] + Gx1[5]*Gx2[51] + Gx1[6]*Gx2[61] + Gx1[7]*Gx2[71] + Gx1[8]*Gx2[81] + Gx1[9]*Gx2[91];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[32] + Gx1[4]*Gx2[42] + Gx1[5]*Gx2[52] + Gx1[6]*Gx2[62] + Gx1[7]*Gx2[72] + Gx1[8]*Gx2[82] + Gx1[9]*Gx2[92];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[33] + Gx1[4]*Gx2[43] + Gx1[5]*Gx2[53] + Gx1[6]*Gx2[63] + Gx1[7]*Gx2[73] + Gx1[8]*Gx2[83] + Gx1[9]*Gx2[93];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[34] + Gx1[4]*Gx2[44] + Gx1[5]*Gx2[54] + Gx1[6]*Gx2[64] + Gx1[7]*Gx2[74] + Gx1[8]*Gx2[84] + Gx1[9]*Gx2[94];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[35] + Gx1[4]*Gx2[45] + Gx1[5]*Gx2[55] + Gx1[6]*Gx2[65] + Gx1[7]*Gx2[75] + Gx1[8]*Gx2[85] + Gx1[9]*Gx2[95];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[36] + Gx1[4]*Gx2[46] + Gx1[5]*Gx2[56] + Gx1[6]*Gx2[66] + Gx1[7]*Gx2[76] + Gx1[8]*Gx2[86] + Gx1[9]*Gx2[96];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[27] + Gx1[3]*Gx2[37] + Gx1[4]*Gx2[47] + Gx1[5]*Gx2[57] + Gx1[6]*Gx2[67] + Gx1[7]*Gx2[77] + Gx1[8]*Gx2[87] + Gx1[9]*Gx2[97];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[18] + Gx1[2]*Gx2[28] + Gx1[3]*Gx2[38] + Gx1[4]*Gx2[48] + Gx1[5]*Gx2[58] + Gx1[6]*Gx2[68] + Gx1[7]*Gx2[78] + Gx1[8]*Gx2[88] + Gx1[9]*Gx2[98];
Gx3[9] = + Gx1[0]*Gx2[9] + Gx1[1]*Gx2[19] + Gx1[2]*Gx2[29] + Gx1[3]*Gx2[39] + Gx1[4]*Gx2[49] + Gx1[5]*Gx2[59] + Gx1[6]*Gx2[69] + Gx1[7]*Gx2[79] + Gx1[8]*Gx2[89] + Gx1[9]*Gx2[99];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[10] + Gx1[12]*Gx2[20] + Gx1[13]*Gx2[30] + Gx1[14]*Gx2[40] + Gx1[15]*Gx2[50] + Gx1[16]*Gx2[60] + Gx1[17]*Gx2[70] + Gx1[18]*Gx2[80] + Gx1[19]*Gx2[90];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[11] + Gx1[12]*Gx2[21] + Gx1[13]*Gx2[31] + Gx1[14]*Gx2[41] + Gx1[15]*Gx2[51] + Gx1[16]*Gx2[61] + Gx1[17]*Gx2[71] + Gx1[18]*Gx2[81] + Gx1[19]*Gx2[91];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[12] + Gx1[12]*Gx2[22] + Gx1[13]*Gx2[32] + Gx1[14]*Gx2[42] + Gx1[15]*Gx2[52] + Gx1[16]*Gx2[62] + Gx1[17]*Gx2[72] + Gx1[18]*Gx2[82] + Gx1[19]*Gx2[92];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[13] + Gx1[12]*Gx2[23] + Gx1[13]*Gx2[33] + Gx1[14]*Gx2[43] + Gx1[15]*Gx2[53] + Gx1[16]*Gx2[63] + Gx1[17]*Gx2[73] + Gx1[18]*Gx2[83] + Gx1[19]*Gx2[93];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[14] + Gx1[12]*Gx2[24] + Gx1[13]*Gx2[34] + Gx1[14]*Gx2[44] + Gx1[15]*Gx2[54] + Gx1[16]*Gx2[64] + Gx1[17]*Gx2[74] + Gx1[18]*Gx2[84] + Gx1[19]*Gx2[94];
Gx3[15] = + Gx1[10]*Gx2[5] + Gx1[11]*Gx2[15] + Gx1[12]*Gx2[25] + Gx1[13]*Gx2[35] + Gx1[14]*Gx2[45] + Gx1[15]*Gx2[55] + Gx1[16]*Gx2[65] + Gx1[17]*Gx2[75] + Gx1[18]*Gx2[85] + Gx1[19]*Gx2[95];
Gx3[16] = + Gx1[10]*Gx2[6] + Gx1[11]*Gx2[16] + Gx1[12]*Gx2[26] + Gx1[13]*Gx2[36] + Gx1[14]*Gx2[46] + Gx1[15]*Gx2[56] + Gx1[16]*Gx2[66] + Gx1[17]*Gx2[76] + Gx1[18]*Gx2[86] + Gx1[19]*Gx2[96];
Gx3[17] = + Gx1[10]*Gx2[7] + Gx1[11]*Gx2[17] + Gx1[12]*Gx2[27] + Gx1[13]*Gx2[37] + Gx1[14]*Gx2[47] + Gx1[15]*Gx2[57] + Gx1[16]*Gx2[67] + Gx1[17]*Gx2[77] + Gx1[18]*Gx2[87] + Gx1[19]*Gx2[97];
Gx3[18] = + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[18] + Gx1[12]*Gx2[28] + Gx1[13]*Gx2[38] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[58] + Gx1[16]*Gx2[68] + Gx1[17]*Gx2[78] + Gx1[18]*Gx2[88] + Gx1[19]*Gx2[98];
Gx3[19] = + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[19] + Gx1[12]*Gx2[29] + Gx1[13]*Gx2[39] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[59] + Gx1[16]*Gx2[69] + Gx1[17]*Gx2[79] + Gx1[18]*Gx2[89] + Gx1[19]*Gx2[99];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[10] + Gx1[22]*Gx2[20] + Gx1[23]*Gx2[30] + Gx1[24]*Gx2[40] + Gx1[25]*Gx2[50] + Gx1[26]*Gx2[60] + Gx1[27]*Gx2[70] + Gx1[28]*Gx2[80] + Gx1[29]*Gx2[90];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[11] + Gx1[22]*Gx2[21] + Gx1[23]*Gx2[31] + Gx1[24]*Gx2[41] + Gx1[25]*Gx2[51] + Gx1[26]*Gx2[61] + Gx1[27]*Gx2[71] + Gx1[28]*Gx2[81] + Gx1[29]*Gx2[91];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[12] + Gx1[22]*Gx2[22] + Gx1[23]*Gx2[32] + Gx1[24]*Gx2[42] + Gx1[25]*Gx2[52] + Gx1[26]*Gx2[62] + Gx1[27]*Gx2[72] + Gx1[28]*Gx2[82] + Gx1[29]*Gx2[92];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[13] + Gx1[22]*Gx2[23] + Gx1[23]*Gx2[33] + Gx1[24]*Gx2[43] + Gx1[25]*Gx2[53] + Gx1[26]*Gx2[63] + Gx1[27]*Gx2[73] + Gx1[28]*Gx2[83] + Gx1[29]*Gx2[93];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[14] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[34] + Gx1[24]*Gx2[44] + Gx1[25]*Gx2[54] + Gx1[26]*Gx2[64] + Gx1[27]*Gx2[74] + Gx1[28]*Gx2[84] + Gx1[29]*Gx2[94];
Gx3[25] = + Gx1[20]*Gx2[5] + Gx1[21]*Gx2[15] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[35] + Gx1[24]*Gx2[45] + Gx1[25]*Gx2[55] + Gx1[26]*Gx2[65] + Gx1[27]*Gx2[75] + Gx1[28]*Gx2[85] + Gx1[29]*Gx2[95];
Gx3[26] = + Gx1[20]*Gx2[6] + Gx1[21]*Gx2[16] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[36] + Gx1[24]*Gx2[46] + Gx1[25]*Gx2[56] + Gx1[26]*Gx2[66] + Gx1[27]*Gx2[76] + Gx1[28]*Gx2[86] + Gx1[29]*Gx2[96];
Gx3[27] = + Gx1[20]*Gx2[7] + Gx1[21]*Gx2[17] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[37] + Gx1[24]*Gx2[47] + Gx1[25]*Gx2[57] + Gx1[26]*Gx2[67] + Gx1[27]*Gx2[77] + Gx1[28]*Gx2[87] + Gx1[29]*Gx2[97];
Gx3[28] = + Gx1[20]*Gx2[8] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[38] + Gx1[24]*Gx2[48] + Gx1[25]*Gx2[58] + Gx1[26]*Gx2[68] + Gx1[27]*Gx2[78] + Gx1[28]*Gx2[88] + Gx1[29]*Gx2[98];
Gx3[29] = + Gx1[20]*Gx2[9] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[39] + Gx1[24]*Gx2[49] + Gx1[25]*Gx2[59] + Gx1[26]*Gx2[69] + Gx1[27]*Gx2[79] + Gx1[28]*Gx2[89] + Gx1[29]*Gx2[99];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[20] + Gx1[33]*Gx2[30] + Gx1[34]*Gx2[40] + Gx1[35]*Gx2[50] + Gx1[36]*Gx2[60] + Gx1[37]*Gx2[70] + Gx1[38]*Gx2[80] + Gx1[39]*Gx2[90];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[21] + Gx1[33]*Gx2[31] + Gx1[34]*Gx2[41] + Gx1[35]*Gx2[51] + Gx1[36]*Gx2[61] + Gx1[37]*Gx2[71] + Gx1[38]*Gx2[81] + Gx1[39]*Gx2[91];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[12] + Gx1[32]*Gx2[22] + Gx1[33]*Gx2[32] + Gx1[34]*Gx2[42] + Gx1[35]*Gx2[52] + Gx1[36]*Gx2[62] + Gx1[37]*Gx2[72] + Gx1[38]*Gx2[82] + Gx1[39]*Gx2[92];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[13] + Gx1[32]*Gx2[23] + Gx1[33]*Gx2[33] + Gx1[34]*Gx2[43] + Gx1[35]*Gx2[53] + Gx1[36]*Gx2[63] + Gx1[37]*Gx2[73] + Gx1[38]*Gx2[83] + Gx1[39]*Gx2[93];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[14] + Gx1[32]*Gx2[24] + Gx1[33]*Gx2[34] + Gx1[34]*Gx2[44] + Gx1[35]*Gx2[54] + Gx1[36]*Gx2[64] + Gx1[37]*Gx2[74] + Gx1[38]*Gx2[84] + Gx1[39]*Gx2[94];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[15] + Gx1[32]*Gx2[25] + Gx1[33]*Gx2[35] + Gx1[34]*Gx2[45] + Gx1[35]*Gx2[55] + Gx1[36]*Gx2[65] + Gx1[37]*Gx2[75] + Gx1[38]*Gx2[85] + Gx1[39]*Gx2[95];
Gx3[36] = + Gx1[30]*Gx2[6] + Gx1[31]*Gx2[16] + Gx1[32]*Gx2[26] + Gx1[33]*Gx2[36] + Gx1[34]*Gx2[46] + Gx1[35]*Gx2[56] + Gx1[36]*Gx2[66] + Gx1[37]*Gx2[76] + Gx1[38]*Gx2[86] + Gx1[39]*Gx2[96];
Gx3[37] = + Gx1[30]*Gx2[7] + Gx1[31]*Gx2[17] + Gx1[32]*Gx2[27] + Gx1[33]*Gx2[37] + Gx1[34]*Gx2[47] + Gx1[35]*Gx2[57] + Gx1[36]*Gx2[67] + Gx1[37]*Gx2[77] + Gx1[38]*Gx2[87] + Gx1[39]*Gx2[97];
Gx3[38] = + Gx1[30]*Gx2[8] + Gx1[31]*Gx2[18] + Gx1[32]*Gx2[28] + Gx1[33]*Gx2[38] + Gx1[34]*Gx2[48] + Gx1[35]*Gx2[58] + Gx1[36]*Gx2[68] + Gx1[37]*Gx2[78] + Gx1[38]*Gx2[88] + Gx1[39]*Gx2[98];
Gx3[39] = + Gx1[30]*Gx2[9] + Gx1[31]*Gx2[19] + Gx1[32]*Gx2[29] + Gx1[33]*Gx2[39] + Gx1[34]*Gx2[49] + Gx1[35]*Gx2[59] + Gx1[36]*Gx2[69] + Gx1[37]*Gx2[79] + Gx1[38]*Gx2[89] + Gx1[39]*Gx2[99];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[40] + Gx1[45]*Gx2[50] + Gx1[46]*Gx2[60] + Gx1[47]*Gx2[70] + Gx1[48]*Gx2[80] + Gx1[49]*Gx2[90];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[41] + Gx1[45]*Gx2[51] + Gx1[46]*Gx2[61] + Gx1[47]*Gx2[71] + Gx1[48]*Gx2[81] + Gx1[49]*Gx2[91];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[32] + Gx1[44]*Gx2[42] + Gx1[45]*Gx2[52] + Gx1[46]*Gx2[62] + Gx1[47]*Gx2[72] + Gx1[48]*Gx2[82] + Gx1[49]*Gx2[92];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[33] + Gx1[44]*Gx2[43] + Gx1[45]*Gx2[53] + Gx1[46]*Gx2[63] + Gx1[47]*Gx2[73] + Gx1[48]*Gx2[83] + Gx1[49]*Gx2[93];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[24] + Gx1[43]*Gx2[34] + Gx1[44]*Gx2[44] + Gx1[45]*Gx2[54] + Gx1[46]*Gx2[64] + Gx1[47]*Gx2[74] + Gx1[48]*Gx2[84] + Gx1[49]*Gx2[94];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[25] + Gx1[43]*Gx2[35] + Gx1[44]*Gx2[45] + Gx1[45]*Gx2[55] + Gx1[46]*Gx2[65] + Gx1[47]*Gx2[75] + Gx1[48]*Gx2[85] + Gx1[49]*Gx2[95];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[16] + Gx1[42]*Gx2[26] + Gx1[43]*Gx2[36] + Gx1[44]*Gx2[46] + Gx1[45]*Gx2[56] + Gx1[46]*Gx2[66] + Gx1[47]*Gx2[76] + Gx1[48]*Gx2[86] + Gx1[49]*Gx2[96];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[17] + Gx1[42]*Gx2[27] + Gx1[43]*Gx2[37] + Gx1[44]*Gx2[47] + Gx1[45]*Gx2[57] + Gx1[46]*Gx2[67] + Gx1[47]*Gx2[77] + Gx1[48]*Gx2[87] + Gx1[49]*Gx2[97];
Gx3[48] = + Gx1[40]*Gx2[8] + Gx1[41]*Gx2[18] + Gx1[42]*Gx2[28] + Gx1[43]*Gx2[38] + Gx1[44]*Gx2[48] + Gx1[45]*Gx2[58] + Gx1[46]*Gx2[68] + Gx1[47]*Gx2[78] + Gx1[48]*Gx2[88] + Gx1[49]*Gx2[98];
Gx3[49] = + Gx1[40]*Gx2[9] + Gx1[41]*Gx2[19] + Gx1[42]*Gx2[29] + Gx1[43]*Gx2[39] + Gx1[44]*Gx2[49] + Gx1[45]*Gx2[59] + Gx1[46]*Gx2[69] + Gx1[47]*Gx2[79] + Gx1[48]*Gx2[89] + Gx1[49]*Gx2[99];
Gx3[50] = + Gx1[50]*Gx2[0] + Gx1[51]*Gx2[10] + Gx1[52]*Gx2[20] + Gx1[53]*Gx2[30] + Gx1[54]*Gx2[40] + Gx1[55]*Gx2[50] + Gx1[56]*Gx2[60] + Gx1[57]*Gx2[70] + Gx1[58]*Gx2[80] + Gx1[59]*Gx2[90];
Gx3[51] = + Gx1[50]*Gx2[1] + Gx1[51]*Gx2[11] + Gx1[52]*Gx2[21] + Gx1[53]*Gx2[31] + Gx1[54]*Gx2[41] + Gx1[55]*Gx2[51] + Gx1[56]*Gx2[61] + Gx1[57]*Gx2[71] + Gx1[58]*Gx2[81] + Gx1[59]*Gx2[91];
Gx3[52] = + Gx1[50]*Gx2[2] + Gx1[51]*Gx2[12] + Gx1[52]*Gx2[22] + Gx1[53]*Gx2[32] + Gx1[54]*Gx2[42] + Gx1[55]*Gx2[52] + Gx1[56]*Gx2[62] + Gx1[57]*Gx2[72] + Gx1[58]*Gx2[82] + Gx1[59]*Gx2[92];
Gx3[53] = + Gx1[50]*Gx2[3] + Gx1[51]*Gx2[13] + Gx1[52]*Gx2[23] + Gx1[53]*Gx2[33] + Gx1[54]*Gx2[43] + Gx1[55]*Gx2[53] + Gx1[56]*Gx2[63] + Gx1[57]*Gx2[73] + Gx1[58]*Gx2[83] + Gx1[59]*Gx2[93];
Gx3[54] = + Gx1[50]*Gx2[4] + Gx1[51]*Gx2[14] + Gx1[52]*Gx2[24] + Gx1[53]*Gx2[34] + Gx1[54]*Gx2[44] + Gx1[55]*Gx2[54] + Gx1[56]*Gx2[64] + Gx1[57]*Gx2[74] + Gx1[58]*Gx2[84] + Gx1[59]*Gx2[94];
Gx3[55] = + Gx1[50]*Gx2[5] + Gx1[51]*Gx2[15] + Gx1[52]*Gx2[25] + Gx1[53]*Gx2[35] + Gx1[54]*Gx2[45] + Gx1[55]*Gx2[55] + Gx1[56]*Gx2[65] + Gx1[57]*Gx2[75] + Gx1[58]*Gx2[85] + Gx1[59]*Gx2[95];
Gx3[56] = + Gx1[50]*Gx2[6] + Gx1[51]*Gx2[16] + Gx1[52]*Gx2[26] + Gx1[53]*Gx2[36] + Gx1[54]*Gx2[46] + Gx1[55]*Gx2[56] + Gx1[56]*Gx2[66] + Gx1[57]*Gx2[76] + Gx1[58]*Gx2[86] + Gx1[59]*Gx2[96];
Gx3[57] = + Gx1[50]*Gx2[7] + Gx1[51]*Gx2[17] + Gx1[52]*Gx2[27] + Gx1[53]*Gx2[37] + Gx1[54]*Gx2[47] + Gx1[55]*Gx2[57] + Gx1[56]*Gx2[67] + Gx1[57]*Gx2[77] + Gx1[58]*Gx2[87] + Gx1[59]*Gx2[97];
Gx3[58] = + Gx1[50]*Gx2[8] + Gx1[51]*Gx2[18] + Gx1[52]*Gx2[28] + Gx1[53]*Gx2[38] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[58] + Gx1[56]*Gx2[68] + Gx1[57]*Gx2[78] + Gx1[58]*Gx2[88] + Gx1[59]*Gx2[98];
Gx3[59] = + Gx1[50]*Gx2[9] + Gx1[51]*Gx2[19] + Gx1[52]*Gx2[29] + Gx1[53]*Gx2[39] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[59] + Gx1[56]*Gx2[69] + Gx1[57]*Gx2[79] + Gx1[58]*Gx2[89] + Gx1[59]*Gx2[99];
Gx3[60] = + Gx1[60]*Gx2[0] + Gx1[61]*Gx2[10] + Gx1[62]*Gx2[20] + Gx1[63]*Gx2[30] + Gx1[64]*Gx2[40] + Gx1[65]*Gx2[50] + Gx1[66]*Gx2[60] + Gx1[67]*Gx2[70] + Gx1[68]*Gx2[80] + Gx1[69]*Gx2[90];
Gx3[61] = + Gx1[60]*Gx2[1] + Gx1[61]*Gx2[11] + Gx1[62]*Gx2[21] + Gx1[63]*Gx2[31] + Gx1[64]*Gx2[41] + Gx1[65]*Gx2[51] + Gx1[66]*Gx2[61] + Gx1[67]*Gx2[71] + Gx1[68]*Gx2[81] + Gx1[69]*Gx2[91];
Gx3[62] = + Gx1[60]*Gx2[2] + Gx1[61]*Gx2[12] + Gx1[62]*Gx2[22] + Gx1[63]*Gx2[32] + Gx1[64]*Gx2[42] + Gx1[65]*Gx2[52] + Gx1[66]*Gx2[62] + Gx1[67]*Gx2[72] + Gx1[68]*Gx2[82] + Gx1[69]*Gx2[92];
Gx3[63] = + Gx1[60]*Gx2[3] + Gx1[61]*Gx2[13] + Gx1[62]*Gx2[23] + Gx1[63]*Gx2[33] + Gx1[64]*Gx2[43] + Gx1[65]*Gx2[53] + Gx1[66]*Gx2[63] + Gx1[67]*Gx2[73] + Gx1[68]*Gx2[83] + Gx1[69]*Gx2[93];
Gx3[64] = + Gx1[60]*Gx2[4] + Gx1[61]*Gx2[14] + Gx1[62]*Gx2[24] + Gx1[63]*Gx2[34] + Gx1[64]*Gx2[44] + Gx1[65]*Gx2[54] + Gx1[66]*Gx2[64] + Gx1[67]*Gx2[74] + Gx1[68]*Gx2[84] + Gx1[69]*Gx2[94];
Gx3[65] = + Gx1[60]*Gx2[5] + Gx1[61]*Gx2[15] + Gx1[62]*Gx2[25] + Gx1[63]*Gx2[35] + Gx1[64]*Gx2[45] + Gx1[65]*Gx2[55] + Gx1[66]*Gx2[65] + Gx1[67]*Gx2[75] + Gx1[68]*Gx2[85] + Gx1[69]*Gx2[95];
Gx3[66] = + Gx1[60]*Gx2[6] + Gx1[61]*Gx2[16] + Gx1[62]*Gx2[26] + Gx1[63]*Gx2[36] + Gx1[64]*Gx2[46] + Gx1[65]*Gx2[56] + Gx1[66]*Gx2[66] + Gx1[67]*Gx2[76] + Gx1[68]*Gx2[86] + Gx1[69]*Gx2[96];
Gx3[67] = + Gx1[60]*Gx2[7] + Gx1[61]*Gx2[17] + Gx1[62]*Gx2[27] + Gx1[63]*Gx2[37] + Gx1[64]*Gx2[47] + Gx1[65]*Gx2[57] + Gx1[66]*Gx2[67] + Gx1[67]*Gx2[77] + Gx1[68]*Gx2[87] + Gx1[69]*Gx2[97];
Gx3[68] = + Gx1[60]*Gx2[8] + Gx1[61]*Gx2[18] + Gx1[62]*Gx2[28] + Gx1[63]*Gx2[38] + Gx1[64]*Gx2[48] + Gx1[65]*Gx2[58] + Gx1[66]*Gx2[68] + Gx1[67]*Gx2[78] + Gx1[68]*Gx2[88] + Gx1[69]*Gx2[98];
Gx3[69] = + Gx1[60]*Gx2[9] + Gx1[61]*Gx2[19] + Gx1[62]*Gx2[29] + Gx1[63]*Gx2[39] + Gx1[64]*Gx2[49] + Gx1[65]*Gx2[59] + Gx1[66]*Gx2[69] + Gx1[67]*Gx2[79] + Gx1[68]*Gx2[89] + Gx1[69]*Gx2[99];
Gx3[70] = + Gx1[70]*Gx2[0] + Gx1[71]*Gx2[10] + Gx1[72]*Gx2[20] + Gx1[73]*Gx2[30] + Gx1[74]*Gx2[40] + Gx1[75]*Gx2[50] + Gx1[76]*Gx2[60] + Gx1[77]*Gx2[70] + Gx1[78]*Gx2[80] + Gx1[79]*Gx2[90];
Gx3[71] = + Gx1[70]*Gx2[1] + Gx1[71]*Gx2[11] + Gx1[72]*Gx2[21] + Gx1[73]*Gx2[31] + Gx1[74]*Gx2[41] + Gx1[75]*Gx2[51] + Gx1[76]*Gx2[61] + Gx1[77]*Gx2[71] + Gx1[78]*Gx2[81] + Gx1[79]*Gx2[91];
Gx3[72] = + Gx1[70]*Gx2[2] + Gx1[71]*Gx2[12] + Gx1[72]*Gx2[22] + Gx1[73]*Gx2[32] + Gx1[74]*Gx2[42] + Gx1[75]*Gx2[52] + Gx1[76]*Gx2[62] + Gx1[77]*Gx2[72] + Gx1[78]*Gx2[82] + Gx1[79]*Gx2[92];
Gx3[73] = + Gx1[70]*Gx2[3] + Gx1[71]*Gx2[13] + Gx1[72]*Gx2[23] + Gx1[73]*Gx2[33] + Gx1[74]*Gx2[43] + Gx1[75]*Gx2[53] + Gx1[76]*Gx2[63] + Gx1[77]*Gx2[73] + Gx1[78]*Gx2[83] + Gx1[79]*Gx2[93];
Gx3[74] = + Gx1[70]*Gx2[4] + Gx1[71]*Gx2[14] + Gx1[72]*Gx2[24] + Gx1[73]*Gx2[34] + Gx1[74]*Gx2[44] + Gx1[75]*Gx2[54] + Gx1[76]*Gx2[64] + Gx1[77]*Gx2[74] + Gx1[78]*Gx2[84] + Gx1[79]*Gx2[94];
Gx3[75] = + Gx1[70]*Gx2[5] + Gx1[71]*Gx2[15] + Gx1[72]*Gx2[25] + Gx1[73]*Gx2[35] + Gx1[74]*Gx2[45] + Gx1[75]*Gx2[55] + Gx1[76]*Gx2[65] + Gx1[77]*Gx2[75] + Gx1[78]*Gx2[85] + Gx1[79]*Gx2[95];
Gx3[76] = + Gx1[70]*Gx2[6] + Gx1[71]*Gx2[16] + Gx1[72]*Gx2[26] + Gx1[73]*Gx2[36] + Gx1[74]*Gx2[46] + Gx1[75]*Gx2[56] + Gx1[76]*Gx2[66] + Gx1[77]*Gx2[76] + Gx1[78]*Gx2[86] + Gx1[79]*Gx2[96];
Gx3[77] = + Gx1[70]*Gx2[7] + Gx1[71]*Gx2[17] + Gx1[72]*Gx2[27] + Gx1[73]*Gx2[37] + Gx1[74]*Gx2[47] + Gx1[75]*Gx2[57] + Gx1[76]*Gx2[67] + Gx1[77]*Gx2[77] + Gx1[78]*Gx2[87] + Gx1[79]*Gx2[97];
Gx3[78] = + Gx1[70]*Gx2[8] + Gx1[71]*Gx2[18] + Gx1[72]*Gx2[28] + Gx1[73]*Gx2[38] + Gx1[74]*Gx2[48] + Gx1[75]*Gx2[58] + Gx1[76]*Gx2[68] + Gx1[77]*Gx2[78] + Gx1[78]*Gx2[88] + Gx1[79]*Gx2[98];
Gx3[79] = + Gx1[70]*Gx2[9] + Gx1[71]*Gx2[19] + Gx1[72]*Gx2[29] + Gx1[73]*Gx2[39] + Gx1[74]*Gx2[49] + Gx1[75]*Gx2[59] + Gx1[76]*Gx2[69] + Gx1[77]*Gx2[79] + Gx1[78]*Gx2[89] + Gx1[79]*Gx2[99];
Gx3[80] = + Gx1[80]*Gx2[0] + Gx1[81]*Gx2[10] + Gx1[82]*Gx2[20] + Gx1[83]*Gx2[30] + Gx1[84]*Gx2[40] + Gx1[85]*Gx2[50] + Gx1[86]*Gx2[60] + Gx1[87]*Gx2[70] + Gx1[88]*Gx2[80] + Gx1[89]*Gx2[90];
Gx3[81] = + Gx1[80]*Gx2[1] + Gx1[81]*Gx2[11] + Gx1[82]*Gx2[21] + Gx1[83]*Gx2[31] + Gx1[84]*Gx2[41] + Gx1[85]*Gx2[51] + Gx1[86]*Gx2[61] + Gx1[87]*Gx2[71] + Gx1[88]*Gx2[81] + Gx1[89]*Gx2[91];
Gx3[82] = + Gx1[80]*Gx2[2] + Gx1[81]*Gx2[12] + Gx1[82]*Gx2[22] + Gx1[83]*Gx2[32] + Gx1[84]*Gx2[42] + Gx1[85]*Gx2[52] + Gx1[86]*Gx2[62] + Gx1[87]*Gx2[72] + Gx1[88]*Gx2[82] + Gx1[89]*Gx2[92];
Gx3[83] = + Gx1[80]*Gx2[3] + Gx1[81]*Gx2[13] + Gx1[82]*Gx2[23] + Gx1[83]*Gx2[33] + Gx1[84]*Gx2[43] + Gx1[85]*Gx2[53] + Gx1[86]*Gx2[63] + Gx1[87]*Gx2[73] + Gx1[88]*Gx2[83] + Gx1[89]*Gx2[93];
Gx3[84] = + Gx1[80]*Gx2[4] + Gx1[81]*Gx2[14] + Gx1[82]*Gx2[24] + Gx1[83]*Gx2[34] + Gx1[84]*Gx2[44] + Gx1[85]*Gx2[54] + Gx1[86]*Gx2[64] + Gx1[87]*Gx2[74] + Gx1[88]*Gx2[84] + Gx1[89]*Gx2[94];
Gx3[85] = + Gx1[80]*Gx2[5] + Gx1[81]*Gx2[15] + Gx1[82]*Gx2[25] + Gx1[83]*Gx2[35] + Gx1[84]*Gx2[45] + Gx1[85]*Gx2[55] + Gx1[86]*Gx2[65] + Gx1[87]*Gx2[75] + Gx1[88]*Gx2[85] + Gx1[89]*Gx2[95];
Gx3[86] = + Gx1[80]*Gx2[6] + Gx1[81]*Gx2[16] + Gx1[82]*Gx2[26] + Gx1[83]*Gx2[36] + Gx1[84]*Gx2[46] + Gx1[85]*Gx2[56] + Gx1[86]*Gx2[66] + Gx1[87]*Gx2[76] + Gx1[88]*Gx2[86] + Gx1[89]*Gx2[96];
Gx3[87] = + Gx1[80]*Gx2[7] + Gx1[81]*Gx2[17] + Gx1[82]*Gx2[27] + Gx1[83]*Gx2[37] + Gx1[84]*Gx2[47] + Gx1[85]*Gx2[57] + Gx1[86]*Gx2[67] + Gx1[87]*Gx2[77] + Gx1[88]*Gx2[87] + Gx1[89]*Gx2[97];
Gx3[88] = + Gx1[80]*Gx2[8] + Gx1[81]*Gx2[18] + Gx1[82]*Gx2[28] + Gx1[83]*Gx2[38] + Gx1[84]*Gx2[48] + Gx1[85]*Gx2[58] + Gx1[86]*Gx2[68] + Gx1[87]*Gx2[78] + Gx1[88]*Gx2[88] + Gx1[89]*Gx2[98];
Gx3[89] = + Gx1[80]*Gx2[9] + Gx1[81]*Gx2[19] + Gx1[82]*Gx2[29] + Gx1[83]*Gx2[39] + Gx1[84]*Gx2[49] + Gx1[85]*Gx2[59] + Gx1[86]*Gx2[69] + Gx1[87]*Gx2[79] + Gx1[88]*Gx2[89] + Gx1[89]*Gx2[99];
Gx3[90] = + Gx1[90]*Gx2[0] + Gx1[91]*Gx2[10] + Gx1[92]*Gx2[20] + Gx1[93]*Gx2[30] + Gx1[94]*Gx2[40] + Gx1[95]*Gx2[50] + Gx1[96]*Gx2[60] + Gx1[97]*Gx2[70] + Gx1[98]*Gx2[80] + Gx1[99]*Gx2[90];
Gx3[91] = + Gx1[90]*Gx2[1] + Gx1[91]*Gx2[11] + Gx1[92]*Gx2[21] + Gx1[93]*Gx2[31] + Gx1[94]*Gx2[41] + Gx1[95]*Gx2[51] + Gx1[96]*Gx2[61] + Gx1[97]*Gx2[71] + Gx1[98]*Gx2[81] + Gx1[99]*Gx2[91];
Gx3[92] = + Gx1[90]*Gx2[2] + Gx1[91]*Gx2[12] + Gx1[92]*Gx2[22] + Gx1[93]*Gx2[32] + Gx1[94]*Gx2[42] + Gx1[95]*Gx2[52] + Gx1[96]*Gx2[62] + Gx1[97]*Gx2[72] + Gx1[98]*Gx2[82] + Gx1[99]*Gx2[92];
Gx3[93] = + Gx1[90]*Gx2[3] + Gx1[91]*Gx2[13] + Gx1[92]*Gx2[23] + Gx1[93]*Gx2[33] + Gx1[94]*Gx2[43] + Gx1[95]*Gx2[53] + Gx1[96]*Gx2[63] + Gx1[97]*Gx2[73] + Gx1[98]*Gx2[83] + Gx1[99]*Gx2[93];
Gx3[94] = + Gx1[90]*Gx2[4] + Gx1[91]*Gx2[14] + Gx1[92]*Gx2[24] + Gx1[93]*Gx2[34] + Gx1[94]*Gx2[44] + Gx1[95]*Gx2[54] + Gx1[96]*Gx2[64] + Gx1[97]*Gx2[74] + Gx1[98]*Gx2[84] + Gx1[99]*Gx2[94];
Gx3[95] = + Gx1[90]*Gx2[5] + Gx1[91]*Gx2[15] + Gx1[92]*Gx2[25] + Gx1[93]*Gx2[35] + Gx1[94]*Gx2[45] + Gx1[95]*Gx2[55] + Gx1[96]*Gx2[65] + Gx1[97]*Gx2[75] + Gx1[98]*Gx2[85] + Gx1[99]*Gx2[95];
Gx3[96] = + Gx1[90]*Gx2[6] + Gx1[91]*Gx2[16] + Gx1[92]*Gx2[26] + Gx1[93]*Gx2[36] + Gx1[94]*Gx2[46] + Gx1[95]*Gx2[56] + Gx1[96]*Gx2[66] + Gx1[97]*Gx2[76] + Gx1[98]*Gx2[86] + Gx1[99]*Gx2[96];
Gx3[97] = + Gx1[90]*Gx2[7] + Gx1[91]*Gx2[17] + Gx1[92]*Gx2[27] + Gx1[93]*Gx2[37] + Gx1[94]*Gx2[47] + Gx1[95]*Gx2[57] + Gx1[96]*Gx2[67] + Gx1[97]*Gx2[77] + Gx1[98]*Gx2[87] + Gx1[99]*Gx2[97];
Gx3[98] = + Gx1[90]*Gx2[8] + Gx1[91]*Gx2[18] + Gx1[92]*Gx2[28] + Gx1[93]*Gx2[38] + Gx1[94]*Gx2[48] + Gx1[95]*Gx2[58] + Gx1[96]*Gx2[68] + Gx1[97]*Gx2[78] + Gx1[98]*Gx2[88] + Gx1[99]*Gx2[98];
Gx3[99] = + Gx1[90]*Gx2[9] + Gx1[91]*Gx2[19] + Gx1[92]*Gx2[29] + Gx1[93]*Gx2[39] + Gx1[94]*Gx2[49] + Gx1[95]*Gx2[59] + Gx1[96]*Gx2[69] + Gx1[97]*Gx2[79] + Gx1[98]*Gx2[89] + Gx1[99]*Gx2[99];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24] + Gx1[9]*Gu1[27];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25] + Gx1[9]*Gu1[28];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26] + Gx1[9]*Gu1[29];
Gu2[3] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[6] + Gx1[13]*Gu1[9] + Gx1[14]*Gu1[12] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[21] + Gx1[18]*Gu1[24] + Gx1[19]*Gu1[27];
Gu2[4] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[12]*Gu1[7] + Gx1[13]*Gu1[10] + Gx1[14]*Gu1[13] + Gx1[15]*Gu1[16] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[22] + Gx1[18]*Gu1[25] + Gx1[19]*Gu1[28];
Gu2[5] = + Gx1[10]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[11] + Gx1[14]*Gu1[14] + Gx1[15]*Gu1[17] + Gx1[16]*Gu1[20] + Gx1[17]*Gu1[23] + Gx1[18]*Gu1[26] + Gx1[19]*Gu1[29];
Gu2[6] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[6] + Gx1[23]*Gu1[9] + Gx1[24]*Gu1[12] + Gx1[25]*Gu1[15] + Gx1[26]*Gu1[18] + Gx1[27]*Gu1[21] + Gx1[28]*Gu1[24] + Gx1[29]*Gu1[27];
Gu2[7] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[4] + Gx1[22]*Gu1[7] + Gx1[23]*Gu1[10] + Gx1[24]*Gu1[13] + Gx1[25]*Gu1[16] + Gx1[26]*Gu1[19] + Gx1[27]*Gu1[22] + Gx1[28]*Gu1[25] + Gx1[29]*Gu1[28];
Gu2[8] = + Gx1[20]*Gu1[2] + Gx1[21]*Gu1[5] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[11] + Gx1[24]*Gu1[14] + Gx1[25]*Gu1[17] + Gx1[26]*Gu1[20] + Gx1[27]*Gu1[23] + Gx1[28]*Gu1[26] + Gx1[29]*Gu1[29];
Gu2[9] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[3] + Gx1[32]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[34]*Gu1[12] + Gx1[35]*Gu1[15] + Gx1[36]*Gu1[18] + Gx1[37]*Gu1[21] + Gx1[38]*Gu1[24] + Gx1[39]*Gu1[27];
Gu2[10] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[4] + Gx1[32]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[34]*Gu1[13] + Gx1[35]*Gu1[16] + Gx1[36]*Gu1[19] + Gx1[37]*Gu1[22] + Gx1[38]*Gu1[25] + Gx1[39]*Gu1[28];
Gu2[11] = + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[5] + Gx1[32]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[34]*Gu1[14] + Gx1[35]*Gu1[17] + Gx1[36]*Gu1[20] + Gx1[37]*Gu1[23] + Gx1[38]*Gu1[26] + Gx1[39]*Gu1[29];
Gu2[12] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[3] + Gx1[42]*Gu1[6] + Gx1[43]*Gu1[9] + Gx1[44]*Gu1[12] + Gx1[45]*Gu1[15] + Gx1[46]*Gu1[18] + Gx1[47]*Gu1[21] + Gx1[48]*Gu1[24] + Gx1[49]*Gu1[27];
Gu2[13] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[4] + Gx1[42]*Gu1[7] + Gx1[43]*Gu1[10] + Gx1[44]*Gu1[13] + Gx1[45]*Gu1[16] + Gx1[46]*Gu1[19] + Gx1[47]*Gu1[22] + Gx1[48]*Gu1[25] + Gx1[49]*Gu1[28];
Gu2[14] = + Gx1[40]*Gu1[2] + Gx1[41]*Gu1[5] + Gx1[42]*Gu1[8] + Gx1[43]*Gu1[11] + Gx1[44]*Gu1[14] + Gx1[45]*Gu1[17] + Gx1[46]*Gu1[20] + Gx1[47]*Gu1[23] + Gx1[48]*Gu1[26] + Gx1[49]*Gu1[29];
Gu2[15] = + Gx1[50]*Gu1[0] + Gx1[51]*Gu1[3] + Gx1[52]*Gu1[6] + Gx1[53]*Gu1[9] + Gx1[54]*Gu1[12] + Gx1[55]*Gu1[15] + Gx1[56]*Gu1[18] + Gx1[57]*Gu1[21] + Gx1[58]*Gu1[24] + Gx1[59]*Gu1[27];
Gu2[16] = + Gx1[50]*Gu1[1] + Gx1[51]*Gu1[4] + Gx1[52]*Gu1[7] + Gx1[53]*Gu1[10] + Gx1[54]*Gu1[13] + Gx1[55]*Gu1[16] + Gx1[56]*Gu1[19] + Gx1[57]*Gu1[22] + Gx1[58]*Gu1[25] + Gx1[59]*Gu1[28];
Gu2[17] = + Gx1[50]*Gu1[2] + Gx1[51]*Gu1[5] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[11] + Gx1[54]*Gu1[14] + Gx1[55]*Gu1[17] + Gx1[56]*Gu1[20] + Gx1[57]*Gu1[23] + Gx1[58]*Gu1[26] + Gx1[59]*Gu1[29];
Gu2[18] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[3] + Gx1[62]*Gu1[6] + Gx1[63]*Gu1[9] + Gx1[64]*Gu1[12] + Gx1[65]*Gu1[15] + Gx1[66]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[68]*Gu1[24] + Gx1[69]*Gu1[27];
Gu2[19] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[7] + Gx1[63]*Gu1[10] + Gx1[64]*Gu1[13] + Gx1[65]*Gu1[16] + Gx1[66]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[68]*Gu1[25] + Gx1[69]*Gu1[28];
Gu2[20] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[11] + Gx1[64]*Gu1[14] + Gx1[65]*Gu1[17] + Gx1[66]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[68]*Gu1[26] + Gx1[69]*Gu1[29];
Gu2[21] = + Gx1[70]*Gu1[0] + Gx1[71]*Gu1[3] + Gx1[72]*Gu1[6] + Gx1[73]*Gu1[9] + Gx1[74]*Gu1[12] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[27];
Gu2[22] = + Gx1[70]*Gu1[1] + Gx1[71]*Gu1[4] + Gx1[72]*Gu1[7] + Gx1[73]*Gu1[10] + Gx1[74]*Gu1[13] + Gx1[75]*Gu1[16] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[28];
Gu2[23] = + Gx1[70]*Gu1[2] + Gx1[71]*Gu1[5] + Gx1[72]*Gu1[8] + Gx1[73]*Gu1[11] + Gx1[74]*Gu1[14] + Gx1[75]*Gu1[17] + Gx1[76]*Gu1[20] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[29];
Gu2[24] = + Gx1[80]*Gu1[0] + Gx1[81]*Gu1[3] + Gx1[82]*Gu1[6] + Gx1[83]*Gu1[9] + Gx1[84]*Gu1[12] + Gx1[85]*Gu1[15] + Gx1[86]*Gu1[18] + Gx1[87]*Gu1[21] + Gx1[88]*Gu1[24] + Gx1[89]*Gu1[27];
Gu2[25] = + Gx1[80]*Gu1[1] + Gx1[81]*Gu1[4] + Gx1[82]*Gu1[7] + Gx1[83]*Gu1[10] + Gx1[84]*Gu1[13] + Gx1[85]*Gu1[16] + Gx1[86]*Gu1[19] + Gx1[87]*Gu1[22] + Gx1[88]*Gu1[25] + Gx1[89]*Gu1[28];
Gu2[26] = + Gx1[80]*Gu1[2] + Gx1[81]*Gu1[5] + Gx1[82]*Gu1[8] + Gx1[83]*Gu1[11] + Gx1[84]*Gu1[14] + Gx1[85]*Gu1[17] + Gx1[86]*Gu1[20] + Gx1[87]*Gu1[23] + Gx1[88]*Gu1[26] + Gx1[89]*Gu1[29];
Gu2[27] = + Gx1[90]*Gu1[0] + Gx1[91]*Gu1[3] + Gx1[92]*Gu1[6] + Gx1[93]*Gu1[9] + Gx1[94]*Gu1[12] + Gx1[95]*Gu1[15] + Gx1[96]*Gu1[18] + Gx1[97]*Gu1[21] + Gx1[98]*Gu1[24] + Gx1[99]*Gu1[27];
Gu2[28] = + Gx1[90]*Gu1[1] + Gx1[91]*Gu1[4] + Gx1[92]*Gu1[7] + Gx1[93]*Gu1[10] + Gx1[94]*Gu1[13] + Gx1[95]*Gu1[16] + Gx1[96]*Gu1[19] + Gx1[97]*Gu1[22] + Gx1[98]*Gu1[25] + Gx1[99]*Gu1[28];
Gu2[29] = + Gx1[90]*Gu1[2] + Gx1[91]*Gu1[5] + Gx1[92]*Gu1[8] + Gx1[93]*Gu1[11] + Gx1[94]*Gu1[14] + Gx1[95]*Gu1[17] + Gx1[96]*Gu1[20] + Gx1[97]*Gu1[23] + Gx1[98]*Gu1[26] + Gx1[99]*Gu1[29];
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
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] = R11[0];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] = R11[3];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] = R11[4];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] = R11[5];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] = R11[6];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] = R11[7];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] = R11[8];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] = acadoWorkspace.H[(iCol * 90) + (iRow * 3)];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 90 + 30) + (iRow * 3)];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 90 + 60) + (iRow * 3)];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] = acadoWorkspace.H[(iCol * 90) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 90 + 30) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 90 + 60) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] = acadoWorkspace.H[(iCol * 90) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 90 + 30) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 90 + 60) + (iRow * 3 + 2)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9];
dNew[1] = + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4] + Gx1[15]*dOld[5] + Gx1[16]*dOld[6] + Gx1[17]*dOld[7] + Gx1[18]*dOld[8] + Gx1[19]*dOld[9];
dNew[2] = + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4] + Gx1[25]*dOld[5] + Gx1[26]*dOld[6] + Gx1[27]*dOld[7] + Gx1[28]*dOld[8] + Gx1[29]*dOld[9];
dNew[3] = + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5] + Gx1[36]*dOld[6] + Gx1[37]*dOld[7] + Gx1[38]*dOld[8] + Gx1[39]*dOld[9];
dNew[4] = + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7] + Gx1[48]*dOld[8] + Gx1[49]*dOld[9];
dNew[5] = + Gx1[50]*dOld[0] + Gx1[51]*dOld[1] + Gx1[52]*dOld[2] + Gx1[53]*dOld[3] + Gx1[54]*dOld[4] + Gx1[55]*dOld[5] + Gx1[56]*dOld[6] + Gx1[57]*dOld[7] + Gx1[58]*dOld[8] + Gx1[59]*dOld[9];
dNew[6] = + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9];
dNew[7] = + Gx1[70]*dOld[0] + Gx1[71]*dOld[1] + Gx1[72]*dOld[2] + Gx1[73]*dOld[3] + Gx1[74]*dOld[4] + Gx1[75]*dOld[5] + Gx1[76]*dOld[6] + Gx1[77]*dOld[7] + Gx1[78]*dOld[8] + Gx1[79]*dOld[9];
dNew[8] = + Gx1[80]*dOld[0] + Gx1[81]*dOld[1] + Gx1[82]*dOld[2] + Gx1[83]*dOld[3] + Gx1[84]*dOld[4] + Gx1[85]*dOld[5] + Gx1[86]*dOld[6] + Gx1[87]*dOld[7] + Gx1[88]*dOld[8] + Gx1[89]*dOld[9];
dNew[9] = + Gx1[90]*dOld[0] + Gx1[91]*dOld[1] + Gx1[92]*dOld[2] + Gx1[93]*dOld[3] + Gx1[94]*dOld[4] + Gx1[95]*dOld[5] + Gx1[96]*dOld[6] + Gx1[97]*dOld[7] + Gx1[98]*dOld[8] + Gx1[99]*dOld[9];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4] + acadoWorkspace.QN1[5]*dOld[5] + acadoWorkspace.QN1[6]*dOld[6] + acadoWorkspace.QN1[7]*dOld[7] + acadoWorkspace.QN1[8]*dOld[8] + acadoWorkspace.QN1[9]*dOld[9];
dNew[1] = + acadoWorkspace.QN1[10]*dOld[0] + acadoWorkspace.QN1[11]*dOld[1] + acadoWorkspace.QN1[12]*dOld[2] + acadoWorkspace.QN1[13]*dOld[3] + acadoWorkspace.QN1[14]*dOld[4] + acadoWorkspace.QN1[15]*dOld[5] + acadoWorkspace.QN1[16]*dOld[6] + acadoWorkspace.QN1[17]*dOld[7] + acadoWorkspace.QN1[18]*dOld[8] + acadoWorkspace.QN1[19]*dOld[9];
dNew[2] = + acadoWorkspace.QN1[20]*dOld[0] + acadoWorkspace.QN1[21]*dOld[1] + acadoWorkspace.QN1[22]*dOld[2] + acadoWorkspace.QN1[23]*dOld[3] + acadoWorkspace.QN1[24]*dOld[4] + acadoWorkspace.QN1[25]*dOld[5] + acadoWorkspace.QN1[26]*dOld[6] + acadoWorkspace.QN1[27]*dOld[7] + acadoWorkspace.QN1[28]*dOld[8] + acadoWorkspace.QN1[29]*dOld[9];
dNew[3] = + acadoWorkspace.QN1[30]*dOld[0] + acadoWorkspace.QN1[31]*dOld[1] + acadoWorkspace.QN1[32]*dOld[2] + acadoWorkspace.QN1[33]*dOld[3] + acadoWorkspace.QN1[34]*dOld[4] + acadoWorkspace.QN1[35]*dOld[5] + acadoWorkspace.QN1[36]*dOld[6] + acadoWorkspace.QN1[37]*dOld[7] + acadoWorkspace.QN1[38]*dOld[8] + acadoWorkspace.QN1[39]*dOld[9];
dNew[4] = + acadoWorkspace.QN1[40]*dOld[0] + acadoWorkspace.QN1[41]*dOld[1] + acadoWorkspace.QN1[42]*dOld[2] + acadoWorkspace.QN1[43]*dOld[3] + acadoWorkspace.QN1[44]*dOld[4] + acadoWorkspace.QN1[45]*dOld[5] + acadoWorkspace.QN1[46]*dOld[6] + acadoWorkspace.QN1[47]*dOld[7] + acadoWorkspace.QN1[48]*dOld[8] + acadoWorkspace.QN1[49]*dOld[9];
dNew[5] = + acadoWorkspace.QN1[50]*dOld[0] + acadoWorkspace.QN1[51]*dOld[1] + acadoWorkspace.QN1[52]*dOld[2] + acadoWorkspace.QN1[53]*dOld[3] + acadoWorkspace.QN1[54]*dOld[4] + acadoWorkspace.QN1[55]*dOld[5] + acadoWorkspace.QN1[56]*dOld[6] + acadoWorkspace.QN1[57]*dOld[7] + acadoWorkspace.QN1[58]*dOld[8] + acadoWorkspace.QN1[59]*dOld[9];
dNew[6] = + acadoWorkspace.QN1[60]*dOld[0] + acadoWorkspace.QN1[61]*dOld[1] + acadoWorkspace.QN1[62]*dOld[2] + acadoWorkspace.QN1[63]*dOld[3] + acadoWorkspace.QN1[64]*dOld[4] + acadoWorkspace.QN1[65]*dOld[5] + acadoWorkspace.QN1[66]*dOld[6] + acadoWorkspace.QN1[67]*dOld[7] + acadoWorkspace.QN1[68]*dOld[8] + acadoWorkspace.QN1[69]*dOld[9];
dNew[7] = + acadoWorkspace.QN1[70]*dOld[0] + acadoWorkspace.QN1[71]*dOld[1] + acadoWorkspace.QN1[72]*dOld[2] + acadoWorkspace.QN1[73]*dOld[3] + acadoWorkspace.QN1[74]*dOld[4] + acadoWorkspace.QN1[75]*dOld[5] + acadoWorkspace.QN1[76]*dOld[6] + acadoWorkspace.QN1[77]*dOld[7] + acadoWorkspace.QN1[78]*dOld[8] + acadoWorkspace.QN1[79]*dOld[9];
dNew[8] = + acadoWorkspace.QN1[80]*dOld[0] + acadoWorkspace.QN1[81]*dOld[1] + acadoWorkspace.QN1[82]*dOld[2] + acadoWorkspace.QN1[83]*dOld[3] + acadoWorkspace.QN1[84]*dOld[4] + acadoWorkspace.QN1[85]*dOld[5] + acadoWorkspace.QN1[86]*dOld[6] + acadoWorkspace.QN1[87]*dOld[7] + acadoWorkspace.QN1[88]*dOld[8] + acadoWorkspace.QN1[89]*dOld[9];
dNew[9] = + acadoWorkspace.QN1[90]*dOld[0] + acadoWorkspace.QN1[91]*dOld[1] + acadoWorkspace.QN1[92]*dOld[2] + acadoWorkspace.QN1[93]*dOld[3] + acadoWorkspace.QN1[94]*dOld[4] + acadoWorkspace.QN1[95]*dOld[5] + acadoWorkspace.QN1[96]*dOld[6] + acadoWorkspace.QN1[97]*dOld[7] + acadoWorkspace.QN1[98]*dOld[8] + acadoWorkspace.QN1[99]*dOld[9];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5];
RDy1[1] = + R2[6]*Dy1[0] + R2[7]*Dy1[1] + R2[8]*Dy1[2] + R2[9]*Dy1[3] + R2[10]*Dy1[4] + R2[11]*Dy1[5];
RDy1[2] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5];
QDy1[1] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2] + Q2[9]*Dy1[3] + Q2[10]*Dy1[4] + Q2[11]*Dy1[5];
QDy1[2] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5];
QDy1[3] = + Q2[18]*Dy1[0] + Q2[19]*Dy1[1] + Q2[20]*Dy1[2] + Q2[21]*Dy1[3] + Q2[22]*Dy1[4] + Q2[23]*Dy1[5];
QDy1[4] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5];
QDy1[5] = + Q2[30]*Dy1[0] + Q2[31]*Dy1[1] + Q2[32]*Dy1[2] + Q2[33]*Dy1[3] + Q2[34]*Dy1[4] + Q2[35]*Dy1[5];
QDy1[6] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5];
QDy1[7] = + Q2[42]*Dy1[0] + Q2[43]*Dy1[1] + Q2[44]*Dy1[2] + Q2[45]*Dy1[3] + Q2[46]*Dy1[4] + Q2[47]*Dy1[5];
QDy1[8] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5];
QDy1[9] = + Q2[54]*Dy1[0] + Q2[55]*Dy1[1] + Q2[56]*Dy1[2] + Q2[57]*Dy1[3] + Q2[58]*Dy1[4] + Q2[59]*Dy1[5];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[3]*QDy1[1] + E1[6]*QDy1[2] + E1[9]*QDy1[3] + E1[12]*QDy1[4] + E1[15]*QDy1[5] + E1[18]*QDy1[6] + E1[21]*QDy1[7] + E1[24]*QDy1[8] + E1[27]*QDy1[9];
U1[1] += + E1[1]*QDy1[0] + E1[4]*QDy1[1] + E1[7]*QDy1[2] + E1[10]*QDy1[3] + E1[13]*QDy1[4] + E1[16]*QDy1[5] + E1[19]*QDy1[6] + E1[22]*QDy1[7] + E1[25]*QDy1[8] + E1[28]*QDy1[9];
U1[2] += + E1[2]*QDy1[0] + E1[5]*QDy1[1] + E1[8]*QDy1[2] + E1[11]*QDy1[3] + E1[14]*QDy1[4] + E1[17]*QDy1[5] + E1[20]*QDy1[6] + E1[23]*QDy1[7] + E1[26]*QDy1[8] + E1[29]*QDy1[9];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[3]*Gx1[10] + E1[6]*Gx1[20] + E1[9]*Gx1[30] + E1[12]*Gx1[40] + E1[15]*Gx1[50] + E1[18]*Gx1[60] + E1[21]*Gx1[70] + E1[24]*Gx1[80] + E1[27]*Gx1[90];
H101[1] += + E1[0]*Gx1[1] + E1[3]*Gx1[11] + E1[6]*Gx1[21] + E1[9]*Gx1[31] + E1[12]*Gx1[41] + E1[15]*Gx1[51] + E1[18]*Gx1[61] + E1[21]*Gx1[71] + E1[24]*Gx1[81] + E1[27]*Gx1[91];
H101[2] += + E1[0]*Gx1[2] + E1[3]*Gx1[12] + E1[6]*Gx1[22] + E1[9]*Gx1[32] + E1[12]*Gx1[42] + E1[15]*Gx1[52] + E1[18]*Gx1[62] + E1[21]*Gx1[72] + E1[24]*Gx1[82] + E1[27]*Gx1[92];
H101[3] += + E1[0]*Gx1[3] + E1[3]*Gx1[13] + E1[6]*Gx1[23] + E1[9]*Gx1[33] + E1[12]*Gx1[43] + E1[15]*Gx1[53] + E1[18]*Gx1[63] + E1[21]*Gx1[73] + E1[24]*Gx1[83] + E1[27]*Gx1[93];
H101[4] += + E1[0]*Gx1[4] + E1[3]*Gx1[14] + E1[6]*Gx1[24] + E1[9]*Gx1[34] + E1[12]*Gx1[44] + E1[15]*Gx1[54] + E1[18]*Gx1[64] + E1[21]*Gx1[74] + E1[24]*Gx1[84] + E1[27]*Gx1[94];
H101[5] += + E1[0]*Gx1[5] + E1[3]*Gx1[15] + E1[6]*Gx1[25] + E1[9]*Gx1[35] + E1[12]*Gx1[45] + E1[15]*Gx1[55] + E1[18]*Gx1[65] + E1[21]*Gx1[75] + E1[24]*Gx1[85] + E1[27]*Gx1[95];
H101[6] += + E1[0]*Gx1[6] + E1[3]*Gx1[16] + E1[6]*Gx1[26] + E1[9]*Gx1[36] + E1[12]*Gx1[46] + E1[15]*Gx1[56] + E1[18]*Gx1[66] + E1[21]*Gx1[76] + E1[24]*Gx1[86] + E1[27]*Gx1[96];
H101[7] += + E1[0]*Gx1[7] + E1[3]*Gx1[17] + E1[6]*Gx1[27] + E1[9]*Gx1[37] + E1[12]*Gx1[47] + E1[15]*Gx1[57] + E1[18]*Gx1[67] + E1[21]*Gx1[77] + E1[24]*Gx1[87] + E1[27]*Gx1[97];
H101[8] += + E1[0]*Gx1[8] + E1[3]*Gx1[18] + E1[6]*Gx1[28] + E1[9]*Gx1[38] + E1[12]*Gx1[48] + E1[15]*Gx1[58] + E1[18]*Gx1[68] + E1[21]*Gx1[78] + E1[24]*Gx1[88] + E1[27]*Gx1[98];
H101[9] += + E1[0]*Gx1[9] + E1[3]*Gx1[19] + E1[6]*Gx1[29] + E1[9]*Gx1[39] + E1[12]*Gx1[49] + E1[15]*Gx1[59] + E1[18]*Gx1[69] + E1[21]*Gx1[79] + E1[24]*Gx1[89] + E1[27]*Gx1[99];
H101[10] += + E1[1]*Gx1[0] + E1[4]*Gx1[10] + E1[7]*Gx1[20] + E1[10]*Gx1[30] + E1[13]*Gx1[40] + E1[16]*Gx1[50] + E1[19]*Gx1[60] + E1[22]*Gx1[70] + E1[25]*Gx1[80] + E1[28]*Gx1[90];
H101[11] += + E1[1]*Gx1[1] + E1[4]*Gx1[11] + E1[7]*Gx1[21] + E1[10]*Gx1[31] + E1[13]*Gx1[41] + E1[16]*Gx1[51] + E1[19]*Gx1[61] + E1[22]*Gx1[71] + E1[25]*Gx1[81] + E1[28]*Gx1[91];
H101[12] += + E1[1]*Gx1[2] + E1[4]*Gx1[12] + E1[7]*Gx1[22] + E1[10]*Gx1[32] + E1[13]*Gx1[42] + E1[16]*Gx1[52] + E1[19]*Gx1[62] + E1[22]*Gx1[72] + E1[25]*Gx1[82] + E1[28]*Gx1[92];
H101[13] += + E1[1]*Gx1[3] + E1[4]*Gx1[13] + E1[7]*Gx1[23] + E1[10]*Gx1[33] + E1[13]*Gx1[43] + E1[16]*Gx1[53] + E1[19]*Gx1[63] + E1[22]*Gx1[73] + E1[25]*Gx1[83] + E1[28]*Gx1[93];
H101[14] += + E1[1]*Gx1[4] + E1[4]*Gx1[14] + E1[7]*Gx1[24] + E1[10]*Gx1[34] + E1[13]*Gx1[44] + E1[16]*Gx1[54] + E1[19]*Gx1[64] + E1[22]*Gx1[74] + E1[25]*Gx1[84] + E1[28]*Gx1[94];
H101[15] += + E1[1]*Gx1[5] + E1[4]*Gx1[15] + E1[7]*Gx1[25] + E1[10]*Gx1[35] + E1[13]*Gx1[45] + E1[16]*Gx1[55] + E1[19]*Gx1[65] + E1[22]*Gx1[75] + E1[25]*Gx1[85] + E1[28]*Gx1[95];
H101[16] += + E1[1]*Gx1[6] + E1[4]*Gx1[16] + E1[7]*Gx1[26] + E1[10]*Gx1[36] + E1[13]*Gx1[46] + E1[16]*Gx1[56] + E1[19]*Gx1[66] + E1[22]*Gx1[76] + E1[25]*Gx1[86] + E1[28]*Gx1[96];
H101[17] += + E1[1]*Gx1[7] + E1[4]*Gx1[17] + E1[7]*Gx1[27] + E1[10]*Gx1[37] + E1[13]*Gx1[47] + E1[16]*Gx1[57] + E1[19]*Gx1[67] + E1[22]*Gx1[77] + E1[25]*Gx1[87] + E1[28]*Gx1[97];
H101[18] += + E1[1]*Gx1[8] + E1[4]*Gx1[18] + E1[7]*Gx1[28] + E1[10]*Gx1[38] + E1[13]*Gx1[48] + E1[16]*Gx1[58] + E1[19]*Gx1[68] + E1[22]*Gx1[78] + E1[25]*Gx1[88] + E1[28]*Gx1[98];
H101[19] += + E1[1]*Gx1[9] + E1[4]*Gx1[19] + E1[7]*Gx1[29] + E1[10]*Gx1[39] + E1[13]*Gx1[49] + E1[16]*Gx1[59] + E1[19]*Gx1[69] + E1[22]*Gx1[79] + E1[25]*Gx1[89] + E1[28]*Gx1[99];
H101[20] += + E1[2]*Gx1[0] + E1[5]*Gx1[10] + E1[8]*Gx1[20] + E1[11]*Gx1[30] + E1[14]*Gx1[40] + E1[17]*Gx1[50] + E1[20]*Gx1[60] + E1[23]*Gx1[70] + E1[26]*Gx1[80] + E1[29]*Gx1[90];
H101[21] += + E1[2]*Gx1[1] + E1[5]*Gx1[11] + E1[8]*Gx1[21] + E1[11]*Gx1[31] + E1[14]*Gx1[41] + E1[17]*Gx1[51] + E1[20]*Gx1[61] + E1[23]*Gx1[71] + E1[26]*Gx1[81] + E1[29]*Gx1[91];
H101[22] += + E1[2]*Gx1[2] + E1[5]*Gx1[12] + E1[8]*Gx1[22] + E1[11]*Gx1[32] + E1[14]*Gx1[42] + E1[17]*Gx1[52] + E1[20]*Gx1[62] + E1[23]*Gx1[72] + E1[26]*Gx1[82] + E1[29]*Gx1[92];
H101[23] += + E1[2]*Gx1[3] + E1[5]*Gx1[13] + E1[8]*Gx1[23] + E1[11]*Gx1[33] + E1[14]*Gx1[43] + E1[17]*Gx1[53] + E1[20]*Gx1[63] + E1[23]*Gx1[73] + E1[26]*Gx1[83] + E1[29]*Gx1[93];
H101[24] += + E1[2]*Gx1[4] + E1[5]*Gx1[14] + E1[8]*Gx1[24] + E1[11]*Gx1[34] + E1[14]*Gx1[44] + E1[17]*Gx1[54] + E1[20]*Gx1[64] + E1[23]*Gx1[74] + E1[26]*Gx1[84] + E1[29]*Gx1[94];
H101[25] += + E1[2]*Gx1[5] + E1[5]*Gx1[15] + E1[8]*Gx1[25] + E1[11]*Gx1[35] + E1[14]*Gx1[45] + E1[17]*Gx1[55] + E1[20]*Gx1[65] + E1[23]*Gx1[75] + E1[26]*Gx1[85] + E1[29]*Gx1[95];
H101[26] += + E1[2]*Gx1[6] + E1[5]*Gx1[16] + E1[8]*Gx1[26] + E1[11]*Gx1[36] + E1[14]*Gx1[46] + E1[17]*Gx1[56] + E1[20]*Gx1[66] + E1[23]*Gx1[76] + E1[26]*Gx1[86] + E1[29]*Gx1[96];
H101[27] += + E1[2]*Gx1[7] + E1[5]*Gx1[17] + E1[8]*Gx1[27] + E1[11]*Gx1[37] + E1[14]*Gx1[47] + E1[17]*Gx1[57] + E1[20]*Gx1[67] + E1[23]*Gx1[77] + E1[26]*Gx1[87] + E1[29]*Gx1[97];
H101[28] += + E1[2]*Gx1[8] + E1[5]*Gx1[18] + E1[8]*Gx1[28] + E1[11]*Gx1[38] + E1[14]*Gx1[48] + E1[17]*Gx1[58] + E1[20]*Gx1[68] + E1[23]*Gx1[78] + E1[26]*Gx1[88] + E1[29]*Gx1[98];
H101[29] += + E1[2]*Gx1[9] + E1[5]*Gx1[19] + E1[8]*Gx1[29] + E1[11]*Gx1[39] + E1[14]*Gx1[49] + E1[17]*Gx1[59] + E1[20]*Gx1[69] + E1[23]*Gx1[79] + E1[26]*Gx1[89] + E1[29]*Gx1[99];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 30; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2];
dNew[1] += + E1[3]*U1[0] + E1[4]*U1[1] + E1[5]*U1[2];
dNew[2] += + E1[6]*U1[0] + E1[7]*U1[1] + E1[8]*U1[2];
dNew[3] += + E1[9]*U1[0] + E1[10]*U1[1] + E1[11]*U1[2];
dNew[4] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2];
dNew[5] += + E1[15]*U1[0] + E1[16]*U1[1] + E1[17]*U1[2];
dNew[6] += + E1[18]*U1[0] + E1[19]*U1[1] + E1[20]*U1[2];
dNew[7] += + E1[21]*U1[0] + E1[22]*U1[1] + E1[23]*U1[2];
dNew[8] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2];
dNew[9] += + E1[27]*U1[0] + E1[28]*U1[1] + E1[29]*U1[2];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[10] + Hx[2]*Gx[20] + Hx[3]*Gx[30] + Hx[4]*Gx[40] + Hx[5]*Gx[50] + Hx[6]*Gx[60] + Hx[7]*Gx[70] + Hx[8]*Gx[80] + Hx[9]*Gx[90];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[11] + Hx[2]*Gx[21] + Hx[3]*Gx[31] + Hx[4]*Gx[41] + Hx[5]*Gx[51] + Hx[6]*Gx[61] + Hx[7]*Gx[71] + Hx[8]*Gx[81] + Hx[9]*Gx[91];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[12] + Hx[2]*Gx[22] + Hx[3]*Gx[32] + Hx[4]*Gx[42] + Hx[5]*Gx[52] + Hx[6]*Gx[62] + Hx[7]*Gx[72] + Hx[8]*Gx[82] + Hx[9]*Gx[92];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[13] + Hx[2]*Gx[23] + Hx[3]*Gx[33] + Hx[4]*Gx[43] + Hx[5]*Gx[53] + Hx[6]*Gx[63] + Hx[7]*Gx[73] + Hx[8]*Gx[83] + Hx[9]*Gx[93];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[14] + Hx[2]*Gx[24] + Hx[3]*Gx[34] + Hx[4]*Gx[44] + Hx[5]*Gx[54] + Hx[6]*Gx[64] + Hx[7]*Gx[74] + Hx[8]*Gx[84] + Hx[9]*Gx[94];
A01[5] = + Hx[0]*Gx[5] + Hx[1]*Gx[15] + Hx[2]*Gx[25] + Hx[3]*Gx[35] + Hx[4]*Gx[45] + Hx[5]*Gx[55] + Hx[6]*Gx[65] + Hx[7]*Gx[75] + Hx[8]*Gx[85] + Hx[9]*Gx[95];
A01[6] = + Hx[0]*Gx[6] + Hx[1]*Gx[16] + Hx[2]*Gx[26] + Hx[3]*Gx[36] + Hx[4]*Gx[46] + Hx[5]*Gx[56] + Hx[6]*Gx[66] + Hx[7]*Gx[76] + Hx[8]*Gx[86] + Hx[9]*Gx[96];
A01[7] = + Hx[0]*Gx[7] + Hx[1]*Gx[17] + Hx[2]*Gx[27] + Hx[3]*Gx[37] + Hx[4]*Gx[47] + Hx[5]*Gx[57] + Hx[6]*Gx[67] + Hx[7]*Gx[77] + Hx[8]*Gx[87] + Hx[9]*Gx[97];
A01[8] = + Hx[0]*Gx[8] + Hx[1]*Gx[18] + Hx[2]*Gx[28] + Hx[3]*Gx[38] + Hx[4]*Gx[48] + Hx[5]*Gx[58] + Hx[6]*Gx[68] + Hx[7]*Gx[78] + Hx[8]*Gx[88] + Hx[9]*Gx[98];
A01[9] = + Hx[0]*Gx[9] + Hx[1]*Gx[19] + Hx[2]*Gx[29] + Hx[3]*Gx[39] + Hx[4]*Gx[49] + Hx[5]*Gx[59] + Hx[6]*Gx[69] + Hx[7]*Gx[79] + Hx[8]*Gx[89] + Hx[9]*Gx[99];
A01[10] = + Hx[10]*Gx[0] + Hx[11]*Gx[10] + Hx[12]*Gx[20] + Hx[13]*Gx[30] + Hx[14]*Gx[40] + Hx[15]*Gx[50] + Hx[16]*Gx[60] + Hx[17]*Gx[70] + Hx[18]*Gx[80] + Hx[19]*Gx[90];
A01[11] = + Hx[10]*Gx[1] + Hx[11]*Gx[11] + Hx[12]*Gx[21] + Hx[13]*Gx[31] + Hx[14]*Gx[41] + Hx[15]*Gx[51] + Hx[16]*Gx[61] + Hx[17]*Gx[71] + Hx[18]*Gx[81] + Hx[19]*Gx[91];
A01[12] = + Hx[10]*Gx[2] + Hx[11]*Gx[12] + Hx[12]*Gx[22] + Hx[13]*Gx[32] + Hx[14]*Gx[42] + Hx[15]*Gx[52] + Hx[16]*Gx[62] + Hx[17]*Gx[72] + Hx[18]*Gx[82] + Hx[19]*Gx[92];
A01[13] = + Hx[10]*Gx[3] + Hx[11]*Gx[13] + Hx[12]*Gx[23] + Hx[13]*Gx[33] + Hx[14]*Gx[43] + Hx[15]*Gx[53] + Hx[16]*Gx[63] + Hx[17]*Gx[73] + Hx[18]*Gx[83] + Hx[19]*Gx[93];
A01[14] = + Hx[10]*Gx[4] + Hx[11]*Gx[14] + Hx[12]*Gx[24] + Hx[13]*Gx[34] + Hx[14]*Gx[44] + Hx[15]*Gx[54] + Hx[16]*Gx[64] + Hx[17]*Gx[74] + Hx[18]*Gx[84] + Hx[19]*Gx[94];
A01[15] = + Hx[10]*Gx[5] + Hx[11]*Gx[15] + Hx[12]*Gx[25] + Hx[13]*Gx[35] + Hx[14]*Gx[45] + Hx[15]*Gx[55] + Hx[16]*Gx[65] + Hx[17]*Gx[75] + Hx[18]*Gx[85] + Hx[19]*Gx[95];
A01[16] = + Hx[10]*Gx[6] + Hx[11]*Gx[16] + Hx[12]*Gx[26] + Hx[13]*Gx[36] + Hx[14]*Gx[46] + Hx[15]*Gx[56] + Hx[16]*Gx[66] + Hx[17]*Gx[76] + Hx[18]*Gx[86] + Hx[19]*Gx[96];
A01[17] = + Hx[10]*Gx[7] + Hx[11]*Gx[17] + Hx[12]*Gx[27] + Hx[13]*Gx[37] + Hx[14]*Gx[47] + Hx[15]*Gx[57] + Hx[16]*Gx[67] + Hx[17]*Gx[77] + Hx[18]*Gx[87] + Hx[19]*Gx[97];
A01[18] = + Hx[10]*Gx[8] + Hx[11]*Gx[18] + Hx[12]*Gx[28] + Hx[13]*Gx[38] + Hx[14]*Gx[48] + Hx[15]*Gx[58] + Hx[16]*Gx[68] + Hx[17]*Gx[78] + Hx[18]*Gx[88] + Hx[19]*Gx[98];
A01[19] = + Hx[10]*Gx[9] + Hx[11]*Gx[19] + Hx[12]*Gx[29] + Hx[13]*Gx[39] + Hx[14]*Gx[49] + Hx[15]*Gx[59] + Hx[16]*Gx[69] + Hx[17]*Gx[79] + Hx[18]*Gx[89] + Hx[19]*Gx[99];
A01[20] = + Hx[20]*Gx[0] + Hx[21]*Gx[10] + Hx[22]*Gx[20] + Hx[23]*Gx[30] + Hx[24]*Gx[40] + Hx[25]*Gx[50] + Hx[26]*Gx[60] + Hx[27]*Gx[70] + Hx[28]*Gx[80] + Hx[29]*Gx[90];
A01[21] = + Hx[20]*Gx[1] + Hx[21]*Gx[11] + Hx[22]*Gx[21] + Hx[23]*Gx[31] + Hx[24]*Gx[41] + Hx[25]*Gx[51] + Hx[26]*Gx[61] + Hx[27]*Gx[71] + Hx[28]*Gx[81] + Hx[29]*Gx[91];
A01[22] = + Hx[20]*Gx[2] + Hx[21]*Gx[12] + Hx[22]*Gx[22] + Hx[23]*Gx[32] + Hx[24]*Gx[42] + Hx[25]*Gx[52] + Hx[26]*Gx[62] + Hx[27]*Gx[72] + Hx[28]*Gx[82] + Hx[29]*Gx[92];
A01[23] = + Hx[20]*Gx[3] + Hx[21]*Gx[13] + Hx[22]*Gx[23] + Hx[23]*Gx[33] + Hx[24]*Gx[43] + Hx[25]*Gx[53] + Hx[26]*Gx[63] + Hx[27]*Gx[73] + Hx[28]*Gx[83] + Hx[29]*Gx[93];
A01[24] = + Hx[20]*Gx[4] + Hx[21]*Gx[14] + Hx[22]*Gx[24] + Hx[23]*Gx[34] + Hx[24]*Gx[44] + Hx[25]*Gx[54] + Hx[26]*Gx[64] + Hx[27]*Gx[74] + Hx[28]*Gx[84] + Hx[29]*Gx[94];
A01[25] = + Hx[20]*Gx[5] + Hx[21]*Gx[15] + Hx[22]*Gx[25] + Hx[23]*Gx[35] + Hx[24]*Gx[45] + Hx[25]*Gx[55] + Hx[26]*Gx[65] + Hx[27]*Gx[75] + Hx[28]*Gx[85] + Hx[29]*Gx[95];
A01[26] = + Hx[20]*Gx[6] + Hx[21]*Gx[16] + Hx[22]*Gx[26] + Hx[23]*Gx[36] + Hx[24]*Gx[46] + Hx[25]*Gx[56] + Hx[26]*Gx[66] + Hx[27]*Gx[76] + Hx[28]*Gx[86] + Hx[29]*Gx[96];
A01[27] = + Hx[20]*Gx[7] + Hx[21]*Gx[17] + Hx[22]*Gx[27] + Hx[23]*Gx[37] + Hx[24]*Gx[47] + Hx[25]*Gx[57] + Hx[26]*Gx[67] + Hx[27]*Gx[77] + Hx[28]*Gx[87] + Hx[29]*Gx[97];
A01[28] = + Hx[20]*Gx[8] + Hx[21]*Gx[18] + Hx[22]*Gx[28] + Hx[23]*Gx[38] + Hx[24]*Gx[48] + Hx[25]*Gx[58] + Hx[26]*Gx[68] + Hx[27]*Gx[78] + Hx[28]*Gx[88] + Hx[29]*Gx[98];
A01[29] = + Hx[20]*Gx[9] + Hx[21]*Gx[19] + Hx[22]*Gx[29] + Hx[23]*Gx[39] + Hx[24]*Gx[49] + Hx[25]*Gx[59] + Hx[26]*Gx[69] + Hx[27]*Gx[79] + Hx[28]*Gx[89] + Hx[29]*Gx[99];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 90) + (col * 3)] = + Hx[0]*E[0] + Hx[1]*E[3] + Hx[2]*E[6] + Hx[3]*E[9] + Hx[4]*E[12] + Hx[5]*E[15] + Hx[6]*E[18] + Hx[7]*E[21] + Hx[8]*E[24] + Hx[9]*E[27];
acadoWorkspace.A[(row * 90) + (col * 3 + 1)] = + Hx[0]*E[1] + Hx[1]*E[4] + Hx[2]*E[7] + Hx[3]*E[10] + Hx[4]*E[13] + Hx[5]*E[16] + Hx[6]*E[19] + Hx[7]*E[22] + Hx[8]*E[25] + Hx[9]*E[28];
acadoWorkspace.A[(row * 90) + (col * 3 + 2)] = + Hx[0]*E[2] + Hx[1]*E[5] + Hx[2]*E[8] + Hx[3]*E[11] + Hx[4]*E[14] + Hx[5]*E[17] + Hx[6]*E[20] + Hx[7]*E[23] + Hx[8]*E[26] + Hx[9]*E[29];
acadoWorkspace.A[(row * 90 + 30) + (col * 3)] = + Hx[10]*E[0] + Hx[11]*E[3] + Hx[12]*E[6] + Hx[13]*E[9] + Hx[14]*E[12] + Hx[15]*E[15] + Hx[16]*E[18] + Hx[17]*E[21] + Hx[18]*E[24] + Hx[19]*E[27];
acadoWorkspace.A[(row * 90 + 30) + (col * 3 + 1)] = + Hx[10]*E[1] + Hx[11]*E[4] + Hx[12]*E[7] + Hx[13]*E[10] + Hx[14]*E[13] + Hx[15]*E[16] + Hx[16]*E[19] + Hx[17]*E[22] + Hx[18]*E[25] + Hx[19]*E[28];
acadoWorkspace.A[(row * 90 + 30) + (col * 3 + 2)] = + Hx[10]*E[2] + Hx[11]*E[5] + Hx[12]*E[8] + Hx[13]*E[11] + Hx[14]*E[14] + Hx[15]*E[17] + Hx[16]*E[20] + Hx[17]*E[23] + Hx[18]*E[26] + Hx[19]*E[29];
acadoWorkspace.A[(row * 90 + 60) + (col * 3)] = + Hx[20]*E[0] + Hx[21]*E[3] + Hx[22]*E[6] + Hx[23]*E[9] + Hx[24]*E[12] + Hx[25]*E[15] + Hx[26]*E[18] + Hx[27]*E[21] + Hx[28]*E[24] + Hx[29]*E[27];
acadoWorkspace.A[(row * 90 + 60) + (col * 3 + 1)] = + Hx[20]*E[1] + Hx[21]*E[4] + Hx[22]*E[7] + Hx[23]*E[10] + Hx[24]*E[13] + Hx[25]*E[16] + Hx[26]*E[19] + Hx[27]*E[22] + Hx[28]*E[25] + Hx[29]*E[28];
acadoWorkspace.A[(row * 90 + 60) + (col * 3 + 2)] = + Hx[20]*E[2] + Hx[21]*E[5] + Hx[22]*E[8] + Hx[23]*E[11] + Hx[24]*E[14] + Hx[25]*E[17] + Hx[26]*E[20] + Hx[27]*E[23] + Hx[28]*E[26] + Hx[29]*E[29];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7] + Hx[8]*tmpd[8] + Hx[9]*tmpd[9];
acadoWorkspace.evHxd[1] = + Hx[10]*tmpd[0] + Hx[11]*tmpd[1] + Hx[12]*tmpd[2] + Hx[13]*tmpd[3] + Hx[14]*tmpd[4] + Hx[15]*tmpd[5] + Hx[16]*tmpd[6] + Hx[17]*tmpd[7] + Hx[18]*tmpd[8] + Hx[19]*tmpd[9];
acadoWorkspace.evHxd[2] = + Hx[20]*tmpd[0] + Hx[21]*tmpd[1] + Hx[22]*tmpd[2] + Hx[23]*tmpd[3] + Hx[24]*tmpd[4] + Hx[25]*tmpd[5] + Hx[26]*tmpd[6] + Hx[27]*tmpd[7] + Hx[28]*tmpd[8] + Hx[29]*tmpd[9];
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
const real_t* u = in + 10;
/* Vector of auxiliary variables; number of elements: 43. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(-1.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(1.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(1.0000000000000000e+00);
a[11] = (real_t)(-1.0000000000000000e+00);
a[12] = (xd[8]*a[11]);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(-1.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (xd[1]*a[11]);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(-1.0000000000000000e+00);
a[22] = xd[7];
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(-1.0000000000000000e+00);
a[25] = (real_t)(-1.0000000000000000e+00);
a[26] = (a[24]*a[25]);
a[27] = (real_t)(-1.0000000000000000e+00);
a[28] = (a[27]*xd[7]);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (xd[1]-xd[4]);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(0.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(-1.0000000000000000e+00);
a[37] = (real_t)(0.0000000000000000e+00);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(-1.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(-1.0000000000000000e+00);

/* Compute outputs: */
out[0] = ((xd[5]-xd[0])-u[2]);
out[1] = (((xd[0]-u[2])-(xd[8]*xd[1]))-xd[6]);
out[2] = (((xd[7]*(xd[1]-xd[4]))-(xd[0]-xd[3]))-u[2]);
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
out[14] = a[12];
out[15] = a[13];
out[16] = a[14];
out[17] = a[15];
out[18] = a[16];
out[19] = a[17];
out[20] = a[18];
out[21] = a[19];
out[22] = a[20];
out[23] = a[21];
out[24] = a[22];
out[25] = a[23];
out[26] = a[26];
out[27] = a[28];
out[28] = a[29];
out[29] = a[30];
out[30] = a[31];
out[31] = a[32];
out[32] = a[33];
out[33] = a[34];
out[34] = a[35];
out[35] = a[36];
out[36] = a[37];
out[37] = a[38];
out[38] = a[39];
out[39] = a[40];
out[40] = a[41];
out[41] = a[42];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.d[ 10 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 100 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 30 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 60 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.d[ 20 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGx[ 200 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 120 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 150 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.d[ 30 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGx[ 300 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 210 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 240 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.E[ 270 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.d[ 40 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.evGx[ 400 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 330 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.E[ 390 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.E[ 420 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.d[ 50 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGx[ 500 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 450 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 510 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.E[ 540 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 570 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 150 ]), &(acadoWorkspace.E[ 600 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.d[ 60 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.evGx[ 600 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.E[ 630 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 660 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.E[ 690 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.E[ 750 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.E[ 780 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.E[ 810 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.d[ 70 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.evGx[ 700 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.E[ 840 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.E[ 870 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.E[ 900 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 930 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.E[ 990 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.E[ 1020 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.E[ 1050 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.d[ 80 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.evGx[ 800 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.E[ 1080 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.E[ 1110 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.E[ 1140 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.E[ 1170 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.E[ 1230 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.E[ 1260 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1050 ]), &(acadoWorkspace.E[ 1290 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.E[ 1320 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.d[ 90 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.evGx[ 900 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1350 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.E[ 1380 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.E[ 1410 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1470 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.E[ 1500 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.E[ 1530 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1290 ]), &(acadoWorkspace.E[ 1560 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.E[ 1590 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.E[ 1620 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 100 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 630 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QE[ 690 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.QE[ 750 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 870 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QE[ 930 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.E[ 1050 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1110 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1170 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1290 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1350 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1410 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1470 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.QE[ 1530 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1590 ]), &(acadoWorkspace.QE[ 1590 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 450 ]), &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 630 ]), &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 840 ]), &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1350 ]), &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 660 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 870 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1110 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1380 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 690 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 900 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1140 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1410 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 930 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1170 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 90 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 570 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 750 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1470 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 150 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.H10[ 150 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 780 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.H10[ 150 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 990 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 150 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1230 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 150 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1500 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 150 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 810 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1020 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1260 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1530 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 210 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1050 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.H10[ 210 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1290 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 210 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 210 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1590 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 270 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1620 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.H10[ 270 ]) );

acado_setBlockH11_R1( 0, 0, acadoWorkspace.R1 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 630 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 870 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1110 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1380 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 690 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1410 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 930 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1170 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1440 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 750 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1470 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1500 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 1, 1, &(acadoWorkspace.R1[ 9 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 870 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1110 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1380 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 690 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1410 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 930 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1170 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1440 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 750 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1470 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1500 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 2, 2, &(acadoWorkspace.R1[ 18 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QE[ 690 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1410 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 930 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1170 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1440 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QE[ 750 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1470 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1500 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 3, 3, &(acadoWorkspace.R1[ 27 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QE[ 930 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1170 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 750 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1470 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1500 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 4, 4, &(acadoWorkspace.R1[ 36 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.QE[ 750 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1470 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1500 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 5, 5, &(acadoWorkspace.R1[ 45 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.QE[ 990 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.QE[ 1230 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1500 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 6, 6, &(acadoWorkspace.R1[ 54 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.QE[ 1530 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 7, 7, &(acadoWorkspace.R1[ 63 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1050 ]), &(acadoWorkspace.QE[ 1050 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1290 ]), &(acadoWorkspace.QE[ 1290 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1290 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 72 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1590 ]), &(acadoWorkspace.QE[ 1590 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1590 ]), &(acadoWorkspace.QE[ 1620 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 81 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QE[ 1620 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( &(acadoWorkspace.Q1[ 100 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.Qd[ 50 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.Qd[ 70 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 450 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 630 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 840 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1080 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1350 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 660 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 870 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1110 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1380 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 690 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 900 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1140 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1410 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 930 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1170 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 570 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 750 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1470 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 780 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 990 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1230 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1500 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 810 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1020 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1260 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1530 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1050 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1290 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1590 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1620 ]), &(acadoWorkspace.g[ 27 ]) );
acadoWorkspace.lb[0] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.ub[0] = (real_t)6.5000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)6.5000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)6.5000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)6.5000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.5000000000000002e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)6.5000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)6.5000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)6.5000000000000002e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.5000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)6.5000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)6.5000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)6.5000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.5000000000000002e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)6.5000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)6.5000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)6.5000000000000002e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)6.5000000000000002e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)6.5000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)6.5000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)6.5000000000000002e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - acadoVariables.u[29];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.conValueIn[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.conValueIn[9] = acadoVariables.x[lRun1 * 10 + 9];
acadoWorkspace.conValueIn[10] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.conValueIn[11] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[12] = acadoVariables.u[lRun1 * 3 + 2];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 3] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 3 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 3 + 2] = acadoWorkspace.conValueOut[2];

acadoWorkspace.evHx[lRun1 * 30] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 30 + 1] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 30 + 2] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 30 + 3] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 30 + 4] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 30 + 5] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 30 + 6] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 30 + 7] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 30 + 8] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 30 + 9] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 30 + 10] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 30 + 11] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 30 + 12] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 30 + 13] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 30 + 14] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 30 + 15] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 30 + 16] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 30 + 17] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 30 + 18] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 30 + 19] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 30 + 20] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHx[lRun1 * 30 + 21] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHx[lRun1 * 30 + 22] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHx[lRun1 * 30 + 23] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHx[lRun1 * 30 + 24] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHx[lRun1 * 30 + 25] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHx[lRun1 * 30 + 26] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHx[lRun1 * 30 + 27] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHx[lRun1 * 30 + 28] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evHx[lRun1 * 30 + 29] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evHu[lRun1 * 9] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evHu[lRun1 * 9 + 1] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evHu[lRun1 * 9 + 2] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evHu[lRun1 * 9 + 3] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evHu[lRun1 * 9 + 4] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evHu[lRun1 * 9 + 5] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evHu[lRun1 * 9 + 6] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evHu[lRun1 * 9 + 7] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evHu[lRun1 * 9 + 8] = acadoWorkspace.conValueOut[41];
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
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];
acadoWorkspace.A01[12] = acadoWorkspace.evHx[12];
acadoWorkspace.A01[13] = acadoWorkspace.evHx[13];
acadoWorkspace.A01[14] = acadoWorkspace.evHx[14];
acadoWorkspace.A01[15] = acadoWorkspace.evHx[15];
acadoWorkspace.A01[16] = acadoWorkspace.evHx[16];
acadoWorkspace.A01[17] = acadoWorkspace.evHx[17];
acadoWorkspace.A01[18] = acadoWorkspace.evHx[18];
acadoWorkspace.A01[19] = acadoWorkspace.evHx[19];
acadoWorkspace.A01[20] = acadoWorkspace.evHx[20];
acadoWorkspace.A01[21] = acadoWorkspace.evHx[21];
acadoWorkspace.A01[22] = acadoWorkspace.evHx[22];
acadoWorkspace.A01[23] = acadoWorkspace.evHx[23];
acadoWorkspace.A01[24] = acadoWorkspace.evHx[24];
acadoWorkspace.A01[25] = acadoWorkspace.evHx[25];
acadoWorkspace.A01[26] = acadoWorkspace.evHx[26];
acadoWorkspace.A01[27] = acadoWorkspace.evHx[27];
acadoWorkspace.A01[28] = acadoWorkspace.evHx[28];
acadoWorkspace.A01[29] = acadoWorkspace.evHx[29];

acado_multHxC( &(acadoWorkspace.evHx[ 30 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 30 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.A01[ 90 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.A01[ 150 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.A01[ 210 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.A01[ 240 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.A01[ 270 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 30 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 60 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 90 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 120 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 150 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 180 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 210 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 240 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 270 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 300 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 330 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 360 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 390 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 420 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 450 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 480 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 510 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 540 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 570 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 600 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 630 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 660 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 690 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 720 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 750 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 780 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 810 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 840 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 870 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 900 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 930 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 960 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 990 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 1020 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 1050 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1080 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1110 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1140 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1170 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1200 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1230 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1260 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1290 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 1320 ]), 9, 8 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[30] = acadoWorkspace.evHu[3];
acadoWorkspace.A[31] = acadoWorkspace.evHu[4];
acadoWorkspace.A[32] = acadoWorkspace.evHu[5];
acadoWorkspace.A[60] = acadoWorkspace.evHu[6];
acadoWorkspace.A[61] = acadoWorkspace.evHu[7];
acadoWorkspace.A[62] = acadoWorkspace.evHu[8];
acadoWorkspace.A[93] = acadoWorkspace.evHu[9];
acadoWorkspace.A[94] = acadoWorkspace.evHu[10];
acadoWorkspace.A[95] = acadoWorkspace.evHu[11];
acadoWorkspace.A[123] = acadoWorkspace.evHu[12];
acadoWorkspace.A[124] = acadoWorkspace.evHu[13];
acadoWorkspace.A[125] = acadoWorkspace.evHu[14];
acadoWorkspace.A[153] = acadoWorkspace.evHu[15];
acadoWorkspace.A[154] = acadoWorkspace.evHu[16];
acadoWorkspace.A[155] = acadoWorkspace.evHu[17];
acadoWorkspace.A[186] = acadoWorkspace.evHu[18];
acadoWorkspace.A[187] = acadoWorkspace.evHu[19];
acadoWorkspace.A[188] = acadoWorkspace.evHu[20];
acadoWorkspace.A[216] = acadoWorkspace.evHu[21];
acadoWorkspace.A[217] = acadoWorkspace.evHu[22];
acadoWorkspace.A[218] = acadoWorkspace.evHu[23];
acadoWorkspace.A[246] = acadoWorkspace.evHu[24];
acadoWorkspace.A[247] = acadoWorkspace.evHu[25];
acadoWorkspace.A[248] = acadoWorkspace.evHu[26];
acadoWorkspace.A[279] = acadoWorkspace.evHu[27];
acadoWorkspace.A[280] = acadoWorkspace.evHu[28];
acadoWorkspace.A[281] = acadoWorkspace.evHu[29];
acadoWorkspace.A[309] = acadoWorkspace.evHu[30];
acadoWorkspace.A[310] = acadoWorkspace.evHu[31];
acadoWorkspace.A[311] = acadoWorkspace.evHu[32];
acadoWorkspace.A[339] = acadoWorkspace.evHu[33];
acadoWorkspace.A[340] = acadoWorkspace.evHu[34];
acadoWorkspace.A[341] = acadoWorkspace.evHu[35];
acadoWorkspace.A[372] = acadoWorkspace.evHu[36];
acadoWorkspace.A[373] = acadoWorkspace.evHu[37];
acadoWorkspace.A[374] = acadoWorkspace.evHu[38];
acadoWorkspace.A[402] = acadoWorkspace.evHu[39];
acadoWorkspace.A[403] = acadoWorkspace.evHu[40];
acadoWorkspace.A[404] = acadoWorkspace.evHu[41];
acadoWorkspace.A[432] = acadoWorkspace.evHu[42];
acadoWorkspace.A[433] = acadoWorkspace.evHu[43];
acadoWorkspace.A[434] = acadoWorkspace.evHu[44];
acadoWorkspace.A[465] = acadoWorkspace.evHu[45];
acadoWorkspace.A[466] = acadoWorkspace.evHu[46];
acadoWorkspace.A[467] = acadoWorkspace.evHu[47];
acadoWorkspace.A[495] = acadoWorkspace.evHu[48];
acadoWorkspace.A[496] = acadoWorkspace.evHu[49];
acadoWorkspace.A[497] = acadoWorkspace.evHu[50];
acadoWorkspace.A[525] = acadoWorkspace.evHu[51];
acadoWorkspace.A[526] = acadoWorkspace.evHu[52];
acadoWorkspace.A[527] = acadoWorkspace.evHu[53];
acadoWorkspace.A[558] = acadoWorkspace.evHu[54];
acadoWorkspace.A[559] = acadoWorkspace.evHu[55];
acadoWorkspace.A[560] = acadoWorkspace.evHu[56];
acadoWorkspace.A[588] = acadoWorkspace.evHu[57];
acadoWorkspace.A[589] = acadoWorkspace.evHu[58];
acadoWorkspace.A[590] = acadoWorkspace.evHu[59];
acadoWorkspace.A[618] = acadoWorkspace.evHu[60];
acadoWorkspace.A[619] = acadoWorkspace.evHu[61];
acadoWorkspace.A[620] = acadoWorkspace.evHu[62];
acadoWorkspace.A[651] = acadoWorkspace.evHu[63];
acadoWorkspace.A[652] = acadoWorkspace.evHu[64];
acadoWorkspace.A[653] = acadoWorkspace.evHu[65];
acadoWorkspace.A[681] = acadoWorkspace.evHu[66];
acadoWorkspace.A[682] = acadoWorkspace.evHu[67];
acadoWorkspace.A[683] = acadoWorkspace.evHu[68];
acadoWorkspace.A[711] = acadoWorkspace.evHu[69];
acadoWorkspace.A[712] = acadoWorkspace.evHu[70];
acadoWorkspace.A[713] = acadoWorkspace.evHu[71];
acadoWorkspace.A[744] = acadoWorkspace.evHu[72];
acadoWorkspace.A[745] = acadoWorkspace.evHu[73];
acadoWorkspace.A[746] = acadoWorkspace.evHu[74];
acadoWorkspace.A[774] = acadoWorkspace.evHu[75];
acadoWorkspace.A[775] = acadoWorkspace.evHu[76];
acadoWorkspace.A[776] = acadoWorkspace.evHu[77];
acadoWorkspace.A[804] = acadoWorkspace.evHu[78];
acadoWorkspace.A[805] = acadoWorkspace.evHu[79];
acadoWorkspace.A[806] = acadoWorkspace.evHu[80];
acadoWorkspace.A[837] = acadoWorkspace.evHu[81];
acadoWorkspace.A[838] = acadoWorkspace.evHu[82];
acadoWorkspace.A[839] = acadoWorkspace.evHu[83];
acadoWorkspace.A[867] = acadoWorkspace.evHu[84];
acadoWorkspace.A[868] = acadoWorkspace.evHu[85];
acadoWorkspace.A[869] = acadoWorkspace.evHu[86];
acadoWorkspace.A[897] = acadoWorkspace.evHu[87];
acadoWorkspace.A[898] = acadoWorkspace.evHu[88];
acadoWorkspace.A[899] = acadoWorkspace.evHu[89];
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

acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 18 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 54 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 126 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 162 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 27 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 90 ]) );

acadoWorkspace.QDy[100] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[101] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[102] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[103] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[104] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[105] = + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[106] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[107] = + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[108] = + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[109] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1];

acadoWorkspace.QDy[10] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[99] += acadoWorkspace.Qd[89];
acadoWorkspace.QDy[100] += acadoWorkspace.Qd[90];
acadoWorkspace.QDy[101] += acadoWorkspace.Qd[91];
acadoWorkspace.QDy[102] += acadoWorkspace.Qd[92];
acadoWorkspace.QDy[103] += acadoWorkspace.Qd[93];
acadoWorkspace.QDy[104] += acadoWorkspace.Qd[94];
acadoWorkspace.QDy[105] += acadoWorkspace.Qd[95];
acadoWorkspace.QDy[106] += acadoWorkspace.Qd[96];
acadoWorkspace.QDy[107] += acadoWorkspace.Qd[97];
acadoWorkspace.QDy[108] += acadoWorkspace.Qd[98];
acadoWorkspace.QDy[109] += acadoWorkspace.Qd[99];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 630 ]), &(acadoWorkspace.QDy[ 70 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QDy[ 70 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.QDy[ 70 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 70 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.QDy[ 70 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QDy[ 70 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QDy[ 70 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1050 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1290 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1590 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.g[ 27 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[1] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[2] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[3] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[4] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[5] += + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[6] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[7] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[8] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[9] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[10] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[11] += + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[12] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[13] += + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[14] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[15] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[16] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[17] += + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[18] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[19] += + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[20] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[21] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[22] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[23] += + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[24] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[25] += + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[26] += + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[27] += + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[28] += + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[29] += + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[9];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[9];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];
acadoVariables.x[6] += acadoWorkspace.Dx0[6];
acadoVariables.x[7] += acadoWorkspace.Dx0[7];
acadoVariables.x[8] += acadoWorkspace.Dx0[8];
acadoVariables.x[9] += acadoWorkspace.Dx0[9];

acadoVariables.x[10] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[0];
acadoVariables.x[11] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[1];
acadoVariables.x[12] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[2];
acadoVariables.x[13] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[3];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[4];
acadoVariables.x[15] += + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[5];
acadoVariables.x[16] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[6];
acadoVariables.x[17] += + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[7];
acadoVariables.x[18] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[8];
acadoVariables.x[19] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[9];
acadoVariables.x[20] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[10];
acadoVariables.x[21] += + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[11];
acadoVariables.x[22] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[12];
acadoVariables.x[23] += + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[13];
acadoVariables.x[24] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[14];
acadoVariables.x[25] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[15];
acadoVariables.x[26] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[16];
acadoVariables.x[27] += + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[17];
acadoVariables.x[28] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[18];
acadoVariables.x[29] += + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[19];
acadoVariables.x[30] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[20];
acadoVariables.x[31] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[21];
acadoVariables.x[32] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[22];
acadoVariables.x[33] += + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[23];
acadoVariables.x[34] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[24];
acadoVariables.x[35] += + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[25];
acadoVariables.x[36] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[26];
acadoVariables.x[37] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[27];
acadoVariables.x[38] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[28];
acadoVariables.x[39] += + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[29];
acadoVariables.x[40] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[30];
acadoVariables.x[41] += + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[31];
acadoVariables.x[42] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[32];
acadoVariables.x[43] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[33];
acadoVariables.x[44] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[34];
acadoVariables.x[45] += + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[35];
acadoVariables.x[46] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[36];
acadoVariables.x[47] += + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[37];
acadoVariables.x[48] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[38];
acadoVariables.x[49] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[39];
acadoVariables.x[50] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[40];
acadoVariables.x[51] += + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[41];
acadoVariables.x[52] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[42];
acadoVariables.x[53] += + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[43];
acadoVariables.x[54] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[44];
acadoVariables.x[55] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[45];
acadoVariables.x[56] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[46];
acadoVariables.x[57] += + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[47];
acadoVariables.x[58] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[48];
acadoVariables.x[59] += + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[49];
acadoVariables.x[60] += + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[50];
acadoVariables.x[61] += + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[51];
acadoVariables.x[62] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[52];
acadoVariables.x[63] += + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[53];
acadoVariables.x[64] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[54];
acadoVariables.x[65] += + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[55];
acadoVariables.x[66] += + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[56];
acadoVariables.x[67] += + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[57];
acadoVariables.x[68] += + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[58];
acadoVariables.x[69] += + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[59];
acadoVariables.x[70] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[60];
acadoVariables.x[71] += + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[61];
acadoVariables.x[72] += + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[62];
acadoVariables.x[73] += + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[63];
acadoVariables.x[74] += + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[64];
acadoVariables.x[75] += + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[65];
acadoVariables.x[76] += + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[66];
acadoVariables.x[77] += + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[67];
acadoVariables.x[78] += + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[68];
acadoVariables.x[79] += + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[69];
acadoVariables.x[80] += + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[70];
acadoVariables.x[81] += + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[71];
acadoVariables.x[82] += + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[725]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[726]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[727]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[728]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[729]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[72];
acadoVariables.x[83] += + acadoWorkspace.evGx[730]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[731]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[732]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[733]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[734]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[735]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[736]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[737]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[738]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[739]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[73];
acadoVariables.x[84] += + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[74];
acadoVariables.x[85] += + acadoWorkspace.evGx[750]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[751]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[752]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[753]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[754]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[755]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[756]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[757]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[758]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[759]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[75];
acadoVariables.x[86] += + acadoWorkspace.evGx[760]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[761]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[762]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[763]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[764]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[765]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[766]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[767]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[768]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[769]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[76];
acadoVariables.x[87] += + acadoWorkspace.evGx[770]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[771]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[772]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[773]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[774]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[775]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[776]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[777]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[778]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[779]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[77];
acadoVariables.x[88] += + acadoWorkspace.evGx[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[783]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[784]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[785]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[786]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[787]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[788]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[789]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[78];
acadoVariables.x[89] += + acadoWorkspace.evGx[790]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[791]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[792]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[793]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[794]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[795]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[796]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[797]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[798]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[799]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[79];
acadoVariables.x[90] += + acadoWorkspace.evGx[800]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[801]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[802]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[803]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[804]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[805]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[806]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[807]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[808]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[809]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[80];
acadoVariables.x[91] += + acadoWorkspace.evGx[810]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[811]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[812]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[813]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[814]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[815]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[816]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[817]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[818]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[819]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[81];
acadoVariables.x[92] += + acadoWorkspace.evGx[820]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[821]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[822]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[823]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[824]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[825]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[826]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[827]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[828]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[829]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[82];
acadoVariables.x[93] += + acadoWorkspace.evGx[830]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[831]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[832]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[833]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[834]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[835]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[836]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[837]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[838]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[839]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[83];
acadoVariables.x[94] += + acadoWorkspace.evGx[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[844]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[845]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[846]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[847]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[848]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[849]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[84];
acadoVariables.x[95] += + acadoWorkspace.evGx[850]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[851]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[852]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[853]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[854]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[855]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[856]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[857]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[858]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[859]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[85];
acadoVariables.x[96] += + acadoWorkspace.evGx[860]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[861]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[862]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[863]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[864]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[865]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[866]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[867]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[868]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[869]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[86];
acadoVariables.x[97] += + acadoWorkspace.evGx[870]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[871]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[872]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[873]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[874]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[875]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[876]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[877]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[878]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[879]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[87];
acadoVariables.x[98] += + acadoWorkspace.evGx[880]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[881]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[882]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[883]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[884]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[885]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[886]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[887]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[888]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[889]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[88];
acadoVariables.x[99] += + acadoWorkspace.evGx[890]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[891]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[892]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[893]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[894]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[895]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[896]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[897]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[898]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[899]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[89];
acadoVariables.x[100] += + acadoWorkspace.evGx[900]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[901]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[902]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[903]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[904]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[905]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[906]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[907]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[908]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[909]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[90];
acadoVariables.x[101] += + acadoWorkspace.evGx[910]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[911]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[912]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[913]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[914]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[915]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[916]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[917]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[918]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[919]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[91];
acadoVariables.x[102] += + acadoWorkspace.evGx[920]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[921]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[922]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[923]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[924]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[925]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[926]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[927]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[928]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[929]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[92];
acadoVariables.x[103] += + acadoWorkspace.evGx[930]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[931]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[932]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[933]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[934]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[935]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[936]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[937]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[938]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[939]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[93];
acadoVariables.x[104] += + acadoWorkspace.evGx[940]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[941]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[942]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[943]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[944]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[945]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[946]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[947]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[948]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[949]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[94];
acadoVariables.x[105] += + acadoWorkspace.evGx[950]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[951]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[952]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[953]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[954]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[955]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[956]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[957]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[958]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[959]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[95];
acadoVariables.x[106] += + acadoWorkspace.evGx[960]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[961]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[962]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[963]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[964]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[965]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[966]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[967]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[968]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[969]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[96];
acadoVariables.x[107] += + acadoWorkspace.evGx[970]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[971]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[972]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[973]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[974]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[975]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[976]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[977]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[978]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[979]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[97];
acadoVariables.x[108] += + acadoWorkspace.evGx[980]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[981]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[982]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[983]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[984]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[985]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[986]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[987]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[988]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[989]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[98];
acadoVariables.x[109] += + acadoWorkspace.evGx[990]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[991]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[992]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[993]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[994]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[995]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[996]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[997]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[998]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[999]*acadoWorkspace.Dx0[9] + acadoWorkspace.d[99];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), acadoWorkspace.x, &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 450 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 630 ]), acadoWorkspace.x, &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 690 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 750 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 70 ]) );
acado_multEDu( &(acadoWorkspace.E[ 840 ]), acadoWorkspace.x, &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 870 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 930 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 990 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1050 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1080 ]), acadoWorkspace.x, &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1110 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1170 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1230 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1290 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1350 ]), acadoWorkspace.x, &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1410 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1470 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1530 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1590 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 100 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 100 ]) );
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
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 10];
acadoWorkspace.state[1] = acadoVariables.x[index * 10 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 10 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 10 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 10 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 10 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 10 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 10 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 10 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 10 + 9];
acadoWorkspace.state[140] = acadoVariables.u[index * 3];
acadoWorkspace.state[141] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[142] = acadoVariables.u[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 10 + 10] = acadoWorkspace.state[0];
acadoVariables.x[index * 10 + 11] = acadoWorkspace.state[1];
acadoVariables.x[index * 10 + 12] = acadoWorkspace.state[2];
acadoVariables.x[index * 10 + 13] = acadoWorkspace.state[3];
acadoVariables.x[index * 10 + 14] = acadoWorkspace.state[4];
acadoVariables.x[index * 10 + 15] = acadoWorkspace.state[5];
acadoVariables.x[index * 10 + 16] = acadoWorkspace.state[6];
acadoVariables.x[index * 10 + 17] = acadoWorkspace.state[7];
acadoVariables.x[index * 10 + 18] = acadoWorkspace.state[8];
acadoVariables.x[index * 10 + 19] = acadoWorkspace.state[9];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 10] = acadoVariables.x[index * 10 + 10];
acadoVariables.x[index * 10 + 1] = acadoVariables.x[index * 10 + 11];
acadoVariables.x[index * 10 + 2] = acadoVariables.x[index * 10 + 12];
acadoVariables.x[index * 10 + 3] = acadoVariables.x[index * 10 + 13];
acadoVariables.x[index * 10 + 4] = acadoVariables.x[index * 10 + 14];
acadoVariables.x[index * 10 + 5] = acadoVariables.x[index * 10 + 15];
acadoVariables.x[index * 10 + 6] = acadoVariables.x[index * 10 + 16];
acadoVariables.x[index * 10 + 7] = acadoVariables.x[index * 10 + 17];
acadoVariables.x[index * 10 + 8] = acadoVariables.x[index * 10 + 18];
acadoVariables.x[index * 10 + 9] = acadoVariables.x[index * 10 + 19];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[100] = xEnd[0];
acadoVariables.x[101] = xEnd[1];
acadoVariables.x[102] = xEnd[2];
acadoVariables.x[103] = xEnd[3];
acadoVariables.x[104] = xEnd[4];
acadoVariables.x[105] = xEnd[5];
acadoVariables.x[106] = xEnd[6];
acadoVariables.x[107] = xEnd[7];
acadoVariables.x[108] = xEnd[8];
acadoVariables.x[109] = xEnd[9];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[100];
acadoWorkspace.state[1] = acadoVariables.x[101];
acadoWorkspace.state[2] = acadoVariables.x[102];
acadoWorkspace.state[3] = acadoVariables.x[103];
acadoWorkspace.state[4] = acadoVariables.x[104];
acadoWorkspace.state[5] = acadoVariables.x[105];
acadoWorkspace.state[6] = acadoVariables.x[106];
acadoWorkspace.state[7] = acadoVariables.x[107];
acadoWorkspace.state[8] = acadoVariables.x[108];
acadoWorkspace.state[9] = acadoVariables.x[109];
if (uEnd != 0)
{
acadoWorkspace.state[140] = uEnd[0];
acadoWorkspace.state[141] = uEnd[1];
acadoWorkspace.state[142] = uEnd[2];
}
else
{
acadoWorkspace.state[140] = acadoVariables.u[27];
acadoWorkspace.state[141] = acadoVariables.u[28];
acadoWorkspace.state[142] = acadoVariables.u[29];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[100] = acadoWorkspace.state[0];
acadoVariables.x[101] = acadoWorkspace.state[1];
acadoVariables.x[102] = acadoWorkspace.state[2];
acadoVariables.x[103] = acadoWorkspace.state[3];
acadoVariables.x[104] = acadoWorkspace.state[4];
acadoVariables.x[105] = acadoWorkspace.state[5];
acadoVariables.x[106] = acadoWorkspace.state[6];
acadoVariables.x[107] = acadoWorkspace.state[7];
acadoVariables.x[108] = acadoWorkspace.state[8];
acadoVariables.x[109] = acadoWorkspace.state[9];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[27] = uEnd[0];
acadoVariables.u[28] = uEnd[1];
acadoVariables.u[29] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29];
kkt = fabs( kkt );
for (index = 0; index < 30; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 30; ++index)
{
prd = acadoWorkspace.y[index + 30];
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
/** Row vector of size: 6 */
real_t tmpDy[ 6 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 6] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 6];
acadoWorkspace.Dy[lRun1 * 6 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 6 + 1];
acadoWorkspace.Dy[lRun1 * 6 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 6 + 2];
acadoWorkspace.Dy[lRun1 * 6 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 6 + 3];
acadoWorkspace.Dy[lRun1 * 6 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 6 + 4];
acadoWorkspace.Dy[lRun1 * 6 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 6 + 5];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[100];
acadoWorkspace.objValueIn[1] = acadoVariables.x[101];
acadoWorkspace.objValueIn[2] = acadoVariables.x[102];
acadoWorkspace.objValueIn[3] = acadoVariables.x[103];
acadoWorkspace.objValueIn[4] = acadoVariables.x[104];
acadoWorkspace.objValueIn[5] = acadoVariables.x[105];
acadoWorkspace.objValueIn[6] = acadoVariables.x[106];
acadoWorkspace.objValueIn[7] = acadoVariables.x[107];
acadoWorkspace.objValueIn[8] = acadoVariables.x[108];
acadoWorkspace.objValueIn[9] = acadoVariables.x[109];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[lRun1 * 36] + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[lRun1 * 36 + 6] + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[lRun1 * 36 + 12] + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[lRun1 * 36 + 18] + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[lRun1 * 36 + 24] + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[lRun1 * 36 + 30];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[lRun1 * 36 + 1] + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[lRun1 * 36 + 7] + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[lRun1 * 36 + 13] + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[lRun1 * 36 + 19] + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[lRun1 * 36 + 25] + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[lRun1 * 36 + 31];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[lRun1 * 36 + 2] + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[lRun1 * 36 + 8] + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[lRun1 * 36 + 14] + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[lRun1 * 36 + 20] + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[lRun1 * 36 + 26] + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[lRun1 * 36 + 32];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[lRun1 * 36 + 3] + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[lRun1 * 36 + 9] + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[lRun1 * 36 + 15] + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[lRun1 * 36 + 21] + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[lRun1 * 36 + 27] + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[lRun1 * 36 + 33];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[lRun1 * 36 + 4] + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[lRun1 * 36 + 10] + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[lRun1 * 36 + 16] + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[lRun1 * 36 + 22] + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[lRun1 * 36 + 28] + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[lRun1 * 36 + 34];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[lRun1 * 36 + 5] + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[lRun1 * 36 + 11] + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[lRun1 * 36 + 17] + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[lRun1 * 36 + 23] + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[lRun1 * 36 + 29] + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[lRun1 * 36 + 35];
objVal += + acadoWorkspace.Dy[lRun1 * 6]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 6 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 6 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 6 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 6 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 6 + 5]*tmpDy[5];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

