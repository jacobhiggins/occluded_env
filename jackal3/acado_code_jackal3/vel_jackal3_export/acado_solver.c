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
for (runObj = 0; runObj < 50; ++runObj)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[500];
acadoWorkspace.objValueIn[1] = acadoVariables.x[501];
acadoWorkspace.objValueIn[2] = acadoVariables.x[502];
acadoWorkspace.objValueIn[3] = acadoVariables.x[503];
acadoWorkspace.objValueIn[4] = acadoVariables.x[504];
acadoWorkspace.objValueIn[5] = acadoVariables.x[505];
acadoWorkspace.objValueIn[6] = acadoVariables.x[506];
acadoWorkspace.objValueIn[7] = acadoVariables.x[507];
acadoWorkspace.objValueIn[8] = acadoVariables.x[508];
acadoWorkspace.objValueIn[9] = acadoVariables.x[509];
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
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = R11[0];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = R11[3];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = R11[4];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = R11[5];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = R11[6];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = R11[7];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = R11[8];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3 + 2)];
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
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 300) + (col * 3)] = + Hx[0]*E[0] + Hx[1]*E[3] + Hx[2]*E[6] + Hx[3]*E[9] + Hx[4]*E[12] + Hx[5]*E[15] + Hx[6]*E[18] + Hx[7]*E[21] + Hx[8]*E[24] + Hx[9]*E[27];
acadoWorkspace.A[(row * 300) + (col * 3 + 1)] = + Hx[0]*E[1] + Hx[1]*E[4] + Hx[2]*E[7] + Hx[3]*E[10] + Hx[4]*E[13] + Hx[5]*E[16] + Hx[6]*E[19] + Hx[7]*E[22] + Hx[8]*E[25] + Hx[9]*E[28];
acadoWorkspace.A[(row * 300) + (col * 3 + 2)] = + Hx[0]*E[2] + Hx[1]*E[5] + Hx[2]*E[8] + Hx[3]*E[11] + Hx[4]*E[14] + Hx[5]*E[17] + Hx[6]*E[20] + Hx[7]*E[23] + Hx[8]*E[26] + Hx[9]*E[29];
acadoWorkspace.A[(row * 300 + 150) + (col * 3)] = + Hx[10]*E[0] + Hx[11]*E[3] + Hx[12]*E[6] + Hx[13]*E[9] + Hx[14]*E[12] + Hx[15]*E[15] + Hx[16]*E[18] + Hx[17]*E[21] + Hx[18]*E[24] + Hx[19]*E[27];
acadoWorkspace.A[(row * 300 + 150) + (col * 3 + 1)] = + Hx[10]*E[1] + Hx[11]*E[4] + Hx[12]*E[7] + Hx[13]*E[10] + Hx[14]*E[13] + Hx[15]*E[16] + Hx[16]*E[19] + Hx[17]*E[22] + Hx[18]*E[25] + Hx[19]*E[28];
acadoWorkspace.A[(row * 300 + 150) + (col * 3 + 2)] = + Hx[10]*E[2] + Hx[11]*E[5] + Hx[12]*E[8] + Hx[13]*E[11] + Hx[14]*E[14] + Hx[15]*E[17] + Hx[16]*E[20] + Hx[17]*E[23] + Hx[18]*E[26] + Hx[19]*E[29];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7] + Hx[8]*tmpd[8] + Hx[9]*tmpd[9];
acadoWorkspace.evHxd[1] = + Hx[10]*tmpd[0] + Hx[11]*tmpd[1] + Hx[12]*tmpd[2] + Hx[13]*tmpd[3] + Hx[14]*tmpd[4] + Hx[15]*tmpd[5] + Hx[16]*tmpd[6] + Hx[17]*tmpd[7] + Hx[18]*tmpd[8] + Hx[19]*tmpd[9];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;
/* Vector of auxiliary variables; number of elements: 27. */
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
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(-1.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(-1.0000000000000000e+00);

/* Compute outputs: */
out[0] = ((xd[5]-xd[0])-u[2]);
out[1] = (((xd[0]-u[2])-(xd[8]*xd[1]))-xd[6]);
out[2] = a[0];
out[3] = a[1];
out[4] = a[2];
out[5] = a[3];
out[6] = a[4];
out[7] = a[5];
out[8] = a[6];
out[9] = a[7];
out[10] = a[8];
out[11] = a[9];
out[12] = a[10];
out[13] = a[12];
out[14] = a[13];
out[15] = a[14];
out[16] = a[15];
out[17] = a[16];
out[18] = a[17];
out[19] = a[18];
out[20] = a[19];
out[21] = a[20];
out[22] = a[21];
out[23] = a[22];
out[24] = a[23];
out[25] = a[24];
out[26] = a[25];
out[27] = a[26];
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
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 50; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 100 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 10-10 ]), &(acadoWorkspace.evGx[ lRun1 * 100 ]), &(acadoWorkspace.d[ lRun1 * 10 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 100-100 ]), &(acadoWorkspace.evGx[ lRun1 * 100 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 30 ]), &(acadoWorkspace.E[ lRun3 * 30 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 30 ]), &(acadoWorkspace.E[ lRun3 * 30 ]) );
}

for (lRun1 = 0; lRun1 < 49; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 100 + 100 ]), &(acadoWorkspace.E[ lRun3 * 30 ]), &(acadoWorkspace.QE[ lRun3 * 30 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 30 ]), &(acadoWorkspace.QE[ lRun3 * 30 ]) );
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 30 ]) );
for (lRun2 = lRun1; lRun2 < 50; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 30 ]), &(acadoWorkspace.evGx[ lRun2 * 100 ]), &(acadoWorkspace.H10[ lRun1 * 30 ]) );
}
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 9 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 50; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 30 ]), &(acadoWorkspace.QE[ lRun5 * 30 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 50; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 50; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 30 ]), &(acadoWorkspace.QE[ lRun5 * 30 ]) );
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

acado_multQ1d( &(acadoWorkspace.Q1[ 100 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.Qd[ 50 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.Qd[ 70 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1000 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1100 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.Qd[ 100 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1200 ]), &(acadoWorkspace.d[ 110 ]), &(acadoWorkspace.Qd[ 110 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1300 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1400 ]), &(acadoWorkspace.d[ 130 ]), &(acadoWorkspace.Qd[ 130 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1500 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.Qd[ 140 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1600 ]), &(acadoWorkspace.d[ 150 ]), &(acadoWorkspace.Qd[ 150 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1700 ]), &(acadoWorkspace.d[ 160 ]), &(acadoWorkspace.Qd[ 160 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1800 ]), &(acadoWorkspace.d[ 170 ]), &(acadoWorkspace.Qd[ 170 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1900 ]), &(acadoWorkspace.d[ 180 ]), &(acadoWorkspace.Qd[ 180 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2000 ]), &(acadoWorkspace.d[ 190 ]), &(acadoWorkspace.Qd[ 190 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2100 ]), &(acadoWorkspace.d[ 200 ]), &(acadoWorkspace.Qd[ 200 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2200 ]), &(acadoWorkspace.d[ 210 ]), &(acadoWorkspace.Qd[ 210 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2300 ]), &(acadoWorkspace.d[ 220 ]), &(acadoWorkspace.Qd[ 220 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2400 ]), &(acadoWorkspace.d[ 230 ]), &(acadoWorkspace.Qd[ 230 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2500 ]), &(acadoWorkspace.d[ 240 ]), &(acadoWorkspace.Qd[ 240 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2600 ]), &(acadoWorkspace.d[ 250 ]), &(acadoWorkspace.Qd[ 250 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2700 ]), &(acadoWorkspace.d[ 260 ]), &(acadoWorkspace.Qd[ 260 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2800 ]), &(acadoWorkspace.d[ 270 ]), &(acadoWorkspace.Qd[ 270 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2900 ]), &(acadoWorkspace.d[ 280 ]), &(acadoWorkspace.Qd[ 280 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3000 ]), &(acadoWorkspace.d[ 290 ]), &(acadoWorkspace.Qd[ 290 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3100 ]), &(acadoWorkspace.d[ 300 ]), &(acadoWorkspace.Qd[ 300 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3200 ]), &(acadoWorkspace.d[ 310 ]), &(acadoWorkspace.Qd[ 310 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3300 ]), &(acadoWorkspace.d[ 320 ]), &(acadoWorkspace.Qd[ 320 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3400 ]), &(acadoWorkspace.d[ 330 ]), &(acadoWorkspace.Qd[ 330 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3500 ]), &(acadoWorkspace.d[ 340 ]), &(acadoWorkspace.Qd[ 340 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3600 ]), &(acadoWorkspace.d[ 350 ]), &(acadoWorkspace.Qd[ 350 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3700 ]), &(acadoWorkspace.d[ 360 ]), &(acadoWorkspace.Qd[ 360 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3800 ]), &(acadoWorkspace.d[ 370 ]), &(acadoWorkspace.Qd[ 370 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3900 ]), &(acadoWorkspace.d[ 380 ]), &(acadoWorkspace.Qd[ 380 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4000 ]), &(acadoWorkspace.d[ 390 ]), &(acadoWorkspace.Qd[ 390 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4100 ]), &(acadoWorkspace.d[ 400 ]), &(acadoWorkspace.Qd[ 400 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4200 ]), &(acadoWorkspace.d[ 410 ]), &(acadoWorkspace.Qd[ 410 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4300 ]), &(acadoWorkspace.d[ 420 ]), &(acadoWorkspace.Qd[ 420 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4400 ]), &(acadoWorkspace.d[ 430 ]), &(acadoWorkspace.Qd[ 430 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4500 ]), &(acadoWorkspace.d[ 440 ]), &(acadoWorkspace.Qd[ 440 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4600 ]), &(acadoWorkspace.d[ 450 ]), &(acadoWorkspace.Qd[ 450 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4700 ]), &(acadoWorkspace.d[ 460 ]), &(acadoWorkspace.Qd[ 460 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4800 ]), &(acadoWorkspace.d[ 470 ]), &(acadoWorkspace.Qd[ 470 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4900 ]), &(acadoWorkspace.d[ 480 ]), &(acadoWorkspace.Qd[ 480 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 490 ]), &(acadoWorkspace.Qd[ 490 ]) );

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 50; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 30 ]), &(acadoWorkspace.g[ lRun1 * 3 ]) );
}
}
acadoWorkspace.lb[0] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[91];
acadoWorkspace.lb[92] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[103];
acadoWorkspace.lb[104] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[107];
acadoWorkspace.lb[108] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[111];
acadoWorkspace.lb[112] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[119];
acadoWorkspace.lb[120] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[120];
acadoWorkspace.lb[121] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[121];
acadoWorkspace.lb[122] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[122];
acadoWorkspace.lb[123] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[123];
acadoWorkspace.lb[124] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[124];
acadoWorkspace.lb[125] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[125];
acadoWorkspace.lb[126] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[126];
acadoWorkspace.lb[127] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[127];
acadoWorkspace.lb[128] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[128];
acadoWorkspace.lb[129] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[129];
acadoWorkspace.lb[130] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[130];
acadoWorkspace.lb[131] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[131];
acadoWorkspace.lb[132] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[132];
acadoWorkspace.lb[133] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[133];
acadoWorkspace.lb[134] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[134];
acadoWorkspace.lb[135] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[135];
acadoWorkspace.lb[136] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[136];
acadoWorkspace.lb[137] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[137];
acadoWorkspace.lb[138] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[138];
acadoWorkspace.lb[139] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[139];
acadoWorkspace.lb[140] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[140];
acadoWorkspace.lb[141] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[141];
acadoWorkspace.lb[142] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[142];
acadoWorkspace.lb[143] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[143];
acadoWorkspace.lb[144] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[144];
acadoWorkspace.lb[145] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[145];
acadoWorkspace.lb[146] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[146];
acadoWorkspace.lb[147] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[147];
acadoWorkspace.lb[148] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[148];
acadoWorkspace.lb[149] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[149];
acadoWorkspace.ub[0] = (real_t)5.0000000000000000e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)5.0000000000000000e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)5.0000000000000000e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)5.0000000000000000e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)5.0000000000000000e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)5.0000000000000000e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)5.0000000000000000e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)5.0000000000000000e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)5.0000000000000000e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)5.0000000000000000e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)5.0000000000000000e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)5.0000000000000000e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)5.0000000000000000e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)5.0000000000000000e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)5.0000000000000000e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)5.0000000000000000e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)5.0000000000000000e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)5.0000000000000000e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)5.0000000000000000e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)5.0000000000000000e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)5.0000000000000000e-01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)5.0000000000000000e-01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)5.0000000000000000e-01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)5.0000000000000000e-01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)5.0000000000000000e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)5.0000000000000000e-01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)5.0000000000000000e-01 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)5.0000000000000000e-01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)5.0000000000000000e-01 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)5.0000000000000000e-01 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)5.0000000000000000e-01 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)5.0000000000000000e-01 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+12 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)5.0000000000000000e-01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)5.0000000000000000e-01 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)5.0000000000000000e-01 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)5.0000000000000000e-01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)1.0000000000000000e+12 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)5.0000000000000000e-01 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)5.0000000000000000e-01 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)1.0000000000000000e+12 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)5.0000000000000000e-01 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)5.0000000000000000e-01 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+12 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)5.0000000000000000e-01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)5.0000000000000000e-01 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)5.0000000000000000e-01 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)5.0000000000000000e-01 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)1.0000000000000000e+12 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)5.0000000000000000e-01 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)5.0000000000000000e-01 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)1.0000000000000000e+12 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)5.0000000000000000e-01 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)5.0000000000000000e-01 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.0000000000000000e+12 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)5.0000000000000000e-01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)5.0000000000000000e-01 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+12 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)5.0000000000000000e-01 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)5.0000000000000000e-01 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)1.0000000000000000e+12 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)5.0000000000000000e-01 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)5.0000000000000000e-01 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)1.0000000000000000e+12 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)5.0000000000000000e-01 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)5.0000000000000000e-01 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)1.0000000000000000e+12 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)5.0000000000000000e-01 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)5.0000000000000000e-01 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)1.0000000000000000e+12 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)5.0000000000000000e-01 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)5.0000000000000000e-01 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)1.0000000000000000e+12 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)5.0000000000000000e-01 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)5.0000000000000000e-01 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)1.0000000000000000e+12 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)5.0000000000000000e-01 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)5.0000000000000000e-01 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)1.0000000000000000e+12 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)5.0000000000000000e-01 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)5.0000000000000000e-01 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)1.0000000000000000e+12 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)5.0000000000000000e-01 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)5.0000000000000000e-01 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)1.0000000000000000e+12 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)5.0000000000000000e-01 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)5.0000000000000000e-01 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)1.0000000000000000e+12 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)5.0000000000000000e-01 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)5.0000000000000000e-01 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)1.0000000000000000e+12 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)5.0000000000000000e-01 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)5.0000000000000000e-01 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)1.0000000000000000e+12 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)5.0000000000000000e-01 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)5.0000000000000000e-01 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)1.0000000000000000e+12 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)5.0000000000000000e-01 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)5.0000000000000000e-01 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)1.0000000000000000e+12 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)5.0000000000000000e-01 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)5.0000000000000000e-01 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)1.0000000000000000e+12 - acadoVariables.u[119];
acadoWorkspace.ub[120] = (real_t)5.0000000000000000e-01 - acadoVariables.u[120];
acadoWorkspace.ub[121] = (real_t)5.0000000000000000e-01 - acadoVariables.u[121];
acadoWorkspace.ub[122] = (real_t)1.0000000000000000e+12 - acadoVariables.u[122];
acadoWorkspace.ub[123] = (real_t)5.0000000000000000e-01 - acadoVariables.u[123];
acadoWorkspace.ub[124] = (real_t)5.0000000000000000e-01 - acadoVariables.u[124];
acadoWorkspace.ub[125] = (real_t)1.0000000000000000e+12 - acadoVariables.u[125];
acadoWorkspace.ub[126] = (real_t)5.0000000000000000e-01 - acadoVariables.u[126];
acadoWorkspace.ub[127] = (real_t)5.0000000000000000e-01 - acadoVariables.u[127];
acadoWorkspace.ub[128] = (real_t)1.0000000000000000e+12 - acadoVariables.u[128];
acadoWorkspace.ub[129] = (real_t)5.0000000000000000e-01 - acadoVariables.u[129];
acadoWorkspace.ub[130] = (real_t)5.0000000000000000e-01 - acadoVariables.u[130];
acadoWorkspace.ub[131] = (real_t)1.0000000000000000e+12 - acadoVariables.u[131];
acadoWorkspace.ub[132] = (real_t)5.0000000000000000e-01 - acadoVariables.u[132];
acadoWorkspace.ub[133] = (real_t)5.0000000000000000e-01 - acadoVariables.u[133];
acadoWorkspace.ub[134] = (real_t)1.0000000000000000e+12 - acadoVariables.u[134];
acadoWorkspace.ub[135] = (real_t)5.0000000000000000e-01 - acadoVariables.u[135];
acadoWorkspace.ub[136] = (real_t)5.0000000000000000e-01 - acadoVariables.u[136];
acadoWorkspace.ub[137] = (real_t)1.0000000000000000e+12 - acadoVariables.u[137];
acadoWorkspace.ub[138] = (real_t)5.0000000000000000e-01 - acadoVariables.u[138];
acadoWorkspace.ub[139] = (real_t)5.0000000000000000e-01 - acadoVariables.u[139];
acadoWorkspace.ub[140] = (real_t)1.0000000000000000e+12 - acadoVariables.u[140];
acadoWorkspace.ub[141] = (real_t)5.0000000000000000e-01 - acadoVariables.u[141];
acadoWorkspace.ub[142] = (real_t)5.0000000000000000e-01 - acadoVariables.u[142];
acadoWorkspace.ub[143] = (real_t)1.0000000000000000e+12 - acadoVariables.u[143];
acadoWorkspace.ub[144] = (real_t)5.0000000000000000e-01 - acadoVariables.u[144];
acadoWorkspace.ub[145] = (real_t)5.0000000000000000e-01 - acadoVariables.u[145];
acadoWorkspace.ub[146] = (real_t)1.0000000000000000e+12 - acadoVariables.u[146];
acadoWorkspace.ub[147] = (real_t)5.0000000000000000e-01 - acadoVariables.u[147];
acadoWorkspace.ub[148] = (real_t)5.0000000000000000e-01 - acadoVariables.u[148];
acadoWorkspace.ub[149] = (real_t)1.0000000000000000e+12 - acadoVariables.u[149];

for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
acadoWorkspace.evH[lRun1 * 2] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[1];

acadoWorkspace.evHx[lRun1 * 20] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 20 + 1] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 20 + 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 20 + 3] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 20 + 4] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 20 + 5] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 20 + 6] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 20 + 7] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 20 + 8] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 20 + 9] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 20 + 10] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 20 + 11] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 20 + 12] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 20 + 13] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 20 + 14] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 20 + 15] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 20 + 16] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 20 + 17] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 20 + 18] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 20 + 19] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHu[lRun1 * 6] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHu[lRun1 * 6 + 1] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHu[lRun1 * 6 + 2] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHu[lRun1 * 6 + 3] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHu[lRun1 * 6 + 4] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHu[lRun1 * 6 + 5] = acadoWorkspace.conValueOut[27];
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

acado_multHxC( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 20 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.A01[ 40 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.A01[ 80 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.A01[ 100 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.A01[ 160 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 200 ]), &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.A01[ 200 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 220 ]), &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.A01[ 220 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.A01[ 240 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 260 ]), &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.A01[ 260 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.A01[ 280 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.A01[ 300 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.evGx[ 1500 ]), &(acadoWorkspace.A01[ 320 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 340 ]), &(acadoWorkspace.evGx[ 1600 ]), &(acadoWorkspace.A01[ 340 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.evGx[ 1700 ]), &(acadoWorkspace.A01[ 360 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 380 ]), &(acadoWorkspace.evGx[ 1800 ]), &(acadoWorkspace.A01[ 380 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 400 ]), &(acadoWorkspace.evGx[ 1900 ]), &(acadoWorkspace.A01[ 400 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.evGx[ 2000 ]), &(acadoWorkspace.A01[ 420 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 440 ]), &(acadoWorkspace.evGx[ 2100 ]), &(acadoWorkspace.A01[ 440 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 460 ]), &(acadoWorkspace.evGx[ 2200 ]), &(acadoWorkspace.A01[ 460 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.evGx[ 2300 ]), &(acadoWorkspace.A01[ 480 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 500 ]), &(acadoWorkspace.evGx[ 2400 ]), &(acadoWorkspace.A01[ 500 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 520 ]), &(acadoWorkspace.evGx[ 2500 ]), &(acadoWorkspace.A01[ 520 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 540 ]), &(acadoWorkspace.evGx[ 2600 ]), &(acadoWorkspace.A01[ 540 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 560 ]), &(acadoWorkspace.evGx[ 2700 ]), &(acadoWorkspace.A01[ 560 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 580 ]), &(acadoWorkspace.evGx[ 2800 ]), &(acadoWorkspace.A01[ 580 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 600 ]), &(acadoWorkspace.evGx[ 2900 ]), &(acadoWorkspace.A01[ 600 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 620 ]), &(acadoWorkspace.evGx[ 3000 ]), &(acadoWorkspace.A01[ 620 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 640 ]), &(acadoWorkspace.evGx[ 3100 ]), &(acadoWorkspace.A01[ 640 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 660 ]), &(acadoWorkspace.evGx[ 3200 ]), &(acadoWorkspace.A01[ 660 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 680 ]), &(acadoWorkspace.evGx[ 3300 ]), &(acadoWorkspace.A01[ 680 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 700 ]), &(acadoWorkspace.evGx[ 3400 ]), &(acadoWorkspace.A01[ 700 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 720 ]), &(acadoWorkspace.evGx[ 3500 ]), &(acadoWorkspace.A01[ 720 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 740 ]), &(acadoWorkspace.evGx[ 3600 ]), &(acadoWorkspace.A01[ 740 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 760 ]), &(acadoWorkspace.evGx[ 3700 ]), &(acadoWorkspace.A01[ 760 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 780 ]), &(acadoWorkspace.evGx[ 3800 ]), &(acadoWorkspace.A01[ 780 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 800 ]), &(acadoWorkspace.evGx[ 3900 ]), &(acadoWorkspace.A01[ 800 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 820 ]), &(acadoWorkspace.evGx[ 4000 ]), &(acadoWorkspace.A01[ 820 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 840 ]), &(acadoWorkspace.evGx[ 4100 ]), &(acadoWorkspace.A01[ 840 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 860 ]), &(acadoWorkspace.evGx[ 4200 ]), &(acadoWorkspace.A01[ 860 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 880 ]), &(acadoWorkspace.evGx[ 4300 ]), &(acadoWorkspace.A01[ 880 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 900 ]), &(acadoWorkspace.evGx[ 4400 ]), &(acadoWorkspace.A01[ 900 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 920 ]), &(acadoWorkspace.evGx[ 4500 ]), &(acadoWorkspace.A01[ 920 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 940 ]), &(acadoWorkspace.evGx[ 4600 ]), &(acadoWorkspace.A01[ 940 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 960 ]), &(acadoWorkspace.evGx[ 4700 ]), &(acadoWorkspace.A01[ 960 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 980 ]), &(acadoWorkspace.evGx[ 4800 ]), &(acadoWorkspace.A01[ 980 ]) );

for (lRun2 = 0; lRun2 < 49; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 20 + 20 ]), &(acadoWorkspace.E[ lRun4 * 30 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[150] = acadoWorkspace.evHu[3];
acadoWorkspace.A[151] = acadoWorkspace.evHu[4];
acadoWorkspace.A[152] = acadoWorkspace.evHu[5];
acadoWorkspace.A[303] = acadoWorkspace.evHu[6];
acadoWorkspace.A[304] = acadoWorkspace.evHu[7];
acadoWorkspace.A[305] = acadoWorkspace.evHu[8];
acadoWorkspace.A[453] = acadoWorkspace.evHu[9];
acadoWorkspace.A[454] = acadoWorkspace.evHu[10];
acadoWorkspace.A[455] = acadoWorkspace.evHu[11];
acadoWorkspace.A[606] = acadoWorkspace.evHu[12];
acadoWorkspace.A[607] = acadoWorkspace.evHu[13];
acadoWorkspace.A[608] = acadoWorkspace.evHu[14];
acadoWorkspace.A[756] = acadoWorkspace.evHu[15];
acadoWorkspace.A[757] = acadoWorkspace.evHu[16];
acadoWorkspace.A[758] = acadoWorkspace.evHu[17];
acadoWorkspace.A[909] = acadoWorkspace.evHu[18];
acadoWorkspace.A[910] = acadoWorkspace.evHu[19];
acadoWorkspace.A[911] = acadoWorkspace.evHu[20];
acadoWorkspace.A[1059] = acadoWorkspace.evHu[21];
acadoWorkspace.A[1060] = acadoWorkspace.evHu[22];
acadoWorkspace.A[1061] = acadoWorkspace.evHu[23];
acadoWorkspace.A[1212] = acadoWorkspace.evHu[24];
acadoWorkspace.A[1213] = acadoWorkspace.evHu[25];
acadoWorkspace.A[1214] = acadoWorkspace.evHu[26];
acadoWorkspace.A[1362] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1363] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1364] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1515] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1516] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1517] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1665] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1666] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1667] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1818] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1819] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1820] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1968] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1969] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1970] = acadoWorkspace.evHu[41];
acadoWorkspace.A[2121] = acadoWorkspace.evHu[42];
acadoWorkspace.A[2122] = acadoWorkspace.evHu[43];
acadoWorkspace.A[2123] = acadoWorkspace.evHu[44];
acadoWorkspace.A[2271] = acadoWorkspace.evHu[45];
acadoWorkspace.A[2272] = acadoWorkspace.evHu[46];
acadoWorkspace.A[2273] = acadoWorkspace.evHu[47];
acadoWorkspace.A[2424] = acadoWorkspace.evHu[48];
acadoWorkspace.A[2425] = acadoWorkspace.evHu[49];
acadoWorkspace.A[2426] = acadoWorkspace.evHu[50];
acadoWorkspace.A[2574] = acadoWorkspace.evHu[51];
acadoWorkspace.A[2575] = acadoWorkspace.evHu[52];
acadoWorkspace.A[2576] = acadoWorkspace.evHu[53];
acadoWorkspace.A[2727] = acadoWorkspace.evHu[54];
acadoWorkspace.A[2728] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2729] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2877] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2878] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2879] = acadoWorkspace.evHu[59];
acadoWorkspace.A[3030] = acadoWorkspace.evHu[60];
acadoWorkspace.A[3031] = acadoWorkspace.evHu[61];
acadoWorkspace.A[3032] = acadoWorkspace.evHu[62];
acadoWorkspace.A[3180] = acadoWorkspace.evHu[63];
acadoWorkspace.A[3181] = acadoWorkspace.evHu[64];
acadoWorkspace.A[3182] = acadoWorkspace.evHu[65];
acadoWorkspace.A[3333] = acadoWorkspace.evHu[66];
acadoWorkspace.A[3334] = acadoWorkspace.evHu[67];
acadoWorkspace.A[3335] = acadoWorkspace.evHu[68];
acadoWorkspace.A[3483] = acadoWorkspace.evHu[69];
acadoWorkspace.A[3484] = acadoWorkspace.evHu[70];
acadoWorkspace.A[3485] = acadoWorkspace.evHu[71];
acadoWorkspace.A[3636] = acadoWorkspace.evHu[72];
acadoWorkspace.A[3637] = acadoWorkspace.evHu[73];
acadoWorkspace.A[3638] = acadoWorkspace.evHu[74];
acadoWorkspace.A[3786] = acadoWorkspace.evHu[75];
acadoWorkspace.A[3787] = acadoWorkspace.evHu[76];
acadoWorkspace.A[3788] = acadoWorkspace.evHu[77];
acadoWorkspace.A[3939] = acadoWorkspace.evHu[78];
acadoWorkspace.A[3940] = acadoWorkspace.evHu[79];
acadoWorkspace.A[3941] = acadoWorkspace.evHu[80];
acadoWorkspace.A[4089] = acadoWorkspace.evHu[81];
acadoWorkspace.A[4090] = acadoWorkspace.evHu[82];
acadoWorkspace.A[4091] = acadoWorkspace.evHu[83];
acadoWorkspace.A[4242] = acadoWorkspace.evHu[84];
acadoWorkspace.A[4243] = acadoWorkspace.evHu[85];
acadoWorkspace.A[4244] = acadoWorkspace.evHu[86];
acadoWorkspace.A[4392] = acadoWorkspace.evHu[87];
acadoWorkspace.A[4393] = acadoWorkspace.evHu[88];
acadoWorkspace.A[4394] = acadoWorkspace.evHu[89];
acadoWorkspace.A[4545] = acadoWorkspace.evHu[90];
acadoWorkspace.A[4546] = acadoWorkspace.evHu[91];
acadoWorkspace.A[4547] = acadoWorkspace.evHu[92];
acadoWorkspace.A[4695] = acadoWorkspace.evHu[93];
acadoWorkspace.A[4696] = acadoWorkspace.evHu[94];
acadoWorkspace.A[4697] = acadoWorkspace.evHu[95];
acadoWorkspace.A[4848] = acadoWorkspace.evHu[96];
acadoWorkspace.A[4849] = acadoWorkspace.evHu[97];
acadoWorkspace.A[4850] = acadoWorkspace.evHu[98];
acadoWorkspace.A[4998] = acadoWorkspace.evHu[99];
acadoWorkspace.A[4999] = acadoWorkspace.evHu[100];
acadoWorkspace.A[5000] = acadoWorkspace.evHu[101];
acadoWorkspace.A[5151] = acadoWorkspace.evHu[102];
acadoWorkspace.A[5152] = acadoWorkspace.evHu[103];
acadoWorkspace.A[5153] = acadoWorkspace.evHu[104];
acadoWorkspace.A[5301] = acadoWorkspace.evHu[105];
acadoWorkspace.A[5302] = acadoWorkspace.evHu[106];
acadoWorkspace.A[5303] = acadoWorkspace.evHu[107];
acadoWorkspace.A[5454] = acadoWorkspace.evHu[108];
acadoWorkspace.A[5455] = acadoWorkspace.evHu[109];
acadoWorkspace.A[5456] = acadoWorkspace.evHu[110];
acadoWorkspace.A[5604] = acadoWorkspace.evHu[111];
acadoWorkspace.A[5605] = acadoWorkspace.evHu[112];
acadoWorkspace.A[5606] = acadoWorkspace.evHu[113];
acadoWorkspace.A[5757] = acadoWorkspace.evHu[114];
acadoWorkspace.A[5758] = acadoWorkspace.evHu[115];
acadoWorkspace.A[5759] = acadoWorkspace.evHu[116];
acadoWorkspace.A[5907] = acadoWorkspace.evHu[117];
acadoWorkspace.A[5908] = acadoWorkspace.evHu[118];
acadoWorkspace.A[5909] = acadoWorkspace.evHu[119];
acadoWorkspace.A[6060] = acadoWorkspace.evHu[120];
acadoWorkspace.A[6061] = acadoWorkspace.evHu[121];
acadoWorkspace.A[6062] = acadoWorkspace.evHu[122];
acadoWorkspace.A[6210] = acadoWorkspace.evHu[123];
acadoWorkspace.A[6211] = acadoWorkspace.evHu[124];
acadoWorkspace.A[6212] = acadoWorkspace.evHu[125];
acadoWorkspace.A[6363] = acadoWorkspace.evHu[126];
acadoWorkspace.A[6364] = acadoWorkspace.evHu[127];
acadoWorkspace.A[6365] = acadoWorkspace.evHu[128];
acadoWorkspace.A[6513] = acadoWorkspace.evHu[129];
acadoWorkspace.A[6514] = acadoWorkspace.evHu[130];
acadoWorkspace.A[6515] = acadoWorkspace.evHu[131];
acadoWorkspace.A[6666] = acadoWorkspace.evHu[132];
acadoWorkspace.A[6667] = acadoWorkspace.evHu[133];
acadoWorkspace.A[6668] = acadoWorkspace.evHu[134];
acadoWorkspace.A[6816] = acadoWorkspace.evHu[135];
acadoWorkspace.A[6817] = acadoWorkspace.evHu[136];
acadoWorkspace.A[6818] = acadoWorkspace.evHu[137];
acadoWorkspace.A[6969] = acadoWorkspace.evHu[138];
acadoWorkspace.A[6970] = acadoWorkspace.evHu[139];
acadoWorkspace.A[6971] = acadoWorkspace.evHu[140];
acadoWorkspace.A[7119] = acadoWorkspace.evHu[141];
acadoWorkspace.A[7120] = acadoWorkspace.evHu[142];
acadoWorkspace.A[7121] = acadoWorkspace.evHu[143];
acadoWorkspace.A[7272] = acadoWorkspace.evHu[144];
acadoWorkspace.A[7273] = acadoWorkspace.evHu[145];
acadoWorkspace.A[7274] = acadoWorkspace.evHu[146];
acadoWorkspace.A[7422] = acadoWorkspace.evHu[147];
acadoWorkspace.A[7423] = acadoWorkspace.evHu[148];
acadoWorkspace.A[7424] = acadoWorkspace.evHu[149];
acadoWorkspace.A[7575] = acadoWorkspace.evHu[150];
acadoWorkspace.A[7576] = acadoWorkspace.evHu[151];
acadoWorkspace.A[7577] = acadoWorkspace.evHu[152];
acadoWorkspace.A[7725] = acadoWorkspace.evHu[153];
acadoWorkspace.A[7726] = acadoWorkspace.evHu[154];
acadoWorkspace.A[7727] = acadoWorkspace.evHu[155];
acadoWorkspace.A[7878] = acadoWorkspace.evHu[156];
acadoWorkspace.A[7879] = acadoWorkspace.evHu[157];
acadoWorkspace.A[7880] = acadoWorkspace.evHu[158];
acadoWorkspace.A[8028] = acadoWorkspace.evHu[159];
acadoWorkspace.A[8029] = acadoWorkspace.evHu[160];
acadoWorkspace.A[8030] = acadoWorkspace.evHu[161];
acadoWorkspace.A[8181] = acadoWorkspace.evHu[162];
acadoWorkspace.A[8182] = acadoWorkspace.evHu[163];
acadoWorkspace.A[8183] = acadoWorkspace.evHu[164];
acadoWorkspace.A[8331] = acadoWorkspace.evHu[165];
acadoWorkspace.A[8332] = acadoWorkspace.evHu[166];
acadoWorkspace.A[8333] = acadoWorkspace.evHu[167];
acadoWorkspace.A[8484] = acadoWorkspace.evHu[168];
acadoWorkspace.A[8485] = acadoWorkspace.evHu[169];
acadoWorkspace.A[8486] = acadoWorkspace.evHu[170];
acadoWorkspace.A[8634] = acadoWorkspace.evHu[171];
acadoWorkspace.A[8635] = acadoWorkspace.evHu[172];
acadoWorkspace.A[8636] = acadoWorkspace.evHu[173];
acadoWorkspace.A[8787] = acadoWorkspace.evHu[174];
acadoWorkspace.A[8788] = acadoWorkspace.evHu[175];
acadoWorkspace.A[8789] = acadoWorkspace.evHu[176];
acadoWorkspace.A[8937] = acadoWorkspace.evHu[177];
acadoWorkspace.A[8938] = acadoWorkspace.evHu[178];
acadoWorkspace.A[8939] = acadoWorkspace.evHu[179];
acadoWorkspace.A[9090] = acadoWorkspace.evHu[180];
acadoWorkspace.A[9091] = acadoWorkspace.evHu[181];
acadoWorkspace.A[9092] = acadoWorkspace.evHu[182];
acadoWorkspace.A[9240] = acadoWorkspace.evHu[183];
acadoWorkspace.A[9241] = acadoWorkspace.evHu[184];
acadoWorkspace.A[9242] = acadoWorkspace.evHu[185];
acadoWorkspace.A[9393] = acadoWorkspace.evHu[186];
acadoWorkspace.A[9394] = acadoWorkspace.evHu[187];
acadoWorkspace.A[9395] = acadoWorkspace.evHu[188];
acadoWorkspace.A[9543] = acadoWorkspace.evHu[189];
acadoWorkspace.A[9544] = acadoWorkspace.evHu[190];
acadoWorkspace.A[9545] = acadoWorkspace.evHu[191];
acadoWorkspace.A[9696] = acadoWorkspace.evHu[192];
acadoWorkspace.A[9697] = acadoWorkspace.evHu[193];
acadoWorkspace.A[9698] = acadoWorkspace.evHu[194];
acadoWorkspace.A[9846] = acadoWorkspace.evHu[195];
acadoWorkspace.A[9847] = acadoWorkspace.evHu[196];
acadoWorkspace.A[9848] = acadoWorkspace.evHu[197];
acadoWorkspace.A[9999] = acadoWorkspace.evHu[198];
acadoWorkspace.A[10000] = acadoWorkspace.evHu[199];
acadoWorkspace.A[10001] = acadoWorkspace.evHu[200];
acadoWorkspace.A[10149] = acadoWorkspace.evHu[201];
acadoWorkspace.A[10150] = acadoWorkspace.evHu[202];
acadoWorkspace.A[10151] = acadoWorkspace.evHu[203];
acadoWorkspace.A[10302] = acadoWorkspace.evHu[204];
acadoWorkspace.A[10303] = acadoWorkspace.evHu[205];
acadoWorkspace.A[10304] = acadoWorkspace.evHu[206];
acadoWorkspace.A[10452] = acadoWorkspace.evHu[207];
acadoWorkspace.A[10453] = acadoWorkspace.evHu[208];
acadoWorkspace.A[10454] = acadoWorkspace.evHu[209];
acadoWorkspace.A[10605] = acadoWorkspace.evHu[210];
acadoWorkspace.A[10606] = acadoWorkspace.evHu[211];
acadoWorkspace.A[10607] = acadoWorkspace.evHu[212];
acadoWorkspace.A[10755] = acadoWorkspace.evHu[213];
acadoWorkspace.A[10756] = acadoWorkspace.evHu[214];
acadoWorkspace.A[10757] = acadoWorkspace.evHu[215];
acadoWorkspace.A[10908] = acadoWorkspace.evHu[216];
acadoWorkspace.A[10909] = acadoWorkspace.evHu[217];
acadoWorkspace.A[10910] = acadoWorkspace.evHu[218];
acadoWorkspace.A[11058] = acadoWorkspace.evHu[219];
acadoWorkspace.A[11059] = acadoWorkspace.evHu[220];
acadoWorkspace.A[11060] = acadoWorkspace.evHu[221];
acadoWorkspace.A[11211] = acadoWorkspace.evHu[222];
acadoWorkspace.A[11212] = acadoWorkspace.evHu[223];
acadoWorkspace.A[11213] = acadoWorkspace.evHu[224];
acadoWorkspace.A[11361] = acadoWorkspace.evHu[225];
acadoWorkspace.A[11362] = acadoWorkspace.evHu[226];
acadoWorkspace.A[11363] = acadoWorkspace.evHu[227];
acadoWorkspace.A[11514] = acadoWorkspace.evHu[228];
acadoWorkspace.A[11515] = acadoWorkspace.evHu[229];
acadoWorkspace.A[11516] = acadoWorkspace.evHu[230];
acadoWorkspace.A[11664] = acadoWorkspace.evHu[231];
acadoWorkspace.A[11665] = acadoWorkspace.evHu[232];
acadoWorkspace.A[11666] = acadoWorkspace.evHu[233];
acadoWorkspace.A[11817] = acadoWorkspace.evHu[234];
acadoWorkspace.A[11818] = acadoWorkspace.evHu[235];
acadoWorkspace.A[11819] = acadoWorkspace.evHu[236];
acadoWorkspace.A[11967] = acadoWorkspace.evHu[237];
acadoWorkspace.A[11968] = acadoWorkspace.evHu[238];
acadoWorkspace.A[11969] = acadoWorkspace.evHu[239];
acadoWorkspace.A[12120] = acadoWorkspace.evHu[240];
acadoWorkspace.A[12121] = acadoWorkspace.evHu[241];
acadoWorkspace.A[12122] = acadoWorkspace.evHu[242];
acadoWorkspace.A[12270] = acadoWorkspace.evHu[243];
acadoWorkspace.A[12271] = acadoWorkspace.evHu[244];
acadoWorkspace.A[12272] = acadoWorkspace.evHu[245];
acadoWorkspace.A[12423] = acadoWorkspace.evHu[246];
acadoWorkspace.A[12424] = acadoWorkspace.evHu[247];
acadoWorkspace.A[12425] = acadoWorkspace.evHu[248];
acadoWorkspace.A[12573] = acadoWorkspace.evHu[249];
acadoWorkspace.A[12574] = acadoWorkspace.evHu[250];
acadoWorkspace.A[12575] = acadoWorkspace.evHu[251];
acadoWorkspace.A[12726] = acadoWorkspace.evHu[252];
acadoWorkspace.A[12727] = acadoWorkspace.evHu[253];
acadoWorkspace.A[12728] = acadoWorkspace.evHu[254];
acadoWorkspace.A[12876] = acadoWorkspace.evHu[255];
acadoWorkspace.A[12877] = acadoWorkspace.evHu[256];
acadoWorkspace.A[12878] = acadoWorkspace.evHu[257];
acadoWorkspace.A[13029] = acadoWorkspace.evHu[258];
acadoWorkspace.A[13030] = acadoWorkspace.evHu[259];
acadoWorkspace.A[13031] = acadoWorkspace.evHu[260];
acadoWorkspace.A[13179] = acadoWorkspace.evHu[261];
acadoWorkspace.A[13180] = acadoWorkspace.evHu[262];
acadoWorkspace.A[13181] = acadoWorkspace.evHu[263];
acadoWorkspace.A[13332] = acadoWorkspace.evHu[264];
acadoWorkspace.A[13333] = acadoWorkspace.evHu[265];
acadoWorkspace.A[13334] = acadoWorkspace.evHu[266];
acadoWorkspace.A[13482] = acadoWorkspace.evHu[267];
acadoWorkspace.A[13483] = acadoWorkspace.evHu[268];
acadoWorkspace.A[13484] = acadoWorkspace.evHu[269];
acadoWorkspace.A[13635] = acadoWorkspace.evHu[270];
acadoWorkspace.A[13636] = acadoWorkspace.evHu[271];
acadoWorkspace.A[13637] = acadoWorkspace.evHu[272];
acadoWorkspace.A[13785] = acadoWorkspace.evHu[273];
acadoWorkspace.A[13786] = acadoWorkspace.evHu[274];
acadoWorkspace.A[13787] = acadoWorkspace.evHu[275];
acadoWorkspace.A[13938] = acadoWorkspace.evHu[276];
acadoWorkspace.A[13939] = acadoWorkspace.evHu[277];
acadoWorkspace.A[13940] = acadoWorkspace.evHu[278];
acadoWorkspace.A[14088] = acadoWorkspace.evHu[279];
acadoWorkspace.A[14089] = acadoWorkspace.evHu[280];
acadoWorkspace.A[14090] = acadoWorkspace.evHu[281];
acadoWorkspace.A[14241] = acadoWorkspace.evHu[282];
acadoWorkspace.A[14242] = acadoWorkspace.evHu[283];
acadoWorkspace.A[14243] = acadoWorkspace.evHu[284];
acadoWorkspace.A[14391] = acadoWorkspace.evHu[285];
acadoWorkspace.A[14392] = acadoWorkspace.evHu[286];
acadoWorkspace.A[14393] = acadoWorkspace.evHu[287];
acadoWorkspace.A[14544] = acadoWorkspace.evHu[288];
acadoWorkspace.A[14545] = acadoWorkspace.evHu[289];
acadoWorkspace.A[14546] = acadoWorkspace.evHu[290];
acadoWorkspace.A[14694] = acadoWorkspace.evHu[291];
acadoWorkspace.A[14695] = acadoWorkspace.evHu[292];
acadoWorkspace.A[14696] = acadoWorkspace.evHu[293];
acadoWorkspace.A[14847] = acadoWorkspace.evHu[294];
acadoWorkspace.A[14848] = acadoWorkspace.evHu[295];
acadoWorkspace.A[14849] = acadoWorkspace.evHu[296];
acadoWorkspace.A[14997] = acadoWorkspace.evHu[297];
acadoWorkspace.A[14998] = acadoWorkspace.evHu[298];
acadoWorkspace.A[14999] = acadoWorkspace.evHu[299];
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

acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 200 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 220 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.d[ 110 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 260 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.d[ 130 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.d[ 150 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 340 ]), &(acadoWorkspace.d[ 160 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.d[ 170 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 380 ]), &(acadoWorkspace.d[ 180 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 400 ]), &(acadoWorkspace.d[ 190 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.d[ 200 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 440 ]), &(acadoWorkspace.d[ 210 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 460 ]), &(acadoWorkspace.d[ 220 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.d[ 230 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 500 ]), &(acadoWorkspace.d[ 240 ]), &(acadoWorkspace.lbA[ 50 ]), &(acadoWorkspace.ubA[ 50 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 520 ]), &(acadoWorkspace.d[ 250 ]), &(acadoWorkspace.lbA[ 52 ]), &(acadoWorkspace.ubA[ 52 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 540 ]), &(acadoWorkspace.d[ 260 ]), &(acadoWorkspace.lbA[ 54 ]), &(acadoWorkspace.ubA[ 54 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 560 ]), &(acadoWorkspace.d[ 270 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 580 ]), &(acadoWorkspace.d[ 280 ]), &(acadoWorkspace.lbA[ 58 ]), &(acadoWorkspace.ubA[ 58 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 600 ]), &(acadoWorkspace.d[ 290 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 620 ]), &(acadoWorkspace.d[ 300 ]), &(acadoWorkspace.lbA[ 62 ]), &(acadoWorkspace.ubA[ 62 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 640 ]), &(acadoWorkspace.d[ 310 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 660 ]), &(acadoWorkspace.d[ 320 ]), &(acadoWorkspace.lbA[ 66 ]), &(acadoWorkspace.ubA[ 66 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 680 ]), &(acadoWorkspace.d[ 330 ]), &(acadoWorkspace.lbA[ 68 ]), &(acadoWorkspace.ubA[ 68 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 700 ]), &(acadoWorkspace.d[ 340 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 720 ]), &(acadoWorkspace.d[ 350 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 740 ]), &(acadoWorkspace.d[ 360 ]), &(acadoWorkspace.lbA[ 74 ]), &(acadoWorkspace.ubA[ 74 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 760 ]), &(acadoWorkspace.d[ 370 ]), &(acadoWorkspace.lbA[ 76 ]), &(acadoWorkspace.ubA[ 76 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 780 ]), &(acadoWorkspace.d[ 380 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 800 ]), &(acadoWorkspace.d[ 390 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 820 ]), &(acadoWorkspace.d[ 400 ]), &(acadoWorkspace.lbA[ 82 ]), &(acadoWorkspace.ubA[ 82 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 840 ]), &(acadoWorkspace.d[ 410 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 860 ]), &(acadoWorkspace.d[ 420 ]), &(acadoWorkspace.lbA[ 86 ]), &(acadoWorkspace.ubA[ 86 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 880 ]), &(acadoWorkspace.d[ 430 ]), &(acadoWorkspace.lbA[ 88 ]), &(acadoWorkspace.ubA[ 88 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 900 ]), &(acadoWorkspace.d[ 440 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 920 ]), &(acadoWorkspace.d[ 450 ]), &(acadoWorkspace.lbA[ 92 ]), &(acadoWorkspace.ubA[ 92 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 940 ]), &(acadoWorkspace.d[ 460 ]), &(acadoWorkspace.lbA[ 94 ]), &(acadoWorkspace.ubA[ 94 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 960 ]), &(acadoWorkspace.d[ 470 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 980 ]), &(acadoWorkspace.d[ 480 ]), &(acadoWorkspace.lbA[ 98 ]), &(acadoWorkspace.ubA[ 98 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
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

for (lRun2 = 0; lRun2 < 300; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

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
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 198 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 234 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 306 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 342 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 378 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 414 ]), &(acadoWorkspace.Dy[ 138 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 450 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 468 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 486 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.g[ 81 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 522 ]), &(acadoWorkspace.Dy[ 174 ]), &(acadoWorkspace.g[ 87 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 558 ]), &(acadoWorkspace.Dy[ 186 ]), &(acadoWorkspace.g[ 93 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 594 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.g[ 99 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 612 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 102 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 630 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 105 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 648 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 666 ]), &(acadoWorkspace.Dy[ 222 ]), &(acadoWorkspace.g[ 111 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 684 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 114 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 702 ]), &(acadoWorkspace.Dy[ 234 ]), &(acadoWorkspace.g[ 117 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 120 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 738 ]), &(acadoWorkspace.Dy[ 246 ]), &(acadoWorkspace.g[ 123 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 756 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 126 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 774 ]), &(acadoWorkspace.Dy[ 258 ]), &(acadoWorkspace.g[ 129 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 792 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 132 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 810 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.g[ 135 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 828 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.g[ 138 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 846 ]), &(acadoWorkspace.Dy[ 282 ]), &(acadoWorkspace.g[ 141 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 864 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 144 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 882 ]), &(acadoWorkspace.Dy[ 294 ]), &(acadoWorkspace.g[ 147 ]) );

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
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 660 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 780 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 900 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 150 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 160 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1020 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.QDy[ 170 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1140 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.QDy[ 190 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1200 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 200 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1260 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 210 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1320 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 220 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1380 ]), &(acadoWorkspace.Dy[ 138 ]), &(acadoWorkspace.QDy[ 230 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1440 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 240 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1500 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 250 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1560 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 260 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1620 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.QDy[ 270 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1680 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 280 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1740 ]), &(acadoWorkspace.Dy[ 174 ]), &(acadoWorkspace.QDy[ 290 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1800 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 300 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1860 ]), &(acadoWorkspace.Dy[ 186 ]), &(acadoWorkspace.QDy[ 310 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1920 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 320 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1980 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.QDy[ 330 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2040 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 340 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2100 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 350 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2160 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 360 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2220 ]), &(acadoWorkspace.Dy[ 222 ]), &(acadoWorkspace.QDy[ 370 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2280 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 380 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2340 ]), &(acadoWorkspace.Dy[ 234 ]), &(acadoWorkspace.QDy[ 390 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2400 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 400 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2460 ]), &(acadoWorkspace.Dy[ 246 ]), &(acadoWorkspace.QDy[ 410 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2520 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 420 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2580 ]), &(acadoWorkspace.Dy[ 258 ]), &(acadoWorkspace.QDy[ 430 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2640 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 440 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2700 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.QDy[ 450 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2760 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.QDy[ 460 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2820 ]), &(acadoWorkspace.Dy[ 282 ]), &(acadoWorkspace.QDy[ 470 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2880 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 480 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2940 ]), &(acadoWorkspace.Dy[ 294 ]), &(acadoWorkspace.QDy[ 490 ]) );

acadoWorkspace.QDy[500] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[501] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[502] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[503] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[504] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[505] = + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[506] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[507] = + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[508] = + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[509] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1];

for (lRun2 = 0; lRun2 < 500; ++lRun2)
acadoWorkspace.QDy[lRun2 + 10] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 50; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 30 ]), &(acadoWorkspace.QDy[ lRun2 * 10 + 10 ]), &(acadoWorkspace.g[ lRun1 * 3 ]) );
}
}

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
acadoWorkspace.g[30] += + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[31] += + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[32] += + acadoWorkspace.H10[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[325]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[326]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[327]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[328]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[329]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[33] += + acadoWorkspace.H10[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[335]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[336]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[337]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[338]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[339]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[34] += + acadoWorkspace.H10[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[344]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[345]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[346]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[347]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[348]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[349]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[35] += + acadoWorkspace.H10[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[355]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[356]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[357]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[358]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[359]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[36] += + acadoWorkspace.H10[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[369]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[37] += + acadoWorkspace.H10[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[374]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[375]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[376]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[377]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[378]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[379]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[38] += + acadoWorkspace.H10[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[384]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[385]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[386]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[387]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[388]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[389]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[39] += + acadoWorkspace.H10[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[395]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[396]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[397]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[398]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[399]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[40] += + acadoWorkspace.H10[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[405]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[406]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[407]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[408]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[409]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[41] += + acadoWorkspace.H10[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[414]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[415]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[416]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[417]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[418]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[419]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[42] += + acadoWorkspace.H10[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[426]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[427]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[428]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[429]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[43] += + acadoWorkspace.H10[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[434]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[435]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[436]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[437]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[438]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[439]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[44] += + acadoWorkspace.H10[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[445]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[446]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[447]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[448]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[449]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[45] += + acadoWorkspace.H10[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[455]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[456]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[457]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[458]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[459]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[46] += + acadoWorkspace.H10[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[464]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[465]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[466]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[467]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[468]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[469]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[47] += + acadoWorkspace.H10[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[474]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[475]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[476]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[477]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[478]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[479]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[48] += + acadoWorkspace.H10[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[488]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[489]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[49] += + acadoWorkspace.H10[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[495]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[496]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[497]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[498]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[499]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[50] += + acadoWorkspace.H10[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[504]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[505]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[506]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[507]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[508]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[509]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[51] += + acadoWorkspace.H10[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[515]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[516]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[517]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[518]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[519]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[52] += + acadoWorkspace.H10[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[525]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[526]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[527]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[528]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[529]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[53] += + acadoWorkspace.H10[530]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[531]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[532]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[533]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[534]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[535]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[536]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[537]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[538]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[539]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[54] += + acadoWorkspace.H10[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[549]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[55] += + acadoWorkspace.H10[550]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[551]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[552]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[553]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[554]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[555]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[556]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[557]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[558]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[559]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[56] += + acadoWorkspace.H10[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[565]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[566]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[567]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[568]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[569]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[57] += + acadoWorkspace.H10[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[574]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[575]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[576]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[577]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[578]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[579]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[58] += + acadoWorkspace.H10[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[583]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[584]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[585]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[586]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[587]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[588]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[589]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[59] += + acadoWorkspace.H10[590]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[591]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[592]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[593]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[594]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[595]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[596]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[597]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[598]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[599]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[60] += + acadoWorkspace.H10[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[606]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[607]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[608]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[609]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[61] += + acadoWorkspace.H10[610]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[611]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[612]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[613]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[614]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[615]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[616]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[617]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[618]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[619]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[62] += + acadoWorkspace.H10[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[624]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[625]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[626]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[627]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[628]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[629]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[63] += + acadoWorkspace.H10[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[636]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[637]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[638]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[639]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[64] += + acadoWorkspace.H10[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[644]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[645]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[646]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[647]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[648]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[649]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[65] += + acadoWorkspace.H10[650]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[651]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[652]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[653]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[654]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[655]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[656]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[657]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[658]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[659]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[66] += + acadoWorkspace.H10[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[666]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[667]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[668]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[669]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[67] += + acadoWorkspace.H10[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[674]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[675]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[676]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[677]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[678]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[679]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[68] += + acadoWorkspace.H10[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[683]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[684]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[685]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[686]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[687]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[688]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[689]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[69] += + acadoWorkspace.H10[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[694]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[695]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[696]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[697]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[698]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[699]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[70] += + acadoWorkspace.H10[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[704]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[705]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[706]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[707]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[708]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[709]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[71] += + acadoWorkspace.H10[710]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[711]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[712]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[713]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[714]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[715]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[716]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[717]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[718]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[719]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[72] += + acadoWorkspace.H10[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[725]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[726]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[727]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[728]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[729]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[73] += + acadoWorkspace.H10[730]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[731]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[732]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[733]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[734]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[735]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[736]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[737]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[738]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[739]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[74] += + acadoWorkspace.H10[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[744]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[745]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[746]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[747]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[748]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[749]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[75] += + acadoWorkspace.H10[750]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[751]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[752]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[753]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[754]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[755]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[756]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[757]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[758]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[759]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[76] += + acadoWorkspace.H10[760]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[761]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[762]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[763]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[764]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[765]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[766]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[767]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[768]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[769]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[77] += + acadoWorkspace.H10[770]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[771]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[772]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[773]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[774]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[775]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[776]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[777]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[778]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[779]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[78] += + acadoWorkspace.H10[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[783]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[784]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[785]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[786]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[787]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[788]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[789]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[79] += + acadoWorkspace.H10[790]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[791]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[792]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[793]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[794]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[795]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[796]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[797]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[798]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[799]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[80] += + acadoWorkspace.H10[800]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[801]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[802]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[803]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[804]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[805]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[806]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[807]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[808]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[809]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[81] += + acadoWorkspace.H10[810]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[811]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[812]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[813]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[814]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[815]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[816]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[817]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[818]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[819]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[82] += + acadoWorkspace.H10[820]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[821]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[822]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[823]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[824]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[825]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[826]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[827]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[828]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[829]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[83] += + acadoWorkspace.H10[830]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[831]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[832]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[833]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[834]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[835]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[836]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[837]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[838]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[839]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[84] += + acadoWorkspace.H10[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[844]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[845]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[846]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[847]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[848]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[849]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[85] += + acadoWorkspace.H10[850]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[851]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[852]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[853]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[854]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[855]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[856]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[857]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[858]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[859]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[86] += + acadoWorkspace.H10[860]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[861]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[862]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[863]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[864]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[865]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[866]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[867]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[868]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[869]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[87] += + acadoWorkspace.H10[870]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[871]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[872]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[873]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[874]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[875]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[876]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[877]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[878]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[879]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[88] += + acadoWorkspace.H10[880]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[881]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[882]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[883]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[884]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[885]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[886]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[887]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[888]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[889]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[89] += + acadoWorkspace.H10[890]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[891]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[892]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[893]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[894]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[895]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[896]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[897]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[898]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[899]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[90] += + acadoWorkspace.H10[900]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[901]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[902]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[903]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[904]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[905]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[906]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[907]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[908]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[909]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[91] += + acadoWorkspace.H10[910]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[911]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[912]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[913]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[914]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[915]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[916]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[917]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[918]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[919]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[92] += + acadoWorkspace.H10[920]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[921]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[922]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[923]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[924]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[925]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[926]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[927]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[928]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[929]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[93] += + acadoWorkspace.H10[930]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[931]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[932]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[933]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[934]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[935]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[936]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[937]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[938]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[939]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[94] += + acadoWorkspace.H10[940]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[941]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[942]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[943]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[944]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[945]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[946]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[947]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[948]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[949]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[95] += + acadoWorkspace.H10[950]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[951]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[952]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[953]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[954]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[955]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[956]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[957]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[958]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[959]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[96] += + acadoWorkspace.H10[960]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[961]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[962]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[963]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[964]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[965]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[966]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[967]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[968]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[969]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[97] += + acadoWorkspace.H10[970]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[971]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[972]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[973]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[974]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[975]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[976]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[977]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[978]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[979]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[98] += + acadoWorkspace.H10[980]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[981]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[982]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[983]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[984]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[985]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[986]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[987]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[988]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[989]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[99] += + acadoWorkspace.H10[990]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[991]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[992]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[993]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[994]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[995]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[996]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[997]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[998]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[999]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[100] += + acadoWorkspace.H10[1000]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1001]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1002]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1003]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1004]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1005]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1006]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1007]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1008]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1009]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[101] += + acadoWorkspace.H10[1010]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1011]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1012]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1013]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1014]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1015]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1016]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1017]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1018]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1019]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[102] += + acadoWorkspace.H10[1020]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1021]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1022]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1023]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1024]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1025]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1026]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1027]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1028]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1029]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[103] += + acadoWorkspace.H10[1030]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1031]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1032]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1033]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1034]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1035]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1036]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1037]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1038]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1039]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[104] += + acadoWorkspace.H10[1040]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1041]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1042]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1043]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1044]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1045]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1046]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1047]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1048]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1049]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[105] += + acadoWorkspace.H10[1050]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1051]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1052]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1053]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1054]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1055]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1056]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1057]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1058]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1059]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[106] += + acadoWorkspace.H10[1060]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1061]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1062]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1063]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1064]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1065]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1066]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1067]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1068]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1069]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[107] += + acadoWorkspace.H10[1070]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1071]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1072]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1073]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1074]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1075]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1076]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1077]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1078]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1079]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[108] += + acadoWorkspace.H10[1080]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1081]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1082]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1083]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1084]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1085]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1086]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1087]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1088]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1089]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[109] += + acadoWorkspace.H10[1090]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1091]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1092]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1093]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1094]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1095]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1096]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1097]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1098]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1099]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[110] += + acadoWorkspace.H10[1100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1103]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1104]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1105]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1106]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1107]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1108]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1109]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[111] += + acadoWorkspace.H10[1110]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1111]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1112]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1113]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1114]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1115]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1116]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1117]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1118]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1119]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[112] += + acadoWorkspace.H10[1120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1125]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1126]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1127]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1128]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1129]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[113] += + acadoWorkspace.H10[1130]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1131]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1132]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1133]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1134]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1135]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1136]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1137]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1138]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1139]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[114] += + acadoWorkspace.H10[1140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1144]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1145]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1146]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1147]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1148]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1149]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[115] += + acadoWorkspace.H10[1150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1154]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1155]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1156]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1157]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1158]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1159]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[116] += + acadoWorkspace.H10[1160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1163]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1164]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1165]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1166]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1167]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1168]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1169]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[117] += + acadoWorkspace.H10[1170]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1171]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1172]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1173]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1174]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1175]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1176]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1177]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1178]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1179]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[118] += + acadoWorkspace.H10[1180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1184]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1185]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1186]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1187]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1188]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1189]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[119] += + acadoWorkspace.H10[1190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1194]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1195]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1196]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1197]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1198]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1199]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[120] += + acadoWorkspace.H10[1200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1204]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1205]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1206]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1207]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1208]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1209]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[121] += + acadoWorkspace.H10[1210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1214]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1215]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1216]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1217]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1218]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1219]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[122] += + acadoWorkspace.H10[1220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1223]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1224]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1225]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1226]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1227]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1228]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1229]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[123] += + acadoWorkspace.H10[1230]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1231]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1232]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1233]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1234]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1235]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1236]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1237]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1238]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1239]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[124] += + acadoWorkspace.H10[1240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1243]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1244]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1245]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1246]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1247]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1248]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1249]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[125] += + acadoWorkspace.H10[1250]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1251]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1252]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1253]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1254]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1255]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1256]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1257]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1258]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1259]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[126] += + acadoWorkspace.H10[1260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1263]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1264]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1265]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1266]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1267]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1268]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1269]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[127] += + acadoWorkspace.H10[1270]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1271]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1272]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1273]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1274]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1275]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1276]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1277]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1278]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1279]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[128] += + acadoWorkspace.H10[1280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1283]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1284]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1285]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1286]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1287]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1288]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1289]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[129] += + acadoWorkspace.H10[1290]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1291]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1292]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1293]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1294]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1295]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1296]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1297]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1298]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1299]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[130] += + acadoWorkspace.H10[1300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1302]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1303]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1304]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1305]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1306]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1307]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1308]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1309]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[131] += + acadoWorkspace.H10[1310]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1311]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1312]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1313]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1314]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1315]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1316]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1317]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1318]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1319]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[132] += + acadoWorkspace.H10[1320]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1321]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1322]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1323]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1324]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1325]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1326]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1327]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1328]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1329]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[133] += + acadoWorkspace.H10[1330]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1331]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1332]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1333]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1334]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1335]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1336]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1337]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1338]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1339]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[134] += + acadoWorkspace.H10[1340]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1341]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1342]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1343]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1344]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1345]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1346]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1347]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1348]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1349]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[135] += + acadoWorkspace.H10[1350]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1351]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1352]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1353]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1354]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1355]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1356]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1357]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1358]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1359]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[136] += + acadoWorkspace.H10[1360]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1361]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1362]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1363]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1364]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1365]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1366]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1367]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1368]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1369]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[137] += + acadoWorkspace.H10[1370]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1371]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1372]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1373]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1374]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1375]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1376]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1377]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1378]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1379]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[138] += + acadoWorkspace.H10[1380]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1381]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1382]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1383]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1384]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1385]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1386]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1387]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1388]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1389]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[139] += + acadoWorkspace.H10[1390]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1391]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1392]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1393]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1394]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1395]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1396]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1397]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1398]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1399]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[140] += + acadoWorkspace.H10[1400]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1401]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1402]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1403]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1404]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1405]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1406]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1407]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1408]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1409]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[141] += + acadoWorkspace.H10[1410]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1411]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1412]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1413]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1414]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1415]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1416]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1417]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1418]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1419]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[142] += + acadoWorkspace.H10[1420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1422]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1423]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1424]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1425]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1426]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1427]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1428]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1429]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[143] += + acadoWorkspace.H10[1430]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1431]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1432]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1433]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1434]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1435]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1436]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1437]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1438]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1439]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[144] += + acadoWorkspace.H10[1440]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1441]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1442]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1443]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1444]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1445]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1446]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1447]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1448]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1449]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[145] += + acadoWorkspace.H10[1450]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1451]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1452]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1453]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1454]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1455]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1456]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1457]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1458]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1459]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[146] += + acadoWorkspace.H10[1460]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1461]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1462]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1463]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1464]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1465]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1466]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1467]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1468]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1469]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[147] += + acadoWorkspace.H10[1470]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1471]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1472]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1473]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1474]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1475]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1476]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1477]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1478]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1479]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[148] += + acadoWorkspace.H10[1480]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1481]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1482]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1483]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1484]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1485]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1486]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1487]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1488]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1489]*acadoWorkspace.Dx0[9];
acadoWorkspace.g[149] += + acadoWorkspace.H10[1490]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1491]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1492]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1493]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1494]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1495]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1496]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1497]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1498]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1499]*acadoWorkspace.Dx0[9];

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
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[369]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[374]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[375]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[376]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[377]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[378]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[379]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[384]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[385]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[386]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[387]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[388]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[389]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[395]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[396]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[397]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[398]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[399]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[405]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[406]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[407]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[408]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[409]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[414]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[415]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[416]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[417]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[418]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[419]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[426]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[427]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[428]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[429]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[434]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[435]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[436]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[437]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[438]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[439]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[445]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[446]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[447]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[448]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[449]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[455]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[456]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[457]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[458]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[459]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[464]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[465]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[466]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[467]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[468]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[469]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[474]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[475]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[476]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[477]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[478]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[479]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[488]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[489]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[495]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[496]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[497]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[498]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[499]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[504]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[505]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[506]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[507]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[508]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[509]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[515]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[516]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[517]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[518]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[519]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[525]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[526]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[527]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[528]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[529]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[530]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[531]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[532]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[533]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[534]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[535]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[536]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[537]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[538]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[539]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[549]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[550]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[551]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[552]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[553]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[554]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[555]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[556]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[557]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[558]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[559]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[565]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[566]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[567]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[568]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[569]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[574]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[575]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[576]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[577]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[578]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[579]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[583]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[584]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[585]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[586]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[587]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[588]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[589]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[590]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[591]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[592]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[593]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[594]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[595]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[596]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[597]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[598]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[599]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[606]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[607]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[608]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[609]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[610]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[611]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[612]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[613]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[614]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[615]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[616]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[617]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[618]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[619]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[624]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[625]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[626]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[627]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[628]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[629]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[636]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[637]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[638]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[639]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[644]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[645]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[646]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[647]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[648]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[649]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[650]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[651]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[652]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[653]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[654]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[655]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[656]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[657]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[658]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[659]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[666]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[667]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[668]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[669]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[674]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[675]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[676]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[677]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[678]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[679]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[683]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[684]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[685]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[686]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[687]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[688]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[689]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[694]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[695]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[696]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[697]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[698]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[699]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[704]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[705]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[706]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[707]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[708]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[709]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[710]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[711]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[712]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[713]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[714]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[715]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[716]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[717]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[718]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[719]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[725]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[726]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[727]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[728]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[729]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[730]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[731]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[732]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[733]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[734]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[735]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[736]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[737]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[738]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[739]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[744]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[745]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[746]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[747]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[748]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[749]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[750]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[751]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[752]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[753]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[754]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[755]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[756]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[757]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[758]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[759]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[760]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[761]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[762]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[763]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[764]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[765]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[766]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[767]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[768]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[769]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[770]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[771]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[772]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[773]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[774]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[775]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[776]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[777]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[778]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[779]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[783]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[784]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[785]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[786]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[787]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[788]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[789]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[790]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[791]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[792]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[793]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[794]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[795]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[796]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[797]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[798]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[799]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[800]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[801]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[802]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[803]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[804]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[805]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[806]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[807]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[808]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[809]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[810]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[811]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[812]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[813]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[814]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[815]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[816]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[817]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[818]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[819]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[820]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[821]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[822]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[823]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[824]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[825]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[826]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[827]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[828]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[829]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[830]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[831]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[832]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[833]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[834]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[835]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[836]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[837]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[838]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[839]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[844]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[845]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[846]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[847]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[848]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[849]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[850]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[851]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[852]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[853]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[854]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[855]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[856]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[857]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[858]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[859]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[860]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[861]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[862]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[863]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[864]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[865]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[866]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[867]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[868]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[869]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[870]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[871]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[872]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[873]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[874]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[875]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[876]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[877]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[878]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[879]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[88] = + acadoWorkspace.A01[880]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[881]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[882]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[883]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[884]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[885]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[886]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[887]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[888]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[889]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[89] = + acadoWorkspace.A01[890]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[891]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[892]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[893]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[894]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[895]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[896]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[897]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[898]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[899]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[90] = + acadoWorkspace.A01[900]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[901]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[902]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[903]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[904]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[905]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[906]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[907]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[908]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[909]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[91] = + acadoWorkspace.A01[910]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[911]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[912]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[913]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[914]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[915]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[916]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[917]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[918]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[919]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[92] = + acadoWorkspace.A01[920]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[921]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[922]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[923]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[924]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[925]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[926]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[927]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[928]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[929]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[93] = + acadoWorkspace.A01[930]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[931]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[932]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[933]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[934]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[935]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[936]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[937]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[938]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[939]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[94] = + acadoWorkspace.A01[940]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[941]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[942]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[943]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[944]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[945]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[946]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[947]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[948]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[949]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[95] = + acadoWorkspace.A01[950]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[951]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[952]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[953]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[954]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[955]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[956]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[957]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[958]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[959]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[96] = + acadoWorkspace.A01[960]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[961]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[962]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[963]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[964]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[965]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[966]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[967]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[968]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[969]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[97] = + acadoWorkspace.A01[970]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[971]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[972]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[973]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[974]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[975]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[976]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[977]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[978]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[979]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[98] = + acadoWorkspace.A01[980]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[981]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[982]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[983]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[984]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[985]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[986]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[987]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[988]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[989]*acadoWorkspace.Dx0[9];
acadoWorkspace.pacA01Dx0[99] = + acadoWorkspace.A01[990]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[991]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[992]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[993]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[994]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[995]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[996]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[997]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[998]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[999]*acadoWorkspace.Dx0[9];
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
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.lbA[80] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.lbA[81] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.lbA[82] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.lbA[83] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.lbA[84] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.lbA[85] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.lbA[86] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.lbA[87] -= acadoWorkspace.pacA01Dx0[87];
acadoWorkspace.lbA[88] -= acadoWorkspace.pacA01Dx0[88];
acadoWorkspace.lbA[89] -= acadoWorkspace.pacA01Dx0[89];
acadoWorkspace.lbA[90] -= acadoWorkspace.pacA01Dx0[90];
acadoWorkspace.lbA[91] -= acadoWorkspace.pacA01Dx0[91];
acadoWorkspace.lbA[92] -= acadoWorkspace.pacA01Dx0[92];
acadoWorkspace.lbA[93] -= acadoWorkspace.pacA01Dx0[93];
acadoWorkspace.lbA[94] -= acadoWorkspace.pacA01Dx0[94];
acadoWorkspace.lbA[95] -= acadoWorkspace.pacA01Dx0[95];
acadoWorkspace.lbA[96] -= acadoWorkspace.pacA01Dx0[96];
acadoWorkspace.lbA[97] -= acadoWorkspace.pacA01Dx0[97];
acadoWorkspace.lbA[98] -= acadoWorkspace.pacA01Dx0[98];
acadoWorkspace.lbA[99] -= acadoWorkspace.pacA01Dx0[99];

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
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.ubA[80] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.ubA[81] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.ubA[82] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.ubA[83] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.ubA[84] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.ubA[85] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.ubA[86] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.ubA[87] -= acadoWorkspace.pacA01Dx0[87];
acadoWorkspace.ubA[88] -= acadoWorkspace.pacA01Dx0[88];
acadoWorkspace.ubA[89] -= acadoWorkspace.pacA01Dx0[89];
acadoWorkspace.ubA[90] -= acadoWorkspace.pacA01Dx0[90];
acadoWorkspace.ubA[91] -= acadoWorkspace.pacA01Dx0[91];
acadoWorkspace.ubA[92] -= acadoWorkspace.pacA01Dx0[92];
acadoWorkspace.ubA[93] -= acadoWorkspace.pacA01Dx0[93];
acadoWorkspace.ubA[94] -= acadoWorkspace.pacA01Dx0[94];
acadoWorkspace.ubA[95] -= acadoWorkspace.pacA01Dx0[95];
acadoWorkspace.ubA[96] -= acadoWorkspace.pacA01Dx0[96];
acadoWorkspace.ubA[97] -= acadoWorkspace.pacA01Dx0[97];
acadoWorkspace.ubA[98] -= acadoWorkspace.pacA01Dx0[98];
acadoWorkspace.ubA[99] -= acadoWorkspace.pacA01Dx0[99];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];


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

for (lRun1 = 0; lRun1 < 500; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 10; ++lRun3)
{
t += + acadoWorkspace.evGx[(lRun1 * 10) + (lRun3)]*acadoWorkspace.Dx0[(lRun3) + (lRun2)];
}
acadoVariables.x[(lRun1 + 10) + (lRun2)] += t + acadoWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 30 ]), &(acadoWorkspace.x[ lRun2 * 3 ]), &(acadoVariables.x[ lRun1 * 10 + 10 ]) );
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
for (index = 0; index < 50; ++index)
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
acadoVariables.x[500] = xEnd[0];
acadoVariables.x[501] = xEnd[1];
acadoVariables.x[502] = xEnd[2];
acadoVariables.x[503] = xEnd[3];
acadoVariables.x[504] = xEnd[4];
acadoVariables.x[505] = xEnd[5];
acadoVariables.x[506] = xEnd[6];
acadoVariables.x[507] = xEnd[7];
acadoVariables.x[508] = xEnd[8];
acadoVariables.x[509] = xEnd[9];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[500];
acadoWorkspace.state[1] = acadoVariables.x[501];
acadoWorkspace.state[2] = acadoVariables.x[502];
acadoWorkspace.state[3] = acadoVariables.x[503];
acadoWorkspace.state[4] = acadoVariables.x[504];
acadoWorkspace.state[5] = acadoVariables.x[505];
acadoWorkspace.state[6] = acadoVariables.x[506];
acadoWorkspace.state[7] = acadoVariables.x[507];
acadoWorkspace.state[8] = acadoVariables.x[508];
acadoWorkspace.state[9] = acadoVariables.x[509];
if (uEnd != 0)
{
acadoWorkspace.state[140] = uEnd[0];
acadoWorkspace.state[141] = uEnd[1];
acadoWorkspace.state[142] = uEnd[2];
}
else
{
acadoWorkspace.state[140] = acadoVariables.u[147];
acadoWorkspace.state[141] = acadoVariables.u[148];
acadoWorkspace.state[142] = acadoVariables.u[149];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[500] = acadoWorkspace.state[0];
acadoVariables.x[501] = acadoWorkspace.state[1];
acadoVariables.x[502] = acadoWorkspace.state[2];
acadoVariables.x[503] = acadoWorkspace.state[3];
acadoVariables.x[504] = acadoWorkspace.state[4];
acadoVariables.x[505] = acadoWorkspace.state[5];
acadoVariables.x[506] = acadoWorkspace.state[6];
acadoVariables.x[507] = acadoWorkspace.state[7];
acadoVariables.x[508] = acadoWorkspace.state[8];
acadoVariables.x[509] = acadoWorkspace.state[9];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 49; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[147] = uEnd[0];
acadoVariables.u[148] = uEnd[1];
acadoVariables.u[149] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147] + acadoWorkspace.g[148]*acadoWorkspace.x[148] + acadoWorkspace.g[149]*acadoWorkspace.x[149];
kkt = fabs( kkt );
for (index = 0; index < 150; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 100; ++index)
{
prd = acadoWorkspace.y[index + 150];
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

for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[500];
acadoWorkspace.objValueIn[1] = acadoVariables.x[501];
acadoWorkspace.objValueIn[2] = acadoVariables.x[502];
acadoWorkspace.objValueIn[3] = acadoVariables.x[503];
acadoWorkspace.objValueIn[4] = acadoVariables.x[504];
acadoWorkspace.objValueIn[5] = acadoVariables.x[505];
acadoWorkspace.objValueIn[6] = acadoVariables.x[506];
acadoWorkspace.objValueIn[7] = acadoVariables.x[507];
acadoWorkspace.objValueIn[8] = acadoVariables.x[508];
acadoWorkspace.objValueIn[9] = acadoVariables.x[509];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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

