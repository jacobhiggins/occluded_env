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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 15;
/* Vector of auxiliary variables; number of elements: 8. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((xd[0])*(xd[0]));
a[1] = (pow((xd[1]/xd[0]),2));
a[2] = (atan((xd[1]/xd[0])));
a[3] = ((xd[1])*(xd[1]));
a[4] = (pow((xd[0]-xd[8]),2));
a[5] = (pow(((xd[1]-xd[9])/(xd[0]-xd[8])),2));
a[6] = (atan(((xd[1]-xd[9])/(xd[0]-xd[8]))));
a[7] = (pow((xd[1]-xd[9]),2));

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = xd[4];
out[3] = xd[5];
out[4] = u[0];
out[5] = u[1];
out[6] = (((((xd[3]/xd[0])-((xd[1]*xd[2])/a[0]))/((real_t)(1.0000000000000000e+00)+a[1]))/xd[1])-((a[2]*xd[3])/a[3]));
out[7] = (((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[4]))/((real_t)(1.0000000000000000e+00)+a[5]))/(xd[1]-xd[9]))-((xd[3]*a[6])/a[7]));
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = u[2];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 58. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[1] = (a[0]*a[0]);
a[2] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[3] = ((xd[0])*(xd[0]));
a[4] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[5] = (a[4]*a[4]);
a[6] = (pow((xd[1]/xd[0]),2));
a[7] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[6]));
a[8] = ((real_t)(2.0000000000000000e+00)*(xd[1]/xd[0]));
a[9] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[10] = (a[9]*a[9]);
a[11] = (a[8]*((real_t)(0.0000000000000000e+00)-(xd[1]*a[10])));
a[12] = (a[7]*a[7]);
a[13] = ((real_t)(1.0000000000000000e+00)/xd[1]);
a[14] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[15] = (a[14]*a[14]);
a[16] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow((xd[1]/xd[0]),2))));
a[17] = (((real_t)(0.0000000000000000e+00)-(xd[1]*a[15]))*a[16]);
a[18] = ((xd[1])*(xd[1]));
a[19] = ((real_t)(1.0000000000000000e+00)/a[18]);
a[20] = (a[8]*a[9]);
a[21] = (a[13]*a[13]);
a[22] = (a[14]*a[16]);
a[23] = (atan((xd[1]/xd[0])));
a[24] = ((real_t)(2.0000000000000000e+00)*xd[1]);
a[25] = (a[19]*a[19]);
a[26] = ((real_t)(1.0000000000000000e+00)/(xd[0]-xd[8]));
a[27] = (a[26]*a[26]);
a[28] = ((real_t)(2.0000000000000000e+00)*(xd[0]-xd[8]));
a[29] = (pow((xd[0]-xd[8]),2));
a[30] = ((real_t)(1.0000000000000000e+00)/a[29]);
a[31] = (a[30]*a[30]);
a[32] = (pow(((xd[1]-xd[9])/(xd[0]-xd[8])),2));
a[33] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[32]));
a[34] = ((real_t)(2.0000000000000000e+00)*((xd[1]-xd[9])/(xd[0]-xd[8])));
a[35] = ((real_t)(1.0000000000000000e+00)/(xd[0]-xd[8]));
a[36] = (a[35]*a[35]);
a[37] = (a[34]*((real_t)(0.0000000000000000e+00)-((xd[1]-xd[9])*a[36])));
a[38] = (a[33]*a[33]);
a[39] = ((real_t)(1.0000000000000000e+00)/(xd[1]-xd[9]));
a[40] = ((real_t)(1.0000000000000000e+00)/(xd[0]-xd[8]));
a[41] = (a[40]*a[40]);
a[42] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((xd[1]-xd[9])/(xd[0]-xd[8])),2))));
a[43] = (((real_t)(0.0000000000000000e+00)-((xd[1]-xd[9])*a[41]))*a[42]);
a[44] = (pow((xd[1]-xd[9]),2));
a[45] = ((real_t)(1.0000000000000000e+00)/a[44]);
a[46] = (a[34]*a[35]);
a[47] = (a[39]*a[39]);
a[48] = (a[40]*a[42]);
a[49] = (atan(((xd[1]-xd[9])/(xd[0]-xd[8]))));
a[50] = ((real_t)(2.0000000000000000e+00)*(xd[1]-xd[9]));
a[51] = (a[45]*a[45]);
a[52] = (a[28]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)));
a[53] = (a[34]*((real_t)(0.0000000000000000e+00)-(((xd[1]-xd[9])*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[36])));
a[54] = (((real_t)(0.0000000000000000e+00)-(((xd[1]-xd[9])*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[41]))*a[42]);
a[55] = (a[34]*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[35]));
a[56] = ((((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[40])*a[42]);
a[57] = (a[50]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(1.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(1.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(1.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(1.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(1.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(1.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (((((((real_t)(0.0000000000000000e+00)-(xd[3]*a[1]))-((real_t)(0.0000000000000000e+00)-(((xd[1]*xd[2])*a[2])*a[5])))*a[7])-((((xd[3]/xd[0])-((xd[1]*xd[2])/a[3]))*a[11])*a[12]))*a[13])-((a[17]*xd[3])*a[19]));
out[109] = (((((((real_t)(0.0000000000000000e+00)-(xd[2]*a[4]))*a[7])-((((xd[3]/xd[0])-((xd[1]*xd[2])/a[3]))*a[20])*a[12]))*a[13])-((((xd[3]/xd[0])-((xd[1]*xd[2])/a[3]))/((real_t)(1.0000000000000000e+00)+a[6]))*a[21]))-(((a[22]*xd[3])*a[19])-(((a[23]*xd[3])*a[24])*a[25])));
out[110] = ((((real_t)(0.0000000000000000e+00)-(xd[1]*a[4]))*a[7])*a[13]);
out[111] = (((a[0]*a[7])*a[13])-(a[23]*a[19]));
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (((((((real_t)(0.0000000000000000e+00)-(xd[3]*a[27]))-((real_t)(0.0000000000000000e+00)-(((xd[2]*(xd[1]-xd[9]))*a[28])*a[31])))*a[33])-((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[29]))*a[37])*a[38]))*a[39])-((xd[3]*a[43])*a[45]));
out[127] = (((((((real_t)(0.0000000000000000e+00)-(xd[2]*a[30]))*a[33])-((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[29]))*a[46])*a[38]))*a[39])-((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[29]))/((real_t)(1.0000000000000000e+00)+a[32]))*a[47]))-(((xd[3]*a[48])*a[45])-(((xd[3]*a[49])*a[50])*a[51])));
out[128] = ((((real_t)(0.0000000000000000e+00)-((xd[1]-xd[9])*a[30]))*a[33])*a[39]);
out[129] = (((a[26]*a[33])*a[39])-(a[49]*a[45]));
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (((((((real_t)(0.0000000000000000e+00)-((xd[3]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[27]))-((real_t)(0.0000000000000000e+00)-(((xd[2]*(xd[1]-xd[9]))*a[52])*a[31])))*a[33])-((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[29]))*a[53])*a[38]))*a[39])-((xd[3]*a[54])*a[45]));
out[135] = (((((((real_t)(0.0000000000000000e+00)-((xd[2]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[30]))*a[33])-((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[29]))*a[55])*a[38]))*a[39])-(((((xd[3]/(xd[0]-xd[8]))-((xd[2]*(xd[1]-xd[9]))/a[29]))/((real_t)(1.0000000000000000e+00)+a[32]))*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[47]))-(((xd[3]*a[56])*a[45])-(((xd[3]*a[49])*a[57])*a[51])));
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(0.0000000000000000e+00);
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(0.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (real_t)(0.0000000000000000e+00);
out[203] = (real_t)(0.0000000000000000e+00);
out[204] = (real_t)(0.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (real_t)(0.0000000000000000e+00);
out[208] = (real_t)(0.0000000000000000e+00);
out[209] = (real_t)(0.0000000000000000e+00);
out[210] = (real_t)(0.0000000000000000e+00);
out[211] = (real_t)(0.0000000000000000e+00);
out[212] = (real_t)(0.0000000000000000e+00);
out[213] = (real_t)(0.0000000000000000e+00);
out[214] = (real_t)(0.0000000000000000e+00);
out[215] = (real_t)(0.0000000000000000e+00);
out[216] = (real_t)(0.0000000000000000e+00);
out[217] = (real_t)(0.0000000000000000e+00);
out[218] = (real_t)(0.0000000000000000e+00);
out[219] = (real_t)(0.0000000000000000e+00);
out[220] = (real_t)(0.0000000000000000e+00);
out[221] = (real_t)(0.0000000000000000e+00);
out[222] = (real_t)(0.0000000000000000e+00);
out[223] = (real_t)(0.0000000000000000e+00);
out[224] = (real_t)(0.0000000000000000e+00);
out[225] = (real_t)(0.0000000000000000e+00);
out[226] = (real_t)(0.0000000000000000e+00);
out[227] = (real_t)(0.0000000000000000e+00);
out[228] = (real_t)(0.0000000000000000e+00);
out[229] = (real_t)(0.0000000000000000e+00);
out[230] = (real_t)(0.0000000000000000e+00);
out[231] = (real_t)(0.0000000000000000e+00);
out[232] = (real_t)(0.0000000000000000e+00);
out[233] = (real_t)(0.0000000000000000e+00);
out[234] = (real_t)(0.0000000000000000e+00);
out[235] = (real_t)(0.0000000000000000e+00);
out[236] = (real_t)(0.0000000000000000e+00);
out[237] = (real_t)(0.0000000000000000e+00);
out[238] = (real_t)(0.0000000000000000e+00);
out[239] = (real_t)(0.0000000000000000e+00);
out[240] = (real_t)(0.0000000000000000e+00);
out[241] = (real_t)(0.0000000000000000e+00);
out[242] = (real_t)(0.0000000000000000e+00);
out[243] = (real_t)(0.0000000000000000e+00);
out[244] = (real_t)(0.0000000000000000e+00);
out[245] = (real_t)(0.0000000000000000e+00);
out[246] = (real_t)(0.0000000000000000e+00);
out[247] = (real_t)(0.0000000000000000e+00);
out[248] = (real_t)(0.0000000000000000e+00);
out[249] = (real_t)(0.0000000000000000e+00);
out[250] = (real_t)(0.0000000000000000e+00);
out[251] = (real_t)(0.0000000000000000e+00);
out[252] = (real_t)(0.0000000000000000e+00);
out[253] = (real_t)(0.0000000000000000e+00);
out[254] = (real_t)(0.0000000000000000e+00);
out[255] = (real_t)(0.0000000000000000e+00);
out[256] = (real_t)(0.0000000000000000e+00);
out[257] = (real_t)(0.0000000000000000e+00);
out[258] = (real_t)(0.0000000000000000e+00);
out[259] = (real_t)(0.0000000000000000e+00);
out[260] = (real_t)(0.0000000000000000e+00);
out[261] = (real_t)(0.0000000000000000e+00);
out[262] = (real_t)(0.0000000000000000e+00);
out[263] = (real_t)(0.0000000000000000e+00);
out[264] = (real_t)(0.0000000000000000e+00);
out[265] = (real_t)(0.0000000000000000e+00);
out[266] = (real_t)(0.0000000000000000e+00);
out[267] = (real_t)(0.0000000000000000e+00);
out[268] = (real_t)(0.0000000000000000e+00);
out[269] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim30_triangular( real_t* const A, real_t* const b )
{

b[29] = b[29]/A[899];
b[28] -= + A[869]*b[29];
b[28] = b[28]/A[868];
b[27] -= + A[839]*b[29];
b[27] -= + A[838]*b[28];
b[27] = b[27]/A[837];
b[26] -= + A[809]*b[29];
b[26] -= + A[808]*b[28];
b[26] -= + A[807]*b[27];
b[26] = b[26]/A[806];
b[25] -= + A[779]*b[29];
b[25] -= + A[778]*b[28];
b[25] -= + A[777]*b[27];
b[25] -= + A[776]*b[26];
b[25] = b[25]/A[775];
b[24] -= + A[749]*b[29];
b[24] -= + A[748]*b[28];
b[24] -= + A[747]*b[27];
b[24] -= + A[746]*b[26];
b[24] -= + A[745]*b[25];
b[24] = b[24]/A[744];
b[23] -= + A[719]*b[29];
b[23] -= + A[718]*b[28];
b[23] -= + A[717]*b[27];
b[23] -= + A[716]*b[26];
b[23] -= + A[715]*b[25];
b[23] -= + A[714]*b[24];
b[23] = b[23]/A[713];
b[22] -= + A[689]*b[29];
b[22] -= + A[688]*b[28];
b[22] -= + A[687]*b[27];
b[22] -= + A[686]*b[26];
b[22] -= + A[685]*b[25];
b[22] -= + A[684]*b[24];
b[22] -= + A[683]*b[23];
b[22] = b[22]/A[682];
b[21] -= + A[659]*b[29];
b[21] -= + A[658]*b[28];
b[21] -= + A[657]*b[27];
b[21] -= + A[656]*b[26];
b[21] -= + A[655]*b[25];
b[21] -= + A[654]*b[24];
b[21] -= + A[653]*b[23];
b[21] -= + A[652]*b[22];
b[21] = b[21]/A[651];
b[20] -= + A[629]*b[29];
b[20] -= + A[628]*b[28];
b[20] -= + A[627]*b[27];
b[20] -= + A[626]*b[26];
b[20] -= + A[625]*b[25];
b[20] -= + A[624]*b[24];
b[20] -= + A[623]*b[23];
b[20] -= + A[622]*b[22];
b[20] -= + A[621]*b[21];
b[20] = b[20]/A[620];
b[19] -= + A[599]*b[29];
b[19] -= + A[598]*b[28];
b[19] -= + A[597]*b[27];
b[19] -= + A[596]*b[26];
b[19] -= + A[595]*b[25];
b[19] -= + A[594]*b[24];
b[19] -= + A[593]*b[23];
b[19] -= + A[592]*b[22];
b[19] -= + A[591]*b[21];
b[19] -= + A[590]*b[20];
b[19] = b[19]/A[589];
b[18] -= + A[569]*b[29];
b[18] -= + A[568]*b[28];
b[18] -= + A[567]*b[27];
b[18] -= + A[566]*b[26];
b[18] -= + A[565]*b[25];
b[18] -= + A[564]*b[24];
b[18] -= + A[563]*b[23];
b[18] -= + A[562]*b[22];
b[18] -= + A[561]*b[21];
b[18] -= + A[560]*b[20];
b[18] -= + A[559]*b[19];
b[18] = b[18]/A[558];
b[17] -= + A[539]*b[29];
b[17] -= + A[538]*b[28];
b[17] -= + A[537]*b[27];
b[17] -= + A[536]*b[26];
b[17] -= + A[535]*b[25];
b[17] -= + A[534]*b[24];
b[17] -= + A[533]*b[23];
b[17] -= + A[532]*b[22];
b[17] -= + A[531]*b[21];
b[17] -= + A[530]*b[20];
b[17] -= + A[529]*b[19];
b[17] -= + A[528]*b[18];
b[17] = b[17]/A[527];
b[16] -= + A[509]*b[29];
b[16] -= + A[508]*b[28];
b[16] -= + A[507]*b[27];
b[16] -= + A[506]*b[26];
b[16] -= + A[505]*b[25];
b[16] -= + A[504]*b[24];
b[16] -= + A[503]*b[23];
b[16] -= + A[502]*b[22];
b[16] -= + A[501]*b[21];
b[16] -= + A[500]*b[20];
b[16] -= + A[499]*b[19];
b[16] -= + A[498]*b[18];
b[16] -= + A[497]*b[17];
b[16] = b[16]/A[496];
b[15] -= + A[479]*b[29];
b[15] -= + A[478]*b[28];
b[15] -= + A[477]*b[27];
b[15] -= + A[476]*b[26];
b[15] -= + A[475]*b[25];
b[15] -= + A[474]*b[24];
b[15] -= + A[473]*b[23];
b[15] -= + A[472]*b[22];
b[15] -= + A[471]*b[21];
b[15] -= + A[470]*b[20];
b[15] -= + A[469]*b[19];
b[15] -= + A[468]*b[18];
b[15] -= + A[467]*b[17];
b[15] -= + A[466]*b[16];
b[15] = b[15]/A[465];
b[14] -= + A[449]*b[29];
b[14] -= + A[448]*b[28];
b[14] -= + A[447]*b[27];
b[14] -= + A[446]*b[26];
b[14] -= + A[445]*b[25];
b[14] -= + A[444]*b[24];
b[14] -= + A[443]*b[23];
b[14] -= + A[442]*b[22];
b[14] -= + A[441]*b[21];
b[14] -= + A[440]*b[20];
b[14] -= + A[439]*b[19];
b[14] -= + A[438]*b[18];
b[14] -= + A[437]*b[17];
b[14] -= + A[436]*b[16];
b[14] -= + A[435]*b[15];
b[14] = b[14]/A[434];
b[13] -= + A[419]*b[29];
b[13] -= + A[418]*b[28];
b[13] -= + A[417]*b[27];
b[13] -= + A[416]*b[26];
b[13] -= + A[415]*b[25];
b[13] -= + A[414]*b[24];
b[13] -= + A[413]*b[23];
b[13] -= + A[412]*b[22];
b[13] -= + A[411]*b[21];
b[13] -= + A[410]*b[20];
b[13] -= + A[409]*b[19];
b[13] -= + A[408]*b[18];
b[13] -= + A[407]*b[17];
b[13] -= + A[406]*b[16];
b[13] -= + A[405]*b[15];
b[13] -= + A[404]*b[14];
b[13] = b[13]/A[403];
b[12] -= + A[389]*b[29];
b[12] -= + A[388]*b[28];
b[12] -= + A[387]*b[27];
b[12] -= + A[386]*b[26];
b[12] -= + A[385]*b[25];
b[12] -= + A[384]*b[24];
b[12] -= + A[383]*b[23];
b[12] -= + A[382]*b[22];
b[12] -= + A[381]*b[21];
b[12] -= + A[380]*b[20];
b[12] -= + A[379]*b[19];
b[12] -= + A[378]*b[18];
b[12] -= + A[377]*b[17];
b[12] -= + A[376]*b[16];
b[12] -= + A[375]*b[15];
b[12] -= + A[374]*b[14];
b[12] -= + A[373]*b[13];
b[12] = b[12]/A[372];
b[11] -= + A[359]*b[29];
b[11] -= + A[358]*b[28];
b[11] -= + A[357]*b[27];
b[11] -= + A[356]*b[26];
b[11] -= + A[355]*b[25];
b[11] -= + A[354]*b[24];
b[11] -= + A[353]*b[23];
b[11] -= + A[352]*b[22];
b[11] -= + A[351]*b[21];
b[11] -= + A[350]*b[20];
b[11] -= + A[349]*b[19];
b[11] -= + A[348]*b[18];
b[11] -= + A[347]*b[17];
b[11] -= + A[346]*b[16];
b[11] -= + A[345]*b[15];
b[11] -= + A[344]*b[14];
b[11] -= + A[343]*b[13];
b[11] -= + A[342]*b[12];
b[11] = b[11]/A[341];
b[10] -= + A[329]*b[29];
b[10] -= + A[328]*b[28];
b[10] -= + A[327]*b[27];
b[10] -= + A[326]*b[26];
b[10] -= + A[325]*b[25];
b[10] -= + A[324]*b[24];
b[10] -= + A[323]*b[23];
b[10] -= + A[322]*b[22];
b[10] -= + A[321]*b[21];
b[10] -= + A[320]*b[20];
b[10] -= + A[319]*b[19];
b[10] -= + A[318]*b[18];
b[10] -= + A[317]*b[17];
b[10] -= + A[316]*b[16];
b[10] -= + A[315]*b[15];
b[10] -= + A[314]*b[14];
b[10] -= + A[313]*b[13];
b[10] -= + A[312]*b[12];
b[10] -= + A[311]*b[11];
b[10] = b[10]/A[310];
b[9] -= + A[299]*b[29];
b[9] -= + A[298]*b[28];
b[9] -= + A[297]*b[27];
b[9] -= + A[296]*b[26];
b[9] -= + A[295]*b[25];
b[9] -= + A[294]*b[24];
b[9] -= + A[293]*b[23];
b[9] -= + A[292]*b[22];
b[9] -= + A[291]*b[21];
b[9] -= + A[290]*b[20];
b[9] -= + A[289]*b[19];
b[9] -= + A[288]*b[18];
b[9] -= + A[287]*b[17];
b[9] -= + A[286]*b[16];
b[9] -= + A[285]*b[15];
b[9] -= + A[284]*b[14];
b[9] -= + A[283]*b[13];
b[9] -= + A[282]*b[12];
b[9] -= + A[281]*b[11];
b[9] -= + A[280]*b[10];
b[9] = b[9]/A[279];
b[8] -= + A[269]*b[29];
b[8] -= + A[268]*b[28];
b[8] -= + A[267]*b[27];
b[8] -= + A[266]*b[26];
b[8] -= + A[265]*b[25];
b[8] -= + A[264]*b[24];
b[8] -= + A[263]*b[23];
b[8] -= + A[262]*b[22];
b[8] -= + A[261]*b[21];
b[8] -= + A[260]*b[20];
b[8] -= + A[259]*b[19];
b[8] -= + A[258]*b[18];
b[8] -= + A[257]*b[17];
b[8] -= + A[256]*b[16];
b[8] -= + A[255]*b[15];
b[8] -= + A[254]*b[14];
b[8] -= + A[253]*b[13];
b[8] -= + A[252]*b[12];
b[8] -= + A[251]*b[11];
b[8] -= + A[250]*b[10];
b[8] -= + A[249]*b[9];
b[8] = b[8]/A[248];
b[7] -= + A[239]*b[29];
b[7] -= + A[238]*b[28];
b[7] -= + A[237]*b[27];
b[7] -= + A[236]*b[26];
b[7] -= + A[235]*b[25];
b[7] -= + A[234]*b[24];
b[7] -= + A[233]*b[23];
b[7] -= + A[232]*b[22];
b[7] -= + A[231]*b[21];
b[7] -= + A[230]*b[20];
b[7] -= + A[229]*b[19];
b[7] -= + A[228]*b[18];
b[7] -= + A[227]*b[17];
b[7] -= + A[226]*b[16];
b[7] -= + A[225]*b[15];
b[7] -= + A[224]*b[14];
b[7] -= + A[223]*b[13];
b[7] -= + A[222]*b[12];
b[7] -= + A[221]*b[11];
b[7] -= + A[220]*b[10];
b[7] -= + A[219]*b[9];
b[7] -= + A[218]*b[8];
b[7] = b[7]/A[217];
b[6] -= + A[209]*b[29];
b[6] -= + A[208]*b[28];
b[6] -= + A[207]*b[27];
b[6] -= + A[206]*b[26];
b[6] -= + A[205]*b[25];
b[6] -= + A[204]*b[24];
b[6] -= + A[203]*b[23];
b[6] -= + A[202]*b[22];
b[6] -= + A[201]*b[21];
b[6] -= + A[200]*b[20];
b[6] -= + A[199]*b[19];
b[6] -= + A[198]*b[18];
b[6] -= + A[197]*b[17];
b[6] -= + A[196]*b[16];
b[6] -= + A[195]*b[15];
b[6] -= + A[194]*b[14];
b[6] -= + A[193]*b[13];
b[6] -= + A[192]*b[12];
b[6] -= + A[191]*b[11];
b[6] -= + A[190]*b[10];
b[6] -= + A[189]*b[9];
b[6] -= + A[188]*b[8];
b[6] -= + A[187]*b[7];
b[6] = b[6]/A[186];
b[5] -= + A[179]*b[29];
b[5] -= + A[178]*b[28];
b[5] -= + A[177]*b[27];
b[5] -= + A[176]*b[26];
b[5] -= + A[175]*b[25];
b[5] -= + A[174]*b[24];
b[5] -= + A[173]*b[23];
b[5] -= + A[172]*b[22];
b[5] -= + A[171]*b[21];
b[5] -= + A[170]*b[20];
b[5] -= + A[169]*b[19];
b[5] -= + A[168]*b[18];
b[5] -= + A[167]*b[17];
b[5] -= + A[166]*b[16];
b[5] -= + A[165]*b[15];
b[5] -= + A[164]*b[14];
b[5] -= + A[163]*b[13];
b[5] -= + A[162]*b[12];
b[5] -= + A[161]*b[11];
b[5] -= + A[160]*b[10];
b[5] -= + A[159]*b[9];
b[5] -= + A[158]*b[8];
b[5] -= + A[157]*b[7];
b[5] -= + A[156]*b[6];
b[5] = b[5]/A[155];
b[4] -= + A[149]*b[29];
b[4] -= + A[148]*b[28];
b[4] -= + A[147]*b[27];
b[4] -= + A[146]*b[26];
b[4] -= + A[145]*b[25];
b[4] -= + A[144]*b[24];
b[4] -= + A[143]*b[23];
b[4] -= + A[142]*b[22];
b[4] -= + A[141]*b[21];
b[4] -= + A[140]*b[20];
b[4] -= + A[139]*b[19];
b[4] -= + A[138]*b[18];
b[4] -= + A[137]*b[17];
b[4] -= + A[136]*b[16];
b[4] -= + A[135]*b[15];
b[4] -= + A[134]*b[14];
b[4] -= + A[133]*b[13];
b[4] -= + A[132]*b[12];
b[4] -= + A[131]*b[11];
b[4] -= + A[130]*b[10];
b[4] -= + A[129]*b[9];
b[4] -= + A[128]*b[8];
b[4] -= + A[127]*b[7];
b[4] -= + A[126]*b[6];
b[4] -= + A[125]*b[5];
b[4] = b[4]/A[124];
b[3] -= + A[119]*b[29];
b[3] -= + A[118]*b[28];
b[3] -= + A[117]*b[27];
b[3] -= + A[116]*b[26];
b[3] -= + A[115]*b[25];
b[3] -= + A[114]*b[24];
b[3] -= + A[113]*b[23];
b[3] -= + A[112]*b[22];
b[3] -= + A[111]*b[21];
b[3] -= + A[110]*b[20];
b[3] -= + A[109]*b[19];
b[3] -= + A[108]*b[18];
b[3] -= + A[107]*b[17];
b[3] -= + A[106]*b[16];
b[3] -= + A[105]*b[15];
b[3] -= + A[104]*b[14];
b[3] -= + A[103]*b[13];
b[3] -= + A[102]*b[12];
b[3] -= + A[101]*b[11];
b[3] -= + A[100]*b[10];
b[3] -= + A[99]*b[9];
b[3] -= + A[98]*b[8];
b[3] -= + A[97]*b[7];
b[3] -= + A[96]*b[6];
b[3] -= + A[95]*b[5];
b[3] -= + A[94]*b[4];
b[3] = b[3]/A[93];
b[2] -= + A[89]*b[29];
b[2] -= + A[88]*b[28];
b[2] -= + A[87]*b[27];
b[2] -= + A[86]*b[26];
b[2] -= + A[85]*b[25];
b[2] -= + A[84]*b[24];
b[2] -= + A[83]*b[23];
b[2] -= + A[82]*b[22];
b[2] -= + A[81]*b[21];
b[2] -= + A[80]*b[20];
b[2] -= + A[79]*b[19];
b[2] -= + A[78]*b[18];
b[2] -= + A[77]*b[17];
b[2] -= + A[76]*b[16];
b[2] -= + A[75]*b[15];
b[2] -= + A[74]*b[14];
b[2] -= + A[73]*b[13];
b[2] -= + A[72]*b[12];
b[2] -= + A[71]*b[11];
b[2] -= + A[70]*b[10];
b[2] -= + A[69]*b[9];
b[2] -= + A[68]*b[8];
b[2] -= + A[67]*b[7];
b[2] -= + A[66]*b[6];
b[2] -= + A[65]*b[5];
b[2] -= + A[64]*b[4];
b[2] -= + A[63]*b[3];
b[2] = b[2]/A[62];
b[1] -= + A[59]*b[29];
b[1] -= + A[58]*b[28];
b[1] -= + A[57]*b[27];
b[1] -= + A[56]*b[26];
b[1] -= + A[55]*b[25];
b[1] -= + A[54]*b[24];
b[1] -= + A[53]*b[23];
b[1] -= + A[52]*b[22];
b[1] -= + A[51]*b[21];
b[1] -= + A[50]*b[20];
b[1] -= + A[49]*b[19];
b[1] -= + A[48]*b[18];
b[1] -= + A[47]*b[17];
b[1] -= + A[46]*b[16];
b[1] -= + A[45]*b[15];
b[1] -= + A[44]*b[14];
b[1] -= + A[43]*b[13];
b[1] -= + A[42]*b[12];
b[1] -= + A[41]*b[11];
b[1] -= + A[40]*b[10];
b[1] -= + A[39]*b[9];
b[1] -= + A[38]*b[8];
b[1] -= + A[37]*b[7];
b[1] -= + A[36]*b[6];
b[1] -= + A[35]*b[5];
b[1] -= + A[34]*b[4];
b[1] -= + A[33]*b[3];
b[1] -= + A[32]*b[2];
b[1] = b[1]/A[31];
b[0] -= + A[29]*b[29];
b[0] -= + A[28]*b[28];
b[0] -= + A[27]*b[27];
b[0] -= + A[26]*b[26];
b[0] -= + A[25]*b[25];
b[0] -= + A[24]*b[24];
b[0] -= + A[23]*b[23];
b[0] -= + A[22]*b[22];
b[0] -= + A[21]*b[21];
b[0] -= + A[20]*b[20];
b[0] -= + A[19]*b[19];
b[0] -= + A[18]*b[18];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim30_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 30; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (29); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*30+i]);
	for( j=(i+1); j < 30; j++ ) {
		temp = fabs(A[j*30+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 30; ++k)
{
	acadoWorkspace.rk_dim30_swap = A[i*30+k];
	A[i*30+k] = A[indexMax*30+k];
	A[indexMax*30+k] = acadoWorkspace.rk_dim30_swap;
}
	acadoWorkspace.rk_dim30_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim30_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*30+i];
	for( j=i+1; j < 30; j++ ) {
		A[j*30+i] = -A[j*30+i]/A[i*30+i];
		for( k=i+1; k < 30; k++ ) {
			A[j*30+k] += A[j*30+i] * A[i*30+k];
		}
		b[j] += A[j*30+i] * b[i];
	}
}
det *= A[899];
det = fabs(det);
acado_solve_dim30_triangular( A, b );
return det;
}

void acado_solve_dim30_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim30_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim30_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim30_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim30_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim30_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim30_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim30_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim30_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim30_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim30_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim30_bPerm[10] = b[rk_perm[10]];
acadoWorkspace.rk_dim30_bPerm[11] = b[rk_perm[11]];
acadoWorkspace.rk_dim30_bPerm[12] = b[rk_perm[12]];
acadoWorkspace.rk_dim30_bPerm[13] = b[rk_perm[13]];
acadoWorkspace.rk_dim30_bPerm[14] = b[rk_perm[14]];
acadoWorkspace.rk_dim30_bPerm[15] = b[rk_perm[15]];
acadoWorkspace.rk_dim30_bPerm[16] = b[rk_perm[16]];
acadoWorkspace.rk_dim30_bPerm[17] = b[rk_perm[17]];
acadoWorkspace.rk_dim30_bPerm[18] = b[rk_perm[18]];
acadoWorkspace.rk_dim30_bPerm[19] = b[rk_perm[19]];
acadoWorkspace.rk_dim30_bPerm[20] = b[rk_perm[20]];
acadoWorkspace.rk_dim30_bPerm[21] = b[rk_perm[21]];
acadoWorkspace.rk_dim30_bPerm[22] = b[rk_perm[22]];
acadoWorkspace.rk_dim30_bPerm[23] = b[rk_perm[23]];
acadoWorkspace.rk_dim30_bPerm[24] = b[rk_perm[24]];
acadoWorkspace.rk_dim30_bPerm[25] = b[rk_perm[25]];
acadoWorkspace.rk_dim30_bPerm[26] = b[rk_perm[26]];
acadoWorkspace.rk_dim30_bPerm[27] = b[rk_perm[27]];
acadoWorkspace.rk_dim30_bPerm[28] = b[rk_perm[28]];
acadoWorkspace.rk_dim30_bPerm[29] = b[rk_perm[29]];
acadoWorkspace.rk_dim30_bPerm[1] += A[30]*acadoWorkspace.rk_dim30_bPerm[0];

acadoWorkspace.rk_dim30_bPerm[2] += A[60]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[2] += A[61]*acadoWorkspace.rk_dim30_bPerm[1];

acadoWorkspace.rk_dim30_bPerm[3] += A[90]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[3] += A[91]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[3] += A[92]*acadoWorkspace.rk_dim30_bPerm[2];

acadoWorkspace.rk_dim30_bPerm[4] += A[120]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[4] += A[121]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[4] += A[122]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[4] += A[123]*acadoWorkspace.rk_dim30_bPerm[3];

acadoWorkspace.rk_dim30_bPerm[5] += A[150]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[5] += A[151]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[5] += A[152]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[5] += A[153]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[5] += A[154]*acadoWorkspace.rk_dim30_bPerm[4];

acadoWorkspace.rk_dim30_bPerm[6] += A[180]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[6] += A[181]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[6] += A[182]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[6] += A[183]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[6] += A[184]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[6] += A[185]*acadoWorkspace.rk_dim30_bPerm[5];

acadoWorkspace.rk_dim30_bPerm[7] += A[210]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[7] += A[211]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[7] += A[212]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[7] += A[213]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[7] += A[214]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[7] += A[215]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[7] += A[216]*acadoWorkspace.rk_dim30_bPerm[6];

acadoWorkspace.rk_dim30_bPerm[8] += A[240]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[8] += A[241]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[8] += A[242]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[8] += A[243]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[8] += A[244]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[8] += A[245]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[8] += A[246]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[8] += A[247]*acadoWorkspace.rk_dim30_bPerm[7];

acadoWorkspace.rk_dim30_bPerm[9] += A[270]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[9] += A[271]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[9] += A[272]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[9] += A[273]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[9] += A[274]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[9] += A[275]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[9] += A[276]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[9] += A[277]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[9] += A[278]*acadoWorkspace.rk_dim30_bPerm[8];

acadoWorkspace.rk_dim30_bPerm[10] += A[300]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[10] += A[301]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[10] += A[302]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[10] += A[303]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[10] += A[304]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[10] += A[305]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[10] += A[306]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[10] += A[307]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[10] += A[308]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[10] += A[309]*acadoWorkspace.rk_dim30_bPerm[9];

acadoWorkspace.rk_dim30_bPerm[11] += A[330]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[11] += A[331]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[11] += A[332]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[11] += A[333]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[11] += A[334]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[11] += A[335]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[11] += A[336]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[11] += A[337]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[11] += A[338]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[11] += A[339]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[11] += A[340]*acadoWorkspace.rk_dim30_bPerm[10];

acadoWorkspace.rk_dim30_bPerm[12] += A[360]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[12] += A[361]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[12] += A[362]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[12] += A[363]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[12] += A[364]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[12] += A[365]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[12] += A[366]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[12] += A[367]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[12] += A[368]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[12] += A[369]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[12] += A[370]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[12] += A[371]*acadoWorkspace.rk_dim30_bPerm[11];

acadoWorkspace.rk_dim30_bPerm[13] += A[390]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[13] += A[391]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[13] += A[392]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[13] += A[393]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[13] += A[394]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[13] += A[395]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[13] += A[396]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[13] += A[397]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[13] += A[398]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[13] += A[399]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[13] += A[400]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[13] += A[401]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[13] += A[402]*acadoWorkspace.rk_dim30_bPerm[12];

acadoWorkspace.rk_dim30_bPerm[14] += A[420]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[14] += A[421]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[14] += A[422]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[14] += A[423]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[14] += A[424]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[14] += A[425]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[14] += A[426]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[14] += A[427]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[14] += A[428]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[14] += A[429]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[14] += A[430]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[14] += A[431]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[14] += A[432]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[14] += A[433]*acadoWorkspace.rk_dim30_bPerm[13];

acadoWorkspace.rk_dim30_bPerm[15] += A[450]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[15] += A[451]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[15] += A[452]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[15] += A[453]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[15] += A[454]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[15] += A[455]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[15] += A[456]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[15] += A[457]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[15] += A[458]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[15] += A[459]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[15] += A[460]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[15] += A[461]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[15] += A[462]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[15] += A[463]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[15] += A[464]*acadoWorkspace.rk_dim30_bPerm[14];

acadoWorkspace.rk_dim30_bPerm[16] += A[480]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[16] += A[481]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[16] += A[482]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[16] += A[483]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[16] += A[484]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[16] += A[485]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[16] += A[486]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[16] += A[487]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[16] += A[488]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[16] += A[489]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[16] += A[490]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[16] += A[491]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[16] += A[492]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[16] += A[493]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[16] += A[494]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[16] += A[495]*acadoWorkspace.rk_dim30_bPerm[15];

acadoWorkspace.rk_dim30_bPerm[17] += A[510]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[17] += A[511]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[17] += A[512]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[17] += A[513]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[17] += A[514]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[17] += A[515]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[17] += A[516]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[17] += A[517]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[17] += A[518]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[17] += A[519]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[17] += A[520]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[17] += A[521]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[17] += A[522]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[17] += A[523]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[17] += A[524]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[17] += A[525]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[17] += A[526]*acadoWorkspace.rk_dim30_bPerm[16];

acadoWorkspace.rk_dim30_bPerm[18] += A[540]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[18] += A[541]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[18] += A[542]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[18] += A[543]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[18] += A[544]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[18] += A[545]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[18] += A[546]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[18] += A[547]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[18] += A[548]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[18] += A[549]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[18] += A[550]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[18] += A[551]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[18] += A[552]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[18] += A[553]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[18] += A[554]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[18] += A[555]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[18] += A[556]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[18] += A[557]*acadoWorkspace.rk_dim30_bPerm[17];

acadoWorkspace.rk_dim30_bPerm[19] += A[570]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[19] += A[571]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[19] += A[572]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[19] += A[573]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[19] += A[574]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[19] += A[575]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[19] += A[576]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[19] += A[577]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[19] += A[578]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[19] += A[579]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[19] += A[580]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[19] += A[581]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[19] += A[582]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[19] += A[583]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[19] += A[584]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[19] += A[585]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[19] += A[586]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[19] += A[587]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[19] += A[588]*acadoWorkspace.rk_dim30_bPerm[18];

acadoWorkspace.rk_dim30_bPerm[20] += A[600]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[20] += A[601]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[20] += A[602]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[20] += A[603]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[20] += A[604]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[20] += A[605]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[20] += A[606]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[20] += A[607]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[20] += A[608]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[20] += A[609]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[20] += A[610]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[20] += A[611]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[20] += A[612]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[20] += A[613]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[20] += A[614]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[20] += A[615]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[20] += A[616]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[20] += A[617]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[20] += A[618]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[20] += A[619]*acadoWorkspace.rk_dim30_bPerm[19];

acadoWorkspace.rk_dim30_bPerm[21] += A[630]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[21] += A[631]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[21] += A[632]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[21] += A[633]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[21] += A[634]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[21] += A[635]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[21] += A[636]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[21] += A[637]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[21] += A[638]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[21] += A[639]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[21] += A[640]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[21] += A[641]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[21] += A[642]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[21] += A[643]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[21] += A[644]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[21] += A[645]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[21] += A[646]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[21] += A[647]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[21] += A[648]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[21] += A[649]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[21] += A[650]*acadoWorkspace.rk_dim30_bPerm[20];

acadoWorkspace.rk_dim30_bPerm[22] += A[660]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[22] += A[661]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[22] += A[662]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[22] += A[663]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[22] += A[664]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[22] += A[665]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[22] += A[666]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[22] += A[667]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[22] += A[668]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[22] += A[669]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[22] += A[670]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[22] += A[671]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[22] += A[672]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[22] += A[673]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[22] += A[674]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[22] += A[675]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[22] += A[676]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[22] += A[677]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[22] += A[678]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[22] += A[679]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[22] += A[680]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[22] += A[681]*acadoWorkspace.rk_dim30_bPerm[21];

acadoWorkspace.rk_dim30_bPerm[23] += A[690]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[23] += A[691]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[23] += A[692]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[23] += A[693]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[23] += A[694]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[23] += A[695]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[23] += A[696]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[23] += A[697]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[23] += A[698]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[23] += A[699]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[23] += A[700]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[23] += A[701]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[23] += A[702]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[23] += A[703]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[23] += A[704]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[23] += A[705]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[23] += A[706]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[23] += A[707]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[23] += A[708]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[23] += A[709]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[23] += A[710]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[23] += A[711]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[23] += A[712]*acadoWorkspace.rk_dim30_bPerm[22];

acadoWorkspace.rk_dim30_bPerm[24] += A[720]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[24] += A[721]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[24] += A[722]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[24] += A[723]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[24] += A[724]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[24] += A[725]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[24] += A[726]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[24] += A[727]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[24] += A[728]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[24] += A[729]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[24] += A[730]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[24] += A[731]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[24] += A[732]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[24] += A[733]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[24] += A[734]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[24] += A[735]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[24] += A[736]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[24] += A[737]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[24] += A[738]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[24] += A[739]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[24] += A[740]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[24] += A[741]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[24] += A[742]*acadoWorkspace.rk_dim30_bPerm[22];
acadoWorkspace.rk_dim30_bPerm[24] += A[743]*acadoWorkspace.rk_dim30_bPerm[23];

acadoWorkspace.rk_dim30_bPerm[25] += A[750]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[25] += A[751]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[25] += A[752]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[25] += A[753]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[25] += A[754]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[25] += A[755]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[25] += A[756]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[25] += A[757]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[25] += A[758]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[25] += A[759]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[25] += A[760]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[25] += A[761]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[25] += A[762]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[25] += A[763]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[25] += A[764]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[25] += A[765]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[25] += A[766]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[25] += A[767]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[25] += A[768]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[25] += A[769]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[25] += A[770]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[25] += A[771]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[25] += A[772]*acadoWorkspace.rk_dim30_bPerm[22];
acadoWorkspace.rk_dim30_bPerm[25] += A[773]*acadoWorkspace.rk_dim30_bPerm[23];
acadoWorkspace.rk_dim30_bPerm[25] += A[774]*acadoWorkspace.rk_dim30_bPerm[24];

acadoWorkspace.rk_dim30_bPerm[26] += A[780]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[26] += A[781]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[26] += A[782]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[26] += A[783]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[26] += A[784]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[26] += A[785]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[26] += A[786]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[26] += A[787]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[26] += A[788]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[26] += A[789]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[26] += A[790]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[26] += A[791]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[26] += A[792]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[26] += A[793]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[26] += A[794]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[26] += A[795]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[26] += A[796]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[26] += A[797]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[26] += A[798]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[26] += A[799]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[26] += A[800]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[26] += A[801]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[26] += A[802]*acadoWorkspace.rk_dim30_bPerm[22];
acadoWorkspace.rk_dim30_bPerm[26] += A[803]*acadoWorkspace.rk_dim30_bPerm[23];
acadoWorkspace.rk_dim30_bPerm[26] += A[804]*acadoWorkspace.rk_dim30_bPerm[24];
acadoWorkspace.rk_dim30_bPerm[26] += A[805]*acadoWorkspace.rk_dim30_bPerm[25];

acadoWorkspace.rk_dim30_bPerm[27] += A[810]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[27] += A[811]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[27] += A[812]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[27] += A[813]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[27] += A[814]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[27] += A[815]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[27] += A[816]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[27] += A[817]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[27] += A[818]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[27] += A[819]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[27] += A[820]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[27] += A[821]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[27] += A[822]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[27] += A[823]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[27] += A[824]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[27] += A[825]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[27] += A[826]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[27] += A[827]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[27] += A[828]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[27] += A[829]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[27] += A[830]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[27] += A[831]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[27] += A[832]*acadoWorkspace.rk_dim30_bPerm[22];
acadoWorkspace.rk_dim30_bPerm[27] += A[833]*acadoWorkspace.rk_dim30_bPerm[23];
acadoWorkspace.rk_dim30_bPerm[27] += A[834]*acadoWorkspace.rk_dim30_bPerm[24];
acadoWorkspace.rk_dim30_bPerm[27] += A[835]*acadoWorkspace.rk_dim30_bPerm[25];
acadoWorkspace.rk_dim30_bPerm[27] += A[836]*acadoWorkspace.rk_dim30_bPerm[26];

acadoWorkspace.rk_dim30_bPerm[28] += A[840]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[28] += A[841]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[28] += A[842]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[28] += A[843]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[28] += A[844]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[28] += A[845]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[28] += A[846]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[28] += A[847]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[28] += A[848]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[28] += A[849]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[28] += A[850]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[28] += A[851]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[28] += A[852]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[28] += A[853]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[28] += A[854]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[28] += A[855]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[28] += A[856]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[28] += A[857]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[28] += A[858]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[28] += A[859]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[28] += A[860]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[28] += A[861]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[28] += A[862]*acadoWorkspace.rk_dim30_bPerm[22];
acadoWorkspace.rk_dim30_bPerm[28] += A[863]*acadoWorkspace.rk_dim30_bPerm[23];
acadoWorkspace.rk_dim30_bPerm[28] += A[864]*acadoWorkspace.rk_dim30_bPerm[24];
acadoWorkspace.rk_dim30_bPerm[28] += A[865]*acadoWorkspace.rk_dim30_bPerm[25];
acadoWorkspace.rk_dim30_bPerm[28] += A[866]*acadoWorkspace.rk_dim30_bPerm[26];
acadoWorkspace.rk_dim30_bPerm[28] += A[867]*acadoWorkspace.rk_dim30_bPerm[27];

acadoWorkspace.rk_dim30_bPerm[29] += A[870]*acadoWorkspace.rk_dim30_bPerm[0];
acadoWorkspace.rk_dim30_bPerm[29] += A[871]*acadoWorkspace.rk_dim30_bPerm[1];
acadoWorkspace.rk_dim30_bPerm[29] += A[872]*acadoWorkspace.rk_dim30_bPerm[2];
acadoWorkspace.rk_dim30_bPerm[29] += A[873]*acadoWorkspace.rk_dim30_bPerm[3];
acadoWorkspace.rk_dim30_bPerm[29] += A[874]*acadoWorkspace.rk_dim30_bPerm[4];
acadoWorkspace.rk_dim30_bPerm[29] += A[875]*acadoWorkspace.rk_dim30_bPerm[5];
acadoWorkspace.rk_dim30_bPerm[29] += A[876]*acadoWorkspace.rk_dim30_bPerm[6];
acadoWorkspace.rk_dim30_bPerm[29] += A[877]*acadoWorkspace.rk_dim30_bPerm[7];
acadoWorkspace.rk_dim30_bPerm[29] += A[878]*acadoWorkspace.rk_dim30_bPerm[8];
acadoWorkspace.rk_dim30_bPerm[29] += A[879]*acadoWorkspace.rk_dim30_bPerm[9];
acadoWorkspace.rk_dim30_bPerm[29] += A[880]*acadoWorkspace.rk_dim30_bPerm[10];
acadoWorkspace.rk_dim30_bPerm[29] += A[881]*acadoWorkspace.rk_dim30_bPerm[11];
acadoWorkspace.rk_dim30_bPerm[29] += A[882]*acadoWorkspace.rk_dim30_bPerm[12];
acadoWorkspace.rk_dim30_bPerm[29] += A[883]*acadoWorkspace.rk_dim30_bPerm[13];
acadoWorkspace.rk_dim30_bPerm[29] += A[884]*acadoWorkspace.rk_dim30_bPerm[14];
acadoWorkspace.rk_dim30_bPerm[29] += A[885]*acadoWorkspace.rk_dim30_bPerm[15];
acadoWorkspace.rk_dim30_bPerm[29] += A[886]*acadoWorkspace.rk_dim30_bPerm[16];
acadoWorkspace.rk_dim30_bPerm[29] += A[887]*acadoWorkspace.rk_dim30_bPerm[17];
acadoWorkspace.rk_dim30_bPerm[29] += A[888]*acadoWorkspace.rk_dim30_bPerm[18];
acadoWorkspace.rk_dim30_bPerm[29] += A[889]*acadoWorkspace.rk_dim30_bPerm[19];
acadoWorkspace.rk_dim30_bPerm[29] += A[890]*acadoWorkspace.rk_dim30_bPerm[20];
acadoWorkspace.rk_dim30_bPerm[29] += A[891]*acadoWorkspace.rk_dim30_bPerm[21];
acadoWorkspace.rk_dim30_bPerm[29] += A[892]*acadoWorkspace.rk_dim30_bPerm[22];
acadoWorkspace.rk_dim30_bPerm[29] += A[893]*acadoWorkspace.rk_dim30_bPerm[23];
acadoWorkspace.rk_dim30_bPerm[29] += A[894]*acadoWorkspace.rk_dim30_bPerm[24];
acadoWorkspace.rk_dim30_bPerm[29] += A[895]*acadoWorkspace.rk_dim30_bPerm[25];
acadoWorkspace.rk_dim30_bPerm[29] += A[896]*acadoWorkspace.rk_dim30_bPerm[26];
acadoWorkspace.rk_dim30_bPerm[29] += A[897]*acadoWorkspace.rk_dim30_bPerm[27];
acadoWorkspace.rk_dim30_bPerm[29] += A[898]*acadoWorkspace.rk_dim30_bPerm[28];


acado_solve_dim30_triangular( A, acadoWorkspace.rk_dim30_bPerm );
b[0] = acadoWorkspace.rk_dim30_bPerm[0];
b[1] = acadoWorkspace.rk_dim30_bPerm[1];
b[2] = acadoWorkspace.rk_dim30_bPerm[2];
b[3] = acadoWorkspace.rk_dim30_bPerm[3];
b[4] = acadoWorkspace.rk_dim30_bPerm[4];
b[5] = acadoWorkspace.rk_dim30_bPerm[5];
b[6] = acadoWorkspace.rk_dim30_bPerm[6];
b[7] = acadoWorkspace.rk_dim30_bPerm[7];
b[8] = acadoWorkspace.rk_dim30_bPerm[8];
b[9] = acadoWorkspace.rk_dim30_bPerm[9];
b[10] = acadoWorkspace.rk_dim30_bPerm[10];
b[11] = acadoWorkspace.rk_dim30_bPerm[11];
b[12] = acadoWorkspace.rk_dim30_bPerm[12];
b[13] = acadoWorkspace.rk_dim30_bPerm[13];
b[14] = acadoWorkspace.rk_dim30_bPerm[14];
b[15] = acadoWorkspace.rk_dim30_bPerm[15];
b[16] = acadoWorkspace.rk_dim30_bPerm[16];
b[17] = acadoWorkspace.rk_dim30_bPerm[17];
b[18] = acadoWorkspace.rk_dim30_bPerm[18];
b[19] = acadoWorkspace.rk_dim30_bPerm[19];
b[20] = acadoWorkspace.rk_dim30_bPerm[20];
b[21] = acadoWorkspace.rk_dim30_bPerm[21];
b[22] = acadoWorkspace.rk_dim30_bPerm[22];
b[23] = acadoWorkspace.rk_dim30_bPerm[23];
b[24] = acadoWorkspace.rk_dim30_bPerm[24];
b[25] = acadoWorkspace.rk_dim30_bPerm[25];
b[26] = acadoWorkspace.rk_dim30_bPerm[26];
b[27] = acadoWorkspace.rk_dim30_bPerm[27];
b[28] = acadoWorkspace.rk_dim30_bPerm[28];
b[29] = acadoWorkspace.rk_dim30_bPerm[29];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 6.2500000000000003e-03, 1.3466878364870323e-02, 
-9.6687836487032168e-04, 6.2500000000000003e-03 };


/* Fixed step size:0.025 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[15] = rk_eta[285];
acadoWorkspace.rk_xxx[16] = rk_eta[286];
acadoWorkspace.rk_xxx[17] = rk_eta[287];

for (run = 0; run < 4; ++run)
{
if( run > 0 ) {
for (i = 0; i < 15; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 18] = rk_eta[i * 15 + 15];
acadoWorkspace.rk_diffsPrev2[i * 18 + 1] = rk_eta[i * 15 + 16];
acadoWorkspace.rk_diffsPrev2[i * 18 + 2] = rk_eta[i * 15 + 17];
acadoWorkspace.rk_diffsPrev2[i * 18 + 3] = rk_eta[i * 15 + 18];
acadoWorkspace.rk_diffsPrev2[i * 18 + 4] = rk_eta[i * 15 + 19];
acadoWorkspace.rk_diffsPrev2[i * 18 + 5] = rk_eta[i * 15 + 20];
acadoWorkspace.rk_diffsPrev2[i * 18 + 6] = rk_eta[i * 15 + 21];
acadoWorkspace.rk_diffsPrev2[i * 18 + 7] = rk_eta[i * 15 + 22];
acadoWorkspace.rk_diffsPrev2[i * 18 + 8] = rk_eta[i * 15 + 23];
acadoWorkspace.rk_diffsPrev2[i * 18 + 9] = rk_eta[i * 15 + 24];
acadoWorkspace.rk_diffsPrev2[i * 18 + 10] = rk_eta[i * 15 + 25];
acadoWorkspace.rk_diffsPrev2[i * 18 + 11] = rk_eta[i * 15 + 26];
acadoWorkspace.rk_diffsPrev2[i * 18 + 12] = rk_eta[i * 15 + 27];
acadoWorkspace.rk_diffsPrev2[i * 18 + 13] = rk_eta[i * 15 + 28];
acadoWorkspace.rk_diffsPrev2[i * 18 + 14] = rk_eta[i * 15 + 29];
acadoWorkspace.rk_diffsPrev2[i * 18 + 15] = rk_eta[i * 3 + 240];
acadoWorkspace.rk_diffsPrev2[i * 18 + 16] = rk_eta[i * 3 + 241];
acadoWorkspace.rk_diffsPrev2[i * 18 + 17] = rk_eta[i * 3 + 242];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 15; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 270 ]) );
for (j = 0; j < 15; ++j)
{
tmp_index1 = (run1 * 15) + (j);
acadoWorkspace.rk_A[tmp_index1 * 30] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 7] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 9] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 10] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 11] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 12] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 13] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 13)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 14] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 14)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 30) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 30 + 15] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 16] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 17] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 18] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 19] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 20] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 21] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 22] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 23] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 24] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 25] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 26] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 27] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 28] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 13)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 29] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 14)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 30) + (j + 15)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 15] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 15 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 15 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 15 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 15 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 15 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 15 + 6] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 15 + 7] = acadoWorkspace.rk_kkk[run1 + 14] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 15 + 8] = acadoWorkspace.rk_kkk[run1 + 16] - acadoWorkspace.rk_rhsTemp[8];
acadoWorkspace.rk_b[run1 * 15 + 9] = acadoWorkspace.rk_kkk[run1 + 18] - acadoWorkspace.rk_rhsTemp[9];
acadoWorkspace.rk_b[run1 * 15 + 10] = acadoWorkspace.rk_kkk[run1 + 20] - acadoWorkspace.rk_rhsTemp[10];
acadoWorkspace.rk_b[run1 * 15 + 11] = acadoWorkspace.rk_kkk[run1 + 22] - acadoWorkspace.rk_rhsTemp[11];
acadoWorkspace.rk_b[run1 * 15 + 12] = acadoWorkspace.rk_kkk[run1 + 24] - acadoWorkspace.rk_rhsTemp[12];
acadoWorkspace.rk_b[run1 * 15 + 13] = acadoWorkspace.rk_kkk[run1 + 26] - acadoWorkspace.rk_rhsTemp[13];
acadoWorkspace.rk_b[run1 * 15 + 14] = acadoWorkspace.rk_kkk[run1 + 28] - acadoWorkspace.rk_rhsTemp[14];
}
det = acado_solve_dim30_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim30_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 15];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 15 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 15 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 15 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 15 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 15 + 5];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 15 + 6];
acadoWorkspace.rk_kkk[j + 14] += acadoWorkspace.rk_b[j * 15 + 7];
acadoWorkspace.rk_kkk[j + 16] += acadoWorkspace.rk_b[j * 15 + 8];
acadoWorkspace.rk_kkk[j + 18] += acadoWorkspace.rk_b[j * 15 + 9];
acadoWorkspace.rk_kkk[j + 20] += acadoWorkspace.rk_b[j * 15 + 10];
acadoWorkspace.rk_kkk[j + 22] += acadoWorkspace.rk_b[j * 15 + 11];
acadoWorkspace.rk_kkk[j + 24] += acadoWorkspace.rk_b[j * 15 + 12];
acadoWorkspace.rk_kkk[j + 26] += acadoWorkspace.rk_b[j * 15 + 13];
acadoWorkspace.rk_kkk[j + 28] += acadoWorkspace.rk_b[j * 15 + 14];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 15; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 15] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 15 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 15 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 15 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 15 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 15 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 15 + 6] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 15 + 7] = acadoWorkspace.rk_kkk[run1 + 14] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 15 + 8] = acadoWorkspace.rk_kkk[run1 + 16] - acadoWorkspace.rk_rhsTemp[8];
acadoWorkspace.rk_b[run1 * 15 + 9] = acadoWorkspace.rk_kkk[run1 + 18] - acadoWorkspace.rk_rhsTemp[9];
acadoWorkspace.rk_b[run1 * 15 + 10] = acadoWorkspace.rk_kkk[run1 + 20] - acadoWorkspace.rk_rhsTemp[10];
acadoWorkspace.rk_b[run1 * 15 + 11] = acadoWorkspace.rk_kkk[run1 + 22] - acadoWorkspace.rk_rhsTemp[11];
acadoWorkspace.rk_b[run1 * 15 + 12] = acadoWorkspace.rk_kkk[run1 + 24] - acadoWorkspace.rk_rhsTemp[12];
acadoWorkspace.rk_b[run1 * 15 + 13] = acadoWorkspace.rk_kkk[run1 + 26] - acadoWorkspace.rk_rhsTemp[13];
acadoWorkspace.rk_b[run1 * 15 + 14] = acadoWorkspace.rk_kkk[run1 + 28] - acadoWorkspace.rk_rhsTemp[14];
}
acado_solve_dim30_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim30_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 15];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 15 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 15 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 15 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 15 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 15 + 5];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 15 + 6];
acadoWorkspace.rk_kkk[j + 14] += acadoWorkspace.rk_b[j * 15 + 7];
acadoWorkspace.rk_kkk[j + 16] += acadoWorkspace.rk_b[j * 15 + 8];
acadoWorkspace.rk_kkk[j + 18] += acadoWorkspace.rk_b[j * 15 + 9];
acadoWorkspace.rk_kkk[j + 20] += acadoWorkspace.rk_b[j * 15 + 10];
acadoWorkspace.rk_kkk[j + 22] += acadoWorkspace.rk_b[j * 15 + 11];
acadoWorkspace.rk_kkk[j + 24] += acadoWorkspace.rk_b[j * 15 + 12];
acadoWorkspace.rk_kkk[j + 26] += acadoWorkspace.rk_b[j * 15 + 13];
acadoWorkspace.rk_kkk[j + 28] += acadoWorkspace.rk_b[j * 15 + 14];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 15; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 270 ]) );
for (j = 0; j < 15; ++j)
{
tmp_index1 = (run1 * 15) + (j);
acadoWorkspace.rk_A[tmp_index1 * 30] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 7] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 9] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 10] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 11] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 12] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 13] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 13)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 14] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 14)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 30) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 30 + 15] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 16] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 17] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 18] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 19] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 20] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 21] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 22] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 23] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 24] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 25] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 26] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 27] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 28] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 13)];
acadoWorkspace.rk_A[tmp_index1 * 30 + 29] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 270) + (j * 18 + 14)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 30) + (j + 15)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 15; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 15] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1)];
acadoWorkspace.rk_b[i * 15 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 18)];
acadoWorkspace.rk_b[i * 15 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 36)];
acadoWorkspace.rk_b[i * 15 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 54)];
acadoWorkspace.rk_b[i * 15 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 72)];
acadoWorkspace.rk_b[i * 15 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 90)];
acadoWorkspace.rk_b[i * 15 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 108)];
acadoWorkspace.rk_b[i * 15 + 7] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 126)];
acadoWorkspace.rk_b[i * 15 + 8] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 144)];
acadoWorkspace.rk_b[i * 15 + 9] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 162)];
acadoWorkspace.rk_b[i * 15 + 10] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 180)];
acadoWorkspace.rk_b[i * 15 + 11] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 198)];
acadoWorkspace.rk_b[i * 15 + 12] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 216)];
acadoWorkspace.rk_b[i * 15 + 13] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 234)];
acadoWorkspace.rk_b[i * 15 + 14] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (run1 + 252)];
}
if( 0 == run1 ) {
det = acado_solve_dim30_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim30_perm );
}
 else {
acado_solve_dim30_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim30_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 15];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 15 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 15 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 15 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 15 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 15 + 5];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 15 + 6];
acadoWorkspace.rk_diffK[i + 14] = acadoWorkspace.rk_b[i * 15 + 7];
acadoWorkspace.rk_diffK[i + 16] = acadoWorkspace.rk_b[i * 15 + 8];
acadoWorkspace.rk_diffK[i + 18] = acadoWorkspace.rk_b[i * 15 + 9];
acadoWorkspace.rk_diffK[i + 20] = acadoWorkspace.rk_b[i * 15 + 10];
acadoWorkspace.rk_diffK[i + 22] = acadoWorkspace.rk_b[i * 15 + 11];
acadoWorkspace.rk_diffK[i + 24] = acadoWorkspace.rk_b[i * 15 + 12];
acadoWorkspace.rk_diffK[i + 26] = acadoWorkspace.rk_b[i * 15 + 13];
acadoWorkspace.rk_diffK[i + 28] = acadoWorkspace.rk_b[i * 15 + 14];
}
for (i = 0; i < 15; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 18) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 18) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 15; ++j)
{
tmp_index1 = (i * 15) + (j);
tmp_index2 = (run1) + (j * 18);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 270) + (tmp_index2 + 15)];
}
}
acado_solve_dim30_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim30_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 15];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 15 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 15 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 15 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 15 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 15 + 5];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 15 + 6];
acadoWorkspace.rk_diffK[i + 14] = acadoWorkspace.rk_b[i * 15 + 7];
acadoWorkspace.rk_diffK[i + 16] = acadoWorkspace.rk_b[i * 15 + 8];
acadoWorkspace.rk_diffK[i + 18] = acadoWorkspace.rk_b[i * 15 + 9];
acadoWorkspace.rk_diffK[i + 20] = acadoWorkspace.rk_b[i * 15 + 10];
acadoWorkspace.rk_diffK[i + 22] = acadoWorkspace.rk_b[i * 15 + 11];
acadoWorkspace.rk_diffK[i + 24] = acadoWorkspace.rk_b[i * 15 + 12];
acadoWorkspace.rk_diffK[i + 26] = acadoWorkspace.rk_b[i * 15 + 13];
acadoWorkspace.rk_diffK[i + 28] = acadoWorkspace.rk_b[i * 15 + 14];
}
for (i = 0; i < 15; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 18) + (run1 + 15)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[1]*(real_t)1.2500000000000001e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[2]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[3]*(real_t)1.2500000000000001e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[4]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[5]*(real_t)1.2500000000000001e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[6]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[7]*(real_t)1.2500000000000001e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[8]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[9]*(real_t)1.2500000000000001e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[10]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[11]*(real_t)1.2500000000000001e-02;
rk_eta[6] += + acadoWorkspace.rk_kkk[12]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[13]*(real_t)1.2500000000000001e-02;
rk_eta[7] += + acadoWorkspace.rk_kkk[14]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[15]*(real_t)1.2500000000000001e-02;
rk_eta[8] += + acadoWorkspace.rk_kkk[16]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[17]*(real_t)1.2500000000000001e-02;
rk_eta[9] += + acadoWorkspace.rk_kkk[18]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[19]*(real_t)1.2500000000000001e-02;
rk_eta[10] += + acadoWorkspace.rk_kkk[20]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[21]*(real_t)1.2500000000000001e-02;
rk_eta[11] += + acadoWorkspace.rk_kkk[22]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[23]*(real_t)1.2500000000000001e-02;
rk_eta[12] += + acadoWorkspace.rk_kkk[24]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[25]*(real_t)1.2500000000000001e-02;
rk_eta[13] += + acadoWorkspace.rk_kkk[26]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[27]*(real_t)1.2500000000000001e-02;
rk_eta[14] += + acadoWorkspace.rk_kkk[28]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_kkk[29]*(real_t)1.2500000000000001e-02;
if( run == 0 ) {
for (i = 0; i < 15; ++i)
{
for (j = 0; j < 15; ++j)
{
tmp_index2 = (j) + (i * 15);
rk_eta[tmp_index2 + 15] = acadoWorkspace.rk_diffsNew2[(i * 18) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 240] = acadoWorkspace.rk_diffsNew2[(i * 18) + (j + 15)];
}
}
}
else {
for (i = 0; i < 15; ++i)
{
for (j = 0; j < 15; ++j)
{
tmp_index2 = (j) + (i * 15);
rk_eta[tmp_index2 + 15] = + acadoWorkspace.rk_diffsNew2[i * 18]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 1]*acadoWorkspace.rk_diffsPrev2[j + 18];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 2]*acadoWorkspace.rk_diffsPrev2[j + 36];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 3]*acadoWorkspace.rk_diffsPrev2[j + 54];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 4]*acadoWorkspace.rk_diffsPrev2[j + 72];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 5]*acadoWorkspace.rk_diffsPrev2[j + 90];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 6]*acadoWorkspace.rk_diffsPrev2[j + 108];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 7]*acadoWorkspace.rk_diffsPrev2[j + 126];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 8]*acadoWorkspace.rk_diffsPrev2[j + 144];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 9]*acadoWorkspace.rk_diffsPrev2[j + 162];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 10]*acadoWorkspace.rk_diffsPrev2[j + 180];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 11]*acadoWorkspace.rk_diffsPrev2[j + 198];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 12]*acadoWorkspace.rk_diffsPrev2[j + 216];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 13]*acadoWorkspace.rk_diffsPrev2[j + 234];
rk_eta[tmp_index2 + 15] += + acadoWorkspace.rk_diffsNew2[i * 18 + 14]*acadoWorkspace.rk_diffsPrev2[j + 252];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 240] = acadoWorkspace.rk_diffsNew2[(i * 18) + (j + 15)];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18]*acadoWorkspace.rk_diffsPrev2[j + 15];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 1]*acadoWorkspace.rk_diffsPrev2[j + 33];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 2]*acadoWorkspace.rk_diffsPrev2[j + 51];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 3]*acadoWorkspace.rk_diffsPrev2[j + 69];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 4]*acadoWorkspace.rk_diffsPrev2[j + 87];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 5]*acadoWorkspace.rk_diffsPrev2[j + 105];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 6]*acadoWorkspace.rk_diffsPrev2[j + 123];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 7]*acadoWorkspace.rk_diffsPrev2[j + 141];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 8]*acadoWorkspace.rk_diffsPrev2[j + 159];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 9]*acadoWorkspace.rk_diffsPrev2[j + 177];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 10]*acadoWorkspace.rk_diffsPrev2[j + 195];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 11]*acadoWorkspace.rk_diffsPrev2[j + 213];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 12]*acadoWorkspace.rk_diffsPrev2[j + 231];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 13]*acadoWorkspace.rk_diffsPrev2[j + 249];
rk_eta[tmp_index2 + 240] += + acadoWorkspace.rk_diffsNew2[i * 18 + 14]*acadoWorkspace.rk_diffsPrev2[j + 267];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 2.5000000000000000e-01;
}
for (i = 0; i < 15; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



