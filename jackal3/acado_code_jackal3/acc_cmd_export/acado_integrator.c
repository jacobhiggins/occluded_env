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
const real_t* u = in + 14;
/* Vector of auxiliary variables; number of elements: 8. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((xd[0])*(xd[0]));
a[1] = (pow((xd[1]/xd[0]),2));
a[2] = (atan((xd[1]/xd[0])));
a[3] = ((xd[1])*(xd[1]));
a[4] = (pow((xd[0]-xd[6]),2));
a[5] = (pow(((xd[1]-xd[7])/(xd[0]-xd[6])),2));
a[6] = (atan(((xd[1]-xd[7])/(xd[0]-xd[6]))));
a[7] = (pow((xd[1]-xd[7]),2));

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = u[0];
out[3] = u[1];
out[4] = (((((xd[3]/xd[0])-((xd[1]*xd[2])/a[0]))/((real_t)(1.0000000000000000e+00)+a[1]))/xd[1])-((a[2]*xd[3])/a[3]));
out[5] = (((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[4]))/((real_t)(1.0000000000000000e+00)+a[5]))/(xd[1]-xd[7]))-((xd[3]*a[6])/a[7]));
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = u[2];
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
a[26] = ((real_t)(1.0000000000000000e+00)/(xd[0]-xd[6]));
a[27] = (a[26]*a[26]);
a[28] = ((real_t)(2.0000000000000000e+00)*(xd[0]-xd[6]));
a[29] = (pow((xd[0]-xd[6]),2));
a[30] = ((real_t)(1.0000000000000000e+00)/a[29]);
a[31] = (a[30]*a[30]);
a[32] = (pow(((xd[1]-xd[7])/(xd[0]-xd[6])),2));
a[33] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[32]));
a[34] = ((real_t)(2.0000000000000000e+00)*((xd[1]-xd[7])/(xd[0]-xd[6])));
a[35] = ((real_t)(1.0000000000000000e+00)/(xd[0]-xd[6]));
a[36] = (a[35]*a[35]);
a[37] = (a[34]*((real_t)(0.0000000000000000e+00)-((xd[1]-xd[7])*a[36])));
a[38] = (a[33]*a[33]);
a[39] = ((real_t)(1.0000000000000000e+00)/(xd[1]-xd[7]));
a[40] = ((real_t)(1.0000000000000000e+00)/(xd[0]-xd[6]));
a[41] = (a[40]*a[40]);
a[42] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((xd[1]-xd[7])/(xd[0]-xd[6])),2))));
a[43] = (((real_t)(0.0000000000000000e+00)-((xd[1]-xd[7])*a[41]))*a[42]);
a[44] = (pow((xd[1]-xd[7]),2));
a[45] = ((real_t)(1.0000000000000000e+00)/a[44]);
a[46] = (a[34]*a[35]);
a[47] = (a[39]*a[39]);
a[48] = (a[40]*a[42]);
a[49] = (atan(((xd[1]-xd[7])/(xd[0]-xd[6]))));
a[50] = ((real_t)(2.0000000000000000e+00)*(xd[1]-xd[7]));
a[51] = (a[45]*a[45]);
a[52] = (a[28]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)));
a[53] = (a[34]*((real_t)(0.0000000000000000e+00)-(((xd[1]-xd[7])*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[36])));
a[54] = (((real_t)(0.0000000000000000e+00)-(((xd[1]-xd[7])*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[41]))*a[42]);
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
out[20] = (real_t)(1.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
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
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(1.0000000000000000e+00);
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
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(1.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (((((((real_t)(0.0000000000000000e+00)-(xd[3]*a[1]))-((real_t)(0.0000000000000000e+00)-(((xd[1]*xd[2])*a[2])*a[5])))*a[7])-((((xd[3]/xd[0])-((xd[1]*xd[2])/a[3]))*a[11])*a[12]))*a[13])-((a[17]*xd[3])*a[19]));
out[69] = (((((((real_t)(0.0000000000000000e+00)-(xd[2]*a[4]))*a[7])-((((xd[3]/xd[0])-((xd[1]*xd[2])/a[3]))*a[20])*a[12]))*a[13])-((((xd[3]/xd[0])-((xd[1]*xd[2])/a[3]))/((real_t)(1.0000000000000000e+00)+a[6]))*a[21]))-(((a[22]*xd[3])*a[19])-(((a[23]*xd[3])*a[24])*a[25])));
out[70] = ((((real_t)(0.0000000000000000e+00)-(xd[1]*a[4]))*a[7])*a[13]);
out[71] = (((a[0]*a[7])*a[13])-(a[23]*a[19]));
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
out[85] = (((((((real_t)(0.0000000000000000e+00)-(xd[3]*a[27]))-((real_t)(0.0000000000000000e+00)-(((xd[2]*(xd[1]-xd[7]))*a[28])*a[31])))*a[33])-((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[29]))*a[37])*a[38]))*a[39])-((xd[3]*a[43])*a[45]));
out[86] = (((((((real_t)(0.0000000000000000e+00)-(xd[2]*a[30]))*a[33])-((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[29]))*a[46])*a[38]))*a[39])-((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[29]))/((real_t)(1.0000000000000000e+00)+a[32]))*a[47]))-(((xd[3]*a[48])*a[45])-(((xd[3]*a[49])*a[50])*a[51])));
out[87] = ((((real_t)(0.0000000000000000e+00)-((xd[1]-xd[7])*a[30]))*a[33])*a[39]);
out[88] = (((a[26]*a[33])*a[39])-(a[49]*a[45]));
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (((((((real_t)(0.0000000000000000e+00)-((xd[3]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[27]))-((real_t)(0.0000000000000000e+00)-(((xd[2]*(xd[1]-xd[7]))*a[52])*a[31])))*a[33])-((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[29]))*a[53])*a[38]))*a[39])-((xd[3]*a[54])*a[45]));
out[92] = (((((((real_t)(0.0000000000000000e+00)-((xd[2]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[30]))*a[33])-((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[29]))*a[55])*a[38]))*a[39])-(((((xd[3]/(xd[0]-xd[6]))-((xd[2]*(xd[1]-xd[7]))/a[29]))/((real_t)(1.0000000000000000e+00)+a[32]))*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)))*a[47]))-(((xd[3]*a[56])*a[45])-(((xd[3]*a[49])*a[57])*a[51])));
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
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
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
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
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
out[237] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim28_triangular( real_t* const A, real_t* const b )
{

b[27] = b[27]/A[783];
b[26] -= + A[755]*b[27];
b[26] = b[26]/A[754];
b[25] -= + A[727]*b[27];
b[25] -= + A[726]*b[26];
b[25] = b[25]/A[725];
b[24] -= + A[699]*b[27];
b[24] -= + A[698]*b[26];
b[24] -= + A[697]*b[25];
b[24] = b[24]/A[696];
b[23] -= + A[671]*b[27];
b[23] -= + A[670]*b[26];
b[23] -= + A[669]*b[25];
b[23] -= + A[668]*b[24];
b[23] = b[23]/A[667];
b[22] -= + A[643]*b[27];
b[22] -= + A[642]*b[26];
b[22] -= + A[641]*b[25];
b[22] -= + A[640]*b[24];
b[22] -= + A[639]*b[23];
b[22] = b[22]/A[638];
b[21] -= + A[615]*b[27];
b[21] -= + A[614]*b[26];
b[21] -= + A[613]*b[25];
b[21] -= + A[612]*b[24];
b[21] -= + A[611]*b[23];
b[21] -= + A[610]*b[22];
b[21] = b[21]/A[609];
b[20] -= + A[587]*b[27];
b[20] -= + A[586]*b[26];
b[20] -= + A[585]*b[25];
b[20] -= + A[584]*b[24];
b[20] -= + A[583]*b[23];
b[20] -= + A[582]*b[22];
b[20] -= + A[581]*b[21];
b[20] = b[20]/A[580];
b[19] -= + A[559]*b[27];
b[19] -= + A[558]*b[26];
b[19] -= + A[557]*b[25];
b[19] -= + A[556]*b[24];
b[19] -= + A[555]*b[23];
b[19] -= + A[554]*b[22];
b[19] -= + A[553]*b[21];
b[19] -= + A[552]*b[20];
b[19] = b[19]/A[551];
b[18] -= + A[531]*b[27];
b[18] -= + A[530]*b[26];
b[18] -= + A[529]*b[25];
b[18] -= + A[528]*b[24];
b[18] -= + A[527]*b[23];
b[18] -= + A[526]*b[22];
b[18] -= + A[525]*b[21];
b[18] -= + A[524]*b[20];
b[18] -= + A[523]*b[19];
b[18] = b[18]/A[522];
b[17] -= + A[503]*b[27];
b[17] -= + A[502]*b[26];
b[17] -= + A[501]*b[25];
b[17] -= + A[500]*b[24];
b[17] -= + A[499]*b[23];
b[17] -= + A[498]*b[22];
b[17] -= + A[497]*b[21];
b[17] -= + A[496]*b[20];
b[17] -= + A[495]*b[19];
b[17] -= + A[494]*b[18];
b[17] = b[17]/A[493];
b[16] -= + A[475]*b[27];
b[16] -= + A[474]*b[26];
b[16] -= + A[473]*b[25];
b[16] -= + A[472]*b[24];
b[16] -= + A[471]*b[23];
b[16] -= + A[470]*b[22];
b[16] -= + A[469]*b[21];
b[16] -= + A[468]*b[20];
b[16] -= + A[467]*b[19];
b[16] -= + A[466]*b[18];
b[16] -= + A[465]*b[17];
b[16] = b[16]/A[464];
b[15] -= + A[447]*b[27];
b[15] -= + A[446]*b[26];
b[15] -= + A[445]*b[25];
b[15] -= + A[444]*b[24];
b[15] -= + A[443]*b[23];
b[15] -= + A[442]*b[22];
b[15] -= + A[441]*b[21];
b[15] -= + A[440]*b[20];
b[15] -= + A[439]*b[19];
b[15] -= + A[438]*b[18];
b[15] -= + A[437]*b[17];
b[15] -= + A[436]*b[16];
b[15] = b[15]/A[435];
b[14] -= + A[419]*b[27];
b[14] -= + A[418]*b[26];
b[14] -= + A[417]*b[25];
b[14] -= + A[416]*b[24];
b[14] -= + A[415]*b[23];
b[14] -= + A[414]*b[22];
b[14] -= + A[413]*b[21];
b[14] -= + A[412]*b[20];
b[14] -= + A[411]*b[19];
b[14] -= + A[410]*b[18];
b[14] -= + A[409]*b[17];
b[14] -= + A[408]*b[16];
b[14] -= + A[407]*b[15];
b[14] = b[14]/A[406];
b[13] -= + A[391]*b[27];
b[13] -= + A[390]*b[26];
b[13] -= + A[389]*b[25];
b[13] -= + A[388]*b[24];
b[13] -= + A[387]*b[23];
b[13] -= + A[386]*b[22];
b[13] -= + A[385]*b[21];
b[13] -= + A[384]*b[20];
b[13] -= + A[383]*b[19];
b[13] -= + A[382]*b[18];
b[13] -= + A[381]*b[17];
b[13] -= + A[380]*b[16];
b[13] -= + A[379]*b[15];
b[13] -= + A[378]*b[14];
b[13] = b[13]/A[377];
b[12] -= + A[363]*b[27];
b[12] -= + A[362]*b[26];
b[12] -= + A[361]*b[25];
b[12] -= + A[360]*b[24];
b[12] -= + A[359]*b[23];
b[12] -= + A[358]*b[22];
b[12] -= + A[357]*b[21];
b[12] -= + A[356]*b[20];
b[12] -= + A[355]*b[19];
b[12] -= + A[354]*b[18];
b[12] -= + A[353]*b[17];
b[12] -= + A[352]*b[16];
b[12] -= + A[351]*b[15];
b[12] -= + A[350]*b[14];
b[12] -= + A[349]*b[13];
b[12] = b[12]/A[348];
b[11] -= + A[335]*b[27];
b[11] -= + A[334]*b[26];
b[11] -= + A[333]*b[25];
b[11] -= + A[332]*b[24];
b[11] -= + A[331]*b[23];
b[11] -= + A[330]*b[22];
b[11] -= + A[329]*b[21];
b[11] -= + A[328]*b[20];
b[11] -= + A[327]*b[19];
b[11] -= + A[326]*b[18];
b[11] -= + A[325]*b[17];
b[11] -= + A[324]*b[16];
b[11] -= + A[323]*b[15];
b[11] -= + A[322]*b[14];
b[11] -= + A[321]*b[13];
b[11] -= + A[320]*b[12];
b[11] = b[11]/A[319];
b[10] -= + A[307]*b[27];
b[10] -= + A[306]*b[26];
b[10] -= + A[305]*b[25];
b[10] -= + A[304]*b[24];
b[10] -= + A[303]*b[23];
b[10] -= + A[302]*b[22];
b[10] -= + A[301]*b[21];
b[10] -= + A[300]*b[20];
b[10] -= + A[299]*b[19];
b[10] -= + A[298]*b[18];
b[10] -= + A[297]*b[17];
b[10] -= + A[296]*b[16];
b[10] -= + A[295]*b[15];
b[10] -= + A[294]*b[14];
b[10] -= + A[293]*b[13];
b[10] -= + A[292]*b[12];
b[10] -= + A[291]*b[11];
b[10] = b[10]/A[290];
b[9] -= + A[279]*b[27];
b[9] -= + A[278]*b[26];
b[9] -= + A[277]*b[25];
b[9] -= + A[276]*b[24];
b[9] -= + A[275]*b[23];
b[9] -= + A[274]*b[22];
b[9] -= + A[273]*b[21];
b[9] -= + A[272]*b[20];
b[9] -= + A[271]*b[19];
b[9] -= + A[270]*b[18];
b[9] -= + A[269]*b[17];
b[9] -= + A[268]*b[16];
b[9] -= + A[267]*b[15];
b[9] -= + A[266]*b[14];
b[9] -= + A[265]*b[13];
b[9] -= + A[264]*b[12];
b[9] -= + A[263]*b[11];
b[9] -= + A[262]*b[10];
b[9] = b[9]/A[261];
b[8] -= + A[251]*b[27];
b[8] -= + A[250]*b[26];
b[8] -= + A[249]*b[25];
b[8] -= + A[248]*b[24];
b[8] -= + A[247]*b[23];
b[8] -= + A[246]*b[22];
b[8] -= + A[245]*b[21];
b[8] -= + A[244]*b[20];
b[8] -= + A[243]*b[19];
b[8] -= + A[242]*b[18];
b[8] -= + A[241]*b[17];
b[8] -= + A[240]*b[16];
b[8] -= + A[239]*b[15];
b[8] -= + A[238]*b[14];
b[8] -= + A[237]*b[13];
b[8] -= + A[236]*b[12];
b[8] -= + A[235]*b[11];
b[8] -= + A[234]*b[10];
b[8] -= + A[233]*b[9];
b[8] = b[8]/A[232];
b[7] -= + A[223]*b[27];
b[7] -= + A[222]*b[26];
b[7] -= + A[221]*b[25];
b[7] -= + A[220]*b[24];
b[7] -= + A[219]*b[23];
b[7] -= + A[218]*b[22];
b[7] -= + A[217]*b[21];
b[7] -= + A[216]*b[20];
b[7] -= + A[215]*b[19];
b[7] -= + A[214]*b[18];
b[7] -= + A[213]*b[17];
b[7] -= + A[212]*b[16];
b[7] -= + A[211]*b[15];
b[7] -= + A[210]*b[14];
b[7] -= + A[209]*b[13];
b[7] -= + A[208]*b[12];
b[7] -= + A[207]*b[11];
b[7] -= + A[206]*b[10];
b[7] -= + A[205]*b[9];
b[7] -= + A[204]*b[8];
b[7] = b[7]/A[203];
b[6] -= + A[195]*b[27];
b[6] -= + A[194]*b[26];
b[6] -= + A[193]*b[25];
b[6] -= + A[192]*b[24];
b[6] -= + A[191]*b[23];
b[6] -= + A[190]*b[22];
b[6] -= + A[189]*b[21];
b[6] -= + A[188]*b[20];
b[6] -= + A[187]*b[19];
b[6] -= + A[186]*b[18];
b[6] -= + A[185]*b[17];
b[6] -= + A[184]*b[16];
b[6] -= + A[183]*b[15];
b[6] -= + A[182]*b[14];
b[6] -= + A[181]*b[13];
b[6] -= + A[180]*b[12];
b[6] -= + A[179]*b[11];
b[6] -= + A[178]*b[10];
b[6] -= + A[177]*b[9];
b[6] -= + A[176]*b[8];
b[6] -= + A[175]*b[7];
b[6] = b[6]/A[174];
b[5] -= + A[167]*b[27];
b[5] -= + A[166]*b[26];
b[5] -= + A[165]*b[25];
b[5] -= + A[164]*b[24];
b[5] -= + A[163]*b[23];
b[5] -= + A[162]*b[22];
b[5] -= + A[161]*b[21];
b[5] -= + A[160]*b[20];
b[5] -= + A[159]*b[19];
b[5] -= + A[158]*b[18];
b[5] -= + A[157]*b[17];
b[5] -= + A[156]*b[16];
b[5] -= + A[155]*b[15];
b[5] -= + A[154]*b[14];
b[5] -= + A[153]*b[13];
b[5] -= + A[152]*b[12];
b[5] -= + A[151]*b[11];
b[5] -= + A[150]*b[10];
b[5] -= + A[149]*b[9];
b[5] -= + A[148]*b[8];
b[5] -= + A[147]*b[7];
b[5] -= + A[146]*b[6];
b[5] = b[5]/A[145];
b[4] -= + A[139]*b[27];
b[4] -= + A[138]*b[26];
b[4] -= + A[137]*b[25];
b[4] -= + A[136]*b[24];
b[4] -= + A[135]*b[23];
b[4] -= + A[134]*b[22];
b[4] -= + A[133]*b[21];
b[4] -= + A[132]*b[20];
b[4] -= + A[131]*b[19];
b[4] -= + A[130]*b[18];
b[4] -= + A[129]*b[17];
b[4] -= + A[128]*b[16];
b[4] -= + A[127]*b[15];
b[4] -= + A[126]*b[14];
b[4] -= + A[125]*b[13];
b[4] -= + A[124]*b[12];
b[4] -= + A[123]*b[11];
b[4] -= + A[122]*b[10];
b[4] -= + A[121]*b[9];
b[4] -= + A[120]*b[8];
b[4] -= + A[119]*b[7];
b[4] -= + A[118]*b[6];
b[4] -= + A[117]*b[5];
b[4] = b[4]/A[116];
b[3] -= + A[111]*b[27];
b[3] -= + A[110]*b[26];
b[3] -= + A[109]*b[25];
b[3] -= + A[108]*b[24];
b[3] -= + A[107]*b[23];
b[3] -= + A[106]*b[22];
b[3] -= + A[105]*b[21];
b[3] -= + A[104]*b[20];
b[3] -= + A[103]*b[19];
b[3] -= + A[102]*b[18];
b[3] -= + A[101]*b[17];
b[3] -= + A[100]*b[16];
b[3] -= + A[99]*b[15];
b[3] -= + A[98]*b[14];
b[3] -= + A[97]*b[13];
b[3] -= + A[96]*b[12];
b[3] -= + A[95]*b[11];
b[3] -= + A[94]*b[10];
b[3] -= + A[93]*b[9];
b[3] -= + A[92]*b[8];
b[3] -= + A[91]*b[7];
b[3] -= + A[90]*b[6];
b[3] -= + A[89]*b[5];
b[3] -= + A[88]*b[4];
b[3] = b[3]/A[87];
b[2] -= + A[83]*b[27];
b[2] -= + A[82]*b[26];
b[2] -= + A[81]*b[25];
b[2] -= + A[80]*b[24];
b[2] -= + A[79]*b[23];
b[2] -= + A[78]*b[22];
b[2] -= + A[77]*b[21];
b[2] -= + A[76]*b[20];
b[2] -= + A[75]*b[19];
b[2] -= + A[74]*b[18];
b[2] -= + A[73]*b[17];
b[2] -= + A[72]*b[16];
b[2] -= + A[71]*b[15];
b[2] -= + A[70]*b[14];
b[2] -= + A[69]*b[13];
b[2] -= + A[68]*b[12];
b[2] -= + A[67]*b[11];
b[2] -= + A[66]*b[10];
b[2] -= + A[65]*b[9];
b[2] -= + A[64]*b[8];
b[2] -= + A[63]*b[7];
b[2] -= + A[62]*b[6];
b[2] -= + A[61]*b[5];
b[2] -= + A[60]*b[4];
b[2] -= + A[59]*b[3];
b[2] = b[2]/A[58];
b[1] -= + A[55]*b[27];
b[1] -= + A[54]*b[26];
b[1] -= + A[53]*b[25];
b[1] -= + A[52]*b[24];
b[1] -= + A[51]*b[23];
b[1] -= + A[50]*b[22];
b[1] -= + A[49]*b[21];
b[1] -= + A[48]*b[20];
b[1] -= + A[47]*b[19];
b[1] -= + A[46]*b[18];
b[1] -= + A[45]*b[17];
b[1] -= + A[44]*b[16];
b[1] -= + A[43]*b[15];
b[1] -= + A[42]*b[14];
b[1] -= + A[41]*b[13];
b[1] -= + A[40]*b[12];
b[1] -= + A[39]*b[11];
b[1] -= + A[38]*b[10];
b[1] -= + A[37]*b[9];
b[1] -= + A[36]*b[8];
b[1] -= + A[35]*b[7];
b[1] -= + A[34]*b[6];
b[1] -= + A[33]*b[5];
b[1] -= + A[32]*b[4];
b[1] -= + A[31]*b[3];
b[1] -= + A[30]*b[2];
b[1] = b[1]/A[29];
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

real_t acado_solve_dim28_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 28; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (27); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*28+i]);
	for( j=(i+1); j < 28; j++ ) {
		temp = fabs(A[j*28+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 28; ++k)
{
	acadoWorkspace.rk_dim28_swap = A[i*28+k];
	A[i*28+k] = A[indexMax*28+k];
	A[indexMax*28+k] = acadoWorkspace.rk_dim28_swap;
}
	acadoWorkspace.rk_dim28_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim28_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*28+i];
	for( j=i+1; j < 28; j++ ) {
		A[j*28+i] = -A[j*28+i]/A[i*28+i];
		for( k=i+1; k < 28; k++ ) {
			A[j*28+k] += A[j*28+i] * A[i*28+k];
		}
		b[j] += A[j*28+i] * b[i];
	}
}
det *= A[783];
det = fabs(det);
acado_solve_dim28_triangular( A, b );
return det;
}

void acado_solve_dim28_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim28_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim28_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim28_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim28_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim28_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim28_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim28_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim28_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim28_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim28_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim28_bPerm[10] = b[rk_perm[10]];
acadoWorkspace.rk_dim28_bPerm[11] = b[rk_perm[11]];
acadoWorkspace.rk_dim28_bPerm[12] = b[rk_perm[12]];
acadoWorkspace.rk_dim28_bPerm[13] = b[rk_perm[13]];
acadoWorkspace.rk_dim28_bPerm[14] = b[rk_perm[14]];
acadoWorkspace.rk_dim28_bPerm[15] = b[rk_perm[15]];
acadoWorkspace.rk_dim28_bPerm[16] = b[rk_perm[16]];
acadoWorkspace.rk_dim28_bPerm[17] = b[rk_perm[17]];
acadoWorkspace.rk_dim28_bPerm[18] = b[rk_perm[18]];
acadoWorkspace.rk_dim28_bPerm[19] = b[rk_perm[19]];
acadoWorkspace.rk_dim28_bPerm[20] = b[rk_perm[20]];
acadoWorkspace.rk_dim28_bPerm[21] = b[rk_perm[21]];
acadoWorkspace.rk_dim28_bPerm[22] = b[rk_perm[22]];
acadoWorkspace.rk_dim28_bPerm[23] = b[rk_perm[23]];
acadoWorkspace.rk_dim28_bPerm[24] = b[rk_perm[24]];
acadoWorkspace.rk_dim28_bPerm[25] = b[rk_perm[25]];
acadoWorkspace.rk_dim28_bPerm[26] = b[rk_perm[26]];
acadoWorkspace.rk_dim28_bPerm[27] = b[rk_perm[27]];
acadoWorkspace.rk_dim28_bPerm[1] += A[28]*acadoWorkspace.rk_dim28_bPerm[0];

acadoWorkspace.rk_dim28_bPerm[2] += A[56]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[2] += A[57]*acadoWorkspace.rk_dim28_bPerm[1];

acadoWorkspace.rk_dim28_bPerm[3] += A[84]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[3] += A[85]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[3] += A[86]*acadoWorkspace.rk_dim28_bPerm[2];

acadoWorkspace.rk_dim28_bPerm[4] += A[112]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[4] += A[113]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[4] += A[114]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[4] += A[115]*acadoWorkspace.rk_dim28_bPerm[3];

acadoWorkspace.rk_dim28_bPerm[5] += A[140]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[5] += A[141]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[5] += A[142]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[5] += A[143]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[5] += A[144]*acadoWorkspace.rk_dim28_bPerm[4];

acadoWorkspace.rk_dim28_bPerm[6] += A[168]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[6] += A[169]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[6] += A[170]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[6] += A[171]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[6] += A[172]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[6] += A[173]*acadoWorkspace.rk_dim28_bPerm[5];

acadoWorkspace.rk_dim28_bPerm[7] += A[196]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[7] += A[197]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[7] += A[198]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[7] += A[199]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[7] += A[200]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[7] += A[201]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[7] += A[202]*acadoWorkspace.rk_dim28_bPerm[6];

acadoWorkspace.rk_dim28_bPerm[8] += A[224]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[8] += A[225]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[8] += A[226]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[8] += A[227]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[8] += A[228]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[8] += A[229]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[8] += A[230]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[8] += A[231]*acadoWorkspace.rk_dim28_bPerm[7];

acadoWorkspace.rk_dim28_bPerm[9] += A[252]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[9] += A[253]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[9] += A[254]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[9] += A[255]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[9] += A[256]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[9] += A[257]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[9] += A[258]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[9] += A[259]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[9] += A[260]*acadoWorkspace.rk_dim28_bPerm[8];

acadoWorkspace.rk_dim28_bPerm[10] += A[280]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[10] += A[281]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[10] += A[282]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[10] += A[283]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[10] += A[284]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[10] += A[285]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[10] += A[286]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[10] += A[287]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[10] += A[288]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[10] += A[289]*acadoWorkspace.rk_dim28_bPerm[9];

acadoWorkspace.rk_dim28_bPerm[11] += A[308]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[11] += A[309]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[11] += A[310]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[11] += A[311]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[11] += A[312]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[11] += A[313]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[11] += A[314]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[11] += A[315]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[11] += A[316]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[11] += A[317]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[11] += A[318]*acadoWorkspace.rk_dim28_bPerm[10];

acadoWorkspace.rk_dim28_bPerm[12] += A[336]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[12] += A[337]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[12] += A[338]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[12] += A[339]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[12] += A[340]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[12] += A[341]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[12] += A[342]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[12] += A[343]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[12] += A[344]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[12] += A[345]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[12] += A[346]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[12] += A[347]*acadoWorkspace.rk_dim28_bPerm[11];

acadoWorkspace.rk_dim28_bPerm[13] += A[364]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[13] += A[365]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[13] += A[366]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[13] += A[367]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[13] += A[368]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[13] += A[369]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[13] += A[370]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[13] += A[371]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[13] += A[372]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[13] += A[373]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[13] += A[374]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[13] += A[375]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[13] += A[376]*acadoWorkspace.rk_dim28_bPerm[12];

acadoWorkspace.rk_dim28_bPerm[14] += A[392]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[14] += A[393]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[14] += A[394]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[14] += A[395]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[14] += A[396]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[14] += A[397]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[14] += A[398]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[14] += A[399]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[14] += A[400]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[14] += A[401]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[14] += A[402]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[14] += A[403]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[14] += A[404]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[14] += A[405]*acadoWorkspace.rk_dim28_bPerm[13];

acadoWorkspace.rk_dim28_bPerm[15] += A[420]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[15] += A[421]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[15] += A[422]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[15] += A[423]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[15] += A[424]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[15] += A[425]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[15] += A[426]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[15] += A[427]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[15] += A[428]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[15] += A[429]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[15] += A[430]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[15] += A[431]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[15] += A[432]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[15] += A[433]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[15] += A[434]*acadoWorkspace.rk_dim28_bPerm[14];

acadoWorkspace.rk_dim28_bPerm[16] += A[448]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[16] += A[449]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[16] += A[450]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[16] += A[451]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[16] += A[452]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[16] += A[453]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[16] += A[454]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[16] += A[455]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[16] += A[456]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[16] += A[457]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[16] += A[458]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[16] += A[459]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[16] += A[460]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[16] += A[461]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[16] += A[462]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[16] += A[463]*acadoWorkspace.rk_dim28_bPerm[15];

acadoWorkspace.rk_dim28_bPerm[17] += A[476]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[17] += A[477]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[17] += A[478]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[17] += A[479]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[17] += A[480]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[17] += A[481]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[17] += A[482]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[17] += A[483]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[17] += A[484]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[17] += A[485]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[17] += A[486]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[17] += A[487]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[17] += A[488]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[17] += A[489]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[17] += A[490]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[17] += A[491]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[17] += A[492]*acadoWorkspace.rk_dim28_bPerm[16];

acadoWorkspace.rk_dim28_bPerm[18] += A[504]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[18] += A[505]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[18] += A[506]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[18] += A[507]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[18] += A[508]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[18] += A[509]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[18] += A[510]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[18] += A[511]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[18] += A[512]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[18] += A[513]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[18] += A[514]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[18] += A[515]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[18] += A[516]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[18] += A[517]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[18] += A[518]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[18] += A[519]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[18] += A[520]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[18] += A[521]*acadoWorkspace.rk_dim28_bPerm[17];

acadoWorkspace.rk_dim28_bPerm[19] += A[532]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[19] += A[533]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[19] += A[534]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[19] += A[535]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[19] += A[536]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[19] += A[537]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[19] += A[538]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[19] += A[539]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[19] += A[540]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[19] += A[541]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[19] += A[542]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[19] += A[543]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[19] += A[544]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[19] += A[545]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[19] += A[546]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[19] += A[547]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[19] += A[548]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[19] += A[549]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[19] += A[550]*acadoWorkspace.rk_dim28_bPerm[18];

acadoWorkspace.rk_dim28_bPerm[20] += A[560]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[20] += A[561]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[20] += A[562]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[20] += A[563]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[20] += A[564]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[20] += A[565]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[20] += A[566]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[20] += A[567]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[20] += A[568]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[20] += A[569]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[20] += A[570]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[20] += A[571]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[20] += A[572]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[20] += A[573]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[20] += A[574]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[20] += A[575]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[20] += A[576]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[20] += A[577]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[20] += A[578]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[20] += A[579]*acadoWorkspace.rk_dim28_bPerm[19];

acadoWorkspace.rk_dim28_bPerm[21] += A[588]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[21] += A[589]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[21] += A[590]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[21] += A[591]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[21] += A[592]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[21] += A[593]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[21] += A[594]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[21] += A[595]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[21] += A[596]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[21] += A[597]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[21] += A[598]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[21] += A[599]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[21] += A[600]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[21] += A[601]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[21] += A[602]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[21] += A[603]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[21] += A[604]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[21] += A[605]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[21] += A[606]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[21] += A[607]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[21] += A[608]*acadoWorkspace.rk_dim28_bPerm[20];

acadoWorkspace.rk_dim28_bPerm[22] += A[616]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[22] += A[617]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[22] += A[618]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[22] += A[619]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[22] += A[620]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[22] += A[621]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[22] += A[622]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[22] += A[623]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[22] += A[624]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[22] += A[625]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[22] += A[626]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[22] += A[627]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[22] += A[628]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[22] += A[629]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[22] += A[630]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[22] += A[631]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[22] += A[632]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[22] += A[633]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[22] += A[634]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[22] += A[635]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[22] += A[636]*acadoWorkspace.rk_dim28_bPerm[20];
acadoWorkspace.rk_dim28_bPerm[22] += A[637]*acadoWorkspace.rk_dim28_bPerm[21];

acadoWorkspace.rk_dim28_bPerm[23] += A[644]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[23] += A[645]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[23] += A[646]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[23] += A[647]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[23] += A[648]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[23] += A[649]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[23] += A[650]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[23] += A[651]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[23] += A[652]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[23] += A[653]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[23] += A[654]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[23] += A[655]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[23] += A[656]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[23] += A[657]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[23] += A[658]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[23] += A[659]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[23] += A[660]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[23] += A[661]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[23] += A[662]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[23] += A[663]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[23] += A[664]*acadoWorkspace.rk_dim28_bPerm[20];
acadoWorkspace.rk_dim28_bPerm[23] += A[665]*acadoWorkspace.rk_dim28_bPerm[21];
acadoWorkspace.rk_dim28_bPerm[23] += A[666]*acadoWorkspace.rk_dim28_bPerm[22];

acadoWorkspace.rk_dim28_bPerm[24] += A[672]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[24] += A[673]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[24] += A[674]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[24] += A[675]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[24] += A[676]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[24] += A[677]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[24] += A[678]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[24] += A[679]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[24] += A[680]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[24] += A[681]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[24] += A[682]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[24] += A[683]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[24] += A[684]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[24] += A[685]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[24] += A[686]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[24] += A[687]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[24] += A[688]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[24] += A[689]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[24] += A[690]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[24] += A[691]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[24] += A[692]*acadoWorkspace.rk_dim28_bPerm[20];
acadoWorkspace.rk_dim28_bPerm[24] += A[693]*acadoWorkspace.rk_dim28_bPerm[21];
acadoWorkspace.rk_dim28_bPerm[24] += A[694]*acadoWorkspace.rk_dim28_bPerm[22];
acadoWorkspace.rk_dim28_bPerm[24] += A[695]*acadoWorkspace.rk_dim28_bPerm[23];

acadoWorkspace.rk_dim28_bPerm[25] += A[700]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[25] += A[701]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[25] += A[702]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[25] += A[703]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[25] += A[704]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[25] += A[705]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[25] += A[706]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[25] += A[707]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[25] += A[708]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[25] += A[709]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[25] += A[710]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[25] += A[711]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[25] += A[712]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[25] += A[713]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[25] += A[714]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[25] += A[715]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[25] += A[716]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[25] += A[717]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[25] += A[718]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[25] += A[719]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[25] += A[720]*acadoWorkspace.rk_dim28_bPerm[20];
acadoWorkspace.rk_dim28_bPerm[25] += A[721]*acadoWorkspace.rk_dim28_bPerm[21];
acadoWorkspace.rk_dim28_bPerm[25] += A[722]*acadoWorkspace.rk_dim28_bPerm[22];
acadoWorkspace.rk_dim28_bPerm[25] += A[723]*acadoWorkspace.rk_dim28_bPerm[23];
acadoWorkspace.rk_dim28_bPerm[25] += A[724]*acadoWorkspace.rk_dim28_bPerm[24];

acadoWorkspace.rk_dim28_bPerm[26] += A[728]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[26] += A[729]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[26] += A[730]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[26] += A[731]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[26] += A[732]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[26] += A[733]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[26] += A[734]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[26] += A[735]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[26] += A[736]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[26] += A[737]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[26] += A[738]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[26] += A[739]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[26] += A[740]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[26] += A[741]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[26] += A[742]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[26] += A[743]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[26] += A[744]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[26] += A[745]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[26] += A[746]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[26] += A[747]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[26] += A[748]*acadoWorkspace.rk_dim28_bPerm[20];
acadoWorkspace.rk_dim28_bPerm[26] += A[749]*acadoWorkspace.rk_dim28_bPerm[21];
acadoWorkspace.rk_dim28_bPerm[26] += A[750]*acadoWorkspace.rk_dim28_bPerm[22];
acadoWorkspace.rk_dim28_bPerm[26] += A[751]*acadoWorkspace.rk_dim28_bPerm[23];
acadoWorkspace.rk_dim28_bPerm[26] += A[752]*acadoWorkspace.rk_dim28_bPerm[24];
acadoWorkspace.rk_dim28_bPerm[26] += A[753]*acadoWorkspace.rk_dim28_bPerm[25];

acadoWorkspace.rk_dim28_bPerm[27] += A[756]*acadoWorkspace.rk_dim28_bPerm[0];
acadoWorkspace.rk_dim28_bPerm[27] += A[757]*acadoWorkspace.rk_dim28_bPerm[1];
acadoWorkspace.rk_dim28_bPerm[27] += A[758]*acadoWorkspace.rk_dim28_bPerm[2];
acadoWorkspace.rk_dim28_bPerm[27] += A[759]*acadoWorkspace.rk_dim28_bPerm[3];
acadoWorkspace.rk_dim28_bPerm[27] += A[760]*acadoWorkspace.rk_dim28_bPerm[4];
acadoWorkspace.rk_dim28_bPerm[27] += A[761]*acadoWorkspace.rk_dim28_bPerm[5];
acadoWorkspace.rk_dim28_bPerm[27] += A[762]*acadoWorkspace.rk_dim28_bPerm[6];
acadoWorkspace.rk_dim28_bPerm[27] += A[763]*acadoWorkspace.rk_dim28_bPerm[7];
acadoWorkspace.rk_dim28_bPerm[27] += A[764]*acadoWorkspace.rk_dim28_bPerm[8];
acadoWorkspace.rk_dim28_bPerm[27] += A[765]*acadoWorkspace.rk_dim28_bPerm[9];
acadoWorkspace.rk_dim28_bPerm[27] += A[766]*acadoWorkspace.rk_dim28_bPerm[10];
acadoWorkspace.rk_dim28_bPerm[27] += A[767]*acadoWorkspace.rk_dim28_bPerm[11];
acadoWorkspace.rk_dim28_bPerm[27] += A[768]*acadoWorkspace.rk_dim28_bPerm[12];
acadoWorkspace.rk_dim28_bPerm[27] += A[769]*acadoWorkspace.rk_dim28_bPerm[13];
acadoWorkspace.rk_dim28_bPerm[27] += A[770]*acadoWorkspace.rk_dim28_bPerm[14];
acadoWorkspace.rk_dim28_bPerm[27] += A[771]*acadoWorkspace.rk_dim28_bPerm[15];
acadoWorkspace.rk_dim28_bPerm[27] += A[772]*acadoWorkspace.rk_dim28_bPerm[16];
acadoWorkspace.rk_dim28_bPerm[27] += A[773]*acadoWorkspace.rk_dim28_bPerm[17];
acadoWorkspace.rk_dim28_bPerm[27] += A[774]*acadoWorkspace.rk_dim28_bPerm[18];
acadoWorkspace.rk_dim28_bPerm[27] += A[775]*acadoWorkspace.rk_dim28_bPerm[19];
acadoWorkspace.rk_dim28_bPerm[27] += A[776]*acadoWorkspace.rk_dim28_bPerm[20];
acadoWorkspace.rk_dim28_bPerm[27] += A[777]*acadoWorkspace.rk_dim28_bPerm[21];
acadoWorkspace.rk_dim28_bPerm[27] += A[778]*acadoWorkspace.rk_dim28_bPerm[22];
acadoWorkspace.rk_dim28_bPerm[27] += A[779]*acadoWorkspace.rk_dim28_bPerm[23];
acadoWorkspace.rk_dim28_bPerm[27] += A[780]*acadoWorkspace.rk_dim28_bPerm[24];
acadoWorkspace.rk_dim28_bPerm[27] += A[781]*acadoWorkspace.rk_dim28_bPerm[25];
acadoWorkspace.rk_dim28_bPerm[27] += A[782]*acadoWorkspace.rk_dim28_bPerm[26];


acado_solve_dim28_triangular( A, acadoWorkspace.rk_dim28_bPerm );
b[0] = acadoWorkspace.rk_dim28_bPerm[0];
b[1] = acadoWorkspace.rk_dim28_bPerm[1];
b[2] = acadoWorkspace.rk_dim28_bPerm[2];
b[3] = acadoWorkspace.rk_dim28_bPerm[3];
b[4] = acadoWorkspace.rk_dim28_bPerm[4];
b[5] = acadoWorkspace.rk_dim28_bPerm[5];
b[6] = acadoWorkspace.rk_dim28_bPerm[6];
b[7] = acadoWorkspace.rk_dim28_bPerm[7];
b[8] = acadoWorkspace.rk_dim28_bPerm[8];
b[9] = acadoWorkspace.rk_dim28_bPerm[9];
b[10] = acadoWorkspace.rk_dim28_bPerm[10];
b[11] = acadoWorkspace.rk_dim28_bPerm[11];
b[12] = acadoWorkspace.rk_dim28_bPerm[12];
b[13] = acadoWorkspace.rk_dim28_bPerm[13];
b[14] = acadoWorkspace.rk_dim28_bPerm[14];
b[15] = acadoWorkspace.rk_dim28_bPerm[15];
b[16] = acadoWorkspace.rk_dim28_bPerm[16];
b[17] = acadoWorkspace.rk_dim28_bPerm[17];
b[18] = acadoWorkspace.rk_dim28_bPerm[18];
b[19] = acadoWorkspace.rk_dim28_bPerm[19];
b[20] = acadoWorkspace.rk_dim28_bPerm[20];
b[21] = acadoWorkspace.rk_dim28_bPerm[21];
b[22] = acadoWorkspace.rk_dim28_bPerm[22];
b[23] = acadoWorkspace.rk_dim28_bPerm[23];
b[24] = acadoWorkspace.rk_dim28_bPerm[24];
b[25] = acadoWorkspace.rk_dim28_bPerm[25];
b[26] = acadoWorkspace.rk_dim28_bPerm[26];
b[27] = acadoWorkspace.rk_dim28_bPerm[27];
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
acadoWorkspace.rk_xxx[14] = rk_eta[252];
acadoWorkspace.rk_xxx[15] = rk_eta[253];
acadoWorkspace.rk_xxx[16] = rk_eta[254];

for (run = 0; run < 5; ++run)
{
if( run > 0 ) {
for (i = 0; i < 14; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 17] = rk_eta[i * 14 + 14];
acadoWorkspace.rk_diffsPrev2[i * 17 + 1] = rk_eta[i * 14 + 15];
acadoWorkspace.rk_diffsPrev2[i * 17 + 2] = rk_eta[i * 14 + 16];
acadoWorkspace.rk_diffsPrev2[i * 17 + 3] = rk_eta[i * 14 + 17];
acadoWorkspace.rk_diffsPrev2[i * 17 + 4] = rk_eta[i * 14 + 18];
acadoWorkspace.rk_diffsPrev2[i * 17 + 5] = rk_eta[i * 14 + 19];
acadoWorkspace.rk_diffsPrev2[i * 17 + 6] = rk_eta[i * 14 + 20];
acadoWorkspace.rk_diffsPrev2[i * 17 + 7] = rk_eta[i * 14 + 21];
acadoWorkspace.rk_diffsPrev2[i * 17 + 8] = rk_eta[i * 14 + 22];
acadoWorkspace.rk_diffsPrev2[i * 17 + 9] = rk_eta[i * 14 + 23];
acadoWorkspace.rk_diffsPrev2[i * 17 + 10] = rk_eta[i * 14 + 24];
acadoWorkspace.rk_diffsPrev2[i * 17 + 11] = rk_eta[i * 14 + 25];
acadoWorkspace.rk_diffsPrev2[i * 17 + 12] = rk_eta[i * 14 + 26];
acadoWorkspace.rk_diffsPrev2[i * 17 + 13] = rk_eta[i * 14 + 27];
acadoWorkspace.rk_diffsPrev2[i * 17 + 14] = rk_eta[i * 3 + 210];
acadoWorkspace.rk_diffsPrev2[i * 17 + 15] = rk_eta[i * 3 + 211];
acadoWorkspace.rk_diffsPrev2[i * 17 + 16] = rk_eta[i * 3 + 212];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 14; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 238 ]) );
for (j = 0; j < 14; ++j)
{
tmp_index1 = (run1 * 14) + (j);
acadoWorkspace.rk_A[tmp_index1 * 28] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 7] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 9] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 10] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 11] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 12] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 13] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 13)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 28) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 28 + 14] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 15] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 16] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 17] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 18] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 19] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 20] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 21] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 22] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 23] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 24] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 25] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 26] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 27] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 13)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 28) + (j + 14)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 14] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 14 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 14 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 14 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 14 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 14 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 14 + 6] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 14 + 7] = acadoWorkspace.rk_kkk[run1 + 14] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 14 + 8] = acadoWorkspace.rk_kkk[run1 + 16] - acadoWorkspace.rk_rhsTemp[8];
acadoWorkspace.rk_b[run1 * 14 + 9] = acadoWorkspace.rk_kkk[run1 + 18] - acadoWorkspace.rk_rhsTemp[9];
acadoWorkspace.rk_b[run1 * 14 + 10] = acadoWorkspace.rk_kkk[run1 + 20] - acadoWorkspace.rk_rhsTemp[10];
acadoWorkspace.rk_b[run1 * 14 + 11] = acadoWorkspace.rk_kkk[run1 + 22] - acadoWorkspace.rk_rhsTemp[11];
acadoWorkspace.rk_b[run1 * 14 + 12] = acadoWorkspace.rk_kkk[run1 + 24] - acadoWorkspace.rk_rhsTemp[12];
acadoWorkspace.rk_b[run1 * 14 + 13] = acadoWorkspace.rk_kkk[run1 + 26] - acadoWorkspace.rk_rhsTemp[13];
}
det = acado_solve_dim28_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim28_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 14];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 14 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 14 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 14 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 14 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 14 + 5];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 14 + 6];
acadoWorkspace.rk_kkk[j + 14] += acadoWorkspace.rk_b[j * 14 + 7];
acadoWorkspace.rk_kkk[j + 16] += acadoWorkspace.rk_b[j * 14 + 8];
acadoWorkspace.rk_kkk[j + 18] += acadoWorkspace.rk_b[j * 14 + 9];
acadoWorkspace.rk_kkk[j + 20] += acadoWorkspace.rk_b[j * 14 + 10];
acadoWorkspace.rk_kkk[j + 22] += acadoWorkspace.rk_b[j * 14 + 11];
acadoWorkspace.rk_kkk[j + 24] += acadoWorkspace.rk_b[j * 14 + 12];
acadoWorkspace.rk_kkk[j + 26] += acadoWorkspace.rk_b[j * 14 + 13];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 14; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 14] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 14 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 14 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 14 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 14 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 14 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 14 + 6] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 14 + 7] = acadoWorkspace.rk_kkk[run1 + 14] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 14 + 8] = acadoWorkspace.rk_kkk[run1 + 16] - acadoWorkspace.rk_rhsTemp[8];
acadoWorkspace.rk_b[run1 * 14 + 9] = acadoWorkspace.rk_kkk[run1 + 18] - acadoWorkspace.rk_rhsTemp[9];
acadoWorkspace.rk_b[run1 * 14 + 10] = acadoWorkspace.rk_kkk[run1 + 20] - acadoWorkspace.rk_rhsTemp[10];
acadoWorkspace.rk_b[run1 * 14 + 11] = acadoWorkspace.rk_kkk[run1 + 22] - acadoWorkspace.rk_rhsTemp[11];
acadoWorkspace.rk_b[run1 * 14 + 12] = acadoWorkspace.rk_kkk[run1 + 24] - acadoWorkspace.rk_rhsTemp[12];
acadoWorkspace.rk_b[run1 * 14 + 13] = acadoWorkspace.rk_kkk[run1 + 26] - acadoWorkspace.rk_rhsTemp[13];
}
acado_solve_dim28_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim28_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 14];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 14 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 14 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 14 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 14 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 14 + 5];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 14 + 6];
acadoWorkspace.rk_kkk[j + 14] += acadoWorkspace.rk_b[j * 14 + 7];
acadoWorkspace.rk_kkk[j + 16] += acadoWorkspace.rk_b[j * 14 + 8];
acadoWorkspace.rk_kkk[j + 18] += acadoWorkspace.rk_b[j * 14 + 9];
acadoWorkspace.rk_kkk[j + 20] += acadoWorkspace.rk_b[j * 14 + 10];
acadoWorkspace.rk_kkk[j + 22] += acadoWorkspace.rk_b[j * 14 + 11];
acadoWorkspace.rk_kkk[j + 24] += acadoWorkspace.rk_b[j * 14 + 12];
acadoWorkspace.rk_kkk[j + 26] += acadoWorkspace.rk_b[j * 14 + 13];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 14; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 238 ]) );
for (j = 0; j < 14; ++j)
{
tmp_index1 = (run1 * 14) + (j);
acadoWorkspace.rk_A[tmp_index1 * 28] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 7] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 9] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 10] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 11] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 12] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 13] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 13)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 28) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 28 + 14] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 15] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 16] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 17] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 18] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 19] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 20] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 21] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 22] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 23] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 24] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 25] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 26] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 28 + 27] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 238) + (j * 17 + 13)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 28) + (j + 14)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 14; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 14] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1)];
acadoWorkspace.rk_b[i * 14 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 17)];
acadoWorkspace.rk_b[i * 14 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 34)];
acadoWorkspace.rk_b[i * 14 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 51)];
acadoWorkspace.rk_b[i * 14 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 68)];
acadoWorkspace.rk_b[i * 14 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 85)];
acadoWorkspace.rk_b[i * 14 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 102)];
acadoWorkspace.rk_b[i * 14 + 7] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 119)];
acadoWorkspace.rk_b[i * 14 + 8] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 136)];
acadoWorkspace.rk_b[i * 14 + 9] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 153)];
acadoWorkspace.rk_b[i * 14 + 10] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 170)];
acadoWorkspace.rk_b[i * 14 + 11] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 187)];
acadoWorkspace.rk_b[i * 14 + 12] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 204)];
acadoWorkspace.rk_b[i * 14 + 13] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (run1 + 221)];
}
if( 0 == run1 ) {
det = acado_solve_dim28_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim28_perm );
}
 else {
acado_solve_dim28_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim28_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 14];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 14 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 14 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 14 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 14 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 14 + 5];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 14 + 6];
acadoWorkspace.rk_diffK[i + 14] = acadoWorkspace.rk_b[i * 14 + 7];
acadoWorkspace.rk_diffK[i + 16] = acadoWorkspace.rk_b[i * 14 + 8];
acadoWorkspace.rk_diffK[i + 18] = acadoWorkspace.rk_b[i * 14 + 9];
acadoWorkspace.rk_diffK[i + 20] = acadoWorkspace.rk_b[i * 14 + 10];
acadoWorkspace.rk_diffK[i + 22] = acadoWorkspace.rk_b[i * 14 + 11];
acadoWorkspace.rk_diffK[i + 24] = acadoWorkspace.rk_b[i * 14 + 12];
acadoWorkspace.rk_diffK[i + 26] = acadoWorkspace.rk_b[i * 14 + 13];
}
for (i = 0; i < 14; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 17) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 17) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 14; ++j)
{
tmp_index1 = (i * 14) + (j);
tmp_index2 = (run1) + (j * 17);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 238) + (tmp_index2 + 14)];
}
}
acado_solve_dim28_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim28_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 14];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 14 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 14 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 14 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 14 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 14 + 5];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 14 + 6];
acadoWorkspace.rk_diffK[i + 14] = acadoWorkspace.rk_b[i * 14 + 7];
acadoWorkspace.rk_diffK[i + 16] = acadoWorkspace.rk_b[i * 14 + 8];
acadoWorkspace.rk_diffK[i + 18] = acadoWorkspace.rk_b[i * 14 + 9];
acadoWorkspace.rk_diffK[i + 20] = acadoWorkspace.rk_b[i * 14 + 10];
acadoWorkspace.rk_diffK[i + 22] = acadoWorkspace.rk_b[i * 14 + 11];
acadoWorkspace.rk_diffK[i + 24] = acadoWorkspace.rk_b[i * 14 + 12];
acadoWorkspace.rk_diffK[i + 26] = acadoWorkspace.rk_b[i * 14 + 13];
}
for (i = 0; i < 14; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 17) + (run1 + 14)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
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
if( run == 0 ) {
for (i = 0; i < 14; ++i)
{
for (j = 0; j < 14; ++j)
{
tmp_index2 = (j) + (i * 14);
rk_eta[tmp_index2 + 14] = acadoWorkspace.rk_diffsNew2[(i * 17) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 210] = acadoWorkspace.rk_diffsNew2[(i * 17) + (j + 14)];
}
}
}
else {
for (i = 0; i < 14; ++i)
{
for (j = 0; j < 14; ++j)
{
tmp_index2 = (j) + (i * 14);
rk_eta[tmp_index2 + 14] = + acadoWorkspace.rk_diffsNew2[i * 17]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 1]*acadoWorkspace.rk_diffsPrev2[j + 17];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 2]*acadoWorkspace.rk_diffsPrev2[j + 34];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 3]*acadoWorkspace.rk_diffsPrev2[j + 51];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 4]*acadoWorkspace.rk_diffsPrev2[j + 68];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 5]*acadoWorkspace.rk_diffsPrev2[j + 85];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 6]*acadoWorkspace.rk_diffsPrev2[j + 102];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 7]*acadoWorkspace.rk_diffsPrev2[j + 119];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 8]*acadoWorkspace.rk_diffsPrev2[j + 136];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 9]*acadoWorkspace.rk_diffsPrev2[j + 153];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 10]*acadoWorkspace.rk_diffsPrev2[j + 170];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 11]*acadoWorkspace.rk_diffsPrev2[j + 187];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 12]*acadoWorkspace.rk_diffsPrev2[j + 204];
rk_eta[tmp_index2 + 14] += + acadoWorkspace.rk_diffsNew2[i * 17 + 13]*acadoWorkspace.rk_diffsPrev2[j + 221];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 210] = acadoWorkspace.rk_diffsNew2[(i * 17) + (j + 14)];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17]*acadoWorkspace.rk_diffsPrev2[j + 14];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 1]*acadoWorkspace.rk_diffsPrev2[j + 31];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 2]*acadoWorkspace.rk_diffsPrev2[j + 48];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 3]*acadoWorkspace.rk_diffsPrev2[j + 65];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 4]*acadoWorkspace.rk_diffsPrev2[j + 82];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 5]*acadoWorkspace.rk_diffsPrev2[j + 99];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 6]*acadoWorkspace.rk_diffsPrev2[j + 116];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 7]*acadoWorkspace.rk_diffsPrev2[j + 133];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 8]*acadoWorkspace.rk_diffsPrev2[j + 150];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 9]*acadoWorkspace.rk_diffsPrev2[j + 167];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 10]*acadoWorkspace.rk_diffsPrev2[j + 184];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 11]*acadoWorkspace.rk_diffsPrev2[j + 201];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 12]*acadoWorkspace.rk_diffsPrev2[j + 218];
rk_eta[tmp_index2 + 210] += + acadoWorkspace.rk_diffsNew2[i * 17 + 13]*acadoWorkspace.rk_diffsPrev2[j + 235];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 2.0000000000000001e-01;
}
for (i = 0; i < 14; ++i)
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



