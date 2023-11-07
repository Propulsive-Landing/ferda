#include "pm_std.h"
#include "math.h"
void pm_math_lin_alg_geSolve2x2(const real_T pm_math_F2l4p_g4sn02huHNflQjMH[4]
,const real_T b[2],real_T x[2]);void pm_math_lin_alg_geSolve2x2(const real_T
pm_math_F2l4p_g4sn02huHNflQjMH[4],const real_T b[2],real_T x[2]){if(fabs(
pm_math_F2l4p_g4sn02huHNflQjMH[0])>=fabs(pm_math_F2l4p_g4sn02huHNflQjMH[1])&&
fabs(pm_math_F2l4p_g4sn02huHNflQjMH[0])>=fabs(pm_math_F2l4p_g4sn02huHNflQjMH[2
])&&fabs(pm_math_F2l4p_g4sn02huHNflQjMH[0])>=fabs(
pm_math_F2l4p_g4sn02huHNflQjMH[3])){const real_T pm_math_V2__YrimeI4E_yWnhKofpy
=pm_math_F2l4p_g4sn02huHNflQjMH[1]/pm_math_F2l4p_g4sn02huHNflQjMH[0];x[1]=(b[1
]-pm_math_V2__YrimeI4E_yWnhKofpy*b[0])/(pm_math_F2l4p_g4sn02huHNflQjMH[3]-
pm_math_V2__YrimeI4E_yWnhKofpy*pm_math_F2l4p_g4sn02huHNflQjMH[2]);x[0]=(b[0]-
pm_math_F2l4p_g4sn02huHNflQjMH[2]*x[1])/pm_math_F2l4p_g4sn02huHNflQjMH[0];}
else if(fabs(pm_math_F2l4p_g4sn02huHNflQjMH[1])>=fabs(
pm_math_F2l4p_g4sn02huHNflQjMH[2])&&fabs(pm_math_F2l4p_g4sn02huHNflQjMH[1])>=
fabs(pm_math_F2l4p_g4sn02huHNflQjMH[3])){const real_T
pm_math_V2__YrimeI4E_yWnhKofpy=pm_math_F2l4p_g4sn02huHNflQjMH[0]/
pm_math_F2l4p_g4sn02huHNflQjMH[1];x[1]=(b[0]-pm_math_V2__YrimeI4E_yWnhKofpy*b[
1])/(pm_math_F2l4p_g4sn02huHNflQjMH[2]-pm_math_V2__YrimeI4E_yWnhKofpy*
pm_math_F2l4p_g4sn02huHNflQjMH[3]);x[0]=(b[1]-pm_math_F2l4p_g4sn02huHNflQjMH[3
]*x[1])/pm_math_F2l4p_g4sn02huHNflQjMH[1];}else if(fabs(
pm_math_F2l4p_g4sn02huHNflQjMH[2])>=fabs(pm_math_F2l4p_g4sn02huHNflQjMH[3])){
const real_T pm_math_V2__YrimeI4E_yWnhKofpy=pm_math_F2l4p_g4sn02huHNflQjMH[3]/
pm_math_F2l4p_g4sn02huHNflQjMH[2];x[0]=(b[1]-pm_math_V2__YrimeI4E_yWnhKofpy*b[
0])/(pm_math_F2l4p_g4sn02huHNflQjMH[1]-pm_math_V2__YrimeI4E_yWnhKofpy*
pm_math_F2l4p_g4sn02huHNflQjMH[0]);x[1]=(b[0]-pm_math_F2l4p_g4sn02huHNflQjMH[0
]*x[0])/pm_math_F2l4p_g4sn02huHNflQjMH[2];}else{const real_T
pm_math_V2__YrimeI4E_yWnhKofpy=pm_math_F2l4p_g4sn02huHNflQjMH[2]/
pm_math_F2l4p_g4sn02huHNflQjMH[3];x[0]=(b[0]-pm_math_V2__YrimeI4E_yWnhKofpy*b[
1])/(pm_math_F2l4p_g4sn02huHNflQjMH[0]-pm_math_V2__YrimeI4E_yWnhKofpy*
pm_math_F2l4p_g4sn02huHNflQjMH[1]);x[1]=(b[1]-pm_math_F2l4p_g4sn02huHNflQjMH[1
]*x[0])/pm_math_F2l4p_g4sn02huHNflQjMH[3];}}
