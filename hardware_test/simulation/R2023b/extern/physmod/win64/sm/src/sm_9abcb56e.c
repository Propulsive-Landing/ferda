#include "pm_std.h"
#include "math.h"
#include "string.h"
#include "stddef.h"
#include "pm_std.h"
boolean_T sm_core_math_bestFitPlane3D(size_t sm_VN1ZzlM6fFlqZPfvzFe_XE,const
double*sm_FyV0AI7EXfWmgeHWBvw3x0,double*sm__jScRWP4tutfaHlU_RjDxz,double*
sm_FSzIyLZpToGyYTlwh2NE5V);struct sm_Vy1mCbsM_oOrYTVxpROczp{double x;double
sm_FzyLWRgau0pMYq2XSI3ETL;double sm_FBDi_PCg670TjHgJTNPcHr;};typedef struct
sm_Vy1mCbsM_oOrYTVxpROczp sm_kd__PjR5tCWObue2RSyCuW;static double
sm_VolitXZcmXCyYioLtCkEVH(double a,double b,double sm_FFZbGh27ya8eem_J_hUtAZ){
const double sm_Vzz7fHz6oElvjujXUEM3YM=a+b;const double
sm_Vo9I5RFcPal_bXJl4MT4zD=a-b;const double sm_V2__YrimeI4E_yWnhKofpy=sqrt(
sm_Vo9I5RFcPal_bXJl4MT4zD*sm_Vo9I5RFcPal_bXJl4MT4zD+4*
sm_FFZbGh27ya8eem_J_hUtAZ*sm_FFZbGh27ya8eem_J_hUtAZ);const double
sm_FtUIbPki1dtk_TdP7id9dk=fabs(sm_Vzz7fHz6oElvjujXUEM3YM+
sm_V2__YrimeI4E_yWnhKofpy);const double sm__7kbyq4YepGcWqaK7gDLqh=fabs(
sm_Vzz7fHz6oElvjujXUEM3YM-sm_V2__YrimeI4E_yWnhKofpy);const double
sm_Vv2_W48_DbO9We5P8cdZyM=sm_FtUIbPki1dtk_TdP7id9dk<sm__7kbyq4YepGcWqaK7gDLqh?
sm_FtUIbPki1dtk_TdP7id9dk:sm__7kbyq4YepGcWqaK7gDLqh;const double
sm_F0_GTpKxKWl2hTmn6u1b_q=sm_FtUIbPki1dtk_TdP7id9dk>sm__7kbyq4YepGcWqaK7gDLqh?
sm_FtUIbPki1dtk_TdP7id9dk:sm__7kbyq4YepGcWqaK7gDLqh;return
sm_Vv2_W48_DbO9We5P8cdZyM==0.0?pmf_get_real_max():(sm_F0_GTpKxKWl2hTmn6u1b_q/
sm_Vv2_W48_DbO9We5P8cdZyM);}boolean_T sm_core_math_bestFitPlane3D(size_t
sm_VN1ZzlM6fFlqZPfvzFe_XE,const double*sm_V17805WDHPt5jXZn9Clfwr,double*
sm_FJxjBa_HFXWXaPdZfDjY5g,double*sm_FSzIyLZpToGyYTlwh2NE5V){const
sm_kd__PjR5tCWObue2RSyCuW*sm_FyV0AI7EXfWmgeHWBvw3x0=(const
sm_kd__PjR5tCWObue2RSyCuW*)sm_V17805WDHPt5jXZn9Clfwr;sm_kd__PjR5tCWObue2RSyCuW
*sm__jScRWP4tutfaHlU_RjDxz=(sm_kd__PjR5tCWObue2RSyCuW*)
sm_FJxjBa_HFXWXaPdZfDjY5g;size_t sm_kwrB3ZoKf7OufTHWaHJV7a;int
sm_kEbBObcYFIxUZ5_77V3CO_,sm_VgJW5ZqpwPpuY1inYtaofQ,sm_V1pxxydYEnWVVi3QKesjvf;
int sm_FQzVh6whWGt6iPSem8N9Wg;double sm_FrBiO20tGR8EZu6tAervEE[3][3]={{0,0,0},
{0,0,0},{0,0,0}};if(sm_VN1ZzlM6fFlqZPfvzFe_XE<3)return false;
sm__jScRWP4tutfaHlU_RjDxz->x=sm_FyV0AI7EXfWmgeHWBvw3x0->x;
sm__jScRWP4tutfaHlU_RjDxz->sm_FzyLWRgau0pMYq2XSI3ETL=sm_FyV0AI7EXfWmgeHWBvw3x0
->sm_FzyLWRgau0pMYq2XSI3ETL;sm__jScRWP4tutfaHlU_RjDxz->
sm_FBDi_PCg670TjHgJTNPcHr=sm_FyV0AI7EXfWmgeHWBvw3x0->sm_FBDi_PCg670TjHgJTNPcHr
;for(sm_kwrB3ZoKf7OufTHWaHJV7a=1;sm_kwrB3ZoKf7OufTHWaHJV7a<
sm_VN1ZzlM6fFlqZPfvzFe_XE;++sm_kwrB3ZoKf7OufTHWaHJV7a){
sm__jScRWP4tutfaHlU_RjDxz->x+=sm_FyV0AI7EXfWmgeHWBvw3x0[
sm_kwrB3ZoKf7OufTHWaHJV7a].x;sm__jScRWP4tutfaHlU_RjDxz->
sm_FzyLWRgau0pMYq2XSI3ETL+=sm_FyV0AI7EXfWmgeHWBvw3x0[sm_kwrB3ZoKf7OufTHWaHJV7a
].sm_FzyLWRgau0pMYq2XSI3ETL;sm__jScRWP4tutfaHlU_RjDxz->
sm_FBDi_PCg670TjHgJTNPcHr+=sm_FyV0AI7EXfWmgeHWBvw3x0[sm_kwrB3ZoKf7OufTHWaHJV7a
].sm_FBDi_PCg670TjHgJTNPcHr;}sm__jScRWP4tutfaHlU_RjDxz->x/=(double)(
sm_VN1ZzlM6fFlqZPfvzFe_XE);sm__jScRWP4tutfaHlU_RjDxz->
sm_FzyLWRgau0pMYq2XSI3ETL/=(double)(sm_VN1ZzlM6fFlqZPfvzFe_XE);
sm__jScRWP4tutfaHlU_RjDxz->sm_FBDi_PCg670TjHgJTNPcHr/=(double)(
sm_VN1ZzlM6fFlqZPfvzFe_XE);for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_VN1ZzlM6fFlqZPfvzFe_XE;++
sm_kwrB3ZoKf7OufTHWaHJV7a){sm_kd__PjR5tCWObue2RSyCuW sm__m28pQuK4Gtaf9vgo0L0DX
;sm__m28pQuK4Gtaf9vgo0L0DX.x=sm_FyV0AI7EXfWmgeHWBvw3x0[
sm_kwrB3ZoKf7OufTHWaHJV7a].x-sm__jScRWP4tutfaHlU_RjDxz->x;
sm__m28pQuK4Gtaf9vgo0L0DX.sm_FzyLWRgau0pMYq2XSI3ETL=sm_FyV0AI7EXfWmgeHWBvw3x0[
sm_kwrB3ZoKf7OufTHWaHJV7a].sm_FzyLWRgau0pMYq2XSI3ETL-sm__jScRWP4tutfaHlU_RjDxz
->sm_FzyLWRgau0pMYq2XSI3ETL;sm__m28pQuK4Gtaf9vgo0L0DX.
sm_FBDi_PCg670TjHgJTNPcHr=sm_FyV0AI7EXfWmgeHWBvw3x0[sm_kwrB3ZoKf7OufTHWaHJV7a]
.sm_FBDi_PCg670TjHgJTNPcHr-sm__jScRWP4tutfaHlU_RjDxz->
sm_FBDi_PCg670TjHgJTNPcHr;sm_FrBiO20tGR8EZu6tAervEE[0][0]+=
sm__m28pQuK4Gtaf9vgo0L0DX.x*sm__m28pQuK4Gtaf9vgo0L0DX.x;
sm_FrBiO20tGR8EZu6tAervEE[1][1]+=sm__m28pQuK4Gtaf9vgo0L0DX.
sm_FzyLWRgau0pMYq2XSI3ETL*sm__m28pQuK4Gtaf9vgo0L0DX.sm_FzyLWRgau0pMYq2XSI3ETL;
sm_FrBiO20tGR8EZu6tAervEE[2][2]+=sm__m28pQuK4Gtaf9vgo0L0DX.
sm_FBDi_PCg670TjHgJTNPcHr*sm__m28pQuK4Gtaf9vgo0L0DX.sm_FBDi_PCg670TjHgJTNPcHr;
sm_FrBiO20tGR8EZu6tAervEE[1][2]+=sm__m28pQuK4Gtaf9vgo0L0DX.
sm_FzyLWRgau0pMYq2XSI3ETL*sm__m28pQuK4Gtaf9vgo0L0DX.sm_FBDi_PCg670TjHgJTNPcHr;
sm_FrBiO20tGR8EZu6tAervEE[2][0]+=sm__m28pQuK4Gtaf9vgo0L0DX.
sm_FBDi_PCg670TjHgJTNPcHr*sm__m28pQuK4Gtaf9vgo0L0DX.x;
sm_FrBiO20tGR8EZu6tAervEE[0][1]+=sm__m28pQuK4Gtaf9vgo0L0DX.x*
sm__m28pQuK4Gtaf9vgo0L0DX.sm_FzyLWRgau0pMYq2XSI3ETL;}sm_FrBiO20tGR8EZu6tAervEE
[2][1]=sm_FrBiO20tGR8EZu6tAervEE[1][2];sm_FrBiO20tGR8EZu6tAervEE[0][2]=
sm_FrBiO20tGR8EZu6tAervEE[2][0];sm_FrBiO20tGR8EZu6tAervEE[1][0]=
sm_FrBiO20tGR8EZu6tAervEE[0][1];{double sm_kW5qn8w_fndXfilNILrk9M,
sm_kL3raftxOmhNXTEBwQPA7T=pmf_get_real_max();sm_FQzVh6whWGt6iPSem8N9Wg= -1;for
(sm_V1pxxydYEnWVVi3QKesjvf=0;sm_V1pxxydYEnWVVi3QKesjvf<3;++
sm_V1pxxydYEnWVVi3QKesjvf){sm_kEbBObcYFIxUZ5_77V3CO_=sm_V1pxxydYEnWVVi3QKesjvf
==2?0:(sm_V1pxxydYEnWVVi3QKesjvf+1);sm_VgJW5ZqpwPpuY1inYtaofQ=3-(
sm_V1pxxydYEnWVVi3QKesjvf+sm_kEbBObcYFIxUZ5_77V3CO_);sm_kW5qn8w_fndXfilNILrk9M
=sm_VolitXZcmXCyYioLtCkEVH(sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_
][sm_kEbBObcYFIxUZ5_77V3CO_],sm_FrBiO20tGR8EZu6tAervEE[
sm_VgJW5ZqpwPpuY1inYtaofQ][sm_VgJW5ZqpwPpuY1inYtaofQ],
sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][sm_VgJW5ZqpwPpuY1inYtaofQ
]);if(sm_kW5qn8w_fndXfilNILrk9M<sm_kL3raftxOmhNXTEBwQPA7T){
sm_kL3raftxOmhNXTEBwQPA7T=sm_kW5qn8w_fndXfilNILrk9M;sm_FQzVh6whWGt6iPSem8N9Wg=
sm_V1pxxydYEnWVVi3QKesjvf;}}if(sm_FQzVh6whWGt6iPSem8N9Wg<0)return false;
sm_V1pxxydYEnWVVi3QKesjvf=sm_FQzVh6whWGt6iPSem8N9Wg;sm_kEbBObcYFIxUZ5_77V3CO_=
sm_V1pxxydYEnWVVi3QKesjvf==2?0:(sm_V1pxxydYEnWVVi3QKesjvf+1);
sm_VgJW5ZqpwPpuY1inYtaofQ=3-(sm_V1pxxydYEnWVVi3QKesjvf+
sm_kEbBObcYFIxUZ5_77V3CO_);if(sm_FrBiO20tGR8EZu6tAervEE[
sm_kEbBObcYFIxUZ5_77V3CO_][sm_kEbBObcYFIxUZ5_77V3CO_]<
sm_FrBiO20tGR8EZu6tAervEE[sm_VgJW5ZqpwPpuY1inYtaofQ][sm_VgJW5ZqpwPpuY1inYtaofQ
]){int sm_Fk2O4u6vQUpibmbv8Kjgnn=sm_kEbBObcYFIxUZ5_77V3CO_;
sm_kEbBObcYFIxUZ5_77V3CO_=sm_VgJW5ZqpwPpuY1inYtaofQ;sm_VgJW5ZqpwPpuY1inYtaofQ=
sm_Fk2O4u6vQUpibmbv8Kjgnn;}}if(sm_FrBiO20tGR8EZu6tAervEE[
sm_kEbBObcYFIxUZ5_77V3CO_][sm_kEbBObcYFIxUZ5_77V3CO_]==0.0)return false;{const
double sm_FFZbGh27ya8eem_J_hUtAZ=sm_FrBiO20tGR8EZu6tAervEE[
sm_kEbBObcYFIxUZ5_77V3CO_][sm_VgJW5ZqpwPpuY1inYtaofQ]/
sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][sm_kEbBObcYFIxUZ5_77V3CO_
];const double sm_V94OHC32_a85j1a8nI0gLx=sm_FrBiO20tGR8EZu6tAervEE[
sm_VgJW5ZqpwPpuY1inYtaofQ][sm_VgJW5ZqpwPpuY1inYtaofQ]-
sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][sm_VgJW5ZqpwPpuY1inYtaofQ
]*sm_FFZbGh27ya8eem_J_hUtAZ;double sm_kPKKHEzx9slcbTkiKgmTSq=0.0;if(
sm_V94OHC32_a85j1a8nI0gLx==0.0)return false;sm_FSzIyLZpToGyYTlwh2NE5V[
sm_VgJW5ZqpwPpuY1inYtaofQ]= -(sm_FrBiO20tGR8EZu6tAervEE[
sm_VgJW5ZqpwPpuY1inYtaofQ][sm_V1pxxydYEnWVVi3QKesjvf]-
sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][sm_V1pxxydYEnWVVi3QKesjvf
]*sm_FFZbGh27ya8eem_J_hUtAZ)/sm_V94OHC32_a85j1a8nI0gLx;
sm_FSzIyLZpToGyYTlwh2NE5V[sm_kEbBObcYFIxUZ5_77V3CO_]=(-
sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][sm_V1pxxydYEnWVVi3QKesjvf
]-sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][
sm_VgJW5ZqpwPpuY1inYtaofQ]*sm_FSzIyLZpToGyYTlwh2NE5V[sm_VgJW5ZqpwPpuY1inYtaofQ
])/sm_FrBiO20tGR8EZu6tAervEE[sm_kEbBObcYFIxUZ5_77V3CO_][
sm_kEbBObcYFIxUZ5_77V3CO_];sm_FSzIyLZpToGyYTlwh2NE5V[sm_V1pxxydYEnWVVi3QKesjvf
]= +1.0;sm_kPKKHEzx9slcbTkiKgmTSq=sqrt(sm_FSzIyLZpToGyYTlwh2NE5V[0]*
sm_FSzIyLZpToGyYTlwh2NE5V[0]+sm_FSzIyLZpToGyYTlwh2NE5V[1]*
sm_FSzIyLZpToGyYTlwh2NE5V[1]+sm_FSzIyLZpToGyYTlwh2NE5V[2]*
sm_FSzIyLZpToGyYTlwh2NE5V[2]);sm_FSzIyLZpToGyYTlwh2NE5V[0]/=
sm_kPKKHEzx9slcbTkiKgmTSq;sm_FSzIyLZpToGyYTlwh2NE5V[1]/=
sm_kPKKHEzx9slcbTkiKgmTSq;sm_FSzIyLZpToGyYTlwh2NE5V[2]/=
sm_kPKKHEzx9slcbTkiKgmTSq;}return true;}
