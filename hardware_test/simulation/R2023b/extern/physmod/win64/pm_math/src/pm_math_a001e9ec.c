#include "pm_std.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "pm_std.h"
#include "pm_std.h"
void pm_math_lin_alg_vectorAdd(uint32_T n,const double*
pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,
double*pm_math_Vzz7fHz6oElvjujXUEM3YM);void pm_math_lin_alg_vectorSubtract(
uint32_T n,const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*pm_math_Vo9I5RFcPal_bXJl4MT4zD);void
pm_math_lin_alg_vectorScale(uint32_T n,double pm_math_FQferGZUKft3_i5GvYy4Oy,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*
pm_math_kMXfgG5aJNtdgeZBD1F9s7);void pm_math_lin_alg_vectorNegate(uint32_T n,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*
pm_math_Vmg5DyKiTS4YVaa_KIUPx2);void pm_math_lin_alg_matrixAssignStrided(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FnKRRzrQB6GPWacR5qZ_N1,uint32_T pm_math_VSkCOkBlf_lY_aOxP7FKDg,const
double*pm_math__Q_i1Z0_CMGpfPqpuLrnMT,double*pm_math_kyfq6L_eQbdYcPuOovpRDW);
void pm_math_lin_alg_matrixTransposeAdd(uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math__Q_i1Z0_CMGpfPqpuLrnMT);void
pm_math_lin_alg_matrixTransposeSubtract(uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_kyfq6L_eQbdYcPuOovpRDW);void
pm_math_lin_alg_matrixMultiply(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T pm_math_V2__YrimeI4E_yWnhKofpy,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void
pm_math_lin_alg_matrixMultiplyStrided(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T pm_math_V2__YrimeI4E_yWnhKofpy,uint32_T n,uint32_T
pm_math__8PlegfDdmpxaD3yVABBka,uint32_T pm_math_khoq8zTqfel4cuvza1xV8U,
uint32_T pm_math__Cskocqf_YOea9zRlVhiAe,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void
pm_math_lin_alg_matrixTransposeMultiply(uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void
pm_math_lin_alg_matrixMultiplyTranspose(uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void pm_math_lin_alg_scaleColumns(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_V5FB4OLCBN0eVqxcC6tIlV);void pm_math_lin_alg_scaleRows(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_knjWFknSIL0bYi3EKhrrJ4);void pm_math_lin_alg_inverseScaleColumns
(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_V5FB4OLCBN0eVqxcC6tIlV);void pm_math_lin_alg_inverseScaleRows(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_knjWFknSIL0bYi3EKhrrJ4);void pm_math_lin_alg_zeroMajor(uint32_T
pm_math_kRZY2SDSifK6ia_3EbLye7,uint32_T pm_math_FEXg1wwjA_S7cyqxiD_lWO,const
int32_T*pm_math_VU18hM_2GHSHWue20_mnXQ,double*pm_math_F2l4p_g4sn02huHNflQjMH);
void pm_math_lin_alg_reduceMatrix(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,uint32_T pm_math_FTSHHVQYO78NWDDPfQSm12,const int32_T*
pm_math_VLZhFoWjGJ0Q_DeI0FI6Gm,double*pm_math_F2l4p_g4sn02huHNflQjMH);void
pm_math_lin_alg_expandVector(uint32_T n,uint32_T pm_math_kZSgglHk8kWUemlk_e6TDr
,const int32_T*pm_math_VD0M3tgCkkSEiHjnw1wnK0,double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ);boolean_T pm_math_lin_alg_hasUniqueRows(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_VRZCD_UL_ESThy75dC9J8D);void
pm_math_lin_alg_matrixVectorMultiply(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_kcMuz7xytzhifPfhbjJNYL);void
pm_math_lin_alg_matrixVectorMultiplyStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_kcMuz7xytzhifPfhbjJNYL);void
pm_math_lin_alg_matrixTransposeVectorMultiply(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_F9DWOuzPFtdljqhs7NoH_Y);void pm_math_lin_alg_addMatrixVectorProduct(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_addMatrixVectorProductStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_addMatrixTransposeVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_subtractMatrixVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_subtractMatrixVectorProductStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_subtractMatrixTransposeVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);double pm_math_lin_alg_dotProduct(uint32_T n,
const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ);double pm_math_lin_alg_norm(uint32_T n,const
double*pm_math_VgJW5ZqpwPpuY1inYtaofQ);double pm_math_lin_alg_inf_norm(
uint32_T n,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ);double
pm_math_lin_alg_computeQuadraticTerm(uint32_T n,const double*
pm_math__EIaXP4H4ZKuZmaTyPyymE,const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ);void
pm_math_lin_alg_transposeColMajor(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,double*
pm_math_kXii4Miq3Y_ShPn9EHS3D5);void pm_math_lin_alg_transposeRowMajor(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_kXii4Miq3Y_ShPn9EHS3D5);void
pm_math_lin_alg_convertColToRowMajor(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_k_pxeHD3zfxkcatRbNa25_,double*
pm_math_F4CqmafKSNK3_y4WQubdlT);void pm_math_lin_alg_convertRowToColMajor(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F4CqmafKSNK3_y4WQubdlT,double*pm_math_k_pxeHD3zfxkcatRbNa25_);void
pm_math_lin_alg_vectorAdd(uint32_T n,const double*
pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,
double*pm_math_Vzz7fHz6oElvjujXUEM3YM){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a
;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a)*pm_math_Vzz7fHz6oElvjujXUEM3YM++= *
pm_math_kEbBObcYFIxUZ5_77V3CO_++ +*pm_math_VgJW5ZqpwPpuY1inYtaofQ++;}void
pm_math_lin_alg_vectorSubtract(uint32_T n,const double*
pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,
double*pm_math_Vo9I5RFcPal_bXJl4MT4zD){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a
;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a)*pm_math_Vo9I5RFcPal_bXJl4MT4zD++= *
pm_math_kEbBObcYFIxUZ5_77V3CO_++-*pm_math_VgJW5ZqpwPpuY1inYtaofQ++;}void
pm_math_lin_alg_vectorScale(uint32_T n,double pm_math_FQferGZUKft3_i5GvYy4Oy,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*
pm_math_kMXfgG5aJNtdgeZBD1F9s7){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a)*pm_math_kMXfgG5aJNtdgeZBD1F9s7++=
pm_math_FQferGZUKft3_i5GvYy4Oy**pm_math_VgJW5ZqpwPpuY1inYtaofQ++;}void
pm_math_lin_alg_vectorNegate(uint32_T n,const double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*pm_math_Vmg5DyKiTS4YVaa_KIUPx2){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++pm_math_kwrB3ZoKf7OufTHWaHJV7a)*
pm_math_Vmg5DyKiTS4YVaa_KIUPx2++= -*pm_math_VgJW5ZqpwPpuY1inYtaofQ++;}void
pm_math_lin_alg_matrixAssignStrided(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,uint32_T pm_math_FnKRRzrQB6GPWacR5qZ_N1,uint32_T
pm_math_VSkCOkBlf_lY_aOxP7FKDg,const double*pm_math__Q_i1Z0_CMGpfPqpuLrnMT,
double*pm_math_kyfq6L_eQbdYcPuOovpRDW){const size_t
pm_math_VJ5XflXQeVxzfeJc_QsT_V=pm_math_VLHhnPUiNQpve5VIL9P3O9*sizeof(double);
uint32_T pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math__Q_i1Z0_CMGpfPqpuLrnMT+=pm_math_FnKRRzrQB6GPWacR5qZ_N1,
pm_math_kyfq6L_eQbdYcPuOovpRDW+=pm_math_VSkCOkBlf_lY_aOxP7FKDg)memcpy(
pm_math_kyfq6L_eQbdYcPuOovpRDW,pm_math__Q_i1Z0_CMGpfPqpuLrnMT,
pm_math_VJ5XflXQeVxzfeJc_QsT_V);}void pm_math_lin_alg_matrixTransposeAdd(
uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,double*
pm_math__Q_i1Z0_CMGpfPqpuLrnMT){uint32_T pm_math_FFZbGh27ya8eem_J_hUtAZ,
pm_math_kUQBO1dSP8_IVqRAUx4R8G,pm_math_FA31lxz3H2h_iPVaaSpgUt,
pm_math_kc8tbonejwtIhqZehiH_wJ;for(pm_math_FFZbGh27ya8eem_J_hUtAZ=0,
pm_math_FA31lxz3H2h_iPVaaSpgUt=0,pm_math_kc8tbonejwtIhqZehiH_wJ=0;
pm_math_FFZbGh27ya8eem_J_hUtAZ<n;++pm_math_FFZbGh27ya8eem_J_hUtAZ,
pm_math_FA31lxz3H2h_iPVaaSpgUt+=pm_math_FFZbGh27ya8eem_J_hUtAZ,
pm_math_kc8tbonejwtIhqZehiH_wJ=pm_math_FA31lxz3H2h_iPVaaSpgUt){
pm_math__Q_i1Z0_CMGpfPqpuLrnMT[pm_math_FA31lxz3H2h_iPVaaSpgUt]=2.0*
pm_math_F2l4p_g4sn02huHNflQjMH[pm_math_FA31lxz3H2h_iPVaaSpgUt];++
pm_math_FA31lxz3H2h_iPVaaSpgUt,pm_math_kc8tbonejwtIhqZehiH_wJ+=n;for(
pm_math_kUQBO1dSP8_IVqRAUx4R8G=pm_math_FFZbGh27ya8eem_J_hUtAZ+1;
pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++pm_math_kUQBO1dSP8_IVqRAUx4R8G,++
pm_math_FA31lxz3H2h_iPVaaSpgUt,pm_math_kc8tbonejwtIhqZehiH_wJ+=n)
pm_math__Q_i1Z0_CMGpfPqpuLrnMT[pm_math_FA31lxz3H2h_iPVaaSpgUt]=
pm_math__Q_i1Z0_CMGpfPqpuLrnMT[pm_math_kc8tbonejwtIhqZehiH_wJ]=
pm_math_F2l4p_g4sn02huHNflQjMH[pm_math_FA31lxz3H2h_iPVaaSpgUt]+
pm_math_F2l4p_g4sn02huHNflQjMH[pm_math_kc8tbonejwtIhqZehiH_wJ];}}void
pm_math_lin_alg_matrixTransposeSubtract(uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_kyfq6L_eQbdYcPuOovpRDW){uint32_T
pm_math_FFZbGh27ya8eem_J_hUtAZ,pm_math_kUQBO1dSP8_IVqRAUx4R8G,
pm_math_FA31lxz3H2h_iPVaaSpgUt,pm_math_kc8tbonejwtIhqZehiH_wJ;for(
pm_math_FFZbGh27ya8eem_J_hUtAZ=0,pm_math_FA31lxz3H2h_iPVaaSpgUt=0,
pm_math_kc8tbonejwtIhqZehiH_wJ=0;pm_math_FFZbGh27ya8eem_J_hUtAZ<n;++
pm_math_FFZbGh27ya8eem_J_hUtAZ,pm_math_FA31lxz3H2h_iPVaaSpgUt+=
pm_math_FFZbGh27ya8eem_J_hUtAZ,pm_math_kc8tbonejwtIhqZehiH_wJ=
pm_math_FA31lxz3H2h_iPVaaSpgUt){pm_math_kyfq6L_eQbdYcPuOovpRDW[
pm_math_FA31lxz3H2h_iPVaaSpgUt]=0.0;++pm_math_FA31lxz3H2h_iPVaaSpgUt,
pm_math_kc8tbonejwtIhqZehiH_wJ+=n;for(pm_math_kUQBO1dSP8_IVqRAUx4R8G=
pm_math_FFZbGh27ya8eem_J_hUtAZ+1;pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++
pm_math_kUQBO1dSP8_IVqRAUx4R8G,++pm_math_FA31lxz3H2h_iPVaaSpgUt,
pm_math_kc8tbonejwtIhqZehiH_wJ+=n){pm_math_kyfq6L_eQbdYcPuOovpRDW[
pm_math_FA31lxz3H2h_iPVaaSpgUt]=pm_math_F2l4p_g4sn02huHNflQjMH[
pm_math_FA31lxz3H2h_iPVaaSpgUt]-pm_math_F2l4p_g4sn02huHNflQjMH[
pm_math_kc8tbonejwtIhqZehiH_wJ];pm_math_kyfq6L_eQbdYcPuOovpRDW[
pm_math_kc8tbonejwtIhqZehiH_wJ]= -pm_math_kyfq6L_eQbdYcPuOovpRDW[
pm_math_FA31lxz3H2h_iPVaaSpgUt];}}}void pm_math_lin_alg_matrixMultiply(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,const double*
pm_math_Vqiy96WqvuhCaXm5e_vvT0,double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_FQferGZUKft3_i5GvYy4Oy;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_Vqiy96WqvuhCaXm5e_vvT0+=pm_math_V2__YrimeI4E_yWnhKofpy)for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a,++
pm_math_FStFcQlyAJ_dVy4kGZXBPQ){const double*pm_kplAJmOlA30feiNcOzi7oj=
pm_math_F2l4p_g4sn02huHNflQjMH+pm_math_kwrB3ZoKf7OufTHWaHJV7a;*
pm_math_FStFcQlyAJ_dVy4kGZXBPQ=0.0;for(pm_math_FQferGZUKft3_i5GvYy4Oy=0;
pm_math_FQferGZUKft3_i5GvYy4Oy<pm_math_V2__YrimeI4E_yWnhKofpy;++
pm_math_FQferGZUKft3_i5GvYy4Oy,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_VLHhnPUiNQpve5VIL9P3O9,++pm_math_Vqiy96WqvuhCaXm5e_vvT0)*
pm_math_FStFcQlyAJ_dVy4kGZXBPQ+= *pm_kplAJmOlA30feiNcOzi7oj**
pm_math_Vqiy96WqvuhCaXm5e_vvT0;pm_math_Vqiy96WqvuhCaXm5e_vvT0-=
pm_math_V2__YrimeI4E_yWnhKofpy;}}void pm_math_lin_alg_matrixMultiplyStrided(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T n,uint32_T pm_math__8PlegfDdmpxaD3yVABBka,uint32_T
pm_math_khoq8zTqfel4cuvza1xV8U,uint32_T pm_math__Cskocqf_YOea9zRlVhiAe,const
double*pm_math_F2l4p_g4sn02huHNflQjMH,const double*
pm_math_Vqiy96WqvuhCaXm5e_vvT0,double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_FQferGZUKft3_i5GvYy4Oy;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_Vqiy96WqvuhCaXm5e_vvT0+=pm_math_khoq8zTqfel4cuvza1xV8U,
pm_math_FStFcQlyAJ_dVy4kGZXBPQ+=(pm_math__Cskocqf_YOea9zRlVhiAe-
pm_math_VLHhnPUiNQpve5VIL9P3O9))for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a,++pm_math_FStFcQlyAJ_dVy4kGZXBPQ){const double*
pm_kplAJmOlA30feiNcOzi7oj=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;*pm_math_FStFcQlyAJ_dVy4kGZXBPQ=0.0;for(
pm_math_FQferGZUKft3_i5GvYy4Oy=0;pm_math_FQferGZUKft3_i5GvYy4Oy<
pm_math_V2__YrimeI4E_yWnhKofpy;++pm_math_FQferGZUKft3_i5GvYy4Oy,
pm_kplAJmOlA30feiNcOzi7oj+=pm_math__8PlegfDdmpxaD3yVABBka,++
pm_math_Vqiy96WqvuhCaXm5e_vvT0)*pm_math_FStFcQlyAJ_dVy4kGZXBPQ+= *
pm_kplAJmOlA30feiNcOzi7oj**pm_math_Vqiy96WqvuhCaXm5e_vvT0;
pm_math_Vqiy96WqvuhCaXm5e_vvT0-=pm_math_V2__YrimeI4E_yWnhKofpy;}}void
pm_math_lin_alg_matrixTransposeMultiply(uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a
,pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_FQferGZUKft3_i5GvYy4Oy;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_Vqiy96WqvuhCaXm5e_vvT0+=
pm_math_V2__YrimeI4E_yWnhKofpy)for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a,++pm_math_FStFcQlyAJ_dVy4kGZXBPQ){const double*
pm_Fr_bHKkQKFWbfi50VWd5Pw=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a*pm_math_V2__YrimeI4E_yWnhKofpy;*
pm_math_FStFcQlyAJ_dVy4kGZXBPQ=0.0;for(pm_math_FQferGZUKft3_i5GvYy4Oy=0;
pm_math_FQferGZUKft3_i5GvYy4Oy<pm_math_V2__YrimeI4E_yWnhKofpy;++
pm_math_FQferGZUKft3_i5GvYy4Oy,++pm_Fr_bHKkQKFWbfi50VWd5Pw,++
pm_math_Vqiy96WqvuhCaXm5e_vvT0)*pm_math_FStFcQlyAJ_dVy4kGZXBPQ+= *
pm_Fr_bHKkQKFWbfi50VWd5Pw**pm_math_Vqiy96WqvuhCaXm5e_vvT0;
pm_math_Vqiy96WqvuhCaXm5e_vvT0-=pm_math_V2__YrimeI4E_yWnhKofpy;}}void
pm_math_lin_alg_matrixMultiplyTranspose(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9
,uint32_T pm_math_V2__YrimeI4E_yWnhKofpy,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a
,pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_FQferGZUKft3_i5GvYy4Oy;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,++pm_math_Vqiy96WqvuhCaXm5e_vvT0)for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a,++
pm_math_FStFcQlyAJ_dVy4kGZXBPQ){const double*pm_kplAJmOlA30feiNcOzi7oj=
pm_math_F2l4p_g4sn02huHNflQjMH+pm_math_kwrB3ZoKf7OufTHWaHJV7a;*
pm_math_FStFcQlyAJ_dVy4kGZXBPQ=0.0;for(pm_math_FQferGZUKft3_i5GvYy4Oy=0;
pm_math_FQferGZUKft3_i5GvYy4Oy<pm_math_V2__YrimeI4E_yWnhKofpy;++
pm_math_FQferGZUKft3_i5GvYy4Oy,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_VLHhnPUiNQpve5VIL9P3O9,pm_math_Vqiy96WqvuhCaXm5e_vvT0+=n)*
pm_math_FStFcQlyAJ_dVy4kGZXBPQ+= *pm_kplAJmOlA30feiNcOzi7oj**
pm_math_Vqiy96WqvuhCaXm5e_vvT0;pm_math_Vqiy96WqvuhCaXm5e_vvT0-=n*
pm_math_V2__YrimeI4E_yWnhKofpy;}}void pm_math_lin_alg_scaleColumns(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_V5FB4OLCBN0eVqxcC6tIlV){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a
,pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++pm_math_kyp6uAyJE40UVuAQNEYzS1,++
pm_math_FQferGZUKft3_i5GvYy4Oy)for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a)*pm_math_V5FB4OLCBN0eVqxcC6tIlV++= *
pm_math_F2l4p_g4sn02huHNflQjMH++**pm_math_FQferGZUKft3_i5GvYy4Oy;}void
pm_math_lin_alg_scaleRows(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,
const double*pm_math_F2l4p_g4sn02huHNflQjMH,const double*
pm_math_FQferGZUKft3_i5GvYy4Oy,double*pm_math_knjWFknSIL0bYi3EKhrrJ4){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1){for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a)*pm_math_knjWFknSIL0bYi3EKhrrJ4++= *
pm_math_F2l4p_g4sn02huHNflQjMH++**pm_math_FQferGZUKft3_i5GvYy4Oy++;
pm_math_FQferGZUKft3_i5GvYy4Oy-=pm_math_VLHhnPUiNQpve5VIL9P3O9;}}void
pm_math_lin_alg_inverseScaleColumns(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,const double*
pm_math_FQferGZUKft3_i5GvYy4Oy,double*pm_math_V5FB4OLCBN0eVqxcC6tIlV){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,++pm_math_FQferGZUKft3_i5GvYy4Oy)for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a)*
pm_math_V5FB4OLCBN0eVqxcC6tIlV++= *pm_math_F2l4p_g4sn02huHNflQjMH++/ *
pm_math_FQferGZUKft3_i5GvYy4Oy;}void pm_math_lin_alg_inverseScaleRows(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_knjWFknSIL0bYi3EKhrrJ4){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a
,pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++pm_math_kyp6uAyJE40UVuAQNEYzS1){for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a)*
pm_math_knjWFknSIL0bYi3EKhrrJ4++= *pm_math_F2l4p_g4sn02huHNflQjMH++/ *
pm_math_FQferGZUKft3_i5GvYy4Oy++;pm_math_FQferGZUKft3_i5GvYy4Oy-=
pm_math_VLHhnPUiNQpve5VIL9P3O9;}}void pm_math_lin_alg_zeroMajor(uint32_T
pm_math_kRZY2SDSifK6ia_3EbLye7,uint32_T pm_math_FEXg1wwjA_S7cyqxiD_lWO,const
int32_T*pm_math_VU18hM_2GHSHWue20_mnXQ,double*pm_math_F2l4p_g4sn02huHNflQjMH){
uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;if(
pm_math_FEXg1wwjA_S7cyqxiD_lWO==0)return;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_kRZY2SDSifK6ia_3EbLye7;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a){if(*pm_math_VU18hM_2GHSHWue20_mnXQ++==0){for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_FEXg1wwjA_S7cyqxiD_lWO;++pm_math_kyp6uAyJE40UVuAQNEYzS1)*
pm_math_F2l4p_g4sn02huHNflQjMH++=0.0;}else pm_math_F2l4p_g4sn02huHNflQjMH+=
pm_math_FEXg1wwjA_S7cyqxiD_lWO;}}void pm_math_lin_alg_reduceMatrix(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FTSHHVQYO78NWDDPfQSm12,const int32_T*pm_math_VLZhFoWjGJ0Q_DeI0FI6Gm,
double*pm_math_F2l4p_g4sn02huHNflQjMH){if(pm_math_FTSHHVQYO78NWDDPfQSm12<n){
double*dst=pm_math_F2l4p_g4sn02huHNflQjMH;const double*
pm_math_Vs6xonQX17Krc5vXv8T_zq=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_FTSHHVQYO78NWDDPfQSm12*pm_math_VLHhnPUiNQpve5VIL9P3O9;uint32_T
pm_math_FFZbGh27ya8eem_J_hUtAZ;for(pm_math_FFZbGh27ya8eem_J_hUtAZ=0;
pm_math_FFZbGh27ya8eem_J_hUtAZ<pm_math_FTSHHVQYO78NWDDPfQSm12&&(uint32_T)(
pm_math_VLZhFoWjGJ0Q_DeI0FI6Gm[pm_math_FFZbGh27ya8eem_J_hUtAZ])==
pm_math_FFZbGh27ya8eem_J_hUtAZ;++pm_math_FFZbGh27ya8eem_J_hUtAZ){}dst+=
pm_math_FFZbGh27ya8eem_J_hUtAZ*pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math_VLZhFoWjGJ0Q_DeI0FI6Gm+=pm_math_FFZbGh27ya8eem_J_hUtAZ;for(;dst<
pm_math_Vs6xonQX17Krc5vXv8T_zq;dst+=pm_math_VLHhnPUiNQpve5VIL9P3O9)memcpy(dst,
pm_math_F2l4p_g4sn02huHNflQjMH+*pm_math_VLZhFoWjGJ0Q_DeI0FI6Gm++*
pm_math_VLHhnPUiNQpve5VIL9P3O9,pm_math_VLHhnPUiNQpve5VIL9P3O9*sizeof(double));
}}void pm_math_lin_alg_expandVector(uint32_T n,uint32_T
pm_math_kZSgglHk8kWUemlk_e6TDr,const int32_T*pm_math_VD0M3tgCkkSEiHjnw1wnK0,
double*pm_math_VgJW5ZqpwPpuY1inYtaofQ){if(pm_math_kZSgglHk8kWUemlk_e6TDr!=n){
double*dst=pm_math_VgJW5ZqpwPpuY1inYtaofQ+n-1;int32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math_kZSgglHk8kWUemlk_e6TDr-1;pm_math_kwrB3ZoKf7OufTHWaHJV7a!= -1;--
pm_math_kwrB3ZoKf7OufTHWaHJV7a){const int32_T pm_math_kyp6uAyJE40UVuAQNEYzS1=
pm_math_VD0M3tgCkkSEiHjnw1wnK0[pm_math_kwrB3ZoKf7OufTHWaHJV7a];while(dst>
pm_math_VgJW5ZqpwPpuY1inYtaofQ+pm_math_kyp6uAyJE40UVuAQNEYzS1)*dst--=0.0;*dst
--=pm_math_VgJW5ZqpwPpuY1inYtaofQ[pm_math_kwrB3ZoKf7OufTHWaHJV7a];}while(dst>=
pm_math_VgJW5ZqpwPpuY1inYtaofQ)*dst--=0.0;}}static int
pm_math_FoLhSZn5_ExBge8dhw9j_1(const void*pm_math__CopB1Elqbd6X90_N_6bRS,const
void*pm_math_FVu4yWFc9d4ehDv5E9qYRs){const double
pm_math_kSsdzO885HhJeHMABtS2BC=((const double*)pm_math__CopB1Elqbd6X90_N_6bRS)
[0];const double pm_math_ko7kMWny89W6cDu2IplA0_=((const double*)
pm_math_FVu4yWFc9d4ehDv5E9qYRs)[0];if(pm_math_kSsdzO885HhJeHMABtS2BC!=
pm_math_ko7kMWny89W6cDu2IplA0_)return pm_math_kSsdzO885HhJeHMABtS2BC<
pm_math_ko7kMWny89W6cDu2IplA0_?-1:+1;return 0;}static int
pm_math__Y4K59xr3rxYd58mRLekv7(const void*pm_math__CopB1Elqbd6X90_N_6bRS,const
void*pm_math_FVu4yWFc9d4ehDv5E9qYRs){const double
pm_math_khJQLpj2wG_9WXIKVs0_NV=((const double*)pm_math__CopB1Elqbd6X90_N_6bRS)
[1];const double pm_math_FvGQymtSo3CXam_B7eduv_=((const double*)
pm_math_FVu4yWFc9d4ehDv5E9qYRs)[1];const int pm_math_VzjQUCcXrYl1he7quOQNiR=
pm_math_FoLhSZn5_ExBge8dhw9j_1(pm_math__CopB1Elqbd6X90_N_6bRS,
pm_math_FVu4yWFc9d4ehDv5E9qYRs);if(pm_math_VzjQUCcXrYl1he7quOQNiR!=0)return
pm_math_VzjQUCcXrYl1he7quOQNiR;if(pm_math_khJQLpj2wG_9WXIKVs0_NV!=
pm_math_FvGQymtSo3CXam_B7eduv_)return pm_math_khJQLpj2wG_9WXIKVs0_NV<
pm_math_FvGQymtSo3CXam_B7eduv_?-1:+1;return 0;}static int
pm_math__sPT76MDkBWXgDAbunj7cL(const void*pm_math__CopB1Elqbd6X90_N_6bRS,const
void*pm_math_FVu4yWFc9d4ehDv5E9qYRs){const double
pm_math_FL5bp_UEfNxBg9PuTifU9l=((const double*)pm_math__CopB1Elqbd6X90_N_6bRS)
[2];const double pm_math_kq9j_7xAmq4rhDwZFLz4Jx=((const double*)
pm_math_FVu4yWFc9d4ehDv5E9qYRs)[2];const int pm_math_V6ywWkwB0VOHauISfb1xPk=
pm_math__Y4K59xr3rxYd58mRLekv7(pm_math__CopB1Elqbd6X90_N_6bRS,
pm_math_FVu4yWFc9d4ehDv5E9qYRs);if(pm_math_V6ywWkwB0VOHauISfb1xPk!=0)return
pm_math_V6ywWkwB0VOHauISfb1xPk;if(pm_math_FL5bp_UEfNxBg9PuTifU9l!=
pm_math_kq9j_7xAmq4rhDwZFLz4Jx)return pm_math_FL5bp_UEfNxBg9PuTifU9l<
pm_math_kq9j_7xAmq4rhDwZFLz4Jx?-1:+1;return 0;}static int
pm_math_kZQvxYsVMz0za9BP0Hqe2o(const void*pm_math__CopB1Elqbd6X90_N_6bRS,const
void*pm_math_FVu4yWFc9d4ehDv5E9qYRs){int pm_math_kwrB3ZoKf7OufTHWaHJV7a;const
double*pm_math_VNiSkKPmQtlgia6GFipUyI=(const double*)
pm_math__CopB1Elqbd6X90_N_6bRS;const double*pm_math_VoT_RGczZNCSXeXIdoFoz0=(
const double*)pm_math_FVu4yWFc9d4ehDv5E9qYRs;const int
pm_math_kpL6NzuwGohXciuEn1oD5K=(int)pm_math_VNiSkKPmQtlgia6GFipUyI[0];(void)0;
;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=1;pm_math_kwrB3ZoKf7OufTHWaHJV7a<=
pm_math_kpL6NzuwGohXciuEn1oD5K;++pm_math_kwrB3ZoKf7OufTHWaHJV7a)if(
pm_math_VNiSkKPmQtlgia6GFipUyI[pm_math_kwrB3ZoKf7OufTHWaHJV7a]!=
pm_math_VoT_RGczZNCSXeXIdoFoz0[pm_math_kwrB3ZoKf7OufTHWaHJV7a])return
pm_math_VNiSkKPmQtlgia6GFipUyI[pm_math_kwrB3ZoKf7OufTHWaHJV7a]<
pm_math_VoT_RGczZNCSXeXIdoFoz0[pm_math_kwrB3ZoKf7OufTHWaHJV7a]?-1:+1;return 0;
}boolean_T pm_math_lin_alg_hasUniqueRows(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_VRZCD_UL_ESThy75dC9J8D){uint32_T
pm_math__Nbhi7GoB0l3bHuUaflNOK,pm_math_kQFjKSCrXkh8XysrlbuTCa,
pm_math_VcBckTaOApdoiXZKqBr514;int(*pm_math__mErSnV2pYxFViTC19vh58)(const void
*,const void*);if(pm_math_VLHhnPUiNQpve5VIL9P3O9<=1)return true;if(n==0)return
false;pm_math_lin_alg_convertColToRowMajor(pm_math_VLHhnPUiNQpve5VIL9P3O9,n,
pm_math_F2l4p_g4sn02huHNflQjMH,pm_math_VRZCD_UL_ESThy75dC9J8D);if(n>3){double*
dst=pm_math_VRZCD_UL_ESThy75dC9J8D+(pm_math_VLHhnPUiNQpve5VIL9P3O9*(n+1))-1;
double*src=pm_math_VRZCD_UL_ESThy75dC9J8D+(pm_math_VLHhnPUiNQpve5VIL9P3O9*n)-1
;for(pm_math__Nbhi7GoB0l3bHuUaflNOK=pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math__Nbhi7GoB0l3bHuUaflNOK>0;--pm_math__Nbhi7GoB0l3bHuUaflNOK){for(
pm_math_kQFjKSCrXkh8XysrlbuTCa=n;pm_math_kQFjKSCrXkh8XysrlbuTCa>0;--
pm_math_kQFjKSCrXkh8XysrlbuTCa)*dst--= *src--;*dst--=(double)n;}}if(n==1)
pm_math__mErSnV2pYxFViTC19vh58= &pm_math_FoLhSZn5_ExBge8dhw9j_1;else if(n==2)
pm_math__mErSnV2pYxFViTC19vh58= &pm_math__Y4K59xr3rxYd58mRLekv7;else if(n==3)
pm_math__mErSnV2pYxFViTC19vh58= &pm_math__sPT76MDkBWXgDAbunj7cL;else
pm_math__mErSnV2pYxFViTC19vh58= &pm_math_kZQvxYsVMz0za9BP0Hqe2o;
pm_math_VcBckTaOApdoiXZKqBr514=n>3?n+1:n;qsort(pm_math_VRZCD_UL_ESThy75dC9J8D,
pm_math_VLHhnPUiNQpve5VIL9P3O9,pm_math_VcBckTaOApdoiXZKqBr514*sizeof(double),
pm_math__mErSnV2pYxFViTC19vh58);for(pm_math__Nbhi7GoB0l3bHuUaflNOK=0;
pm_math__Nbhi7GoB0l3bHuUaflNOK<pm_math_VLHhnPUiNQpve5VIL9P3O9-1;++
pm_math__Nbhi7GoB0l3bHuUaflNOK)if(pm_math__mErSnV2pYxFViTC19vh58(&
pm_math_VRZCD_UL_ESThy75dC9J8D[pm_math__Nbhi7GoB0l3bHuUaflNOK*
pm_math_VcBckTaOApdoiXZKqBr514],&pm_math_VRZCD_UL_ESThy75dC9J8D[(
pm_math__Nbhi7GoB0l3bHuUaflNOK+1)*pm_math_VcBckTaOApdoiXZKqBr514])==0)return
false;return true;}void pm_math_lin_alg_matrixVectorMultiply(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a,++pm_math_FzyLWRgau0pMYq2XSI3ETL){const double*
pm_kplAJmOlA30feiNcOzi7oj=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;*pm_math_FzyLWRgau0pMYq2XSI3ETL=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_VLHhnPUiNQpve5VIL9P3O9,++x)*pm_math_FzyLWRgau0pMYq2XSI3ETL+= *
pm_kplAJmOlA30feiNcOzi7oj**x;x-=n;}}void
pm_math_lin_alg_matrixVectorMultiplyStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a,++
pm_math_FzyLWRgau0pMYq2XSI3ETL){const double*pm_kplAJmOlA30feiNcOzi7oj=
pm_math_F2l4p_g4sn02huHNflQjMH+pm_math_kwrB3ZoKf7OufTHWaHJV7a;*
pm_math_FzyLWRgau0pMYq2XSI3ETL=0.0;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_kplAJmOlA30feiNcOzi7oj+=pm_math_FxajFA_1TahGZXVH9j9Gmx,++x)*
pm_math_FzyLWRgau0pMYq2XSI3ETL+= *pm_kplAJmOlA30feiNcOzi7oj**x;x-=n;}}void
pm_math_lin_alg_matrixTransposeVectorMultiply(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++pm_math_kwrB3ZoKf7OufTHWaHJV7a,++
pm_math_FzyLWRgau0pMYq2XSI3ETL){*pm_math_FzyLWRgau0pMYq2XSI3ETL=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kyp6uAyJE40UVuAQNEYzS1)*
pm_math_FzyLWRgau0pMYq2XSI3ETL+= *pm_math_F2l4p_g4sn02huHNflQjMH++**x++;x-=
pm_math_VLHhnPUiNQpve5VIL9P3O9;}}void pm_math_lin_alg_addMatrixVectorProduct(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kyp6uAyJE40UVuAQNEYzS1;if(n==0)return;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a){const double*
pm_kplAJmOlA30feiNcOzi7oj=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;double pm_math_V23f63bJK0S_gaBdf6FQ0z=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_VLHhnPUiNQpve5VIL9P3O9,++x)pm_math_V23f63bJK0S_gaBdf6FQ0z+= *
pm_kplAJmOlA30feiNcOzi7oj**x;x-=n;*pm_math_FzyLWRgau0pMYq2XSI3ETL++ +=
pm_math_V23f63bJK0S_gaBdf6FQ0z;}}void
pm_math_lin_alg_addMatrixVectorProductStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;if(n==0)return;
for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a){const double*
pm_kplAJmOlA30feiNcOzi7oj=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;double pm_math_V23f63bJK0S_gaBdf6FQ0z=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_FxajFA_1TahGZXVH9j9Gmx,++x)pm_math_V23f63bJK0S_gaBdf6FQ0z+= *
pm_kplAJmOlA30feiNcOzi7oj**x;x-=n;*pm_math_FzyLWRgau0pMYq2XSI3ETL++ +=
pm_math_V23f63bJK0S_gaBdf6FQ0z;}}void
pm_math_lin_alg_addMatrixTransposeVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kyp6uAyJE40UVuAQNEYzS1;if(pm_math_VLHhnPUiNQpve5VIL9P3O9==0)return;for
(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a){double pm_math_V23f63bJK0S_gaBdf6FQ0z=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kyp6uAyJE40UVuAQNEYzS1)
pm_math_V23f63bJK0S_gaBdf6FQ0z+= *pm_math_F2l4p_g4sn02huHNflQjMH++**x++;x-=
pm_math_VLHhnPUiNQpve5VIL9P3O9;*pm_math_FzyLWRgau0pMYq2XSI3ETL++ +=
pm_math_V23f63bJK0S_gaBdf6FQ0z;}}void
pm_math_lin_alg_subtractMatrixVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kyp6uAyJE40UVuAQNEYzS1;if(n==0)return;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a){const double*
pm_kplAJmOlA30feiNcOzi7oj=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;double pm_math_V23f63bJK0S_gaBdf6FQ0z=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_VLHhnPUiNQpve5VIL9P3O9,++x)pm_math_V23f63bJK0S_gaBdf6FQ0z+= *
pm_kplAJmOlA30feiNcOzi7oj**x;x-=n;*pm_math_FzyLWRgau0pMYq2XSI3ETL++-=
pm_math_V23f63bJK0S_gaBdf6FQ0z;}}void
pm_math_lin_alg_subtractMatrixVectorProductStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;if(n==0)return;
for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kwrB3ZoKf7OufTHWaHJV7a){const double*
pm_kplAJmOlA30feiNcOzi7oj=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;double pm_math_V23f63bJK0S_gaBdf6FQ0z=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;++
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_kplAJmOlA30feiNcOzi7oj+=
pm_math_FxajFA_1TahGZXVH9j9Gmx,++x)pm_math_V23f63bJK0S_gaBdf6FQ0z+= *
pm_kplAJmOlA30feiNcOzi7oj**x;x-=n;*pm_math_FzyLWRgau0pMYq2XSI3ETL++-=
pm_math_V23f63bJK0S_gaBdf6FQ0z;}}void
pm_math_lin_alg_subtractMatrixTransposeVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kyp6uAyJE40UVuAQNEYzS1;if(pm_math_VLHhnPUiNQpve5VIL9P3O9==0)return;for
(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a){double pm_math_V23f63bJK0S_gaBdf6FQ0z=0.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kyp6uAyJE40UVuAQNEYzS1)
pm_math_V23f63bJK0S_gaBdf6FQ0z+= *pm_math_F2l4p_g4sn02huHNflQjMH++**x++;x-=
pm_math_VLHhnPUiNQpve5VIL9P3O9;*pm_math_FzyLWRgau0pMYq2XSI3ETL++-=
pm_math_V23f63bJK0S_gaBdf6FQ0z;}}double pm_math_lin_alg_dotProduct(uint32_T n,
const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a;double
x=0.0;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a)x+= *pm_math_kEbBObcYFIxUZ5_77V3CO_++**
pm_math_VgJW5ZqpwPpuY1inYtaofQ++;return x;}double pm_math_lin_alg_norm(
uint32_T n,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ){double x=0.0;const
double*const pm_math_Vs6xonQX17Krc5vXv8T_zq=pm_math_VgJW5ZqpwPpuY1inYtaofQ+n;
while(pm_math_VgJW5ZqpwPpuY1inYtaofQ<pm_math_Vs6xonQX17Krc5vXv8T_zq){const
double pm_math_FL1llpmubk_sXeD10Sbe3f= *pm_math_VgJW5ZqpwPpuY1inYtaofQ;x+=
pm_math_FL1llpmubk_sXeD10Sbe3f*pm_math_FL1llpmubk_sXeD10Sbe3f;++
pm_math_VgJW5ZqpwPpuY1inYtaofQ;}return sqrt(x);}double pm_math_lin_alg_inf_norm
(uint32_T n,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ){double x=0.0;const
double*const pm_math_Vs6xonQX17Krc5vXv8T_zq=pm_math_VgJW5ZqpwPpuY1inYtaofQ+n;
while(pm_math_VgJW5ZqpwPpuY1inYtaofQ<pm_math_Vs6xonQX17Krc5vXv8T_zq){const
double pm_math_FL1llpmubk_sXeD10Sbe3f=fabs(*pm_math_VgJW5ZqpwPpuY1inYtaofQ);x=
pm_math_FL1llpmubk_sXeD10Sbe3f<x?x:pm_math_FL1llpmubk_sXeD10Sbe3f;++
pm_math_VgJW5ZqpwPpuY1inYtaofQ;}return x;}double
pm_math_lin_alg_computeQuadraticTerm(uint32_T n,const double*
pm_math__EIaXP4H4ZKuZmaTyPyymE,const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ){uint32_T
pm_math_kUQBO1dSP8_IVqRAUx4R8G,pm_math_FFZbGh27ya8eem_J_hUtAZ;double x=0.0;for
(pm_math_FFZbGh27ya8eem_J_hUtAZ=0;pm_math_FFZbGh27ya8eem_J_hUtAZ<n;++
pm_math_FFZbGh27ya8eem_J_hUtAZ){double pm_math_FzyLWRgau0pMYq2XSI3ETL=0.0;for(
pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++
pm_math_kUQBO1dSP8_IVqRAUx4R8G)pm_math_FzyLWRgau0pMYq2XSI3ETL+= *
pm_math__EIaXP4H4ZKuZmaTyPyymE++**pm_math_kEbBObcYFIxUZ5_77V3CO_++;
pm_math_kEbBObcYFIxUZ5_77V3CO_-=n;x+=pm_math_FzyLWRgau0pMYq2XSI3ETL**
pm_math_VgJW5ZqpwPpuY1inYtaofQ++;}return x;}
