#ifndef __pm_math_eig_h__
#define __pm_math_eig_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void pm_math_lin_alg_eig(uint32_T n,const real_T*A,const real_T*B,real_T*d_re,
real_T*d_im,real_T*V_re,real_T*V_im,real_T*realWork,int32_T*int32Work);
uint32_T pm_math_lin_alg_eigReqdRealWorkSize(uint32_T n);uint32_T
pm_math_lin_alg_eigReqdInt32WorkSize(uint32_T n);boolean_T
pm_math_lin_alg_orthonormalizeRealEigenvectorsWrtSpdB(uint32_T n,const real_T*
B,real_T*V,real_T*work);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_eig_h__ */
