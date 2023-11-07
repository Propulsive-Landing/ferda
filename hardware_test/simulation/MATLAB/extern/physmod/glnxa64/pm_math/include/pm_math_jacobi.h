#ifndef __pm_math_jacobi_h__
#define __pm_math_jacobi_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void pm_math_lin_alg_jacobi3(const real_T*diag,const real_T*offDiag,real_T*
eigenvalues,real_T*R);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_jacobi_h__ */
