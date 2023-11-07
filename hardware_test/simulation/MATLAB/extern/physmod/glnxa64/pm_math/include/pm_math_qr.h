#ifndef __pm_math_qr_h__
#define __pm_math_qr_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void pm_math_lin_alg_qrFactor(uint32_T m,uint32_T n,real_T*A,real_T*qraux,
int32_T*jpvt,real_T*work);uint32_T pm_math_lin_alg_getRankWithTol(const real_T
*qr,uint32_T m,uint32_T n,real_T rel_tol);boolean_T pm_math_lin_alg_qrSolveTall
(uint32_T m,uint32_T n,const real_T*qr,const real_T*qraux,int32_T*jpvt,real_T
relTol,real_T*bx);int32_T pm_math_lin_alg_qrSolveWide(const uint32_T m,const
uint32_T n,const real_T*qr,const real_T*qraux,const int32_T*jpvt,const real_T*
b,real_T relTol,real_T*x);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_qr_h__ */
