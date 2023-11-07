#ifndef __pm_math_svd_h__
#define __pm_math_svd_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void pm_math_lin_alg_svd(uint32_T m,uint32_T n,const real_T*A,boolean_T doFull
,real_T*Sdiag,real_T*U,real_T*V,real_T*work);uint32_T
pm_math_lin_alg_svdReqdWorkSize(uint32_T m,uint32_T n,boolean_T doFull);void
pm_math_lin_alg_svdSolve(uint32_T m,uint32_T n,const real_T*A,const real_T*b,
real_T rankTol,real_T*x,real_T*work);uint32_T
pm_math_lin_alg_svdSolveReqdWorkSize(uint32_T m,uint32_T n);uint32_T
pm_math_lin_alg_svdSolveFromFactorization(uint32_T m,uint32_T n,const real_T*
Sdiag,const real_T*U,const real_T*V,const real_T*b,real_T rankTol,real_T*x,
real_T*work);uint32_T pm_math_lin_alg_svdSolveFromFactorizationReqdWorkSize(
uint32_T m,uint32_T n);uint32_T pm_math_lin_alg_svdSolveAdaptive(uint32_T m,
uint32_T n,const real_T*A,const real_T*b,real_T minRankTol,real_T maxRankTol,
real_T improvementThreshold,real_T*x,real_T*work);uint32_T
pm_math_lin_alg_svdSolveAdaptiveReqdWorkSize(uint32_T m,uint32_T n);uint32_T
pm_math_lin_alg_svdComputeRank(uint32_T m,uint32_T n,const real_T*Sdiag,real_T
rankTol);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_svd_h__ */
