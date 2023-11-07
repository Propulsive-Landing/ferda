#ifndef __pm_math_tridiagonal_solve_h__
#define __pm_math_tridiagonal_solve_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
boolean_T pm_math_lin_alg_solveTriDiagPDSymm(real_T*d,real_T*ud,real_T*b,const
uint32_T n);boolean_T pm_math_lin_alg_solveTriDiagSymm(real_T*d,real_T*ud,
real_T*b,const uint32_T n);boolean_T pm_math_lin_alg_solveTriDiagSymmPer(
real_T*d,real_T*ud,double c,real_T*b,const uint32_T n,real_T*w);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_tridiagonal_solve_h__ */
