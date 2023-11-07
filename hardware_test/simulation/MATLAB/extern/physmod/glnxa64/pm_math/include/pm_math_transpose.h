#ifndef __pm_math_transpose_h__
#define __pm_math_transpose_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void pm_math_lin_alg_transposeColMajor(uint32_T m,uint32_T n,const double*A,
double*At);void pm_math_lin_alg_transposeRowMajor(uint32_T m,uint32_T n,const
double*A,double*At);void pm_math_lin_alg_convertColToRowMajor(uint32_T m,
uint32_T n,const double*Acol,double*Arow);void
pm_math_lin_alg_convertRowToColMajor(uint32_T m,uint32_T n,const double*Arow,
double*Acol);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_transpose_h__ */
