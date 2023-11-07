#ifndef __sm_mechanicsMath_h__
#define __sm_mechanicsMath_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void sm_core_math_computeIndicesNonzeroElements(uint32_T n,const int32_T*u,
int32_T*v);boolean_T sm_core_math_anyIsInf(uint32_T n,const real_T*v);
boolean_T sm_core_math_anyIsNaN(uint32_T n,const real_T*v);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_mechanicsMath_h__ */
