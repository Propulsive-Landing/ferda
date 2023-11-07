#ifndef __sm_funcMinSearch1D_h__
#define __sm_funcMinSearch1D_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct SolutionPairTag{real_T X;real_T FX;};typedef struct SolutionPairTag
SolutionPair;SolutionPair sm_core_FuncMinSearch1D(real_T(*F)(real_T),real_T min
,real_T max);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_funcMinSearch1D_h__ */
