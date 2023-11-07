#ifndef __sm_body_algorithms_h__
#define __sm_body_algorithms_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void sm_core_compiler_sortIndices(uint32_T n,const real_T*v,int32_T*iv);
int32_T sm_core_compiler_uniqueTol(uint32_T m,const real_T*u,const int32_T*iu,
real_T tol,real_T*v,int32_T*iv);void sm_core_compiler_subdividePartition(
uint32_T m,const real_T*u,uint32_T n,real_T*v,uint32_T p,int32_T*idxs,real_T*
workr,int32_T*worki);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_body_algorithms_h__ */
