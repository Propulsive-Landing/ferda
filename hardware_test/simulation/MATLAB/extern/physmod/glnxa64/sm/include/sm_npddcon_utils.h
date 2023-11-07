#ifndef __sm_npddcon_utils_h__
#define __sm_npddcon_utils_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void sm_core_NonPlanarDrumDrumConstraint_CompScratchpad(const real_T*
trackedState,const real_T*ib_root,const real_T*jb_root,const real_T*kb_root,
const real_T*if_root,const real_T*jf_root,const real_T*kf_root,const real_T*
delta,real_T rb,real_T rf,real_T sigmaB,real_T sigmaF,real_T*spArray);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_npddcon_utils_h__ */
