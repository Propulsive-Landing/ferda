#ifndef __sm_smoothStickSlipFrictionLaw_h__
#define __sm_smoothStickSlipFrictionLaw_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
real_T sm_core_compiler_smoothStickSlipMu(real_T vel,real_T coeffDynamicFric,
real_T staticFricMult,real_T velocityDivisor);void
sm_core_compiler_computeSmoothStickSlipConstants(real_T coeffStaticFric,real_T
coeffDynamicFric,real_T criticalVelocity,real_T*staticFricMult,real_T*
velocityDivisor);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_smoothStickSlipFrictionLaw_h__ */
