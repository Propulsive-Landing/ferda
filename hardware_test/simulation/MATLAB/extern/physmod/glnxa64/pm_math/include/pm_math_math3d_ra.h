#ifndef __pm_math_math3d_ra_h__
#define __pm_math_math3d_ra_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
real_T pm_math_Vector3_dot_ra(const real_T*v0,const real_T*v1);void
pm_math_Vector3_cross_ra(const real_T*v0,const real_T*v1,real_T*result);void
pm_math_Vector3_compOrthogonalBasis_ra(const real_T*v,real_T*i,real_T*j,real_T
*k);void pm_math_Quaternion_compose_ra(const real_T*Q0,const real_T*Q1,real_T*
result);void pm_math_Quaternion_composeInverse_ra(const real_T*Q0,const real_T
*Q1,real_T*result);void pm_math_Quaternion_inverseCompose_ra(const real_T*Q0,
const real_T*Q1,real_T*result);void pm_math_Quaternion_xform_ra(const real_T*Q
,const real_T*v,real_T*result);void pm_math_Quaternion_inverseXform_ra(const
real_T*Q,const real_T*v,real_T*result);void pm_math_Quaternion_compDeriv_ra(
const real_T*Q,const real_T*omega,real_T*deriv);void
pm_math_Quaternion_Matrix3x3Ctor_ra(const real_T*R,real_T*Q);void
pm_math_Matrix3x3_compose_ra(const real_T*A0,const real_T*A1,real_T*result);
void pm_math_Matrix3x3_composeTranspose_ra(const real_T*A0,const real_T*A1,
real_T*result);void pm_math_Matrix3x3_transposeCompose_ra(const real_T*A0,
const real_T*A1,real_T*result);void pm_math_Matrix3x3_preCross_ra(const real_T
*A,const real_T*v,real_T*result);void pm_math_Matrix3x3_postCross_ra(const
real_T*A,const real_T*v,real_T*result);void pm_math_Matrix3x3_xform_ra(const
real_T*A,const real_T*v,real_T*result);void pm_math_Matrix3x3_transposeXform_ra
(const real_T*A,const real_T*v,real_T*result);void
pm_math_Matrix3x3_minRotation_ra(const real_T*Q,real_T*R,int perm[3]);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __pm_math_math3d_ra_h__ */
