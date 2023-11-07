#ifndef __sm_geometrySupportFunctions_h__
#define __sm_geometrySupportFunctions_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void polyhedronSupport(const void*geom,const pm_math_Vector3*dir,
pm_math_Vector3*point);void brickSupport(const void*geom,const pm_math_Vector3
*dir,pm_math_Vector3*point);void cylinderSupport(const void*geom,const
pm_math_Vector3*dir,pm_math_Vector3*point);void sphereSupport(const void*geom,
const pm_math_Vector3*dir,pm_math_Vector3*point);void ellipsoidSupport(const
void*geom,const pm_math_Vector3*dir,pm_math_Vector3*point);void revolveSupport
(const void*geom,const pm_math_Vector3*dir,pm_math_Vector3*point);void
diskSupport(const void*geom,const pm_math_Vector3*dir,pm_math_Vector3*point);
void cylinderSupportAnalytical(const void*geom,const pm_math_Vector3*dir,
pm_math_Vector3*point);void sphereSupportAnalytical(const void*geom,const
pm_math_Vector3*dir,pm_math_Vector3*point);void ellipsoidSupportAnalytical(
const void*geom,const pm_math_Vector3*dir,pm_math_Vector3*point);void
pointSupportAnalytical(const void*geom,const pm_math_Vector3*dir,
pm_math_Vector3*point);void revolveSupportAnalytical(const void*geom,const
pm_math_Vector3*dir,pm_math_Vector3*point);void diskSupportAnalytical(const
void*geom,const pm_math_Vector3*dir,pm_math_Vector3*point);real_T
polyhedronScale(const sm_core_compiler_ConvexPolyhedron*cxpoly);real_T
brickScale(const sm_core_compiler_Brick*brick);real_T cylinderScale(const
sm_core_compiler_Cylinder*cylinder);real_T sphereScale(const
sm_core_compiler_Sphere*sphere);real_T ellipsoidScale(const
sm_core_compiler_Ellipsoid*ellipsoid);real_T revolveScale(const
sm_core_compiler_ConvexRevolve*cxrev);void revolveSupportCylindricalCoords(
const sm_core_compiler_ConvexRevolve*cxrev,const pm_math_Vector3*dir,
pm_math_Vector3*point,real_T*radius,real_T*thetaAngle,boolean_T*angleClipped);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_geometrySupportFunctions_h__ */
