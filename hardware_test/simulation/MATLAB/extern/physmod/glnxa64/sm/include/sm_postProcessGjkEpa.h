#ifndef __sm_postProcessGjkEpa_h__
#define __sm_postProcessGjkEpa_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void improveDiskAccuracy(real_T*min_dist,const sm_core_compiler_Disk*disk,
const pm_math_Transform3*Tdo,const pm_math_Transform3*Tod,pm_math_Vector3*
extremePointDisk,pm_math_Vector3*extremePointOther,pm_math_Vector3*normalDisk,
pm_math_Vector3*normalOther);void improveCylinderAccuracy(real_T*min_dist,
const sm_core_compiler_Cylinder*cylinder,const pm_math_Transform3*Tco,const
pm_math_Transform3*Toc,pm_math_Vector3*extremePointCylinder,pm_math_Vector3*
extremePointOther,pm_math_Vector3*normalCylinder,pm_math_Vector3*normalOther);
void improveSphereAccuracy(real_T*min_dist,const sm_core_compiler_Sphere*
sphere,const pm_math_Transform3*Tso,const pm_math_Transform3*Tos,
pm_math_Vector3*extremePointSphere,pm_math_Vector3*extremePointOther,
pm_math_Vector3*normalSphere,pm_math_Vector3*normalOther);void
improveEllipsoidAccuracy(real_T*min_dist,const sm_core_compiler_Ellipsoid*
ellipsoid,const pm_math_Transform3*Teo,const pm_math_Transform3*Toe,
pm_math_Vector3*extremePointEllipsoid,pm_math_Vector3*extremePointOther,
pm_math_Vector3*normalEllipsoid,pm_math_Vector3*normalOther);boolean_T
improveBrickFaceNormal(const sm_core_compiler_Brick*brick,const
pm_math_Transform3*Tbo,const pm_math_Vector3*extremePointBrick,pm_math_Vector3
*normalBrick,pm_math_Vector3*normalOther);void improveRevolveAccuracy(real_T*
minDist,const sm_core_compiler_ConvexRevolve*cxrev,const pm_math_Transform3*
Tro,const pm_math_Transform3*Tor,pm_math_Vector3*extremePointCxrev,
pm_math_Vector3*extremePointOther,pm_math_Vector3*normalCxrev,pm_math_Vector3*
normalOther);boolean_T improveCylinderCylinderAccuracy(real_T*minDist,const
sm_core_compiler_Cylinder*cylinder0,const sm_core_compiler_Cylinder*cylinder1,
const pm_math_Transform3*T01,const pm_math_Transform3*T10,pm_math_Vector3*
extremePointCylinder0,pm_math_Vector3*extremePointCylinder1,pm_math_Vector3*
normalCylinder0,pm_math_Vector3*normalCYlinder1);real_T DistancePointEllipsoid
(const real_T e0,const real_T e1,const real_T e2,real_T y0,real_T y1,real_T y2
,real_T*x0,real_T*x1,real_T*x2);void normalPointEllipsoid(const
sm_core_compiler_Ellipsoid*ellipsoid,const pm_math_Vector3*extremePoint,
pm_math_Vector3*unitNormal);boolean_T findRevolveRadiusAtPoint(const
sm_core_compiler_ConvexRevolve*rev,const pm_math_Vector3*expR,real_T*radius);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_postProcessGjkEpa_h__ */
