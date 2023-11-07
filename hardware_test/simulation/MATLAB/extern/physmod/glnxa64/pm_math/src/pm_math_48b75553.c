#include "pm_std.h"
#include "string.h"
#include "math.h"
#include "pm_std.h"
real_T pm_math_Vector3_dot_ra(const real_T*pm_math__AuaKMC5koOPbXxPi_MZvt,
const real_T*pm_math_kNtPmLll5l8eiqwtk5kfJv);void pm_math_Vector3_cross_ra(
const real_T*pm_math__AuaKMC5koOPbXxPi_MZvt,const real_T*
pm_math_kNtPmLll5l8eiqwtk5kfJv,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void
pm_math_Vector3_compOrthogonalBasis_ra(const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math_kwrB3ZoKf7OufTHWaHJV7a,real_T*
pm_math_kyp6uAyJE40UVuAQNEYzS1,real_T*pm_math_V2__YrimeI4E_yWnhKofpy);void
pm_math_Quaternion_compose_ra(const real_T*pm_math_FbCdebDtDIhMduMKWA8Khy,
const real_T*pm_math_krgkQdg3ZZ_0ded42Fk_8r,real_T*
pm_math__1Zf2IciMRCub1vvbEr1C4);void pm_math_Quaternion_composeInverse_ra(
const real_T*pm_math_FbCdebDtDIhMduMKWA8Khy,const real_T*
pm_math_krgkQdg3ZZ_0ded42Fk_8r,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void
pm_math_Quaternion_inverseCompose_ra(const real_T*
pm_math_FbCdebDtDIhMduMKWA8Khy,const real_T*pm_math_krgkQdg3ZZ_0ded42Fk_8r,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void pm_math_Quaternion_xform_ra(const
real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void
pm_math_Quaternion_inverseXform_ra(const real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb
,const real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*
pm_math__1Zf2IciMRCub1vvbEr1C4);void pm_math_Quaternion_compDeriv_ra(const
real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,const real_T*
pm_math_kvL2QWFrSblY_eznRgCz87,real_T*pm_math_FGJfoQhjnH_kdaWooDAepH);void
pm_math_Quaternion_Matrix3x3Ctor_ra(const real_T*
pm_math_FqUCZrSGGNOuePgRr82o_8,real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb);void
pm_math_Matrix3x3_compose_ra(const real_T*pm_math_Fcuud3IN0odMZi54a1R_8f,const
real_T*pm_math__09m2ugY6U_OXH9Il_a7Bj,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);
void pm_math_Matrix3x3_composeTranspose_ra(const real_T*
pm_math_Fcuud3IN0odMZi54a1R_8f,const real_T*pm_math__09m2ugY6U_OXH9Il_a7Bj,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void
pm_math_Matrix3x3_transposeCompose_ra(const real_T*
pm_math_Fcuud3IN0odMZi54a1R_8f,const real_T*pm_math__09m2ugY6U_OXH9Il_a7Bj,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void pm_math_Matrix3x3_preCross_ra(
const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void
pm_math_Matrix3x3_postCross_ra(const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,
const real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*
pm_math__1Zf2IciMRCub1vvbEr1C4);void pm_math_Matrix3x3_xform_ra(const real_T*
pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void pm_math_Matrix3x3_transposeXform_ra
(const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4);void
pm_math_Matrix3x3_minRotation_ra(const real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,
real_T*pm_math_FqUCZrSGGNOuePgRr82o_8,int pm_math_kbzF46WM0FtKWeE0s7WR95[3]);
real_T pm_math_Vector3_dot_ra(const real_T*pm_math__AuaKMC5koOPbXxPi_MZvt,
const real_T*pm_math_kNtPmLll5l8eiqwtk5kfJv){return
pm_math__AuaKMC5koOPbXxPi_MZvt[0]*pm_math_kNtPmLll5l8eiqwtk5kfJv[0]+
pm_math__AuaKMC5koOPbXxPi_MZvt[1]*pm_math_kNtPmLll5l8eiqwtk5kfJv[1]+
pm_math__AuaKMC5koOPbXxPi_MZvt[2]*pm_math_kNtPmLll5l8eiqwtk5kfJv[2];}void
pm_math_Vector3_cross_ra(const real_T*pm_math__AuaKMC5koOPbXxPi_MZvt,const
real_T*pm_math_kNtPmLll5l8eiqwtk5kfJv,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(
void)0;;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math__AuaKMC5koOPbXxPi_MZvt[1]*
pm_math_kNtPmLll5l8eiqwtk5kfJv[2]-pm_math__AuaKMC5koOPbXxPi_MZvt[2]*
pm_math_kNtPmLll5l8eiqwtk5kfJv[1];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math__AuaKMC5koOPbXxPi_MZvt[2]*pm_math_kNtPmLll5l8eiqwtk5kfJv[0]-
pm_math__AuaKMC5koOPbXxPi_MZvt[0]*pm_math_kNtPmLll5l8eiqwtk5kfJv[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math__AuaKMC5koOPbXxPi_MZvt[0]*
pm_math_kNtPmLll5l8eiqwtk5kfJv[1]-pm_math__AuaKMC5koOPbXxPi_MZvt[1]*
pm_math_kNtPmLll5l8eiqwtk5kfJv[0];}static void pm_math_FvxVpUGZsRt_iyppjQhBrA(
const real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*
pm_math_VhMZ7Balz3hebuYIOnZcwH){const real_T pm_math_Fkbrfbd5DA0yb1kdOaIHtl=
sqrt(pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]);
pm_math_VhMZ7Balz3hebuYIOnZcwH[0]=pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]/
pm_math_Fkbrfbd5DA0yb1kdOaIHtl;pm_math_VhMZ7Balz3hebuYIOnZcwH[1]=
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]/pm_math_Fkbrfbd5DA0yb1kdOaIHtl;
pm_math_VhMZ7Balz3hebuYIOnZcwH[2]=pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]/
pm_math_Fkbrfbd5DA0yb1kdOaIHtl;}void pm_math_Vector3_compOrthogonalBasis_ra(
const real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*
pm_math_kwrB3ZoKf7OufTHWaHJV7a,real_T*pm_math_kyp6uAyJE40UVuAQNEYzS1,real_T*
pm_math_V2__YrimeI4E_yWnhKofpy){real_T pm_math_Fklj3odRoMCGhX9VnS2RBz,
pm_math___TAiJ__Y1OQWD7mG_iJLJ,pm_math_VCj4vmLqKyOyie7D24jNe6;real_T
pm_math_FEa9HpQub3K9X1Fkj0Fvl_[3]={0.0,0.0,0.0};pm_math_FvxVpUGZsRt_iyppjQhBrA
(pm_math_VgJW5ZqpwPpuY1inYtaofQ,pm_math_V2__YrimeI4E_yWnhKofpy);
pm_math_Fklj3odRoMCGhX9VnS2RBz=fabs(pm_math_V2__YrimeI4E_yWnhKofpy[0]);
pm_math___TAiJ__Y1OQWD7mG_iJLJ=fabs(pm_math_V2__YrimeI4E_yWnhKofpy[1]);
pm_math_VCj4vmLqKyOyie7D24jNe6=fabs(pm_math_V2__YrimeI4E_yWnhKofpy[2]);{const
int pm_math_k0_sG8vQ8u0Bc9IThf_p20=(pm_math_Fklj3odRoMCGhX9VnS2RBz>=
pm_math___TAiJ__Y1OQWD7mG_iJLJ)?((pm_math_Fklj3odRoMCGhX9VnS2RBz>
pm_math_VCj4vmLqKyOyie7D24jNe6)?1:0):((pm_math___TAiJ__Y1OQWD7mG_iJLJ>
pm_math_VCj4vmLqKyOyie7D24jNe6)?2:0);pm_math_FEa9HpQub3K9X1Fkj0Fvl_[
pm_math_k0_sG8vQ8u0Bc9IThf_p20]=1.0;}pm_math_Vector3_cross_ra(
pm_math_V2__YrimeI4E_yWnhKofpy,pm_math_FEa9HpQub3K9X1Fkj0Fvl_,
pm_math_kyp6uAyJE40UVuAQNEYzS1);pm_math_FvxVpUGZsRt_iyppjQhBrA(
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_kyp6uAyJE40UVuAQNEYzS1);
pm_math_Vector3_cross_ra(pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_V2__YrimeI4E_yWnhKofpy,pm_math_kwrB3ZoKf7OufTHWaHJV7a);
pm_math_FvxVpUGZsRt_iyppjQhBrA(pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_kwrB3ZoKf7OufTHWaHJV7a);}void pm_math_Quaternion_compose_ra(const
real_T*pm_math_FbCdebDtDIhMduMKWA8Khy,const real_T*
pm_math_krgkQdg3ZZ_0ded42Fk_8r,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;
;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_FbCdebDtDIhMduMKWA8Khy[0]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]-(pm_math_FbCdebDtDIhMduMKWA8Khy[1]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]+pm_math_FbCdebDtDIhMduMKWA8Khy[2]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]+pm_math_FbCdebDtDIhMduMKWA8Khy[3]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]);*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_FbCdebDtDIhMduMKWA8Khy[0]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]+
pm_math_FbCdebDtDIhMduMKWA8Khy[1]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+
pm_math_FbCdebDtDIhMduMKWA8Khy[2]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]-
pm_math_FbCdebDtDIhMduMKWA8Khy[3]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_FbCdebDtDIhMduMKWA8Khy[0]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]+pm_math_FbCdebDtDIhMduMKWA8Khy[2]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+pm_math_FbCdebDtDIhMduMKWA8Khy[3]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]-pm_math_FbCdebDtDIhMduMKWA8Khy[1]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[3];*pm_math__1Zf2IciMRCub1vvbEr1C4=
pm_math_FbCdebDtDIhMduMKWA8Khy[0]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]+
pm_math_FbCdebDtDIhMduMKWA8Khy[3]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+
pm_math_FbCdebDtDIhMduMKWA8Khy[1]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]-
pm_math_FbCdebDtDIhMduMKWA8Khy[2]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[1];}void
pm_math_Quaternion_composeInverse_ra(const real_T*
pm_math_FbCdebDtDIhMduMKWA8Khy,const real_T*pm_math_krgkQdg3ZZ_0ded42Fk_8r,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;;*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_FbCdebDtDIhMduMKWA8Khy[0]*-
pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]-(pm_math_FbCdebDtDIhMduMKWA8Khy[1]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]+pm_math_FbCdebDtDIhMduMKWA8Khy[2]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]+pm_math_FbCdebDtDIhMduMKWA8Khy[3]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]);*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_FbCdebDtDIhMduMKWA8Khy[0]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]-
pm_math_FbCdebDtDIhMduMKWA8Khy[1]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+
pm_math_FbCdebDtDIhMduMKWA8Khy[2]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]-
pm_math_FbCdebDtDIhMduMKWA8Khy[3]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_FbCdebDtDIhMduMKWA8Khy[0]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]-pm_math_FbCdebDtDIhMduMKWA8Khy[2]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+pm_math_FbCdebDtDIhMduMKWA8Khy[3]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]-pm_math_FbCdebDtDIhMduMKWA8Khy[1]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[3];*pm_math__1Zf2IciMRCub1vvbEr1C4=
pm_math_FbCdebDtDIhMduMKWA8Khy[0]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]-
pm_math_FbCdebDtDIhMduMKWA8Khy[3]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+
pm_math_FbCdebDtDIhMduMKWA8Khy[1]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]-
pm_math_FbCdebDtDIhMduMKWA8Khy[2]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[1];}void
pm_math_Quaternion_inverseCompose_ra(const real_T*
pm_math_FbCdebDtDIhMduMKWA8Khy,const real_T*pm_math_krgkQdg3ZZ_0ded42Fk_8r,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;;*
pm_math__1Zf2IciMRCub1vvbEr1C4++= -pm_math_FbCdebDtDIhMduMKWA8Khy[0]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]-(pm_math_FbCdebDtDIhMduMKWA8Khy[1]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]+pm_math_FbCdebDtDIhMduMKWA8Khy[2]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]+pm_math_FbCdebDtDIhMduMKWA8Khy[3]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]);*pm_math__1Zf2IciMRCub1vvbEr1C4++= -
pm_math_FbCdebDtDIhMduMKWA8Khy[0]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]+
pm_math_FbCdebDtDIhMduMKWA8Khy[1]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+
pm_math_FbCdebDtDIhMduMKWA8Khy[2]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]-
pm_math_FbCdebDtDIhMduMKWA8Khy[3]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++= -pm_math_FbCdebDtDIhMduMKWA8Khy[0]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]+pm_math_FbCdebDtDIhMduMKWA8Khy[2]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+pm_math_FbCdebDtDIhMduMKWA8Khy[3]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[1]-pm_math_FbCdebDtDIhMduMKWA8Khy[1]*
pm_math_krgkQdg3ZZ_0ded42Fk_8r[3];*pm_math__1Zf2IciMRCub1vvbEr1C4= -
pm_math_FbCdebDtDIhMduMKWA8Khy[0]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[3]+
pm_math_FbCdebDtDIhMduMKWA8Khy[3]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[0]+
pm_math_FbCdebDtDIhMduMKWA8Khy[1]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[2]-
pm_math_FbCdebDtDIhMduMKWA8Khy[2]*pm_math_krgkQdg3ZZ_0ded42Fk_8r[1];}void
pm_math_Quaternion_xform_ra(const real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,const
real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){
real_T pm_math_FFhjXMhOoO8gZHlh8Hu3K3[3],pm_math_FEcK0zAOzXKSZq2OQse79O[3];
pm_math_Vector3_cross_ra(pm_math_VnD_HGFKVUOWdDAWvZhyEb+1,
pm_math_VgJW5ZqpwPpuY1inYtaofQ,pm_math_FFhjXMhOoO8gZHlh8Hu3K3);
pm_math_Vector3_cross_ra(pm_math_VnD_HGFKVUOWdDAWvZhyEb+1,
pm_math_FFhjXMhOoO8gZHlh8Hu3K3,pm_math_FEcK0zAOzXKSZq2OQse79O);*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+2.0*(
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*pm_math_FFhjXMhOoO8gZHlh8Hu3K3[0]+
pm_math_FEcK0zAOzXKSZq2OQse79O[0]);*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+2.0*(pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*
pm_math_FFhjXMhOoO8gZHlh8Hu3K3[1]+pm_math_FEcK0zAOzXKSZq2OQse79O[1]);*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]+2.0*(
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*pm_math_FFhjXMhOoO8gZHlh8Hu3K3[2]+
pm_math_FEcK0zAOzXKSZq2OQse79O[2]);}void pm_math_Quaternion_inverseXform_ra(
const real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){real_T
pm_math_FFhjXMhOoO8gZHlh8Hu3K3[3],pm_math_FEcK0zAOzXKSZq2OQse79O[3];
pm_math_Vector3_cross_ra(pm_math_VnD_HGFKVUOWdDAWvZhyEb+1,
pm_math_VgJW5ZqpwPpuY1inYtaofQ,pm_math_FFhjXMhOoO8gZHlh8Hu3K3);
pm_math_Vector3_cross_ra(pm_math_VnD_HGFKVUOWdDAWvZhyEb+1,
pm_math_FFhjXMhOoO8gZHlh8Hu3K3,pm_math_FEcK0zAOzXKSZq2OQse79O);*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+2.0*(-
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*pm_math_FFhjXMhOoO8gZHlh8Hu3K3[0]+
pm_math_FEcK0zAOzXKSZq2OQse79O[0]);*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+2.0*(-pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*
pm_math_FFhjXMhOoO8gZHlh8Hu3K3[1]+pm_math_FEcK0zAOzXKSZq2OQse79O[1]);*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]+2.0*(-
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*pm_math_FFhjXMhOoO8gZHlh8Hu3K3[2]+
pm_math_FEcK0zAOzXKSZq2OQse79O[2]);}void pm_math_Quaternion_compDeriv_ra(const
real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,const real_T*
pm_math_kvL2QWFrSblY_eznRgCz87,real_T*pm_math_FGJfoQhjnH_kdaWooDAepH){real_T
pm_math_kgCuYXt6a6hngHkW2WPaRv[3];pm_math_kgCuYXt6a6hngHkW2WPaRv[0]=0.5*
pm_math_kvL2QWFrSblY_eznRgCz87[0];pm_math_kgCuYXt6a6hngHkW2WPaRv[1]=0.5*
pm_math_kvL2QWFrSblY_eznRgCz87[1];pm_math_kgCuYXt6a6hngHkW2WPaRv[2]=0.5*
pm_math_kvL2QWFrSblY_eznRgCz87[2];*pm_math_FGJfoQhjnH_kdaWooDAepH++= -
pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]*pm_math_kgCuYXt6a6hngHkW2WPaRv[0]-
pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]*pm_math_kgCuYXt6a6hngHkW2WPaRv[1]-
pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]*pm_math_kgCuYXt6a6hngHkW2WPaRv[2];*
pm_math_FGJfoQhjnH_kdaWooDAepH++= +pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*
pm_math_kgCuYXt6a6hngHkW2WPaRv[0]-pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]*
pm_math_kgCuYXt6a6hngHkW2WPaRv[1]+pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]*
pm_math_kgCuYXt6a6hngHkW2WPaRv[2];*pm_math_FGJfoQhjnH_kdaWooDAepH++= +
pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]*pm_math_kgCuYXt6a6hngHkW2WPaRv[0]+
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*pm_math_kgCuYXt6a6hngHkW2WPaRv[1]-
pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]*pm_math_kgCuYXt6a6hngHkW2WPaRv[2];*
pm_math_FGJfoQhjnH_kdaWooDAepH= -pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]*
pm_math_kgCuYXt6a6hngHkW2WPaRv[0]+pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]*
pm_math_kgCuYXt6a6hngHkW2WPaRv[1]+pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*
pm_math_kgCuYXt6a6hngHkW2WPaRv[2];}void pm_math_Quaternion_Matrix3x3Ctor_ra(
const real_T*pm_math_FqUCZrSGGNOuePgRr82o_8,real_T*
pm_math_VnD_HGFKVUOWdDAWvZhyEb){const real_T pm_math_VmbGGnawYJ_FY5eg3Q9VOr=
0.25*(pm_math_FqUCZrSGGNOuePgRr82o_8[0]+pm_math_FqUCZrSGGNOuePgRr82o_8[4]+
pm_math_FqUCZrSGGNOuePgRr82o_8[8]+1.0);const real_T
pm_math_VQ_T199jNSCPYuRLcxtO6j=pm_math_VmbGGnawYJ_FY5eg3Q9VOr-0.5*(
pm_math_FqUCZrSGGNOuePgRr82o_8[4]+pm_math_FqUCZrSGGNOuePgRr82o_8[8]);const
real_T pm_math_VkE6m1GxptxxYDqNBGUEZw=pm_math_VmbGGnawYJ_FY5eg3Q9VOr-0.5*(
pm_math_FqUCZrSGGNOuePgRr82o_8[8]+pm_math_FqUCZrSGGNOuePgRr82o_8[0]);const
real_T pm_math_VcAmFCMQr_KodumW__UKW6=pm_math_VmbGGnawYJ_FY5eg3Q9VOr-0.5*(
pm_math_FqUCZrSGGNOuePgRr82o_8[0]+pm_math_FqUCZrSGGNOuePgRr82o_8[4]);real_T
pm_math_kEbBObcYFIxUZ5_77V3CO_=0.0;const int pm_math_kH3bO_YJW98YWa3J39Vj1I=(
pm_math_VmbGGnawYJ_FY5eg3Q9VOr>pm_math_VQ_T199jNSCPYuRLcxtO6j)?((
pm_math_VmbGGnawYJ_FY5eg3Q9VOr>pm_math_VkE6m1GxptxxYDqNBGUEZw)?((
pm_math_VmbGGnawYJ_FY5eg3Q9VOr>pm_math_VcAmFCMQr_KodumW__UKW6)?0:3):((
pm_math_VkE6m1GxptxxYDqNBGUEZw>pm_math_VcAmFCMQr_KodumW__UKW6)?2:3)):((
pm_math_VQ_T199jNSCPYuRLcxtO6j>pm_math_VkE6m1GxptxxYDqNBGUEZw)?((
pm_math_VQ_T199jNSCPYuRLcxtO6j>pm_math_VcAmFCMQr_KodumW__UKW6)?1:3):((
pm_math_VkE6m1GxptxxYDqNBGUEZw>pm_math_VcAmFCMQr_KodumW__UKW6)?2:3));switch(
pm_math_kH3bO_YJW98YWa3J39Vj1I){case 0:pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]=sqrt(
pm_math_VmbGGnawYJ_FY5eg3Q9VOr);pm_math_kEbBObcYFIxUZ5_77V3CO_=0.25/
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0];pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[7]-pm_math_FqUCZrSGGNOuePgRr82o_8[5])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[2]-pm_math_FqUCZrSGGNOuePgRr82o_8[6])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[3]-pm_math_FqUCZrSGGNOuePgRr82o_8[1])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;break;case 1:pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]=
sqrt(pm_math_VQ_T199jNSCPYuRLcxtO6j);pm_math_kEbBObcYFIxUZ5_77V3CO_=0.25/
pm_math_VnD_HGFKVUOWdDAWvZhyEb[1];pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[7]-pm_math_FqUCZrSGGNOuePgRr82o_8[5])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[1]+pm_math_FqUCZrSGGNOuePgRr82o_8[3])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[2]+pm_math_FqUCZrSGGNOuePgRr82o_8[6])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;break;case 2:pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]=
sqrt(pm_math_VkE6m1GxptxxYDqNBGUEZw);pm_math_kEbBObcYFIxUZ5_77V3CO_=0.25/
pm_math_VnD_HGFKVUOWdDAWvZhyEb[2];pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[2]-pm_math_FqUCZrSGGNOuePgRr82o_8[6])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[5]+pm_math_FqUCZrSGGNOuePgRr82o_8[7])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[3]+pm_math_FqUCZrSGGNOuePgRr82o_8[1])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;break;case 3:pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]=
sqrt(pm_math_VcAmFCMQr_KodumW__UKW6);pm_math_kEbBObcYFIxUZ5_77V3CO_=0.25/
pm_math_VnD_HGFKVUOWdDAWvZhyEb[3];pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[3]-pm_math_FqUCZrSGGNOuePgRr82o_8[1])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[6]+pm_math_FqUCZrSGGNOuePgRr82o_8[2])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]=(
pm_math_FqUCZrSGGNOuePgRr82o_8[7]+pm_math_FqUCZrSGGNOuePgRr82o_8[5])*
pm_math_kEbBObcYFIxUZ5_77V3CO_;break;}pm_math_kEbBObcYFIxUZ5_77V3CO_=1.0/sqrt(
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]+
pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]*pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]+
pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]*pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]+
pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]*pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]);
pm_math_VnD_HGFKVUOWdDAWvZhyEb[0]*=pm_math_kEbBObcYFIxUZ5_77V3CO_;
pm_math_VnD_HGFKVUOWdDAWvZhyEb[1]*=pm_math_kEbBObcYFIxUZ5_77V3CO_;
pm_math_VnD_HGFKVUOWdDAWvZhyEb[2]*=pm_math_kEbBObcYFIxUZ5_77V3CO_;
pm_math_VnD_HGFKVUOWdDAWvZhyEb[3]*=pm_math_kEbBObcYFIxUZ5_77V3CO_;}void
pm_math_Matrix3x3_compose_ra(const real_T*pm_math_Fcuud3IN0odMZi54a1R_8f,const
real_T*pm_math__09m2ugY6U_OXH9Il_a7Bj,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(
void)0;;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[0]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+pm_math_Fcuud3IN0odMZi54a1R_8f[1]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+pm_math_Fcuud3IN0odMZi54a1R_8f[2]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[6];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[0]*pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+
pm_math_Fcuud3IN0odMZi54a1R_8f[1]*pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+
pm_math_Fcuud3IN0odMZi54a1R_8f[2]*pm_math__09m2ugY6U_OXH9Il_a7Bj[7];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[0]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[2]+pm_math_Fcuud3IN0odMZi54a1R_8f[1]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[5]+pm_math_Fcuud3IN0odMZi54a1R_8f[2]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[8];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[3]*pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+
pm_math_Fcuud3IN0odMZi54a1R_8f[4]*pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+
pm_math_Fcuud3IN0odMZi54a1R_8f[5]*pm_math__09m2ugY6U_OXH9Il_a7Bj[6];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[3]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+pm_math_Fcuud3IN0odMZi54a1R_8f[4]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+pm_math_Fcuud3IN0odMZi54a1R_8f[5]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[7];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[3]*pm_math__09m2ugY6U_OXH9Il_a7Bj[2]+
pm_math_Fcuud3IN0odMZi54a1R_8f[4]*pm_math__09m2ugY6U_OXH9Il_a7Bj[5]+
pm_math_Fcuud3IN0odMZi54a1R_8f[5]*pm_math__09m2ugY6U_OXH9Il_a7Bj[8];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[6]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+pm_math_Fcuud3IN0odMZi54a1R_8f[7]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+pm_math_Fcuud3IN0odMZi54a1R_8f[8]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[6];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[6]*pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+
pm_math_Fcuud3IN0odMZi54a1R_8f[7]*pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+
pm_math_Fcuud3IN0odMZi54a1R_8f[8]*pm_math__09m2ugY6U_OXH9Il_a7Bj[7];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_Fcuud3IN0odMZi54a1R_8f[6]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[2]+pm_math_Fcuud3IN0odMZi54a1R_8f[7]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[5]+pm_math_Fcuud3IN0odMZi54a1R_8f[8]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[8];}void pm_math_Matrix3x3_composeTranspose_ra(
const real_T*pm_math_Fcuud3IN0odMZi54a1R_8f,const real_T*
pm_math__09m2ugY6U_OXH9Il_a7Bj,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;
;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[0]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+pm_math_Fcuud3IN0odMZi54a1R_8f[1]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+pm_math_Fcuud3IN0odMZi54a1R_8f[2]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[0]*pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+
pm_math_Fcuud3IN0odMZi54a1R_8f[1]*pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+
pm_math_Fcuud3IN0odMZi54a1R_8f[2]*pm_math__09m2ugY6U_OXH9Il_a7Bj[5];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[0]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[6]+pm_math_Fcuud3IN0odMZi54a1R_8f[1]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[7]+pm_math_Fcuud3IN0odMZi54a1R_8f[2]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[8];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[3]*pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+
pm_math_Fcuud3IN0odMZi54a1R_8f[4]*pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+
pm_math_Fcuud3IN0odMZi54a1R_8f[5]*pm_math__09m2ugY6U_OXH9Il_a7Bj[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[3]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+pm_math_Fcuud3IN0odMZi54a1R_8f[4]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+pm_math_Fcuud3IN0odMZi54a1R_8f[5]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[5];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[3]*pm_math__09m2ugY6U_OXH9Il_a7Bj[6]+
pm_math_Fcuud3IN0odMZi54a1R_8f[4]*pm_math__09m2ugY6U_OXH9Il_a7Bj[7]+
pm_math_Fcuud3IN0odMZi54a1R_8f[5]*pm_math__09m2ugY6U_OXH9Il_a7Bj[8];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[6]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+pm_math_Fcuud3IN0odMZi54a1R_8f[7]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+pm_math_Fcuud3IN0odMZi54a1R_8f[8]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[6]*pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+
pm_math_Fcuud3IN0odMZi54a1R_8f[7]*pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+
pm_math_Fcuud3IN0odMZi54a1R_8f[8]*pm_math__09m2ugY6U_OXH9Il_a7Bj[5];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_Fcuud3IN0odMZi54a1R_8f[6]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[6]+pm_math_Fcuud3IN0odMZi54a1R_8f[7]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[7]+pm_math_Fcuud3IN0odMZi54a1R_8f[8]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[8];}void pm_math_Matrix3x3_transposeCompose_ra(
const real_T*pm_math_Fcuud3IN0odMZi54a1R_8f,const real_T*
pm_math__09m2ugY6U_OXH9Il_a7Bj,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;
;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[0]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+pm_math_Fcuud3IN0odMZi54a1R_8f[3]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+pm_math_Fcuud3IN0odMZi54a1R_8f[6]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[6];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[0]*pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+
pm_math_Fcuud3IN0odMZi54a1R_8f[3]*pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+
pm_math_Fcuud3IN0odMZi54a1R_8f[6]*pm_math__09m2ugY6U_OXH9Il_a7Bj[7];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[0]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[2]+pm_math_Fcuud3IN0odMZi54a1R_8f[3]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[5]+pm_math_Fcuud3IN0odMZi54a1R_8f[6]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[8];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[1]*pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+
pm_math_Fcuud3IN0odMZi54a1R_8f[4]*pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+
pm_math_Fcuud3IN0odMZi54a1R_8f[7]*pm_math__09m2ugY6U_OXH9Il_a7Bj[6];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[1]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+pm_math_Fcuud3IN0odMZi54a1R_8f[4]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+pm_math_Fcuud3IN0odMZi54a1R_8f[7]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[7];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[1]*pm_math__09m2ugY6U_OXH9Il_a7Bj[2]+
pm_math_Fcuud3IN0odMZi54a1R_8f[4]*pm_math__09m2ugY6U_OXH9Il_a7Bj[5]+
pm_math_Fcuud3IN0odMZi54a1R_8f[7]*pm_math__09m2ugY6U_OXH9Il_a7Bj[8];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_Fcuud3IN0odMZi54a1R_8f[2]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[0]+pm_math_Fcuud3IN0odMZi54a1R_8f[5]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[3]+pm_math_Fcuud3IN0odMZi54a1R_8f[8]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[6];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_Fcuud3IN0odMZi54a1R_8f[2]*pm_math__09m2ugY6U_OXH9Il_a7Bj[1]+
pm_math_Fcuud3IN0odMZi54a1R_8f[5]*pm_math__09m2ugY6U_OXH9Il_a7Bj[4]+
pm_math_Fcuud3IN0odMZi54a1R_8f[8]*pm_math__09m2ugY6U_OXH9Il_a7Bj[7];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_Fcuud3IN0odMZi54a1R_8f[2]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[2]+pm_math_Fcuud3IN0odMZi54a1R_8f[5]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[5]+pm_math_Fcuud3IN0odMZi54a1R_8f[8]*
pm_math__09m2ugY6U_OXH9Il_a7Bj[8];}void pm_math_Matrix3x3_preCross_ra(const
real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;
;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[6]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]-pm_math_F2l4p_g4sn02huHNflQjMH[3]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[7]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]-
pm_math_F2l4p_g4sn02huHNflQjMH[4]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[8]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]-pm_math_F2l4p_g4sn02huHNflQjMH[5]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[0]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]-
pm_math_F2l4p_g4sn02huHNflQjMH[6]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[1]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]-pm_math_F2l4p_g4sn02huHNflQjMH[7]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[2]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]-
pm_math_F2l4p_g4sn02huHNflQjMH[8]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[3]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]-pm_math_F2l4p_g4sn02huHNflQjMH[0]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[4]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]-
pm_math_F2l4p_g4sn02huHNflQjMH[1]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_F2l4p_g4sn02huHNflQjMH[5]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]-pm_math_F2l4p_g4sn02huHNflQjMH[2]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1];}void pm_math_Matrix3x3_postCross_ra(const
real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;
;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[1]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]-pm_math_F2l4p_g4sn02huHNflQjMH[2]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[2]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]-
pm_math_F2l4p_g4sn02huHNflQjMH[0]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[0]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]-pm_math_F2l4p_g4sn02huHNflQjMH[1]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[4]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]-
pm_math_F2l4p_g4sn02huHNflQjMH[5]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[5]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]-pm_math_F2l4p_g4sn02huHNflQjMH[3]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[3]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]-
pm_math_F2l4p_g4sn02huHNflQjMH[4]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[7]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2]-pm_math_F2l4p_g4sn02huHNflQjMH[8]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[8]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]-
pm_math_F2l4p_g4sn02huHNflQjMH[6]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_F2l4p_g4sn02huHNflQjMH[6]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]-pm_math_F2l4p_g4sn02huHNflQjMH[7]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0];}void pm_math_Matrix3x3_xform_ra(const real_T
*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*pm_math_VgJW5ZqpwPpuY1inYtaofQ,
real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;;*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[0]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+pm_math_F2l4p_g4sn02huHNflQjMH[1]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+pm_math_F2l4p_g4sn02huHNflQjMH[2]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[3]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+
pm_math_F2l4p_g4sn02huHNflQjMH[4]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+
pm_math_F2l4p_g4sn02huHNflQjMH[5]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[6]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+pm_math_F2l4p_g4sn02huHNflQjMH[7]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+pm_math_F2l4p_g4sn02huHNflQjMH[8]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];}void pm_math_Matrix3x3_transposeXform_ra(
const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,real_T*pm_math__1Zf2IciMRCub1vvbEr1C4){(void)0;
;*pm_math__1Zf2IciMRCub1vvbEr1C4++=pm_math_F2l4p_g4sn02huHNflQjMH[0]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+pm_math_F2l4p_g4sn02huHNflQjMH[3]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+pm_math_F2l4p_g4sn02huHNflQjMH[6]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*pm_math__1Zf2IciMRCub1vvbEr1C4++=
pm_math_F2l4p_g4sn02huHNflQjMH[1]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+
pm_math_F2l4p_g4sn02huHNflQjMH[4]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+
pm_math_F2l4p_g4sn02huHNflQjMH[7]*pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];*
pm_math__1Zf2IciMRCub1vvbEr1C4=pm_math_F2l4p_g4sn02huHNflQjMH[2]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[0]+pm_math_F2l4p_g4sn02huHNflQjMH[5]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[1]+pm_math_F2l4p_g4sn02huHNflQjMH[8]*
pm_math_VgJW5ZqpwPpuY1inYtaofQ[2];}static real_T pm_math_V_FoLVGO37G4eXJuucTSjm
(const real_T*pm_math_F2l4p_g4sn02huHNflQjMH){const real_T*a=
pm_math_F2l4p_g4sn02huHNflQjMH;const real_T*b=pm_math_F2l4p_g4sn02huHNflQjMH+3
;const real_T*pm_math_FFZbGh27ya8eem_J_hUtAZ=pm_math_F2l4p_g4sn02huHNflQjMH+6;
real_T pm_math__8EEqwe2wQpGa1Zrsu6MRI[3];pm_math_Vector3_cross_ra(b,
pm_math_FFZbGh27ya8eem_J_hUtAZ,pm_math__8EEqwe2wQpGa1Zrsu6MRI);return
pm_math_Vector3_dot_ra(a,pm_math__8EEqwe2wQpGa1Zrsu6MRI);}void
pm_math_Matrix3x3_minRotation_ra(const real_T*pm_math_VnD_HGFKVUOWdDAWvZhyEb,
real_T*pm_math_FqUCZrSGGNOuePgRr82o_8,int pm_math_kbzF46WM0FtKWeE0s7WR95[3]){
const boolean_T pm_math_kA1JM95JsV0RcLxI4SAZNK=pm_math_V_FoLVGO37G4eXJuucTSjm(
pm_math_VnD_HGFKVUOWdDAWvZhyEb)<0.0;const int pm_math_V5sGO9ANMS0VVLkblQ_gRK[6
][3]={{0,1,2},{1,2,0},{2,0,1},{0,2,1},{2,1,0},{1,0,2}};const int*
pm_math_VdLWATHg4Lt5gihTpf28WR=NULL;int pm_math_VR3Gzdw6XzCOZ1z6An_brA=0;
real_T pm_math_FoqvJs6gbxKlbP5pszn7ig= -4.0;int pm_math_kwrB3ZoKf7OufTHWaHJV7a
;int pm_math_krYzdgglpzS1gus79MQZpM;for(pm_math_krYzdgglpzS1gus79MQZpM=0;
pm_math_krYzdgglpzS1gus79MQZpM<6;++pm_math_krYzdgglpzS1gus79MQZpM){const int*
pm__lqjegyKuwStj56WZLiC_e=pm_math_V5sGO9ANMS0VVLkblQ_gRK[
pm_math_krYzdgglpzS1gus79MQZpM];const boolean_T pm_math_VDv36i3RLWK7VqXFxwtULD
=pm_math_krYzdgglpzS1gus79MQZpM>=3;int pm_math__Ff9JZhSNPK7VeK_UaPbQQ;for(
pm_math__Ff9JZhSNPK7VeK_UaPbQQ=0;pm_math__Ff9JZhSNPK7VeK_UaPbQQ<8;++
pm_math__Ff9JZhSNPK7VeK_UaPbQQ){const boolean_T pm_math_ktbvjqcPhWdOj9Vukr1STa
=((pm_math__Ff9JZhSNPK7VeK_UaPbQQ+(pm_math__Ff9JZhSNPK7VeK_UaPbQQ>>1)+(
pm_math__Ff9JZhSNPK7VeK_UaPbQQ>>2))&1)==1;real_T pm_math_Vgyn9_XGvLSUaauCGjY4RB
=0.0;if((pm_math_VDv36i3RLWK7VqXFxwtULD^pm_math_ktbvjqcPhWdOj9Vukr1STa)!=
pm_math_kA1JM95JsV0RcLxI4SAZNK)continue;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<3;++pm_math_kwrB3ZoKf7OufTHWaHJV7a){const real_T
pm_math_F32Ql82vv6pW_PYIdpkFQ0=pm_math_VnD_HGFKVUOWdDAWvZhyEb[
pm__lqjegyKuwStj56WZLiC_e[pm_math_kwrB3ZoKf7OufTHWaHJV7a]+3*
pm_math_kwrB3ZoKf7OufTHWaHJV7a];pm_math_Vgyn9_XGvLSUaauCGjY4RB+=(
pm_math__Ff9JZhSNPK7VeK_UaPbQQ&(1<<pm_math_kwrB3ZoKf7OufTHWaHJV7a))==0?+
pm_math_F32Ql82vv6pW_PYIdpkFQ0:-pm_math_F32Ql82vv6pW_PYIdpkFQ0;}if(
pm_math_Vgyn9_XGvLSUaauCGjY4RB>pm_math_FoqvJs6gbxKlbP5pszn7ig){
pm_math_VdLWATHg4Lt5gihTpf28WR=pm__lqjegyKuwStj56WZLiC_e;
pm_math_VR3Gzdw6XzCOZ1z6An_brA=pm_math__Ff9JZhSNPK7VeK_UaPbQQ;
pm_math_FoqvJs6gbxKlbP5pszn7ig=pm_math_Vgyn9_XGvLSUaauCGjY4RB;}}}memcpy(
pm_math_kbzF46WM0FtKWeE0s7WR95,pm_math_VdLWATHg4Lt5gihTpf28WR,3*sizeof(int));{
real_T pm_math_Fk2O4u6vQUpibmbv8Kjgnn[9];if(pm_math_FqUCZrSGGNOuePgRr82o_8==
pm_math_VnD_HGFKVUOWdDAWvZhyEb){memcpy(pm_math_Fk2O4u6vQUpibmbv8Kjgnn,
pm_math_VnD_HGFKVUOWdDAWvZhyEb,9*sizeof(real_T));
pm_math_VnD_HGFKVUOWdDAWvZhyEb=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;}
pm_math_FqUCZrSGGNOuePgRr82o_8[0]=pm_math_VnD_HGFKVUOWdDAWvZhyEb[
pm_math_kbzF46WM0FtKWeE0s7WR95[0]+0],pm_math_FqUCZrSGGNOuePgRr82o_8[3]=
pm_math_VnD_HGFKVUOWdDAWvZhyEb[pm_math_kbzF46WM0FtKWeE0s7WR95[0]+3],
pm_math_FqUCZrSGGNOuePgRr82o_8[6]=pm_math_VnD_HGFKVUOWdDAWvZhyEb[
pm_math_kbzF46WM0FtKWeE0s7WR95[0]+6];pm_math_FqUCZrSGGNOuePgRr82o_8[1]=
pm_math_VnD_HGFKVUOWdDAWvZhyEb[pm_math_kbzF46WM0FtKWeE0s7WR95[1]+0],
pm_math_FqUCZrSGGNOuePgRr82o_8[4]=pm_math_VnD_HGFKVUOWdDAWvZhyEb[
pm_math_kbzF46WM0FtKWeE0s7WR95[1]+3],pm_math_FqUCZrSGGNOuePgRr82o_8[7]=
pm_math_VnD_HGFKVUOWdDAWvZhyEb[pm_math_kbzF46WM0FtKWeE0s7WR95[1]+6];
pm_math_FqUCZrSGGNOuePgRr82o_8[2]=pm_math_VnD_HGFKVUOWdDAWvZhyEb[
pm_math_kbzF46WM0FtKWeE0s7WR95[2]+0],pm_math_FqUCZrSGGNOuePgRr82o_8[5]=
pm_math_VnD_HGFKVUOWdDAWvZhyEb[pm_math_kbzF46WM0FtKWeE0s7WR95[2]+3],
pm_math_FqUCZrSGGNOuePgRr82o_8[8]=pm_math_VnD_HGFKVUOWdDAWvZhyEb[
pm_math_kbzF46WM0FtKWeE0s7WR95[2]+6];}for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<3;++pm_math_kwrB3ZoKf7OufTHWaHJV7a)if((
pm_math_VR3Gzdw6XzCOZ1z6An_brA&(1<<pm_math_kwrB3ZoKf7OufTHWaHJV7a))!=0){real_T
*pm_math_FL1llpmubk_sXeD10Sbe3f=pm_math_FqUCZrSGGNOuePgRr82o_8+
pm_math_kwrB3ZoKf7OufTHWaHJV7a;pm_math_FL1llpmubk_sXeD10Sbe3f[0]= -
pm_math_FL1llpmubk_sXeD10Sbe3f[0];pm_math_FL1llpmubk_sXeD10Sbe3f[3]= -
pm_math_FL1llpmubk_sXeD10Sbe3f[3];pm_math_FL1llpmubk_sXeD10Sbe3f[6]= -
pm_math_FL1llpmubk_sXeD10Sbe3f[6];}}
