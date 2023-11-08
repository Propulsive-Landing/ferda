#include "pm_std.h"
#include "string.h"
#include "pm_std.h"
boolean_T pm_math_lin_alg_solveTriDiagPDSymm(real_T*
pm_math_F32Ql82vv6pW_PYIdpkFQ0,real_T*pm_math_k5Jn8vECoSpVb9vqgBv_An,real_T*b,
const uint32_T n);boolean_T pm_math_lin_alg_solveTriDiagSymm(real_T*
pm_math_F32Ql82vv6pW_PYIdpkFQ0,real_T*pm_math_k5Jn8vECoSpVb9vqgBv_An,real_T*b,
const uint32_T n);boolean_T pm_math_lin_alg_solveTriDiagSymmPer(real_T*
pm_math_F32Ql82vv6pW_PYIdpkFQ0,real_T*pm_math_k5Jn8vECoSpVb9vqgBv_An,double
pm_math_FFZbGh27ya8eem_J_hUtAZ,real_T*b,const uint32_T n,real_T*
pm_math_V1pxxydYEnWVVi3QKesjvf);boolean_T pm_math_lin_alg_solveTriDiagPDSymm(
real_T*pm_math_F32Ql82vv6pW_PYIdpkFQ0,real_T*pm_math_k5Jn8vECoSpVb9vqgBv_An,
real_T*b,const uint32_T n){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a;real_T
pm_math__1eAP9V6_J_NYm_1gTIm7A;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=1;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;pm_math_kwrB3ZoKf7OufTHWaHJV7a++){if(
pm_math_F32Ql82vv6pW_PYIdpkFQ0[pm_math_kwrB3ZoKf7OufTHWaHJV7a]==0.0){return
false;}pm_math__1eAP9V6_J_NYm_1gTIm7A=pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1];pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]=pm_math__1eAP9V6_J_NYm_1gTIm7A/
pm_math_F32Ql82vv6pW_PYIdpkFQ0[pm_math_kwrB3ZoKf7OufTHWaHJV7a-1];
pm_math_F32Ql82vv6pW_PYIdpkFQ0[pm_math_kwrB3ZoKf7OufTHWaHJV7a]-=
pm_math__1eAP9V6_J_NYm_1gTIm7A*pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1];}for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=1;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;pm_math_kwrB3ZoKf7OufTHWaHJV7a++){b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]-=pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]*b[pm_math_kwrB3ZoKf7OufTHWaHJV7a-1];}b[n-1]=
b[n-1]/pm_math_F32Ql82vv6pW_PYIdpkFQ0[n-1];for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
n-1;pm_math_kwrB3ZoKf7OufTHWaHJV7a>=1;pm_math_kwrB3ZoKf7OufTHWaHJV7a--){b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]=b[pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]/
pm_math_F32Ql82vv6pW_PYIdpkFQ0[pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]-
pm_math_k5Jn8vECoSpVb9vqgBv_An[pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]*b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a];}return true;}boolean_T
pm_math_lin_alg_solveTriDiagSymm(real_T*pm_math_F32Ql82vv6pW_PYIdpkFQ0,real_T*
pm_math_k5Jn8vECoSpVb9vqgBv_An,real_T*b,const uint32_T n){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a;real_T pm_math__A03UaHSqLpkZTVbcGFDYA=
pm_math_k5Jn8vECoSpVb9vqgBv_An[0];real_T pm_math_Fnc6VeEzOUpB_mSaEMom_J=0;
real_T pm_math_VJSvDW7ZrxCgjDOLUzUAxM;if(pm_math_F32Ql82vv6pW_PYIdpkFQ0[0]==
0.0)return false;pm_math_k5Jn8vECoSpVb9vqgBv_An[0]=
pm_math_k5Jn8vECoSpVb9vqgBv_An[0]/pm_math_F32Ql82vv6pW_PYIdpkFQ0[0];b[0]=b[0]/
pm_math_F32Ql82vv6pW_PYIdpkFQ0[0];for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=1;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n-1;pm_math_kwrB3ZoKf7OufTHWaHJV7a++){if(
pm_math_F32Ql82vv6pW_PYIdpkFQ0[pm_math_kwrB3ZoKf7OufTHWaHJV7a]==0.0)return
false;pm_math_Fnc6VeEzOUpB_mSaEMom_J=pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a];pm_math_VJSvDW7ZrxCgjDOLUzUAxM=
pm_math_F32Ql82vv6pW_PYIdpkFQ0[pm_math_kwrB3ZoKf7OufTHWaHJV7a]-
pm_math__A03UaHSqLpkZTVbcGFDYA*pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1];if(pm_math_VJSvDW7ZrxCgjDOLUzUAxM==0.0)
return false;pm_math_k5Jn8vECoSpVb9vqgBv_An[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=
pm_math_k5Jn8vECoSpVb9vqgBv_An[pm_math_kwrB3ZoKf7OufTHWaHJV7a]/
pm_math_VJSvDW7ZrxCgjDOLUzUAxM;b[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=(b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]-pm_math__A03UaHSqLpkZTVbcGFDYA*b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1])/pm_math_VJSvDW7ZrxCgjDOLUzUAxM;
pm_math__A03UaHSqLpkZTVbcGFDYA=pm_math_Fnc6VeEzOUpB_mSaEMom_J;}
pm_math_VJSvDW7ZrxCgjDOLUzUAxM=pm_math_F32Ql82vv6pW_PYIdpkFQ0[n-1]-
pm_math__A03UaHSqLpkZTVbcGFDYA*pm_math_k5Jn8vECoSpVb9vqgBv_An[n-2];if(
pm_math_VJSvDW7ZrxCgjDOLUzUAxM==0.0)return false;b[n-1]=(b[n-1]-
pm_math__A03UaHSqLpkZTVbcGFDYA*b[n-2])/pm_math_VJSvDW7ZrxCgjDOLUzUAxM;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=n-1;pm_math_kwrB3ZoKf7OufTHWaHJV7a>=1;
pm_math_kwrB3ZoKf7OufTHWaHJV7a--){b[pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]=b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]-pm_math_k5Jn8vECoSpVb9vqgBv_An[
pm_math_kwrB3ZoKf7OufTHWaHJV7a-1]*b[pm_math_kwrB3ZoKf7OufTHWaHJV7a];}return
true;}boolean_T pm_math_lin_alg_solveTriDiagSymmPer(real_T*
pm_math_F32Ql82vv6pW_PYIdpkFQ0,real_T*pm_math_k5Jn8vECoSpVb9vqgBv_An,real_T
pm_math_FFZbGh27ya8eem_J_hUtAZ,real_T*b,const uint32_T n,real_T*
pm_math_V1pxxydYEnWVVi3QKesjvf){uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a;real_T
*pm_math__UTX96WVikWCcT_zOSX142=pm_math_V1pxxydYEnWVVi3QKesjvf+2*n;real_T
pm_math_k_2Wndsk8m00XXzsIt1jhZ,pm_math_koikYkvKukdAWPsKEX6OcA,
pm_math_kyLoqjPgS40JdDStC1rdx7,pm_math_VlemFGFUagC4YmtaYb7Vq8,
pm_math_kA432fLw6etNd1r6_GPTfg,pm_math__ttRIIbEDxdsVmftp6cVSX;real_T
pm_math_VbvZflEkBEGbV9KQXHxqeH,pm_math__pfVi4G4PgpfbyIE_sOfJZ;real_T
pm_math_VJSvDW7ZrxCgjDOLUzUAxM;pm_math_VbvZflEkBEGbV9KQXHxqeH=
pm_math_k5Jn8vECoSpVb9vqgBv_An[n-2];pm_math__pfVi4G4PgpfbyIE_sOfJZ=
pm_math_FFZbGh27ya8eem_J_hUtAZ;memcpy(pm_math_V1pxxydYEnWVVi3QKesjvf,
pm_math_F32Ql82vv6pW_PYIdpkFQ0,n*sizeof(real_T));memcpy(
pm_math_V1pxxydYEnWVVi3QKesjvf+n,pm_math_k5Jn8vECoSpVb9vqgBv_An,(n-1)*sizeof(
real_T));for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n
;pm_math_kwrB3ZoKf7OufTHWaHJV7a++)pm_math__UTX96WVikWCcT_zOSX142[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]=0.0;pm_math__UTX96WVikWCcT_zOSX142[0]=
pm_math_FFZbGh27ya8eem_J_hUtAZ;pm_math__UTX96WVikWCcT_zOSX142[n-2]=
pm_math_k5Jn8vECoSpVb9vqgBv_An[n-2];if(!pm_math_lin_alg_solveTriDiagSymm(
pm_math_F32Ql82vv6pW_PYIdpkFQ0,pm_math_k5Jn8vECoSpVb9vqgBv_An,b,n-1))return
false;if(!pm_math_lin_alg_solveTriDiagSymm(pm_math_V1pxxydYEnWVVi3QKesjvf,
pm_math_V1pxxydYEnWVVi3QKesjvf+n,pm_math__UTX96WVikWCcT_zOSX142,n-1))return
false;pm_math_k_2Wndsk8m00XXzsIt1jhZ=b[0];pm_math_koikYkvKukdAWPsKEX6OcA=b[n-2
];pm_math_kyLoqjPgS40JdDStC1rdx7=pm_math__UTX96WVikWCcT_zOSX142[0];
pm_math_VlemFGFUagC4YmtaYb7Vq8=pm_math__UTX96WVikWCcT_zOSX142[n-2];
pm_math_kA432fLw6etNd1r6_GPTfg=b[n-1];pm_math_VJSvDW7ZrxCgjDOLUzUAxM=(
pm_math_F32Ql82vv6pW_PYIdpkFQ0[n-1]-pm_math__pfVi4G4PgpfbyIE_sOfJZ*
pm_math_kyLoqjPgS40JdDStC1rdx7-pm_math_VbvZflEkBEGbV9KQXHxqeH*
pm_math_VlemFGFUagC4YmtaYb7Vq8);if(pm_math_VJSvDW7ZrxCgjDOLUzUAxM==0.0)return
false;pm_math__ttRIIbEDxdsVmftp6cVSX=(pm_math_kA432fLw6etNd1r6_GPTfg-
pm_math__pfVi4G4PgpfbyIE_sOfJZ*pm_math_k_2Wndsk8m00XXzsIt1jhZ-
pm_math_VbvZflEkBEGbV9KQXHxqeH*pm_math_koikYkvKukdAWPsKEX6OcA)/
pm_math_VJSvDW7ZrxCgjDOLUzUAxM;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n-1;pm_math_kwrB3ZoKf7OufTHWaHJV7a++){b[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]-=pm_math__UTX96WVikWCcT_zOSX142[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]*pm_math__ttRIIbEDxdsVmftp6cVSX;}b[n-1]=
pm_math__ttRIIbEDxdsVmftp6cVSX;return true;}
