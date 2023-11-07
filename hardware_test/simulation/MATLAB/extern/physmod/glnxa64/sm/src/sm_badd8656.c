#include "pm_std.h"
#include "math.h"
#include "string.h"
#include "stddef.h"
#include "pm_std.h"
void sm_core_math_validNormal(double*sm_Vr9KldtodP0_Va38fjNnWL,const double*
sm_V91rYIvDBkGRba7MvGcPUK,const int sm_F9XXYvbW1X_RXP6D2uUU48);void
sm_core_math_castelEval(const size_t n,const size_t sm_F9XXYvbW1X_RXP6D2uUU48,
const double*sm_FzyLWRgau0pMYq2XSI3ETL,const double sm_kEbBObcYFIxUZ5_77V3CO_,
double*sm_Fbnx9FOLhnW0X1Wd233Ai7);int sm_core_math_indexOfLowerBound(const
size_t n,const double*sm__BS3Tlzr_NtKXuzA0AvWWK,const double
sm_kEbBObcYFIxUZ5_77V3CO_);double sm_core_math_findClosestPointOnBezier(const
size_t sm_FruBC65sPgSaZ5e_Y8cbTC,const size_t sm_F9XXYvbW1X_RXP6D2uUU48,const
double*sm_kTtwL9vkis0Rj11z__fIfM,const double*sm__Fx3AcPRGGpDfPGz1V3ouw,const
double*sm__coIYFxX3sdUZi7GOyTTHD,const double*pm__lqjegyKuwStj56WZLiC_e,double
*sm_FXT4jPWj6stSWaiL5dan3b);double sm_core_math_findClosestPointOnSpline(const
size_t sm_FruBC65sPgSaZ5e_Y8cbTC,const size_t sm_F9XXYvbW1X_RXP6D2uUU48,const
size_t sm_FgdMfCu3V48Waqkzyywcw6,const unsigned char sm__4eZvqLS4xSK_LaNVwLvKT
,int sm_Fz3gDBTbnAdRjexjCHxc_s,const double*sm__C5iw_XK_207dqezwnl6D0,const
double*sm_kTtwL9vkis0Rj11z__fIfM,const double*sm_kU3wi6a_0id2_uiGo0WkXI,const
double*sm__Fx3AcPRGGpDfPGz1V3ouw,const double*sm__coIYFxX3sdUZi7GOyTTHD,const
double*sm_VZrAo961Ev_Ij5VUbwCEtu,double*sm_kp1vFBczgUK0deS6edHd0E);double
sm__2OZs4W2TKx3Xm44SRHYDa(const double*a,const double*b,const size_t
sm_F9XXYvbW1X_RXP6D2uUU48){size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;double
sm__OFtIlKN2Yt8VqNJE2GwhU=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kwrB3ZoKf7OufTHWaHJV7a
++){sm__OFtIlKN2Yt8VqNJE2GwhU+=a[sm_kwrB3ZoKf7OufTHWaHJV7a]*b[
sm_kwrB3ZoKf7OufTHWaHJV7a];}return sm__OFtIlKN2Yt8VqNJE2GwhU;}void
sm_FR9_rqYpgXp_V1OzlcZOmm(double*a,const size_t sm_F9XXYvbW1X_RXP6D2uUU48){
size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kwrB3ZoKf7OufTHWaHJV7a
++){a[sm_kwrB3ZoKf7OufTHWaHJV7a]=0.0;}}void sm_FH112C98EMKI_9byfNwnne(double*a
,const double*b,const double sm_FQferGZUKft3_i5GvYy4Oy,const size_t
sm_F9XXYvbW1X_RXP6D2uUU48){size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(
sm_kwrB3ZoKf7OufTHWaHJV7a;sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;
sm_kwrB3ZoKf7OufTHWaHJV7a++){a[sm_kwrB3ZoKf7OufTHWaHJV7a]=
sm_FQferGZUKft3_i5GvYy4Oy*b[sm_kwrB3ZoKf7OufTHWaHJV7a];}}void
sm_kqG7yu8iQLhFWyc4rARxlo(double*a,const double*b,const double
sm_FQferGZUKft3_i5GvYy4Oy,const size_t sm_F9XXYvbW1X_RXP6D2uUU48){size_t
sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kwrB3ZoKf7OufTHWaHJV7a
++){a[sm_kwrB3ZoKf7OufTHWaHJV7a]+=sm_FQferGZUKft3_i5GvYy4Oy*b[
sm_kwrB3ZoKf7OufTHWaHJV7a];}}double sm__3gTK5cN1E_7_eKMQ9Uy_u(const double*a,
const size_t sm_F9XXYvbW1X_RXP6D2uUU48){double sm_VRYYo80g8Gt_ZyEKRXCsw_=0;
size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kwrB3ZoKf7OufTHWaHJV7a
++){sm_VRYYo80g8Gt_ZyEKRXCsw_+=a[sm_kwrB3ZoKf7OufTHWaHJV7a]*a[
sm_kwrB3ZoKf7OufTHWaHJV7a];}sm_VRYYo80g8Gt_ZyEKRXCsw_=sqrt(
sm_VRYYo80g8Gt_ZyEKRXCsw_);return sm_VRYYo80g8Gt_ZyEKRXCsw_;}double
sm_V6WG9QFC01O2aTR8QNBvNg(const double*a,const double*b,const size_t
sm_F9XXYvbW1X_RXP6D2uUU48){double sm_VRYYo80g8Gt_ZyEKRXCsw_=0;size_t
sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kwrB3ZoKf7OufTHWaHJV7a
++){double sm_F32Ql82vv6pW_PYIdpkFQ0=a[sm_kwrB3ZoKf7OufTHWaHJV7a]-b[
sm_kwrB3ZoKf7OufTHWaHJV7a];sm_VRYYo80g8Gt_ZyEKRXCsw_+=
sm_F32Ql82vv6pW_PYIdpkFQ0*sm_F32Ql82vv6pW_PYIdpkFQ0;}sm_VRYYo80g8Gt_ZyEKRXCsw_
=sqrt(sm_VRYYo80g8Gt_ZyEKRXCsw_);return sm_VRYYo80g8Gt_ZyEKRXCsw_;}void
sm_V33S2jgq4A_ffmu7BVqFTK(double*a,const double*b,const double*
sm_FFZbGh27ya8eem_J_hUtAZ,const size_t sm_F9XXYvbW1X_RXP6D2uUU48){size_t
sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kwrB3ZoKf7OufTHWaHJV7a
++){a[sm_kwrB3ZoKf7OufTHWaHJV7a]=b[sm_kwrB3ZoKf7OufTHWaHJV7a]-
sm_FFZbGh27ya8eem_J_hUtAZ[sm_kwrB3ZoKf7OufTHWaHJV7a];}}void
sm_V8aO54_tT9dNgXBLjsemo2(double*sm_FFZbGh27ya8eem_J_hUtAZ,const double*a,
const double*b){sm_FFZbGh27ya8eem_J_hUtAZ[0]=a[1]*b[2]-a[2]*b[1];
sm_FFZbGh27ya8eem_J_hUtAZ[1]=a[2]*b[0]-a[0]*b[2];sm_FFZbGh27ya8eem_J_hUtAZ[2]=
a[0]*b[1]-a[1]*b[0];}void sm_k5mh49edWDlpg181nZgGhF(double*
sm_FVG9mW_iV00Sd5jvvtdbAo,const size_t pm_FW8nEXbTFjdJhTIKepsgFT,const size_t
pm_FsQ9LRKHRTKdgeP2xWDL07,const size_t sm_VYCTO0OcYBG6WayE_bxS2Q,const double*
sm_FawmFe5oOEC_cmkvZc1Kaa){size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<
pm_FsQ9LRKHRTKdgeP2xWDL07;sm_kwrB3ZoKf7OufTHWaHJV7a++){
sm_FVG9mW_iV00Sd5jvvtdbAo[sm_kwrB3ZoKf7OufTHWaHJV7a*pm_FW8nEXbTFjdJhTIKepsgFT+
sm_VYCTO0OcYBG6WayE_bxS2Q]=sm_FawmFe5oOEC_cmkvZc1Kaa[sm_kwrB3ZoKf7OufTHWaHJV7a
];}}boolean_T sm_kU5zplToL1GFZDH91lQdmx(const size_t n,const double*
sm_FzyLWRgau0pMYq2XSI3ETL,const double sm_FxVx8bRexTl3hDBcGo5O3w){const double
sm_k2hnl_VYjJ45gu25Izyvuo=sm_FzyLWRgau0pMYq2XSI3ETL[0];const double
sm_kVyfVJ6Osq4VcDVUUFt5p5=sm_FzyLWRgau0pMYq2XSI3ETL[n-1];const double
sm_VLHhnPUiNQpve5VIL9P3O9=sm_kVyfVJ6Osq4VcDVUUFt5p5-sm_k2hnl_VYjJ45gu25Izyvuo;
const double sm_VJSvDW7ZrxCgjDOLUzUAxM=sqrt(sm_VLHhnPUiNQpve5VIL9P3O9*
sm_VLHhnPUiNQpve5VIL9P3O9+1);size_t sm_V2__YrimeI4E_yWnhKofpy=1;for(
sm_V2__YrimeI4E_yWnhKofpy=1;sm_V2__YrimeI4E_yWnhKofpy<n;
sm_V2__YrimeI4E_yWnhKofpy++){const double sm_VJ9uUjrFrLtaYqDR1PEPSW=((double)
sm_V2__YrimeI4E_yWnhKofpy)/(n-1.0);const double sm_kiBL1YkUNVG1VyHm3DftAw=fabs
(sm_FzyLWRgau0pMYq2XSI3ETL[sm_V2__YrimeI4E_yWnhKofpy]-
sm_VLHhnPUiNQpve5VIL9P3O9*sm_VJ9uUjrFrLtaYqDR1PEPSW-sm_k2hnl_VYjJ45gu25Izyvuo)
/sm_VJSvDW7ZrxCgjDOLUzUAxM;if(sm_kiBL1YkUNVG1VyHm3DftAw>
sm_FxVx8bRexTl3hDBcGo5O3w)return false;}return true;}double
sm_kyU1cflJQfp9YXibuT1d41(const size_t n,const double*
sm_FzyLWRgau0pMYq2XSI3ETL,const double sm_kEbBObcYFIxUZ5_77V3CO_){static double
sm_Vsnaqb_jwyprXqg3IoFhUA[][11]={{1.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
,{1.0,3.0,3.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{1.0,4.0,6.0,4.0,1.0,0.0,0.0,
0.0,0.0,0.0,0.0},{1.0,5.0,10.0,10.0,5.0,1.0,0.0,0.0,0.0,0.0,0.0},{1.0,6.0,15.0
,20.0,15.0,6.0,1.0,0.0,0.0,0.0,0.0},{1.0,7.0,21.0,35.0,35.0,21.0,7.0,1.0,0.0,
0.0,0.0},{1.0,8.0,28.0,56.0,70.0,56.0,28.0,8.0,1.0,0.0,0.0},{1.0,9.0,36.0,84.0
,126.0,126.0,84.0,36.0,9.0,1.0,0.0},{1.0,10.0,45.0,120.0,210.0,252.0,210.0,
120.0,45.0,10.0,1.0}};size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;size_t
sm_kNbvLmFI_EtAay5OpgNJFz=(n-3);double sm_FkicuWr3zNG_VHwcasJgXR=1.0-
sm_kEbBObcYFIxUZ5_77V3CO_;double sm_Vjy0KawG6hdMVaVqNR12oq=pow(
sm_FkicuWr3zNG_VHwcasJgXR,(double)n-1);double sm_Vj5sbPGWtkW1ju9uQCxxOH=
sm_kEbBObcYFIxUZ5_77V3CO_;double sm_VH3Zs0SzEDWpWu5Hr39bVU=
sm_Vjy0KawG6hdMVaVqNR12oq*sm_FzyLWRgau0pMYq2XSI3ETL[0];if(
sm_FkicuWr3zNG_VHwcasJgXR!=0.0&&sm_kEbBObcYFIxUZ5_77V3CO_!=0.0){for(
sm_kwrB3ZoKf7OufTHWaHJV7a=1;sm_kwrB3ZoKf7OufTHWaHJV7a<n-1;
sm_kwrB3ZoKf7OufTHWaHJV7a++){sm_Vjy0KawG6hdMVaVqNR12oq/=
sm_FkicuWr3zNG_VHwcasJgXR;sm_VH3Zs0SzEDWpWu5Hr39bVU+=sm_Vsnaqb_jwyprXqg3IoFhUA
[sm_kNbvLmFI_EtAay5OpgNJFz][sm_kwrB3ZoKf7OufTHWaHJV7a]*
sm_Vjy0KawG6hdMVaVqNR12oq*sm_Vj5sbPGWtkW1ju9uQCxxOH*sm_FzyLWRgau0pMYq2XSI3ETL[
sm_kwrB3ZoKf7OufTHWaHJV7a];sm_Vj5sbPGWtkW1ju9uQCxxOH*=
sm_kEbBObcYFIxUZ5_77V3CO_;}}sm_VH3Zs0SzEDWpWu5Hr39bVU+=
sm_Vj5sbPGWtkW1ju9uQCxxOH*sm_FzyLWRgau0pMYq2XSI3ETL[n-1];return
sm_VH3Zs0SzEDWpWu5Hr39bVU;}void sm_core_math_castelEval(const size_t n,const
size_t sm_F9XXYvbW1X_RXP6D2uUU48,const double*sm_FzyLWRgau0pMYq2XSI3ETL,const
double sm_kEbBObcYFIxUZ5_77V3CO_,double*sm_Fbnx9FOLhnW0X1Wd233Ai7){if(
sm_kEbBObcYFIxUZ5_77V3CO_==0.0){memcpy(sm_Fbnx9FOLhnW0X1Wd233Ai7,
sm_FzyLWRgau0pMYq2XSI3ETL,sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));}else if(
sm_kEbBObcYFIxUZ5_77V3CO_==1.0||n==1){memcpy(sm_Fbnx9FOLhnW0X1Wd233Ai7,
sm_FzyLWRgau0pMYq2XSI3ETL+(n-1)*sm_F9XXYvbW1X_RXP6D2uUU48,
sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));}else{double
sm_VH3Zs0SzEDWpWu5Hr39bVU[99];size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0,
sm_kyp6uAyJE40UVuAQNEYzS1=0,sm_V2__YrimeI4E_yWnhKofpy=0;const double
sm_FkicuWr3zNG_VHwcasJgXR=1.0-sm_kEbBObcYFIxUZ5_77V3CO_;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n-1;
sm_kwrB3ZoKf7OufTHWaHJV7a++){for(sm_kyp6uAyJE40UVuAQNEYzS1=0;
sm_kyp6uAyJE40UVuAQNEYzS1<sm_F9XXYvbW1X_RXP6D2uUU48;sm_kyp6uAyJE40UVuAQNEYzS1
++){sm_VH3Zs0SzEDWpWu5Hr39bVU[sm_kwrB3ZoKf7OufTHWaHJV7a*
sm_F9XXYvbW1X_RXP6D2uUU48+sm_kyp6uAyJE40UVuAQNEYzS1]=sm_FkicuWr3zNG_VHwcasJgXR
*sm_FzyLWRgau0pMYq2XSI3ETL[sm_kwrB3ZoKf7OufTHWaHJV7a*sm_F9XXYvbW1X_RXP6D2uUU48
+sm_kyp6uAyJE40UVuAQNEYzS1]+sm_kEbBObcYFIxUZ5_77V3CO_*
sm_FzyLWRgau0pMYq2XSI3ETL[(sm_kwrB3ZoKf7OufTHWaHJV7a+1)*
sm_F9XXYvbW1X_RXP6D2uUU48+sm_kyp6uAyJE40UVuAQNEYzS1];}}for(
sm_V2__YrimeI4E_yWnhKofpy=n-2;sm_V2__YrimeI4E_yWnhKofpy>0;
sm_V2__YrimeI4E_yWnhKofpy--){for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_V2__YrimeI4E_yWnhKofpy;sm_kwrB3ZoKf7OufTHWaHJV7a
++){for(sm_kyp6uAyJE40UVuAQNEYzS1=0;sm_kyp6uAyJE40UVuAQNEYzS1<
sm_F9XXYvbW1X_RXP6D2uUU48;sm_kyp6uAyJE40UVuAQNEYzS1++){
sm_VH3Zs0SzEDWpWu5Hr39bVU[sm_kwrB3ZoKf7OufTHWaHJV7a*sm_F9XXYvbW1X_RXP6D2uUU48+
sm_kyp6uAyJE40UVuAQNEYzS1]=sm_FkicuWr3zNG_VHwcasJgXR*sm_VH3Zs0SzEDWpWu5Hr39bVU
[sm_kwrB3ZoKf7OufTHWaHJV7a*sm_F9XXYvbW1X_RXP6D2uUU48+sm_kyp6uAyJE40UVuAQNEYzS1
]+sm_kEbBObcYFIxUZ5_77V3CO_*sm_VH3Zs0SzEDWpWu5Hr39bVU[(
sm_kwrB3ZoKf7OufTHWaHJV7a+1)*sm_F9XXYvbW1X_RXP6D2uUU48+
sm_kyp6uAyJE40UVuAQNEYzS1];}}}memcpy(sm_Fbnx9FOLhnW0X1Wd233Ai7,
sm_VH3Zs0SzEDWpWu5Hr39bVU,sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));}}double
sm_FZwwajt1zMSqi1HID2Kvml(const size_t n,double*sm_FzyLWRgau0pMYq2XSI3ETL,
const double sm_FxVx8bRexTl3hDBcGo5O3w){const size_t sm__NbjXUeO_bdXjLOXjp08yZ
=50;size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0;double sm__1eAP9V6_J_NYm_1gTIm7A=0.5;
double sm_VHnvbMkfc3p_Z13e_ytkks=0.0,sm_VH3Zs0SzEDWpWu5Hr39bVU=0.0,
sm_kLKYzmYft1Scbykv_UbF5S=0.0;double sm_F8VrXMVRQbh6dT2VAEm8u5[10];(void)0;;if
(sm_FzyLWRgau0pMYq2XSI3ETL[0]*sm_FzyLWRgau0pMYq2XSI3ETL[n-1]<0.0){
sm__1eAP9V6_J_NYm_1gTIm7A=sm_FzyLWRgau0pMYq2XSI3ETL[0]/(
sm_FzyLWRgau0pMYq2XSI3ETL[0]-sm_FzyLWRgau0pMYq2XSI3ETL[n-1]);(void)0;;}for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n-1;++
sm_kwrB3ZoKf7OufTHWaHJV7a)sm_F8VrXMVRQbh6dT2VAEm8u5[sm_kwrB3ZoKf7OufTHWaHJV7a]
=((double)n)*(sm_FzyLWRgau0pMYq2XSI3ETL[sm_kwrB3ZoKf7OufTHWaHJV7a+1]-
sm_FzyLWRgau0pMYq2XSI3ETL[sm_kwrB3ZoKf7OufTHWaHJV7a]);sm_core_math_castelEval(
n,1,sm_FzyLWRgau0pMYq2XSI3ETL,sm__1eAP9V6_J_NYm_1gTIm7A,&
sm_VH3Zs0SzEDWpWu5Hr39bVU);sm_core_math_castelEval(n-1,1,
sm_F8VrXMVRQbh6dT2VAEm8u5,sm__1eAP9V6_J_NYm_1gTIm7A,&sm_kLKYzmYft1Scbykv_UbF5S
);if(fabs(sm_VH3Zs0SzEDWpWu5Hr39bVU)>2.0*fabs(sm_kLKYzmYft1Scbykv_UbF5S))
return-1.0;sm_VHnvbMkfc3p_Z13e_ytkks=sm_VH3Zs0SzEDWpWu5Hr39bVU/
sm_kLKYzmYft1Scbykv_UbF5S;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm__NbjXUeO_bdXjLOXjp08yZ;sm_kwrB3ZoKf7OufTHWaHJV7a
++){if(fabs(sm_VHnvbMkfc3p_Z13e_ytkks)<=sm_FxVx8bRexTl3hDBcGo5O3w){break;}
sm__1eAP9V6_J_NYm_1gTIm7A-=sm_VHnvbMkfc3p_Z13e_ytkks;if(
sm__1eAP9V6_J_NYm_1gTIm7A<-1.0||sm__1eAP9V6_J_NYm_1gTIm7A>2.0)return-1.0;
sm_core_math_castelEval(n,1,sm_FzyLWRgau0pMYq2XSI3ETL,
sm__1eAP9V6_J_NYm_1gTIm7A,&sm_VH3Zs0SzEDWpWu5Hr39bVU);sm_core_math_castelEval(
n-1,1,sm_F8VrXMVRQbh6dT2VAEm8u5,sm__1eAP9V6_J_NYm_1gTIm7A,&
sm_kLKYzmYft1Scbykv_UbF5S);if(fabs(sm_VH3Zs0SzEDWpWu5Hr39bVU)>2.0*fabs(
sm_kLKYzmYft1Scbykv_UbF5S))return-1.0;sm_VHnvbMkfc3p_Z13e_ytkks=
sm_VH3Zs0SzEDWpWu5Hr39bVU/sm_kLKYzmYft1Scbykv_UbF5S;}return(
sm_kwrB3ZoKf7OufTHWaHJV7a<sm__NbjXUeO_bdXjLOXjp08yZ)?sm__1eAP9V6_J_NYm_1gTIm7A
:-1.0;}void sm_FKp_ekXsPJtwaup5CTUoOJ(const size_t n,const size_t
sm_kwrB3ZoKf7OufTHWaHJV7a,const double sm_kGFBA5N_QsxsZybUalQ5a2,const double
sm_VPmi9Pi_E8WnWumMULHOT_,double*sm_FWrSwPm1wudqcibPkToJ9D,double*
sm__hQaUQnP4t8b_eCsoixA1g){size_t sm_kyp6uAyJE40UVuAQNEYzS1;
sm__hQaUQnP4t8b_eCsoixA1g[(n-1)-(sm_kwrB3ZoKf7OufTHWaHJV7a-1)]=
sm_FWrSwPm1wudqcibPkToJ9D[n-1];if(sm_kwrB3ZoKf7OufTHWaHJV7a==n){return;}for(
sm_kyp6uAyJE40UVuAQNEYzS1=n-1;sm_kyp6uAyJE40UVuAQNEYzS1>=
sm_kwrB3ZoKf7OufTHWaHJV7a;sm_kyp6uAyJE40UVuAQNEYzS1--){
sm_FWrSwPm1wudqcibPkToJ9D[sm_kyp6uAyJE40UVuAQNEYzS1]=sm_VPmi9Pi_E8WnWumMULHOT_
*sm_FWrSwPm1wudqcibPkToJ9D[sm_kyp6uAyJE40UVuAQNEYzS1-1]+
sm_kGFBA5N_QsxsZybUalQ5a2*sm_FWrSwPm1wudqcibPkToJ9D[sm_kyp6uAyJE40UVuAQNEYzS1]
;}sm_FKp_ekXsPJtwaup5CTUoOJ(n,sm_kwrB3ZoKf7OufTHWaHJV7a+1,
sm_kGFBA5N_QsxsZybUalQ5a2,sm_VPmi9Pi_E8WnWumMULHOT_,sm_FWrSwPm1wudqcibPkToJ9D,
sm__hQaUQnP4t8b_eCsoixA1g);}void sm__mPTwl1kUoKmeqGBJwenxc(const size_t n,
const double sm_kGFBA5N_QsxsZybUalQ5a2,double*sm_FWrSwPm1wudqcibPkToJ9D,double
*sm__hQaUQnP4t8b_eCsoixA1g){sm_FKp_ekXsPJtwaup5CTUoOJ(n,1,
sm_kGFBA5N_QsxsZybUalQ5a2,1.0-sm_kGFBA5N_QsxsZybUalQ5a2,
sm_FWrSwPm1wudqcibPkToJ9D,sm__hQaUQnP4t8b_eCsoixA1g);}size_t
sm_FJRnDQB_gb_MhHhvTtm3yD(const size_t n,const double*
sm_FzyLWRgau0pMYq2XSI3ETL){size_t sm_kwrB3ZoKf7OufTHWaHJV7a=0,
sm_kyp6uAyJE40UVuAQNEYzS1=1;size_t sm__73DLohsLGGXW1jMg6ZSQ_=0;while(
sm_kyp6uAyJE40UVuAQNEYzS1<n){if(sm_FzyLWRgau0pMYq2XSI3ETL[
sm_kyp6uAyJE40UVuAQNEYzS1]==0){sm_kyp6uAyJE40UVuAQNEYzS1++;continue;}if(
sm_FzyLWRgau0pMYq2XSI3ETL[sm_kwrB3ZoKf7OufTHWaHJV7a]*sm_FzyLWRgau0pMYq2XSI3ETL
[sm_kyp6uAyJE40UVuAQNEYzS1]<0)sm__73DLohsLGGXW1jMg6ZSQ_++;
sm_kwrB3ZoKf7OufTHWaHJV7a=sm_kyp6uAyJE40UVuAQNEYzS1;sm_kyp6uAyJE40UVuAQNEYzS1
++;}return sm__73DLohsLGGXW1jMg6ZSQ_;}int sm__ZclC0am9m0aXT_4J934Bs(const
size_t n,unsigned int sm_FcDayhBcBzKeePNPSptq10,double*
sm_FzyLWRgau0pMYq2XSI3ETL,const double sm_kX_3wpayBuGfayGjLiZlmB,const double
sm_FkicuWr3zNG_VHwcasJgXR,const double sm_FxVx8bRexTl3hDBcGo5O3w,double*
sm_FW6qHWPVaIK3geHEb4DSvL){size_t sm__73DLohsLGGXW1jMg6ZSQ_=0,
sm_VND14ejye_86WmozrNaSGJ=0;sm__73DLohsLGGXW1jMg6ZSQ_=
sm_FJRnDQB_gb_MhHhvTtm3yD(n,sm_FzyLWRgau0pMYq2XSI3ETL);if(
sm__73DLohsLGGXW1jMg6ZSQ_==0)return 0;if(sm__73DLohsLGGXW1jMg6ZSQ_==1&&
sm_FcDayhBcBzKeePNPSptq10>=4){const double sm_kEbBObcYFIxUZ5_77V3CO_=
sm_FZwwajt1zMSqi1HID2Kvml(n,sm_FzyLWRgau0pMYq2XSI3ETL,
sm_FxVx8bRexTl3hDBcGo5O3w);if(sm_kEbBObcYFIxUZ5_77V3CO_>=0.0&&
sm_kEbBObcYFIxUZ5_77V3CO_<=1.0){*sm_FW6qHWPVaIK3geHEb4DSvL++=
sm_kX_3wpayBuGfayGjLiZlmB+sm_kEbBObcYFIxUZ5_77V3CO_*(sm_FkicuWr3zNG_VHwcasJgXR
-sm_kX_3wpayBuGfayGjLiZlmB);++sm_VND14ejye_86WmozrNaSGJ;}else if(
sm_FcDayhBcBzKeePNPSptq10>16){sm__73DLohsLGGXW1jMg6ZSQ_=0;}}if(
sm_VND14ejye_86WmozrNaSGJ<sm__73DLohsLGGXW1jMg6ZSQ_){const double
sm_kGFBA5N_QsxsZybUalQ5a2=(sm_kX_3wpayBuGfayGjLiZlmB+sm_FkicuWr3zNG_VHwcasJgXR
)/2.0;int sm_FJ4C7kuNFD_kcXt1_x85B3=0;double sm__hQaUQnP4t8b_eCsoixA1g[11];(
void)0;;sm__mPTwl1kUoKmeqGBJwenxc(n,0.5,sm_FzyLWRgau0pMYq2XSI3ETL,
sm__hQaUQnP4t8b_eCsoixA1g);sm_FJ4C7kuNFD_kcXt1_x85B3=sm__ZclC0am9m0aXT_4J934Bs
(n,sm_FcDayhBcBzKeePNPSptq10+1,sm_FzyLWRgau0pMYq2XSI3ETL,
sm_kX_3wpayBuGfayGjLiZlmB,sm_kGFBA5N_QsxsZybUalQ5a2,sm_FxVx8bRexTl3hDBcGo5O3w,
sm_FW6qHWPVaIK3geHEb4DSvL);sm_VND14ejye_86WmozrNaSGJ=sm_FJ4C7kuNFD_kcXt1_x85B3
;sm_FW6qHWPVaIK3geHEb4DSvL+=sm_FJ4C7kuNFD_kcXt1_x85B3;if(
sm_VND14ejye_86WmozrNaSGJ<sm__73DLohsLGGXW1jMg6ZSQ_){sm_FJ4C7kuNFD_kcXt1_x85B3
=sm__ZclC0am9m0aXT_4J934Bs(n,sm_FcDayhBcBzKeePNPSptq10+1,
sm__hQaUQnP4t8b_eCsoixA1g,sm_kGFBA5N_QsxsZybUalQ5a2,sm_FkicuWr3zNG_VHwcasJgXR,
sm_FxVx8bRexTl3hDBcGo5O3w,sm_FW6qHWPVaIK3geHEb4DSvL);sm_VND14ejye_86WmozrNaSGJ
+=sm_FJ4C7kuNFD_kcXt1_x85B3;sm_FW6qHWPVaIK3geHEb4DSvL+=
sm_FJ4C7kuNFD_kcXt1_x85B3;if(sm_VND14ejye_86WmozrNaSGJ<
sm__73DLohsLGGXW1jMg6ZSQ_&&fabs(sm__hQaUQnP4t8b_eCsoixA1g[0])<
sm_FxVx8bRexTl3hDBcGo5O3w&&fabs(sm__hQaUQnP4t8b_eCsoixA1g[1])>
sm_FxVx8bRexTl3hDBcGo5O3w&&fabs(sm_FzyLWRgau0pMYq2XSI3ETL[n-2])>
sm_FxVx8bRexTl3hDBcGo5O3w){*sm_FW6qHWPVaIK3geHEb4DSvL++=
sm_kGFBA5N_QsxsZybUalQ5a2;++sm_VND14ejye_86WmozrNaSGJ;}}}return(int)
sm_VND14ejye_86WmozrNaSGJ;}int sm__3DuFGHQFu07jmbdOsGWQP(const double*
sm__C5iw_XK_207dqezwnl6D0,const double sm_kEbBObcYFIxUZ5_77V3CO_,const int
sm_Fa35fYhbZA03ZipctuX5h1,const int sm__NbjXUeO_bdXjLOXjp08yZ){int
sm_k_i_xEyf1J8qfXP9AlvysW=sm_Fa35fYhbZA03ZipctuX5h1+((
sm__NbjXUeO_bdXjLOXjp08yZ-sm_Fa35fYhbZA03ZipctuX5h1)/2);if((
sm__C5iw_XK_207dqezwnl6D0[sm_k_i_xEyf1J8qfXP9AlvysW]==
sm_kEbBObcYFIxUZ5_77V3CO_)||(sm_k_i_xEyf1J8qfXP9AlvysW==
sm_Fa35fYhbZA03ZipctuX5h1)){return sm_k_i_xEyf1J8qfXP9AlvysW;}else if(
sm__C5iw_XK_207dqezwnl6D0[sm_k_i_xEyf1J8qfXP9AlvysW]<sm_kEbBObcYFIxUZ5_77V3CO_
){return sm__3DuFGHQFu07jmbdOsGWQP(sm__C5iw_XK_207dqezwnl6D0,
sm_kEbBObcYFIxUZ5_77V3CO_,sm_k_i_xEyf1J8qfXP9AlvysW,sm__NbjXUeO_bdXjLOXjp08yZ)
;}else{return sm__3DuFGHQFu07jmbdOsGWQP(sm__C5iw_XK_207dqezwnl6D0,
sm_kEbBObcYFIxUZ5_77V3CO_,sm_Fa35fYhbZA03ZipctuX5h1,sm_k_i_xEyf1J8qfXP9AlvysW)
;}}int sm_core_math_indexOfLowerBound(const size_t n,const double*
sm__BS3Tlzr_NtKXuzA0AvWWK,const double sm_kEbBObcYFIxUZ5_77V3CO_){const int
sm_VLHhnPUiNQpve5VIL9P3O9=(int)n;if(sm_kEbBObcYFIxUZ5_77V3CO_>
sm__BS3Tlzr_NtKXuzA0AvWWK[sm_VLHhnPUiNQpve5VIL9P3O9-1]){return
sm_VLHhnPUiNQpve5VIL9P3O9-1;}else if(sm_kEbBObcYFIxUZ5_77V3CO_<
sm__BS3Tlzr_NtKXuzA0AvWWK[0]){return-1;}else{return sm__3DuFGHQFu07jmbdOsGWQP(
sm__BS3Tlzr_NtKXuzA0AvWWK,sm_kEbBObcYFIxUZ5_77V3CO_,0,
sm_VLHhnPUiNQpve5VIL9P3O9-1);}}double sm_kNKPUVafBMtJWLgQGKOQes(const size_t
sm_VY7k8tVKVTWabyr34z3F_c,const size_t sm_F9XXYvbW1X_RXP6D2uUU48,const double*
sm_kcga7oDUd_KJZ5ep8UQIl2,const int sm_V78_jX3vbP8Fd1_V6QmV3A,const double*
sm_VeDJvqSU0ktAWXfSef07BJ,const double*sm_VZrAo961Ev_Ij5VUbwCEtu,double*
sm_FXT4jPWj6stSWaiL5dan3b){double sm_kzE_B99SLHhKgDdJ8T40RW[3];double
sm_kQ31u4dAHspehiBpRji8rT=sm_VeDJvqSU0ktAWXfSef07BJ[0];double
sm_F32Ql82vv6pW_PYIdpkFQ0=0;int sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_core_math_castelEval(sm_VY7k8tVKVTWabyr34z3F_c,sm_F9XXYvbW1X_RXP6D2uUU48,
sm_kcga7oDUd_KJZ5ep8UQIl2,sm_kQ31u4dAHspehiBpRji8rT,sm_kzE_B99SLHhKgDdJ8T40RW)
;memcpy(sm_FXT4jPWj6stSWaiL5dan3b,sm_kzE_B99SLHhKgDdJ8T40RW,
sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));sm_F32Ql82vv6pW_PYIdpkFQ0=
sm_V6WG9QFC01O2aTR8QNBvNg(sm_VZrAo961Ev_Ij5VUbwCEtu,sm_kzE_B99SLHhKgDdJ8T40RW,
sm_F9XXYvbW1X_RXP6D2uUU48);for(sm_kwrB3ZoKf7OufTHWaHJV7a=1;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_V78_jX3vbP8Fd1_V6QmV3A;sm_kwrB3ZoKf7OufTHWaHJV7a
++){double sm_Fxl4i6XHkE8cY9kYmlJixt=0;sm_core_math_castelEval(
sm_VY7k8tVKVTWabyr34z3F_c,sm_F9XXYvbW1X_RXP6D2uUU48,sm_kcga7oDUd_KJZ5ep8UQIl2,
sm_VeDJvqSU0ktAWXfSef07BJ[sm_kwrB3ZoKf7OufTHWaHJV7a],sm_kzE_B99SLHhKgDdJ8T40RW
);sm_Fxl4i6XHkE8cY9kYmlJixt=sm_V6WG9QFC01O2aTR8QNBvNg(
sm_VZrAo961Ev_Ij5VUbwCEtu,sm_kzE_B99SLHhKgDdJ8T40RW,sm_F9XXYvbW1X_RXP6D2uUU48)
;if(sm_Fxl4i6XHkE8cY9kYmlJixt<sm_F32Ql82vv6pW_PYIdpkFQ0){
sm_kQ31u4dAHspehiBpRji8rT=sm_VeDJvqSU0ktAWXfSef07BJ[sm_kwrB3ZoKf7OufTHWaHJV7a]
;sm_F32Ql82vv6pW_PYIdpkFQ0=sm_Fxl4i6XHkE8cY9kYmlJixt;memcpy(
sm_FXT4jPWj6stSWaiL5dan3b,sm_kzE_B99SLHhKgDdJ8T40RW,sm_F9XXYvbW1X_RXP6D2uUU48*
sizeof(double));}}return sm_kQ31u4dAHspehiBpRji8rT;}double
sm_core_math_findClosestPointOnBezier(const size_t sm_FruBC65sPgSaZ5e_Y8cbTC,
const size_t sm_F9XXYvbW1X_RXP6D2uUU48,const double*sm_kTtwL9vkis0Rj11z__fIfM,
const double*sm__Fx3AcPRGGpDfPGz1V3ouw,const double*sm__coIYFxX3sdUZi7GOyTTHD,
const double*pm__lqjegyKuwStj56WZLiC_e,double*sm_FXT4jPWj6stSWaiL5dan3b){const
size_t sm_FQferGZUKft3_i5GvYy4Oy=2*sm_FruBC65sPgSaZ5e_Y8cbTC;double
sm_FW6qHWPVaIK3geHEb4DSvL[100];int sm_VND14ejye_86WmozrNaSGJ=0;double
sm_VLVkk9Vyo_KPcT4KZjnq1h=0;double sm_FkP_t3Oro9WRa9eZ_dZtsD[20];size_t
sm_V2__YrimeI4E_yWnhKofpy=0;for(sm_V2__YrimeI4E_yWnhKofpy=0;
sm_V2__YrimeI4E_yWnhKofpy<sm_FQferGZUKft3_i5GvYy4Oy;sm_V2__YrimeI4E_yWnhKofpy
++){sm_FkP_t3Oro9WRa9eZ_dZtsD[sm_V2__YrimeI4E_yWnhKofpy]=(
sm__Fx3AcPRGGpDfPGz1V3ouw[sm_V2__YrimeI4E_yWnhKofpy]-sm__2OZs4W2TKx3Xm44SRHYDa
(sm__coIYFxX3sdUZi7GOyTTHD+sm_V2__YrimeI4E_yWnhKofpy*sm_F9XXYvbW1X_RXP6D2uUU48
,pm__lqjegyKuwStj56WZLiC_e,sm_F9XXYvbW1X_RXP6D2uUU48));}
sm_FW6qHWPVaIK3geHEb4DSvL[0]=0.0;sm_FW6qHWPVaIK3geHEb4DSvL[1]=1.0;
sm_VND14ejye_86WmozrNaSGJ=sm__ZclC0am9m0aXT_4J934Bs(sm_FQferGZUKft3_i5GvYy4Oy,
0,sm_FkP_t3Oro9WRa9eZ_dZtsD,0.0,1.0,1e-14,sm_FW6qHWPVaIK3geHEb4DSvL+2)+2;
sm_VLVkk9Vyo_KPcT4KZjnq1h=sm_kNKPUVafBMtJWLgQGKOQes(sm_FruBC65sPgSaZ5e_Y8cbTC+
1,sm_F9XXYvbW1X_RXP6D2uUU48,sm_kTtwL9vkis0Rj11z__fIfM,
sm_VND14ejye_86WmozrNaSGJ,sm_FW6qHWPVaIK3geHEb4DSvL,pm__lqjegyKuwStj56WZLiC_e,
sm_FXT4jPWj6stSWaiL5dan3b);return sm_VLVkk9Vyo_KPcT4KZjnq1h;}double
sm__NLygguTP_CbXDNx736iLf(double sm_kEbBObcYFIxUZ5_77V3CO_,double
sm_FG350oy5KhSieeGidT6m0n,double sm_k9vhyFTMKOdYgysUqibCZU){return
sm_FG350oy5KhSieeGidT6m0n+sm_kEbBObcYFIxUZ5_77V3CO_*(sm_k9vhyFTMKOdYgysUqibCZU
-sm_FG350oy5KhSieeGidT6m0n);}double sm_core_math_findClosestPointOnSpline(
const size_t sm_FruBC65sPgSaZ5e_Y8cbTC,const size_t sm_F9XXYvbW1X_RXP6D2uUU48,
const size_t sm_FgdMfCu3V48Waqkzyywcw6,const unsigned char
sm__4eZvqLS4xSK_LaNVwLvKT,int sm_Fz3gDBTbnAdRjexjCHxc_s,const double*
sm__C5iw_XK_207dqezwnl6D0,const double*sm_kTtwL9vkis0Rj11z__fIfM,const double*
sm_kU3wi6a_0id2_uiGo0WkXI,const double*sm__Fx3AcPRGGpDfPGz1V3ouw,const double*
sm__coIYFxX3sdUZi7GOyTTHD,const double*sm_VZrAo961Ev_Ij5VUbwCEtu,double*
sm_kp1vFBczgUK0deS6edHd0E){double sm_FfDTppU8N_tOWuLK37x_08=1e-12;size_t
sm_kJyTNOIUd4OGdTtRX2nBEQ=0;while(sm_kJyTNOIUd4OGdTtRX2nBEQ<=
sm_FgdMfCu3V48Waqkzyywcw6){const double*sm__2saWeDC2C8y_DPuXJDkxN=
sm_kTtwL9vkis0Rj11z__fIfM+(sm_FruBC65sPgSaZ5e_Y8cbTC+1)*
sm_F9XXYvbW1X_RXP6D2uUU48*sm_Fz3gDBTbnAdRjexjCHxc_s;const double*
sm__btzaXlSoT8tgXtiOv0cfy=sm_kU3wi6a_0id2_uiGo0WkXI+sm_FruBC65sPgSaZ5e_Y8cbTC*
sm_F9XXYvbW1X_RXP6D2uUU48*sm_Fz3gDBTbnAdRjexjCHxc_s;const double*
sm_FA7KvL74Hu8YfaNM7Q6tdK=sm__Fx3AcPRGGpDfPGz1V3ouw+2*
sm_FruBC65sPgSaZ5e_Y8cbTC*sm_Fz3gDBTbnAdRjexjCHxc_s;const double*
sm__o5uyEtVoyS_amR_J0r9Md=sm__coIYFxX3sdUZi7GOyTTHD+2*
sm_FruBC65sPgSaZ5e_Y8cbTC*sm_F9XXYvbW1X_RXP6D2uUU48*sm_Fz3gDBTbnAdRjexjCHxc_s;
double sm_k1bRC114bIpZaevXlAuq_5[3],sm_V8XMtcd13M8FjLV0KvZZWM[3],
sm_kCrtWxbP_6xBjTvxotKDkD;const double sm___JpvEQgr1h_Wyn6IE25xS=
sm_core_math_findClosestPointOnBezier(sm_FruBC65sPgSaZ5e_Y8cbTC,
sm_F9XXYvbW1X_RXP6D2uUU48,sm__2saWeDC2C8y_DPuXJDkxN,sm_FA7KvL74Hu8YfaNM7Q6tdK,
sm__o5uyEtVoyS_amR_J0r9Md,sm_VZrAo961Ev_Ij5VUbwCEtu,sm_k1bRC114bIpZaevXlAuq_5)
;sm_kJyTNOIUd4OGdTtRX2nBEQ++;if(sm___JpvEQgr1h_Wyn6IE25xS!=0.0&&
sm___JpvEQgr1h_Wyn6IE25xS!=1.0){memcpy(sm_kp1vFBczgUK0deS6edHd0E,
sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));return
sm__NLygguTP_CbXDNx736iLf(sm___JpvEQgr1h_Wyn6IE25xS,sm__C5iw_XK_207dqezwnl6D0[
sm_Fz3gDBTbnAdRjexjCHxc_s],sm__C5iw_XK_207dqezwnl6D0[sm_Fz3gDBTbnAdRjexjCHxc_s
+1]);}sm_V33S2jgq4A_ffmu7BVqFTK(sm_V8XMtcd13M8FjLV0KvZZWM,
sm_VZrAo961Ev_Ij5VUbwCEtu,sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48)
;sm_kCrtWxbP_6xBjTvxotKDkD=sm_V6WG9QFC01O2aTR8QNBvNg(sm_VZrAo961Ev_Ij5VUbwCEtu
,sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48);if(
sm_kCrtWxbP_6xBjTvxotKDkD<sm_FfDTppU8N_tOWuLK37x_08){memcpy(
sm_kp1vFBczgUK0deS6edHd0E,sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48*
sizeof(double));return sm__NLygguTP_CbXDNx736iLf(sm___JpvEQgr1h_Wyn6IE25xS,
sm__C5iw_XK_207dqezwnl6D0[sm_Fz3gDBTbnAdRjexjCHxc_s],sm__C5iw_XK_207dqezwnl6D0
[sm_Fz3gDBTbnAdRjexjCHxc_s+1]);}else{double sm_V91rYIvDBkGRba7MvGcPUK[3];
double sm_VqT0gIZXuM8McuQYYsgQvq;sm_core_math_castelEval(
sm_FruBC65sPgSaZ5e_Y8cbTC,sm_F9XXYvbW1X_RXP6D2uUU48,sm__btzaXlSoT8tgXtiOv0cfy,
sm___JpvEQgr1h_Wyn6IE25xS,sm_V91rYIvDBkGRba7MvGcPUK);sm_VqT0gIZXuM8McuQYYsgQvq
=sm__2OZs4W2TKx3Xm44SRHYDa(sm_V8XMtcd13M8FjLV0KvZZWM,sm_V91rYIvDBkGRba7MvGcPUK
,sm_F9XXYvbW1X_RXP6D2uUU48);if(fabs(sm_VqT0gIZXuM8McuQYYsgQvq)<
sm_FfDTppU8N_tOWuLK37x_08){memcpy(sm_kp1vFBczgUK0deS6edHd0E,
sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));return
sm__NLygguTP_CbXDNx736iLf(sm___JpvEQgr1h_Wyn6IE25xS,sm__C5iw_XK_207dqezwnl6D0[
sm_Fz3gDBTbnAdRjexjCHxc_s],sm__C5iw_XK_207dqezwnl6D0[sm_Fz3gDBTbnAdRjexjCHxc_s
+1]);}else{if(sm_kJyTNOIUd4OGdTtRX2nBEQ>sm_FgdMfCu3V48Waqkzyywcw6){memcpy(
sm_kp1vFBczgUK0deS6edHd0E,sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48*
sizeof(double));return sm__NLygguTP_CbXDNx736iLf(sm___JpvEQgr1h_Wyn6IE25xS,
sm__C5iw_XK_207dqezwnl6D0[sm_FgdMfCu3V48Waqkzyywcw6-1],
sm__C5iw_XK_207dqezwnl6D0[sm_FgdMfCu3V48Waqkzyywcw6]);}if(
sm___JpvEQgr1h_Wyn6IE25xS==0.0){sm_Fz3gDBTbnAdRjexjCHxc_s-=1;if(
sm__4eZvqLS4xSK_LaNVwLvKT){if(sm_Fz3gDBTbnAdRjexjCHxc_s== -1)
sm_Fz3gDBTbnAdRjexjCHxc_s=(int)sm_FgdMfCu3V48Waqkzyywcw6-1;}else{if(
sm_Fz3gDBTbnAdRjexjCHxc_s== -1){memcpy(sm_kp1vFBczgUK0deS6edHd0E,
sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48*sizeof(double));return
sm__NLygguTP_CbXDNx736iLf(sm___JpvEQgr1h_Wyn6IE25xS,sm__C5iw_XK_207dqezwnl6D0[
0],sm__C5iw_XK_207dqezwnl6D0[1]);}}}else{sm_Fz3gDBTbnAdRjexjCHxc_s+=1;if(
sm__4eZvqLS4xSK_LaNVwLvKT){if(sm_Fz3gDBTbnAdRjexjCHxc_s==(int)
sm_FgdMfCu3V48Waqkzyywcw6)sm_Fz3gDBTbnAdRjexjCHxc_s=0;}else{if(
sm_Fz3gDBTbnAdRjexjCHxc_s==(int)sm_FgdMfCu3V48Waqkzyywcw6){memcpy(
sm_kp1vFBczgUK0deS6edHd0E,sm_k1bRC114bIpZaevXlAuq_5,sm_F9XXYvbW1X_RXP6D2uUU48*
sizeof(double));return sm__NLygguTP_CbXDNx736iLf(sm___JpvEQgr1h_Wyn6IE25xS,
sm__C5iw_XK_207dqezwnl6D0[sm_FgdMfCu3V48Waqkzyywcw6-1],
sm__C5iw_XK_207dqezwnl6D0[sm_FgdMfCu3V48Waqkzyywcw6]);}}}}}}return 0;}void
sm_core_math_validNormal(double*sm_Vr9KldtodP0_Va38fjNnWL,const double*
sm_V91rYIvDBkGRba7MvGcPUK,const int sm_F9XXYvbW1X_RXP6D2uUU48){if(
sm__3gTK5cN1E_7_eKMQ9Uy_u(sm_Vr9KldtodP0_Va38fjNnWL,sm_F9XXYvbW1X_RXP6D2uUU48)
<1e-14){if(sm_F9XXYvbW1X_RXP6D2uUU48==2){sm_Vr9KldtodP0_Va38fjNnWL[0]= -
sm_V91rYIvDBkGRba7MvGcPUK[1];sm_Vr9KldtodP0_Va38fjNnWL[1]=
sm_V91rYIvDBkGRba7MvGcPUK[0];}else{double sm_FP1K_cmgj_WFjuZSakIZSO[]={0.0,0.0
,0.0};int sm_VcZDhkN3sT4oW9z4LTO8tH=0;if(fabs(sm_V91rYIvDBkGRba7MvGcPUK[0])>
fabs(sm_V91rYIvDBkGRba7MvGcPUK[1]))sm_VcZDhkN3sT4oW9z4LTO8tH=1;if(fabs(
sm_V91rYIvDBkGRba7MvGcPUK[sm_VcZDhkN3sT4oW9z4LTO8tH])>fabs(
sm_V91rYIvDBkGRba7MvGcPUK[2]))sm_VcZDhkN3sT4oW9z4LTO8tH=2;
sm_FP1K_cmgj_WFjuZSakIZSO[sm_VcZDhkN3sT4oW9z4LTO8tH]=1.0;
sm_V8aO54_tT9dNgXBLjsemo2(sm_Vr9KldtodP0_Va38fjNnWL,sm_V91rYIvDBkGRba7MvGcPUK,
sm_FP1K_cmgj_WFjuZSakIZSO);}}}
