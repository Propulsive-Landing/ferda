#include "pm_std.h"
#include "pm_std.h"
boolean_T pm_math_lin_alg_choleskyFactor(real_T*pm_math_F2l4p_g4sn02huHNflQjMH
,uint32_T n);void pm_math_lin_alg_choleskySolve(const real_T*
pm_math_F8I2q9dciO0aXmFVhOX9ZQ,const real_T*b,uint32_T n,real_T*x,real_T*
pm_math_VRZCD_UL_ESThy75dC9J8D);void pm_math_VWv_QvuNRm4viyJPPMYOkS(const
real_T*pm_math__ut5UfJwzNlZ_XZC_yEgKo,const real_T*b,uint32_T n,boolean_T
pm_math_FEqGkTLIgMdKjaJ1lIV70b,real_T*x);void pm_math_VRlre4EWbUOCcXxk9GINjY(
const real_T*pm_math_Vi4Cp0qK964NYTFMGr9Ttn,const real_T*b,uint32_T n,
boolean_T pm_math_FEqGkTLIgMdKjaJ1lIV70b,real_T*x);
#include "math.h"
boolean_T pm_math_lin_alg_choleskyFactor(real_T*pm_math_F2l4p_g4sn02huHNflQjMH
,uint32_T n){boolean_T pm_math_VqT0gIZXuM8McuQYYsgQvq=false;uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;real_T*pm_math_VycBOLdaYCCCgXQAu_jrEd=
pm_math_F2l4p_g4sn02huHNflQjMH;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;pm_math_kyp6uAyJE40UVuAQNEYzS1++){uint32_T
pm_math_V2__YrimeI4E_yWnhKofpy;real_T pm_math_FQferGZUKft3_i5GvYy4Oy=0.0;
real_T*pm_math_koFRx_yZLHGefDTAtwpZPd=pm_math_F2l4p_g4sn02huHNflQjMH;for(
pm_math_V2__YrimeI4E_yWnhKofpy=0;pm_math_V2__YrimeI4E_yWnhKofpy<
pm_math_kyp6uAyJE40UVuAQNEYzS1;pm_math_V2__YrimeI4E_yWnhKofpy++){real_T
pm_math__1eAP9V6_J_NYm_1gTIm7A=0.0;{real_T*pm_math_VRbT1WwZjwW6f5YGBCFuaD=
pm_math_koFRx_yZLHGefDTAtwpZPd;real_T*pm_math__d7LC10lP38OY1VXN_tFqb=
pm_math_VycBOLdaYCCCgXQAu_jrEd;uint32_T pm_math_FtLpaXfWjlhXVyJeyN2hSL=
pm_math_V2__YrimeI4E_yWnhKofpy;while(pm_math_FtLpaXfWjlhXVyJeyN2hSL-->0){
pm_math__1eAP9V6_J_NYm_1gTIm7A+= *pm_math_VRbT1WwZjwW6f5YGBCFuaD++**
pm_math__d7LC10lP38OY1VXN_tFqb++;}}pm_math__1eAP9V6_J_NYm_1gTIm7A=(
pm_math_VycBOLdaYCCCgXQAu_jrEd[pm_math_V2__YrimeI4E_yWnhKofpy]-
pm_math__1eAP9V6_J_NYm_1gTIm7A)/pm_math_koFRx_yZLHGefDTAtwpZPd[
pm_math_V2__YrimeI4E_yWnhKofpy];pm_math_VycBOLdaYCCCgXQAu_jrEd[
pm_math_V2__YrimeI4E_yWnhKofpy]=pm_math__1eAP9V6_J_NYm_1gTIm7A;
pm_math_koFRx_yZLHGefDTAtwpZPd[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math__1eAP9V6_J_NYm_1gTIm7A;pm_math_FQferGZUKft3_i5GvYy4Oy+=
pm_math__1eAP9V6_J_NYm_1gTIm7A*pm_math__1eAP9V6_J_NYm_1gTIm7A;
pm_math_koFRx_yZLHGefDTAtwpZPd+=n;}pm_math_FQferGZUKft3_i5GvYy4Oy=
pm_math_VycBOLdaYCCCgXQAu_jrEd[pm_math_kyp6uAyJE40UVuAQNEYzS1]-
pm_math_FQferGZUKft3_i5GvYy4Oy;if(pm_math_FQferGZUKft3_i5GvYy4Oy<=0.0){return(
pm_math_VqT0gIZXuM8McuQYYsgQvq=true);}pm_math_VycBOLdaYCCCgXQAu_jrEd[
pm_math_kyp6uAyJE40UVuAQNEYzS1]=sqrt(pm_math_FQferGZUKft3_i5GvYy4Oy);
pm_math_VycBOLdaYCCCgXQAu_jrEd+=n;}return(pm_math_VqT0gIZXuM8McuQYYsgQvq);}
void pm_math_lin_alg_choleskySolve(const real_T*pm_math_knZMtGN5npp9jax3au9uBf
,const real_T*b,uint32_T n,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D){
pm_math_VWv_QvuNRm4viyJPPMYOkS(pm_math_knZMtGN5npp9jax3au9uBf,b,n,false,
pm_math_VRZCD_UL_ESThy75dC9J8D);pm_math_VRlre4EWbUOCcXxk9GINjY(
pm_math_knZMtGN5npp9jax3au9uBf,pm_math_VRZCD_UL_ESThy75dC9J8D,n,false,x);}
