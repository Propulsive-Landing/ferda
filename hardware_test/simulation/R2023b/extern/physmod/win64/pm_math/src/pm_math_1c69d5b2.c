#include "pm_std.h"
#include "pm_std.h"
real_T pm_math_FbClXSm2MEW_gDjN18cA_U(const real_T*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,uint32_T n);real_T
pm_math_V1oag74lTiOv_aubp3aT9k(const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n);real_T
pm_math_VtKWxJZti9h2aTofzH3eQO(const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,
uint32_T n);real_T pm_math_lin_alg_conditionReciprocal(const real_T*
pm_math_knZMtGN5npp9jax3au9uBf,uint32_T n,boolean_T
pm_math_F6d3U0ostIxpXqcgw_2o__,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D);
#include "math.h"
real_T pm_math_lin_alg_conditionReciprocal(const real_T*
pm_math_knZMtGN5npp9jax3au9uBf,uint32_T n,boolean_T
pm_math_F6d3U0ostIxpXqcgw_2o__,real_T*pm_math_FBuROKTP_gCWgT_XDEoCzK){real_T
pm_math__hM0MPHQmaKbVPvKeR6UJX,pm_math__1eAP9V6_J_NYm_1gTIm7A,
pm_math_F3hIP89KiXOg_DgTB0_IFU,pm_math_FQferGZUKft3_i5GvYy4Oy,
pm_math_FWFLvpYQwlWRWihzReKEr9,pm_math__NBiAn7kx4dJiyqYAqJWjS,
pm_math_ksMnaAK8LbS0ZeO5PrFWhV,pm_math_VFhz0mAbWpWAhmXYOn_DjI,
pm_math_kW5qn8w_fndXfilNILrk9M;real_T*pm_math__iCMSSgVO_WBayg2MVVQLH,*
pm_math_kY2hpEmnE1SgiaLqNF_r4Z;const real_T*pm_math_VxeVCtf7FB4RYXZH4ahCSR,*
pm_math__lKes25TwmOt_u_ajawS55,*pm_math_k_Ri1pn8ucCYVqXFmls82f;uint32_T
pm_math_V2__YrimeI4E_yWnhKofpy;const real_T pm_math_F1JiFVQcvUKbXuLGIfRYqM=
pm_math_F6d3U0ostIxpXqcgw_2o__?pm_math_VtKWxJZti9h2aTofzH3eQO(
pm_math_knZMtGN5npp9jax3au9uBf,n):pm_math_V1oag74lTiOv_aubp3aT9k(
pm_math_knZMtGN5npp9jax3au9uBf,n,n);if(pm_math_F1JiFVQcvUKbXuLGIfRYqM==0){
return(0);}pm_math__iCMSSgVO_WBayg2MVVQLH=pm_math_FBuROKTP_gCWgT_XDEoCzK;for(
pm_math_V2__YrimeI4E_yWnhKofpy=n;pm_math_V2__YrimeI4E_yWnhKofpy-->0;){*
pm_math__iCMSSgVO_WBayg2MVVQLH++=0.0;}pm_math__hM0MPHQmaKbVPvKeR6UJX=1.0;
pm_math__iCMSSgVO_WBayg2MVVQLH=pm_math_FBuROKTP_gCWgT_XDEoCzK;
pm_math_VxeVCtf7FB4RYXZH4ahCSR=pm_math_knZMtGN5npp9jax3au9uBf;for(
pm_math_V2__YrimeI4E_yWnhKofpy=0;pm_math_V2__YrimeI4E_yWnhKofpy<n;
pm_math_V2__YrimeI4E_yWnhKofpy++){pm_math__1eAP9V6_J_NYm_1gTIm7A= -(*
pm_math__iCMSSgVO_WBayg2MVVQLH);pm_math__hM0MPHQmaKbVPvKeR6UJX=(
pm_math__1eAP9V6_J_NYm_1gTIm7A>=0)?fabs(pm_math__hM0MPHQmaKbVPvKeR6UJX):-fabs(
pm_math__hM0MPHQmaKbVPvKeR6UJX);pm_math__1eAP9V6_J_NYm_1gTIm7A+=
pm_math__hM0MPHQmaKbVPvKeR6UJX;pm_math_F3hIP89KiXOg_DgTB0_IFU=fabs(*
pm_math_VxeVCtf7FB4RYXZH4ahCSR);if(fabs(pm_math__1eAP9V6_J_NYm_1gTIm7A)>
pm_math_F3hIP89KiXOg_DgTB0_IFU){pm_math_FQferGZUKft3_i5GvYy4Oy=
pm_math_F3hIP89KiXOg_DgTB0_IFU/fabs(pm_math__1eAP9V6_J_NYm_1gTIm7A);
pm_math_kY2hpEmnE1SgiaLqNF_r4Z=pm_math_FBuROKTP_gCWgT_XDEoCzK;{uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=n;
pm_math_kyp6uAyJE40UVuAQNEYzS1-->0;){*pm_math_kY2hpEmnE1SgiaLqNF_r4Z++*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}}pm_math__hM0MPHQmaKbVPvKeR6UJX*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}pm_math__NBiAn7kx4dJiyqYAqJWjS=
pm_math__hM0MPHQmaKbVPvKeR6UJX-*pm_math__iCMSSgVO_WBayg2MVVQLH;
pm_math_ksMnaAK8LbS0ZeO5PrFWhV= -pm_math__hM0MPHQmaKbVPvKeR6UJX-*
pm_math__iCMSSgVO_WBayg2MVVQLH;pm_math_FQferGZUKft3_i5GvYy4Oy=fabs(
pm_math__NBiAn7kx4dJiyqYAqJWjS);pm_math_FWFLvpYQwlWRWihzReKEr9=fabs(
pm_math_ksMnaAK8LbS0ZeO5PrFWhV);if(pm_math_F3hIP89KiXOg_DgTB0_IFU!=0.0){
pm_math__1eAP9V6_J_NYm_1gTIm7A= *pm_math_VxeVCtf7FB4RYXZH4ahCSR;
pm_math__NBiAn7kx4dJiyqYAqJWjS/=pm_math__1eAP9V6_J_NYm_1gTIm7A;
pm_math_ksMnaAK8LbS0ZeO5PrFWhV/=pm_math__1eAP9V6_J_NYm_1gTIm7A;}else{
pm_math__NBiAn7kx4dJiyqYAqJWjS=1.0;pm_math_ksMnaAK8LbS0ZeO5PrFWhV=1.0;}
pm_math_kY2hpEmnE1SgiaLqNF_r4Z=pm_math__iCMSSgVO_WBayg2MVVQLH;
pm_math__lKes25TwmOt_u_ajawS55=pm_math_VxeVCtf7FB4RYXZH4ahCSR;{uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=
pm_math_V2__YrimeI4E_yWnhKofpy+1;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;
pm_math_kyp6uAyJE40UVuAQNEYzS1++){pm_math__lKes25TwmOt_u_ajawS55+=n;
pm_math_F3hIP89KiXOg_DgTB0_IFU= *pm_math__lKes25TwmOt_u_ajawS55;
pm_math__1eAP9V6_J_NYm_1gTIm7A=pm_math_ksMnaAK8LbS0ZeO5PrFWhV*
pm_math_F3hIP89KiXOg_DgTB0_IFU+*(++pm_math_kY2hpEmnE1SgiaLqNF_r4Z);
pm_math_FWFLvpYQwlWRWihzReKEr9+=fabs(pm_math__1eAP9V6_J_NYm_1gTIm7A);*
pm_math_kY2hpEmnE1SgiaLqNF_r4Z+=pm_math__NBiAn7kx4dJiyqYAqJWjS*
pm_math_F3hIP89KiXOg_DgTB0_IFU;pm_math_FQferGZUKft3_i5GvYy4Oy+=fabs(*
pm_math_kY2hpEmnE1SgiaLqNF_r4Z);}}if((pm_math_V2__YrimeI4E_yWnhKofpy+1<n)&&(
pm_math_FQferGZUKft3_i5GvYy4Oy<pm_math_FWFLvpYQwlWRWihzReKEr9)){
pm_math__1eAP9V6_J_NYm_1gTIm7A=pm_math_ksMnaAK8LbS0ZeO5PrFWhV-
pm_math__NBiAn7kx4dJiyqYAqJWjS;pm_math__NBiAn7kx4dJiyqYAqJWjS=
pm_math_ksMnaAK8LbS0ZeO5PrFWhV;pm_math_kY2hpEmnE1SgiaLqNF_r4Z=
pm_math__iCMSSgVO_WBayg2MVVQLH;pm_math__lKes25TwmOt_u_ajawS55=
pm_math_VxeVCtf7FB4RYXZH4ahCSR;{uint32_T pm_math_kyp6uAyJE40UVuAQNEYzS1;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math_V2__YrimeI4E_yWnhKofpy+1;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math__lKes25TwmOt_u_ajawS55+=n;pm_math_F3hIP89KiXOg_DgTB0_IFU= *
pm_math__lKes25TwmOt_u_ajawS55;*(++pm_math_kY2hpEmnE1SgiaLqNF_r4Z)+=
pm_math__1eAP9V6_J_NYm_1gTIm7A*pm_math_F3hIP89KiXOg_DgTB0_IFU;}}}*
pm_math__iCMSSgVO_WBayg2MVVQLH++=pm_math__NBiAn7kx4dJiyqYAqJWjS;
pm_math_VxeVCtf7FB4RYXZH4ahCSR+=n+1;}pm_math_FQferGZUKft3_i5GvYy4Oy=
pm_math_FbClXSm2MEW_gDjN18cA_U(pm_math_FBuROKTP_gCWgT_XDEoCzK,n);
pm_math_FQferGZUKft3_i5GvYy4Oy=1.0/pm_math_FQferGZUKft3_i5GvYy4Oy;
pm_math__iCMSSgVO_WBayg2MVVQLH=pm_math_FBuROKTP_gCWgT_XDEoCzK;for(
pm_math_V2__YrimeI4E_yWnhKofpy=n;pm_math_V2__YrimeI4E_yWnhKofpy-->0;){*
pm_math__iCMSSgVO_WBayg2MVVQLH++*=pm_math_FQferGZUKft3_i5GvYy4Oy;}
pm_math_VFhz0mAbWpWAhmXYOn_DjI=1.0;if(!pm_math_F6d3U0ostIxpXqcgw_2o__){
pm_math__iCMSSgVO_WBayg2MVVQLH=pm_math_FBuROKTP_gCWgT_XDEoCzK+n-1;
pm_math_VxeVCtf7FB4RYXZH4ahCSR=pm_math_knZMtGN5npp9jax3au9uBf+n*n-1;for(
pm_math_V2__YrimeI4E_yWnhKofpy=n-1;pm_math_V2__YrimeI4E_yWnhKofpy!=(uint32_T)-
1;pm_math_V2__YrimeI4E_yWnhKofpy--){if(pm_math_V2__YrimeI4E_yWnhKofpy<n-1){
pm_math_kY2hpEmnE1SgiaLqNF_r4Z=pm_math__iCMSSgVO_WBayg2MVVQLH;
pm_math_k_Ri1pn8ucCYVqXFmls82f=pm_math_VxeVCtf7FB4RYXZH4ahCSR;{uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=
pm_math_V2__YrimeI4E_yWnhKofpy+1;pm_math_kyp6uAyJE40UVuAQNEYzS1<n;
pm_math_kyp6uAyJE40UVuAQNEYzS1++){*pm_math__iCMSSgVO_WBayg2MVVQLH-= *(++
pm_math_k_Ri1pn8ucCYVqXFmls82f)**(++pm_math_kY2hpEmnE1SgiaLqNF_r4Z);}}}if(fabs
(*pm_math__iCMSSgVO_WBayg2MVVQLH)>1.0){pm_math_FQferGZUKft3_i5GvYy4Oy=1.0/fabs
(*pm_math__iCMSSgVO_WBayg2MVVQLH);pm_math_kY2hpEmnE1SgiaLqNF_r4Z=
pm_math_FBuROKTP_gCWgT_XDEoCzK;{uint32_T pm_math_kyp6uAyJE40UVuAQNEYzS1;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=n;pm_math_kyp6uAyJE40UVuAQNEYzS1-->0;){*
pm_math_kY2hpEmnE1SgiaLqNF_r4Z++*=pm_math_FQferGZUKft3_i5GvYy4Oy;}}}
pm_math_VxeVCtf7FB4RYXZH4ahCSR-=n+1;pm_math__iCMSSgVO_WBayg2MVVQLH--;}
pm_math_FQferGZUKft3_i5GvYy4Oy=pm_math_FbClXSm2MEW_gDjN18cA_U(
pm_math_FBuROKTP_gCWgT_XDEoCzK,n);pm_math_FQferGZUKft3_i5GvYy4Oy=1.0/
pm_math_FQferGZUKft3_i5GvYy4Oy;pm_math__iCMSSgVO_WBayg2MVVQLH=
pm_math_FBuROKTP_gCWgT_XDEoCzK;for(pm_math_V2__YrimeI4E_yWnhKofpy=n;
pm_math_V2__YrimeI4E_yWnhKofpy-->0;){*pm_math__iCMSSgVO_WBayg2MVVQLH++*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}pm_math__iCMSSgVO_WBayg2MVVQLH=
pm_math_FBuROKTP_gCWgT_XDEoCzK;pm_math_VxeVCtf7FB4RYXZH4ahCSR=
pm_math_knZMtGN5npp9jax3au9uBf;for(pm_math_V2__YrimeI4E_yWnhKofpy=0;
pm_math_V2__YrimeI4E_yWnhKofpy<n;pm_math_V2__YrimeI4E_yWnhKofpy++){if(
pm_math_V2__YrimeI4E_yWnhKofpy<n-1){pm_math_kY2hpEmnE1SgiaLqNF_r4Z=
pm_math__iCMSSgVO_WBayg2MVVQLH;pm_math_k_Ri1pn8ucCYVqXFmls82f=
pm_math_VxeVCtf7FB4RYXZH4ahCSR;{uint32_T pm_math_kyp6uAyJE40UVuAQNEYzS1;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math_V2__YrimeI4E_yWnhKofpy+1;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;pm_math_kyp6uAyJE40UVuAQNEYzS1++){*(++
pm_math_kY2hpEmnE1SgiaLqNF_r4Z)-= *pm_math__iCMSSgVO_WBayg2MVVQLH**(++
pm_math_k_Ri1pn8ucCYVqXFmls82f);}}}if(fabs(*pm_math__iCMSSgVO_WBayg2MVVQLH)>
1.0){pm_math_FQferGZUKft3_i5GvYy4Oy=1.0/fabs(*pm_math__iCMSSgVO_WBayg2MVVQLH);
pm_math_kY2hpEmnE1SgiaLqNF_r4Z=pm_math_FBuROKTP_gCWgT_XDEoCzK;{uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=n;
pm_math_kyp6uAyJE40UVuAQNEYzS1-->0;){*pm_math_kY2hpEmnE1SgiaLqNF_r4Z++*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}}pm_math_VFhz0mAbWpWAhmXYOn_DjI*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}pm_math_VxeVCtf7FB4RYXZH4ahCSR+=n+1;
pm_math__iCMSSgVO_WBayg2MVVQLH++;}pm_math_FQferGZUKft3_i5GvYy4Oy=
pm_math_FbClXSm2MEW_gDjN18cA_U(pm_math_FBuROKTP_gCWgT_XDEoCzK,n);
pm_math_FQferGZUKft3_i5GvYy4Oy=1.0/pm_math_FQferGZUKft3_i5GvYy4Oy;
pm_math__iCMSSgVO_WBayg2MVVQLH=pm_math_FBuROKTP_gCWgT_XDEoCzK;for(
pm_math_V2__YrimeI4E_yWnhKofpy=n;pm_math_V2__YrimeI4E_yWnhKofpy-->0;){*
pm_math__iCMSSgVO_WBayg2MVVQLH++*=pm_math_FQferGZUKft3_i5GvYy4Oy;}
pm_math_VFhz0mAbWpWAhmXYOn_DjI*=pm_math_FQferGZUKft3_i5GvYy4Oy;}
pm_math__iCMSSgVO_WBayg2MVVQLH=pm_math_FBuROKTP_gCWgT_XDEoCzK+n-1;
pm_math_VxeVCtf7FB4RYXZH4ahCSR=pm_math_knZMtGN5npp9jax3au9uBf+(n-1)*(n+1);for(
pm_math_V2__YrimeI4E_yWnhKofpy=n-1;pm_math_V2__YrimeI4E_yWnhKofpy!=(uint32_T)-
1;pm_math_V2__YrimeI4E_yWnhKofpy--){pm_math__1eAP9V6_J_NYm_1gTIm7A=fabs(*
pm_math_VxeVCtf7FB4RYXZH4ahCSR);pm_math_F3hIP89KiXOg_DgTB0_IFU=fabs(*
pm_math__iCMSSgVO_WBayg2MVVQLH);if(pm_math_F3hIP89KiXOg_DgTB0_IFU>
pm_math__1eAP9V6_J_NYm_1gTIm7A){pm_math_FQferGZUKft3_i5GvYy4Oy=
pm_math__1eAP9V6_J_NYm_1gTIm7A/pm_math_F3hIP89KiXOg_DgTB0_IFU;
pm_math_kY2hpEmnE1SgiaLqNF_r4Z=pm_math_FBuROKTP_gCWgT_XDEoCzK;{uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=n;
pm_math_kyp6uAyJE40UVuAQNEYzS1-->0;){*pm_math_kY2hpEmnE1SgiaLqNF_r4Z++*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}}pm_math_VFhz0mAbWpWAhmXYOn_DjI*=
pm_math_FQferGZUKft3_i5GvYy4Oy;}if(pm_math__1eAP9V6_J_NYm_1gTIm7A==0.0){*
pm_math__iCMSSgVO_WBayg2MVVQLH=1.0;}else{*pm_math__iCMSSgVO_WBayg2MVVQLH/= *
pm_math_VxeVCtf7FB4RYXZH4ahCSR;}pm_math__1eAP9V6_J_NYm_1gTIm7A= -(*
pm_math__iCMSSgVO_WBayg2MVVQLH--);pm_math_kY2hpEmnE1SgiaLqNF_r4Z=
pm_math_FBuROKTP_gCWgT_XDEoCzK;pm_math_k_Ri1pn8ucCYVqXFmls82f=
pm_math_VxeVCtf7FB4RYXZH4ahCSR-pm_math_V2__YrimeI4E_yWnhKofpy;{uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=
pm_math_V2__YrimeI4E_yWnhKofpy;pm_math_kyp6uAyJE40UVuAQNEYzS1-->0;){*
pm_math_kY2hpEmnE1SgiaLqNF_r4Z++ += *pm_math_k_Ri1pn8ucCYVqXFmls82f++*
pm_math__1eAP9V6_J_NYm_1gTIm7A;}}pm_math_VxeVCtf7FB4RYXZH4ahCSR-=n+1;}
pm_math_FQferGZUKft3_i5GvYy4Oy=pm_math_FbClXSm2MEW_gDjN18cA_U(
pm_math_FBuROKTP_gCWgT_XDEoCzK,n);pm_math_FQferGZUKft3_i5GvYy4Oy=1.0/
pm_math_FQferGZUKft3_i5GvYy4Oy;pm_math_VFhz0mAbWpWAhmXYOn_DjI*=
pm_math_FQferGZUKft3_i5GvYy4Oy;pm_math_kW5qn8w_fndXfilNILrk9M=
pm_math_VFhz0mAbWpWAhmXYOn_DjI/pm_math_F1JiFVQcvUKbXuLGIfRYqM;return(
pm_math_kW5qn8w_fndXfilNILrk9M);}
