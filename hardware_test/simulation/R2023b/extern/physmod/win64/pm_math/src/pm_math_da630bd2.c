#include "pm_std.h"
#include "pm_std.h"
void pm_math_lin_alg_qrFactor(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T
n,real_T*pm_math_F2l4p_g4sn02huHNflQjMH,real_T*pm_math_kTQbRrK9Zn49jTgjOBqDPQ,
int32_T*pm_math_FrVYLSQleN4GYXlrWrmRui,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D);
uint32_T pm_math_lin_alg_getRankWithTol(const real_T*
pm_math_VWCrbzOuAe4hay4E_Unnfg,uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,real_T pm_math_kxaGfrxGoyCxZ1cvYvF8ZN);boolean_T
pm_math_lin_alg_qrSolveTall(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n
,const real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,const real_T*
pm_math_kTQbRrK9Zn49jTgjOBqDPQ,int32_T*pm_math_FrVYLSQleN4GYXlrWrmRui,real_T
pm_math_Vwo26eepTNdHYHWZbLLpBj,real_T*pm_math_FYb3BU2fbqC_bHhRn9AW9b);int32_T
pm_math_lin_alg_qrSolveWide(const uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
const uint32_T n,const real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,const real_T*
pm_math_kTQbRrK9Zn49jTgjOBqDPQ,const int32_T*pm_math_FrVYLSQleN4GYXlrWrmRui,
const real_T*b,real_T pm_math_Vwo26eepTNdHYHWZbLLpBj,real_T*x);
#include "pm_std.h"
#include "pm_std.h"
#include "string.h"
#include "math.h"
static real_T*pm_math_FaQn66Qi_V0iYuGMmd5xjd(const real_T*
pm_math_Fjrq38q0w9WrgLyXNhsx1c){union{const real_T*
pm_math_Fjrq38q0w9WrgLyXNhsx1c;real_T*pm_math_kIm3LiXDCKxeWDpwRJ2Oes;}
pm_math_kEbBObcYFIxUZ5_77V3CO_;pm_math_kEbBObcYFIxUZ5_77V3CO_.
pm_math_Fjrq38q0w9WrgLyXNhsx1c=pm_math_Fjrq38q0w9WrgLyXNhsx1c;return
pm_math_kEbBObcYFIxUZ5_77V3CO_.pm_math_kIm3LiXDCKxeWDpwRJ2Oes;}void
pm_math_FhJag9VgpAKgc5kXrC6eK_(const real_T*pm_math_kJeUz19e49pVaDwcttsZep,
uint32_T pm_math_F3nVSXkoDK8VeeL5OmY0_B,real_T*pm_math__WadSiE0LKCAYTdBOZVq99)
{real_T pm_math_VRYYo80g8Gt_ZyEKRXCsw_=0.0;uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a;const real_T*pm_math__ETpmKEb9t_6_uqy2dDtPw=
pm_math_kJeUz19e49pVaDwcttsZep;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_F3nVSXkoDK8VeeL5OmY0_B;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a){{if(fabs(pm_math_VRYYo80g8Gt_ZyEKRXCsw_)>fabs(
*pm_math__ETpmKEb9t_6_uqy2dDtPw)){real_T pm_math_FFvdRuzi2OGlaa4iw1XuCB=(*
pm_math__ETpmKEb9t_6_uqy2dDtPw)/(pm_math_VRYYo80g8Gt_ZyEKRXCsw_);(
pm_math_VRYYo80g8Gt_ZyEKRXCsw_)=(fabs(pm_math_VRYYo80g8Gt_ZyEKRXCsw_)*sqrt(1+
pm_math_FFvdRuzi2OGlaa4iw1XuCB*pm_math_FFvdRuzi2OGlaa4iw1XuCB));}else{if((*
pm_math__ETpmKEb9t_6_uqy2dDtPw)==0.0){(pm_math_VRYYo80g8Gt_ZyEKRXCsw_)=0.0;}
else{real_T pm_math_FFvdRuzi2OGlaa4iw1XuCB=(pm_math_VRYYo80g8Gt_ZyEKRXCsw_)/(*
pm_math__ETpmKEb9t_6_uqy2dDtPw);(pm_math_VRYYo80g8Gt_ZyEKRXCsw_)=(fabs(*
pm_math__ETpmKEb9t_6_uqy2dDtPw)*sqrt(1+pm_math_FFvdRuzi2OGlaa4iw1XuCB*
pm_math_FFvdRuzi2OGlaa4iw1XuCB));}}};pm_math__ETpmKEb9t_6_uqy2dDtPw++;}*
pm_math__WadSiE0LKCAYTdBOZVq99=pm_math_VRYYo80g8Gt_ZyEKRXCsw_;}void
pm_math_lin_alg_qrFactor(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,
real_T*x,real_T*pm_math_kTQbRrK9Zn49jTgjOBqDPQ,int32_T*
pm_math_FrVYLSQleN4GYXlrWrmRui,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_FlcK3Fc_kWOwdeoC78cTfx,pm_math__lO81KuDBk41W9Wd2wAkb0,
pm_math_VGvNcp5jm_l3fmLCC_tyjs,pm_math_FDG3wUulCLxAhHDy8tQ6Zx,
pm_math_kY30jPK_4J8UZy7wWmgk5V,pm_math__epTP49opvlmj9zAk5YIHj,
pm_math_VXXpHDW5mu8_buAXE3tDmk;real_T pm_math__1eAP9V6_J_NYm_1gTIm7A,
pm_math_Frl9Y1Vw32xGeP3KCstBuR,pm_math_kc_Le2kmDm_TbHtWyMgFyz,
pm_math_F3hIP89KiXOg_DgTB0_IFU,pm_math_Fk2O4u6vQUpibmbv8Kjgnn=0.0,*
pm_math__d7gkv9vMLhcYPICPcC5X_,*pm_math_VgZLri__KJl2e14Sx3Jvy6,
pm_math_VhCwJ10QjUxzg5l2ImlZpw;uint32_T pm_math_FNl5HOfkAo_3gyGJx_miiR,
pm_math__3TsTiO5bfOPV19qAs0rQN,pm_math_kUPRnQZwX1dSbqI0HmVq4f,
pm_math_VCTwyEFHYUWMdXuHAtRliU,pm_math__vIqq4PR28dSbuLi1_wpVo;int32_T*
pm_math_VrvNlzoYOE_0WTg_0eva2T;(void)0;;pm_math_FDG3wUulCLxAhHDy8tQ6Zx=0;
pm_math_kY30jPK_4J8UZy7wWmgk5V=n;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math_VrvNlzoYOE_0WTg_0eva2T= &pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_kyp6uAyJE40UVuAQNEYzS1];pm_math_VXXpHDW5mu8_buAXE3tDmk=(*
pm_math_VrvNlzoYOE_0WTg_0eva2T>0);*pm_math_VrvNlzoYOE_0WTg_0eva2T=(*
pm_math_VrvNlzoYOE_0WTg_0eva2T<0)?(-(int32_T)(pm_math_kyp6uAyJE40UVuAQNEYzS1+1
)):((int32_T)pm_math_kyp6uAyJE40UVuAQNEYzS1);if(pm_math_VXXpHDW5mu8_buAXE3tDmk
){if(pm_math_kyp6uAyJE40UVuAQNEYzS1!=pm_math_FDG3wUulCLxAhHDy8tQ6Zx){
pm_math_FNl5HOfkAo_3gyGJx_miiR=pm_math_FDG3wUulCLxAhHDy8tQ6Zx*
pm_math_VLHhnPUiNQpve5VIL9P3O9;pm_math__3TsTiO5bfOPV19qAs0rQN=
pm_math_kyp6uAyJE40UVuAQNEYzS1*pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math__d7gkv9vMLhcYPICPcC5X_=x+pm_math_FNl5HOfkAo_3gyGJx_miiR;
pm_math_VgZLri__KJl2e14Sx3Jvy6=x+pm_math__3TsTiO5bfOPV19qAs0rQN;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){pm_math_Fk2O4u6vQUpibmbv8Kjgnn= *
pm_math__d7gkv9vMLhcYPICPcC5X_;*pm_math__d7gkv9vMLhcYPICPcC5X_= *
pm_math_VgZLri__KJl2e14Sx3Jvy6;*pm_math_VgZLri__KJl2e14Sx3Jvy6=
pm_math_Fk2O4u6vQUpibmbv8Kjgnn;pm_math__d7gkv9vMLhcYPICPcC5X_++;
pm_math_VgZLri__KJl2e14Sx3Jvy6++;}}*pm_math_VrvNlzoYOE_0WTg_0eva2T=
pm_math_FrVYLSQleN4GYXlrWrmRui[pm_math_FDG3wUulCLxAhHDy8tQ6Zx];
pm_math_FrVYLSQleN4GYXlrWrmRui[pm_math_FDG3wUulCLxAhHDy8tQ6Zx++]=
pm_math_kyp6uAyJE40UVuAQNEYzS1;}}pm_math_kY30jPK_4J8UZy7wWmgk5V=n;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math_kY30jPK_4J8UZy7wWmgk5V-1;
pm_math_kyp6uAyJE40UVuAQNEYzS1!=(uint32_T)-1;pm_math_kyp6uAyJE40UVuAQNEYzS1--)
{if(*(pm_math_VrvNlzoYOE_0WTg_0eva2T= &pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_kyp6uAyJE40UVuAQNEYzS1])<0){*pm_math_VrvNlzoYOE_0WTg_0eva2T=(-*
pm_math_VrvNlzoYOE_0WTg_0eva2T)-1;--pm_math_kY30jPK_4J8UZy7wWmgk5V;if(
pm_math_kyp6uAyJE40UVuAQNEYzS1!=pm_math_kY30jPK_4J8UZy7wWmgk5V){
pm_math_FNl5HOfkAo_3gyGJx_miiR=pm_math_kY30jPK_4J8UZy7wWmgk5V*
pm_math_VLHhnPUiNQpve5VIL9P3O9;pm_math__3TsTiO5bfOPV19qAs0rQN=
pm_math_kyp6uAyJE40UVuAQNEYzS1*pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math__d7gkv9vMLhcYPICPcC5X_=x+pm_math_FNl5HOfkAo_3gyGJx_miiR;
pm_math_VgZLri__KJl2e14Sx3Jvy6=x+pm_math__3TsTiO5bfOPV19qAs0rQN;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){pm_math_Fk2O4u6vQUpibmbv8Kjgnn= *
pm_math__d7gkv9vMLhcYPICPcC5X_;*pm_math__d7gkv9vMLhcYPICPcC5X_= *
pm_math_VgZLri__KJl2e14Sx3Jvy6;*pm_math_VgZLri__KJl2e14Sx3Jvy6=
pm_math_Fk2O4u6vQUpibmbv8Kjgnn;pm_math__d7gkv9vMLhcYPICPcC5X_++;
pm_math_VgZLri__KJl2e14Sx3Jvy6++;}pm_math_FlcK3Fc_kWOwdeoC78cTfx=
pm_math_FrVYLSQleN4GYXlrWrmRui[pm_math_kY30jPK_4J8UZy7wWmgk5V];
pm_math_FrVYLSQleN4GYXlrWrmRui[pm_math_kY30jPK_4J8UZy7wWmgk5V]= *
pm_math_VrvNlzoYOE_0WTg_0eva2T;*pm_math_VrvNlzoYOE_0WTg_0eva2T=
pm_math_FlcK3Fc_kWOwdeoC78cTfx;}}}if(pm_math_VLHhnPUiNQpve5VIL9P3O9*n!=0){(
void)0;;for(pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math_FDG3wUulCLxAhHDy8tQ6Zx;
pm_math_kyp6uAyJE40UVuAQNEYzS1<pm_math_kY30jPK_4J8UZy7wWmgk5V;
pm_math_kyp6uAyJE40UVuAQNEYzS1++){pm_math__3TsTiO5bfOPV19qAs0rQN=
pm_math_kyp6uAyJE40UVuAQNEYzS1*pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math_FhJag9VgpAKgc5kXrC6eK_(x+pm_math__3TsTiO5bfOPV19qAs0rQN,
pm_math_VLHhnPUiNQpve5VIL9P3O9,&pm_math_Fk2O4u6vQUpibmbv8Kjgnn);
pm_math_VRZCD_UL_ESThy75dC9J8D[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math_Fk2O4u6vQUpibmbv8Kjgnn;}for(pm_math__lO81KuDBk41W9Wd2wAkb0=0;
pm_math__lO81KuDBk41W9Wd2wAkb0<((pm_math_VLHhnPUiNQpve5VIL9P3O9)<(n)?(
pm_math_VLHhnPUiNQpve5VIL9P3O9):(n));pm_math__lO81KuDBk41W9Wd2wAkb0++){
pm_math__epTP49opvlmj9zAk5YIHj=pm_math_VLHhnPUiNQpve5VIL9P3O9-
pm_math__lO81KuDBk41W9Wd2wAkb0;if(pm_math__lO81KuDBk41W9Wd2wAkb0>=
pm_math_FDG3wUulCLxAhHDy8tQ6Zx&&pm_math__lO81KuDBk41W9Wd2wAkb0+1<
pm_math_kY30jPK_4J8UZy7wWmgk5V){pm_math_kc_Le2kmDm_TbHtWyMgFyz=0.0;
pm_math_VGvNcp5jm_l3fmLCC_tyjs=pm_math__lO81KuDBk41W9Wd2wAkb0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math__lO81KuDBk41W9Wd2wAkb0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<pm_math_kY30jPK_4J8UZy7wWmgk5V;
pm_math_kyp6uAyJE40UVuAQNEYzS1++){if(pm_math_kTQbRrK9Zn49jTgjOBqDPQ[
pm_math_kyp6uAyJE40UVuAQNEYzS1]>pm_math_kc_Le2kmDm_TbHtWyMgFyz){
pm_math_kc_Le2kmDm_TbHtWyMgFyz=pm_math_kTQbRrK9Zn49jTgjOBqDPQ[
pm_math_kyp6uAyJE40UVuAQNEYzS1];pm_math_VGvNcp5jm_l3fmLCC_tyjs=
pm_math_kyp6uAyJE40UVuAQNEYzS1;}}if(pm_math_VGvNcp5jm_l3fmLCC_tyjs!=
pm_math__lO81KuDBk41W9Wd2wAkb0){pm_math_kUPRnQZwX1dSbqI0HmVq4f=
pm_math__lO81KuDBk41W9Wd2wAkb0*pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math__3TsTiO5bfOPV19qAs0rQN=pm_math_VGvNcp5jm_l3fmLCC_tyjs*
pm_math_VLHhnPUiNQpve5VIL9P3O9;pm_math__d7gkv9vMLhcYPICPcC5X_=x+
pm_math_kUPRnQZwX1dSbqI0HmVq4f;pm_math_VgZLri__KJl2e14Sx3Jvy6=x+
pm_math__3TsTiO5bfOPV19qAs0rQN;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math_VLHhnPUiNQpve5VIL9P3O9;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){
pm_math_Fk2O4u6vQUpibmbv8Kjgnn= *pm_math__d7gkv9vMLhcYPICPcC5X_;*
pm_math__d7gkv9vMLhcYPICPcC5X_= *pm_math_VgZLri__KJl2e14Sx3Jvy6;*
pm_math_VgZLri__KJl2e14Sx3Jvy6=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;
pm_math__d7gkv9vMLhcYPICPcC5X_++;pm_math_VgZLri__KJl2e14Sx3Jvy6++;}
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math_VGvNcp5jm_l3fmLCC_tyjs]=
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math__lO81KuDBk41W9Wd2wAkb0];
pm_math_VRZCD_UL_ESThy75dC9J8D[pm_math_VGvNcp5jm_l3fmLCC_tyjs]=
pm_math_VRZCD_UL_ESThy75dC9J8D[pm_math__lO81KuDBk41W9Wd2wAkb0];
pm_math_FlcK3Fc_kWOwdeoC78cTfx=pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_VGvNcp5jm_l3fmLCC_tyjs];pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_VGvNcp5jm_l3fmLCC_tyjs]=pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math__lO81KuDBk41W9Wd2wAkb0];pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math__lO81KuDBk41W9Wd2wAkb0]=pm_math_FlcK3Fc_kWOwdeoC78cTfx;}}
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math__lO81KuDBk41W9Wd2wAkb0]=0.0;if(
pm_math__lO81KuDBk41W9Wd2wAkb0==(pm_math_VLHhnPUiNQpve5VIL9P3O9-1)){continue;}
pm_math_VCTwyEFHYUWMdXuHAtRliU=pm_math__lO81KuDBk41W9Wd2wAkb0*(
pm_math_VLHhnPUiNQpve5VIL9P3O9+1);pm_math__d7gkv9vMLhcYPICPcC5X_=x+
pm_math_VCTwyEFHYUWMdXuHAtRliU;pm_math_FhJag9VgpAKgc5kXrC6eK_(
pm_math__d7gkv9vMLhcYPICPcC5X_,pm_math__epTP49opvlmj9zAk5YIHj,&
pm_math_Frl9Y1Vw32xGeP3KCstBuR);if(fabs(pm_math_Frl9Y1Vw32xGeP3KCstBuR)==0.0){
continue;}if(fabs(*pm_math__d7gkv9vMLhcYPICPcC5X_)!=0.0){
pm_math_Frl9Y1Vw32xGeP3KCstBuR=(*pm_math__d7gkv9vMLhcYPICPcC5X_>=0.0)?fabs(
pm_math_Frl9Y1Vw32xGeP3KCstBuR):-fabs(pm_math_Frl9Y1Vw32xGeP3KCstBuR);}
pm_math_VgZLri__KJl2e14Sx3Jvy6=pm_math__d7gkv9vMLhcYPICPcC5X_;
pm_math_VhCwJ10QjUxzg5l2ImlZpw=5.0e-20*pm_math_Frl9Y1Vw32xGeP3KCstBuR;if(
pm_math_VhCwJ10QjUxzg5l2ImlZpw!=0.0){pm_math_Fk2O4u6vQUpibmbv8Kjgnn=1.0/
pm_math_Frl9Y1Vw32xGeP3KCstBuR;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math__epTP49opvlmj9zAk5YIHj;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){*
pm_math_VgZLri__KJl2e14Sx3Jvy6*=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;
pm_math_VgZLri__KJl2e14Sx3Jvy6++;}}else{for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math__epTP49opvlmj9zAk5YIHj;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){*
pm_math_VgZLri__KJl2e14Sx3Jvy6/=pm_math_Frl9Y1Vw32xGeP3KCstBuR;
pm_math_VgZLri__KJl2e14Sx3Jvy6++;}}*pm_math__d7gkv9vMLhcYPICPcC5X_+=1.0;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math__lO81KuDBk41W9Wd2wAkb0+1;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math__vIqq4PR28dSbuLi1_wpVo=pm_math_kyp6uAyJE40UVuAQNEYzS1*
pm_math_VLHhnPUiNQpve5VIL9P3O9+pm_math__lO81KuDBk41W9Wd2wAkb0;
pm_math_VgZLri__KJl2e14Sx3Jvy6=x+pm_math__vIqq4PR28dSbuLi1_wpVo;
pm_math__1eAP9V6_J_NYm_1gTIm7A=0.0;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math__epTP49opvlmj9zAk5YIHj;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){
pm_math__1eAP9V6_J_NYm_1gTIm7A-=(*pm_math__d7gkv9vMLhcYPICPcC5X_)*(*
pm_math_VgZLri__KJl2e14Sx3Jvy6);pm_math__d7gkv9vMLhcYPICPcC5X_++;
pm_math_VgZLri__KJl2e14Sx3Jvy6++;}pm_math__d7gkv9vMLhcYPICPcC5X_=x+
pm_math_VCTwyEFHYUWMdXuHAtRliU;pm_math_VgZLri__KJl2e14Sx3Jvy6=x+
pm_math__vIqq4PR28dSbuLi1_wpVo;pm_math__1eAP9V6_J_NYm_1gTIm7A/= *
pm_math__d7gkv9vMLhcYPICPcC5X_;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math__epTP49opvlmj9zAk5YIHj;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){*
pm_math_VgZLri__KJl2e14Sx3Jvy6+=pm_math__1eAP9V6_J_NYm_1gTIm7A*(*
pm_math__d7gkv9vMLhcYPICPcC5X_);pm_math__d7gkv9vMLhcYPICPcC5X_++;
pm_math_VgZLri__KJl2e14Sx3Jvy6++;}pm_math__d7gkv9vMLhcYPICPcC5X_=x+
pm_math_VCTwyEFHYUWMdXuHAtRliU;pm_math_VgZLri__KJl2e14Sx3Jvy6=x+
pm_math__vIqq4PR28dSbuLi1_wpVo;if(!(pm_math_kyp6uAyJE40UVuAQNEYzS1>=
pm_math_FDG3wUulCLxAhHDy8tQ6Zx&&pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_kY30jPK_4J8UZy7wWmgk5V)||fabs(pm_math_kTQbRrK9Zn49jTgjOBqDPQ[
pm_math_kyp6uAyJE40UVuAQNEYzS1])==0.0){continue;}
pm_math_F3hIP89KiXOg_DgTB0_IFU=1.0-pow((fabs(*pm_math_VgZLri__KJl2e14Sx3Jvy6)/
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math_kyp6uAyJE40UVuAQNEYzS1]),2.0);if(
pm_math_F3hIP89KiXOg_DgTB0_IFU<0.0)pm_math_F3hIP89KiXOg_DgTB0_IFU=0.0;
pm_math__1eAP9V6_J_NYm_1gTIm7A=pm_math_F3hIP89KiXOg_DgTB0_IFU;
pm_math_F3hIP89KiXOg_DgTB0_IFU=1.0+0.05*pm_math_F3hIP89KiXOg_DgTB0_IFU*pow((
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math_kyp6uAyJE40UVuAQNEYzS1]/
pm_math_VRZCD_UL_ESThy75dC9J8D[pm_math_kyp6uAyJE40UVuAQNEYzS1]),2.0);if(
pm_math_F3hIP89KiXOg_DgTB0_IFU!=1.0){pm_math_Fk2O4u6vQUpibmbv8Kjgnn=sqrt(
pm_math__1eAP9V6_J_NYm_1gTIm7A);pm_math_kTQbRrK9Zn49jTgjOBqDPQ[
pm_math_kyp6uAyJE40UVuAQNEYzS1]*=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;}else{
pm_math_FhJag9VgpAKgc5kXrC6eK_(++pm_math_VgZLri__KJl2e14Sx3Jvy6,
pm_math__epTP49opvlmj9zAk5YIHj-1,&pm_math_Fk2O4u6vQUpibmbv8Kjgnn);
pm_math_VRZCD_UL_ESThy75dC9J8D[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math_kTQbRrK9Zn49jTgjOBqDPQ[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math_Fk2O4u6vQUpibmbv8Kjgnn;}}pm_math_kTQbRrK9Zn49jTgjOBqDPQ[
pm_math__lO81KuDBk41W9Wd2wAkb0]= *pm_math__d7gkv9vMLhcYPICPcC5X_;*
pm_math__d7gkv9vMLhcYPICPcC5X_= -pm_math_Frl9Y1Vw32xGeP3KCstBuR;}};return;}
void pm_math__oRp9ZVvQj0Iem9LsFGyx1(uint32_T n,uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1,const real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,
const real_T*pm_math_VJZIyX7kuiW2iDFXwUKbpG,real_T*
pm_math_FzyLWRgau0pMYq2XSI3ETL){if((pm_math_VWCrbzOuAe4hay4E_Unnfg!=NULL)&&(
fabs(*pm_math_VJZIyX7kuiW2iDFXwUKbpG)!=0.0)){uint32_T
pm_math__1y29sW6WMtEWa9cXa7p7w,pm_math_kwrB3ZoKf7OufTHWaHJV7a,
pm_math_VESVVna1U8KQ_9OvaOghNl;real_T pm_math__1eAP9V6_J_NYm_1gTIm7A,
pm_math__0Gxic3Bpu_HjDevqjQi0l,*pm_math_FYZz1y0gQlxadqVuZF3GGC,*
pm_math_FsXzQT7CFnhMVmsxvJDfxd,*pm_math_V9_K2Uc0FEKCZigwWcNNOt;
pm_math_V9_K2Uc0FEKCZigwWcNNOt=pm_math_FaQn66Qi_V0iYuGMmd5xjd(
pm_math_VWCrbzOuAe4hay4E_Unnfg);pm_math__1y29sW6WMtEWa9cXa7p7w=n-
pm_math_kyp6uAyJE40UVuAQNEYzS1;pm_math_VESVVna1U8KQ_9OvaOghNl=
pm_math_kyp6uAyJE40UVuAQNEYzS1*(n+1);pm_math_FYZz1y0gQlxadqVuZF3GGC=
pm_math_V9_K2Uc0FEKCZigwWcNNOt+pm_math_VESVVna1U8KQ_9OvaOghNl;
pm_math__0Gxic3Bpu_HjDevqjQi0l= *pm_math_FYZz1y0gQlxadqVuZF3GGC;*
pm_math_FYZz1y0gQlxadqVuZF3GGC= *pm_math_VJZIyX7kuiW2iDFXwUKbpG;
pm_math__1eAP9V6_J_NYm_1gTIm7A=0.0;pm_math_FsXzQT7CFnhMVmsxvJDfxd=
pm_math_FzyLWRgau0pMYq2XSI3ETL;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math__1y29sW6WMtEWa9cXa7p7w;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){
pm_math__1eAP9V6_J_NYm_1gTIm7A-=(*pm_math_FYZz1y0gQlxadqVuZF3GGC)*(*
pm_math_FsXzQT7CFnhMVmsxvJDfxd);pm_math_FYZz1y0gQlxadqVuZF3GGC++;
pm_math_FsXzQT7CFnhMVmsxvJDfxd++;}pm_math_FYZz1y0gQlxadqVuZF3GGC=
pm_math_V9_K2Uc0FEKCZigwWcNNOt+pm_math_VESVVna1U8KQ_9OvaOghNl;
pm_math__1eAP9V6_J_NYm_1gTIm7A/= *pm_math_FYZz1y0gQlxadqVuZF3GGC;
pm_math_FsXzQT7CFnhMVmsxvJDfxd=pm_math_FzyLWRgau0pMYq2XSI3ETL;for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=pm_math__1y29sW6WMtEWa9cXa7p7w;
pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){*pm_math_FsXzQT7CFnhMVmsxvJDfxd+=
pm_math__1eAP9V6_J_NYm_1gTIm7A*(*pm_math_FYZz1y0gQlxadqVuZF3GGC);
pm_math_FYZz1y0gQlxadqVuZF3GGC++;pm_math_FsXzQT7CFnhMVmsxvJDfxd++;}
pm_math_FYZz1y0gQlxadqVuZF3GGC=pm_math_V9_K2Uc0FEKCZigwWcNNOt+
pm_math_VESVVna1U8KQ_9OvaOghNl;*pm_math_FYZz1y0gQlxadqVuZF3GGC=
pm_math__0Gxic3Bpu_HjDevqjQi0l;}}uint32_T pm_math_lin_alg_getRankWithTol(const
real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,uint32_T pm_math_V97n_svAQI86jykAS8CiYB,
uint32_T pm_math_kpL6NzuwGohXciuEn1oD5K,real_T pm_math_kxaGfrxGoyCxZ1cvYvF8ZN)
{uint32_T pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_V2__YrimeI4E_yWnhKofpy,
pm_math_VTqv0IWy_NdQj1xkFRaWX8;pm_math_V2__YrimeI4E_yWnhKofpy=(uint32_T)-1;if(
pm_math_VWCrbzOuAe4hay4E_Unnfg!=NULL){real_T pm_math_FfDTppU8N_tOWuLK37x_08=
fabs(*pm_math_VWCrbzOuAe4hay4E_Unnfg)*pm_math_kxaGfrxGoyCxZ1cvYvF8ZN;
pm_math_VTqv0IWy_NdQj1xkFRaWX8=((pm_math_V97n_svAQI86jykAS8CiYB)<(
pm_math_kpL6NzuwGohXciuEn1oD5K)?(pm_math_V97n_svAQI86jykAS8CiYB):(
pm_math_kpL6NzuwGohXciuEn1oD5K));for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<pm_math_VTqv0IWy_NdQj1xkFRaWX8;
pm_math_kyp6uAyJE40UVuAQNEYzS1++){if(fabs(*pm_math_VWCrbzOuAe4hay4E_Unnfg)>
pm_math_FfDTppU8N_tOWuLK37x_08)pm_math_V2__YrimeI4E_yWnhKofpy=
pm_math_kyp6uAyJE40UVuAQNEYzS1;pm_math_VWCrbzOuAe4hay4E_Unnfg+=
pm_math_V97n_svAQI86jykAS8CiYB+1;}}else{(void)0;;}return
pm_math_V2__YrimeI4E_yWnhKofpy+1;}static uint32_T
pm_math_VoKBd7V2eBt4XeyqMzZ15_(const real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,
int32_T pm_math_V97n_svAQI86jykAS8CiYB,int32_T pm_math_kpL6NzuwGohXciuEn1oD5K)
{real_T pm_math_kxaGfrxGoyCxZ1cvYvF8ZN;if(pm_math_VWCrbzOuAe4hay4E_Unnfg!=NULL
){pm_math_kxaGfrxGoyCxZ1cvYvF8ZN=((real_T)((pm_math_V97n_svAQI86jykAS8CiYB)>(
pm_math_kpL6NzuwGohXciuEn1oD5K)?(pm_math_V97n_svAQI86jykAS8CiYB):(
pm_math_kpL6NzuwGohXciuEn1oD5K)))*pmf_get_eps();return
pm_math_lin_alg_getRankWithTol(pm_math_VWCrbzOuAe4hay4E_Unnfg,
pm_math_V97n_svAQI86jykAS8CiYB,pm_math_kpL6NzuwGohXciuEn1oD5K,
pm_math_kxaGfrxGoyCxZ1cvYvF8ZN);}else{(void)0;;return 0;}}int32_T
pm_math_VNXkOckvWTpeiak_RjnMAI(int32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,int32_T
pm_math_V2__YrimeI4E_yWnhKofpy,const real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,
const real_T*pm_math_kTQbRrK9Zn49jTgjOBqDPQ,real_T*
pm_math_FzyLWRgau0pMYq2XSI3ETL){int32_T pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_kSEW12lLld4uY1T_UcY6kY=0;real_T*pm_math_FsXzQT7CFnhMVmsxvJDfxd;const
real_T*pm_math_V54qTRZvRZtVc9X6tsYve9;pm_math_kyp6uAyJE40UVuAQNEYzS1=((
pm_math_V2__YrimeI4E_yWnhKofpy)<(pm_math_VLHhnPUiNQpve5VIL9P3O9-1)?(
pm_math_V2__YrimeI4E_yWnhKofpy):(pm_math_VLHhnPUiNQpve5VIL9P3O9-1));if(
pm_math_kyp6uAyJE40UVuAQNEYzS1<=0){return(pm_math_kSEW12lLld4uY1T_UcY6kY);};(
void)0;;(void)0;;pm_math_V54qTRZvRZtVc9X6tsYve9=pm_math_kTQbRrK9Zn49jTgjOBqDPQ
+pm_math_kyp6uAyJE40UVuAQNEYzS1-1;pm_math_FsXzQT7CFnhMVmsxvJDfxd=
pm_math_FzyLWRgau0pMYq2XSI3ETL+pm_math_kyp6uAyJE40UVuAQNEYzS1-1;while(
pm_math_kyp6uAyJE40UVuAQNEYzS1--){pm_math__oRp9ZVvQj0Iem9LsFGyx1(
pm_math_VLHhnPUiNQpve5VIL9P3O9,pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math_VWCrbzOuAe4hay4E_Unnfg,pm_math_V54qTRZvRZtVc9X6tsYve9--,
pm_math_FsXzQT7CFnhMVmsxvJDfxd--);}return(pm_math_kSEW12lLld4uY1T_UcY6kY);}
int32_T pm_math__COxBXKr6nl6_9maNQXFqi(int32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
int32_T pm_math_V2__YrimeI4E_yWnhKofpy,const real_T*
pm_math_VWCrbzOuAe4hay4E_Unnfg,real_T*pm_math_kTQbRrK9Zn49jTgjOBqDPQ,real_T*
pm_math_FzyLWRgau0pMYq2XSI3ETL){int32_T pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math__n_mbZayf4_ViymCHt71tt,pm_math_kSEW12lLld4uY1T_UcY6kY=0;real_T*
pm_math_V54qTRZvRZtVc9X6tsYve9,*pm_math_FsXzQT7CFnhMVmsxvJDfxd;
pm_math__n_mbZayf4_ViymCHt71tt=((pm_math_V2__YrimeI4E_yWnhKofpy)<(
pm_math_VLHhnPUiNQpve5VIL9P3O9-1)?(pm_math_V2__YrimeI4E_yWnhKofpy):(
pm_math_VLHhnPUiNQpve5VIL9P3O9-1));if(pm_math__n_mbZayf4_ViymCHt71tt<=0){
return(pm_math_kSEW12lLld4uY1T_UcY6kY);}(void)0;;(void)0;;
pm_math_V54qTRZvRZtVc9X6tsYve9=pm_math_kTQbRrK9Zn49jTgjOBqDPQ;
pm_math_FsXzQT7CFnhMVmsxvJDfxd=pm_math_FzyLWRgau0pMYq2XSI3ETL;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math__n_mbZayf4_ViymCHt71tt;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math__oRp9ZVvQj0Iem9LsFGyx1(pm_math_VLHhnPUiNQpve5VIL9P3O9,
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_VWCrbzOuAe4hay4E_Unnfg,
pm_math_V54qTRZvRZtVc9X6tsYve9++,pm_math_FsXzQT7CFnhMVmsxvJDfxd++);}return(
pm_math_kSEW12lLld4uY1T_UcY6kY);}int32_T pm_math_VogMiZsNHthCVeebcsyzjw(
int32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,int32_T pm_math_V2__YrimeI4E_yWnhKofpy,
const real_T*pm_math_VWCrbzOuAe4hay4E_Unnfg,const real_T*
pm_math_kTQbRrK9Zn49jTgjOBqDPQ,real_T*b){int32_T pm_math_kSEW12lLld4uY1T_UcY6kY
=0;if((pm_math_VLHhnPUiNQpve5VIL9P3O9>0)&&(pm_math_V2__YrimeI4E_yWnhKofpy>0)){
int32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1,
pm_math__n_mbZayf4_ViymCHt71tt;real_T pm_math__1eAP9V6_J_NYm_1gTIm7A,*
pm_math__BgOuTo61ahviHhipJwhJx,*pm_math_kv58lO966_pH_ysOUPQUwk;const real_T*
pm_math_FYZz1y0gQlxadqVuZF3GGC,*pm_math_V54qTRZvRZtVc9X6tsYve9;(void)0;;
pm_math__n_mbZayf4_ViymCHt71tt=((pm_math_V2__YrimeI4E_yWnhKofpy)<(
pm_math_VLHhnPUiNQpve5VIL9P3O9-1)?(pm_math_V2__YrimeI4E_yWnhKofpy):(
pm_math_VLHhnPUiNQpve5VIL9P3O9-1));if(pm_math__n_mbZayf4_ViymCHt71tt==0){if(
fabs(*pm_math_VWCrbzOuAe4hay4E_Unnfg)==0.0){pm_math_kSEW12lLld4uY1T_UcY6kY=1;}
else{*b/= *pm_math_VWCrbzOuAe4hay4E_Unnfg;}return(
pm_math_kSEW12lLld4uY1T_UcY6kY);}pm_math_V54qTRZvRZtVc9X6tsYve9=
pm_math_kTQbRrK9Zn49jTgjOBqDPQ;pm_math__BgOuTo61ahviHhipJwhJx=b;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math__n_mbZayf4_ViymCHt71tt;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math__oRp9ZVvQj0Iem9LsFGyx1(pm_math_VLHhnPUiNQpve5VIL9P3O9,
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_VWCrbzOuAe4hay4E_Unnfg,
pm_math_V54qTRZvRZtVc9X6tsYve9++,pm_math__BgOuTo61ahviHhipJwhJx++);}
pm_math_kv58lO966_pH_ysOUPQUwk=b+pm_math_V2__YrimeI4E_yWnhKofpy-1;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math_V2__YrimeI4E_yWnhKofpy-1;
pm_math_kyp6uAyJE40UVuAQNEYzS1>=0;pm_math_kyp6uAyJE40UVuAQNEYzS1--){
pm_math_FYZz1y0gQlxadqVuZF3GGC=pm_math_VWCrbzOuAe4hay4E_Unnfg+
pm_math_kyp6uAyJE40UVuAQNEYzS1*(pm_math_VLHhnPUiNQpve5VIL9P3O9+1);if(fabs(*
pm_math_FYZz1y0gQlxadqVuZF3GGC)==0.0){pm_math_kSEW12lLld4uY1T_UcY6kY=
pm_math_kyp6uAyJE40UVuAQNEYzS1+1;break;}*pm_math_kv58lO966_pH_ysOUPQUwk/= *
pm_math_FYZz1y0gQlxadqVuZF3GGC;if(pm_math_kyp6uAyJE40UVuAQNEYzS1!=0){
pm_math__1eAP9V6_J_NYm_1gTIm7A= -*pm_math_kv58lO966_pH_ysOUPQUwk;
pm_math__BgOuTo61ahviHhipJwhJx=b;pm_math_FYZz1y0gQlxadqVuZF3GGC-=
pm_math_kyp6uAyJE40UVuAQNEYzS1;for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math_kyp6uAyJE40UVuAQNEYzS1;pm_math_kwrB3ZoKf7OufTHWaHJV7a-->0;){*
pm_math__BgOuTo61ahviHhipJwhJx++ +=pm_math__1eAP9V6_J_NYm_1gTIm7A**
pm_math_FYZz1y0gQlxadqVuZF3GGC++;}}pm_math_kv58lO966_pH_ysOUPQUwk--;}}return(
pm_math_kSEW12lLld4uY1T_UcY6kY);}boolean_T pm_math_lin_alg_qrSolveTall(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const real_T*
pm_math_VWCrbzOuAe4hay4E_Unnfg,const real_T*pm_math_kTQbRrK9Zn49jTgjOBqDPQ,
int32_T*pm_math_FrVYLSQleN4GYXlrWrmRui,real_T pm_math_Vwo26eepTNdHYHWZbLLpBj,
real_T*pm_math_FYb3BU2fbqC_bHhRn9AW9b){boolean_T pm_math_VCm_jc1sEWKoVauHxAjXLF
=false;if((pm_math_VLHhnPUiNQpve5VIL9P3O9>0)&&(n>0)){uint32_T
pm_math_kyp6uAyJE40UVuAQNEYzS1,pm_math_V2__YrimeI4E_yWnhKofpy,
pm_math_VTqv0IWy_NdQj1xkFRaWX8;real_T*pm_math__b_mEeaeTMSye952Y8Gn25,
pm_math_Fk2O4u6vQUpibmbv8Kjgnn,pm_math__KU02uumydKLjHi_uSO5mG=0.0;(void)0;;
pm_math_V2__YrimeI4E_yWnhKofpy=(pm_math_Vwo26eepTNdHYHWZbLLpBj==0.0)?
pm_math_VoKBd7V2eBt4XeyqMzZ15_(pm_math_VWCrbzOuAe4hay4E_Unnfg,
pm_math_VLHhnPUiNQpve5VIL9P3O9,n):pm_math_lin_alg_getRankWithTol(
pm_math_VWCrbzOuAe4hay4E_Unnfg,pm_math_VLHhnPUiNQpve5VIL9P3O9,n,
pm_math_Vwo26eepTNdHYHWZbLLpBj);pm_math_VTqv0IWy_NdQj1xkFRaWX8=((
pm_math_VLHhnPUiNQpve5VIL9P3O9)<(n)?(pm_math_VLHhnPUiNQpve5VIL9P3O9):(n));if(
pm_math_V2__YrimeI4E_yWnhKofpy<pm_math_VTqv0IWy_NdQj1xkFRaWX8){
pm_math_VCm_jc1sEWKoVauHxAjXLF=true;}pm_math_VogMiZsNHthCVeebcsyzjw(
pm_math_VLHhnPUiNQpve5VIL9P3O9,pm_math_V2__YrimeI4E_yWnhKofpy,
pm_math_VWCrbzOuAe4hay4E_Unnfg,pm_math_kTQbRrK9Zn49jTgjOBqDPQ,
pm_math_FYb3BU2fbqC_bHhRn9AW9b);pm_math__b_mEeaeTMSye952Y8Gn25=
pm_math_FYb3BU2fbqC_bHhRn9AW9b+pm_math_V2__YrimeI4E_yWnhKofpy;for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=n-pm_math_V2__YrimeI4E_yWnhKofpy;
pm_math_kyp6uAyJE40UVuAQNEYzS1-->0;){*pm_math__b_mEeaeTMSye952Y8Gn25++=
pm_math__KU02uumydKLjHi_uSO5mG;}for(pm_math_kyp6uAyJE40UVuAQNEYzS1=0;
pm_math_kyp6uAyJE40UVuAQNEYzS1<n;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math_V2__YrimeI4E_yWnhKofpy=pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_kyp6uAyJE40UVuAQNEYzS1];while(pm_math_V2__YrimeI4E_yWnhKofpy!=
pm_math_kyp6uAyJE40UVuAQNEYzS1){pm_math_Fk2O4u6vQUpibmbv8Kjgnn= *(
pm_math_FYb3BU2fbqC_bHhRn9AW9b+pm_math_kyp6uAyJE40UVuAQNEYzS1);*(
pm_math_FYb3BU2fbqC_bHhRn9AW9b+pm_math_kyp6uAyJE40UVuAQNEYzS1)= *(
pm_math_FYb3BU2fbqC_bHhRn9AW9b+pm_math_V2__YrimeI4E_yWnhKofpy);*(
pm_math_FYb3BU2fbqC_bHhRn9AW9b+pm_math_V2__YrimeI4E_yWnhKofpy)=
pm_math_Fk2O4u6vQUpibmbv8Kjgnn;pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_kyp6uAyJE40UVuAQNEYzS1]=pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_V2__YrimeI4E_yWnhKofpy];pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_V2__YrimeI4E_yWnhKofpy]=pm_math_V2__YrimeI4E_yWnhKofpy;
pm_math_V2__YrimeI4E_yWnhKofpy=pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_kyp6uAyJE40UVuAQNEYzS1];}}};return pm_math_VCm_jc1sEWKoVauHxAjXLF;}
int32_T pm_math_lin_alg_qrSolveWide(const uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,const uint32_T n,const real_T*
pm_math_VWCrbzOuAe4hay4E_Unnfg,const real_T*pm_math_kTQbRrK9Zn49jTgjOBqDPQ,
const int32_T*pm_math_FrVYLSQleN4GYXlrWrmRui,const real_T*
pm_math_FYb3BU2fbqC_bHhRn9AW9b,real_T pm_math_Vwo26eepTNdHYHWZbLLpBj,real_T*
pm_math_V5gSkPeCSkdzZ1xMyj2v7U){uint32_T pm_math_V2__YrimeI4E_yWnhKofpy=0;if((
pm_math_VLHhnPUiNQpve5VIL9P3O9>0)&&(n>0)){uint32_T
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_kyp6uAyJE40UVuAQNEYzS1;(void)0;;if(
pm_math_FrVYLSQleN4GYXlrWrmRui!=NULL){pm_math_V2__YrimeI4E_yWnhKofpy=(
pm_math_Vwo26eepTNdHYHWZbLLpBj==0.0)?pm_math_VoKBd7V2eBt4XeyqMzZ15_(
pm_math_VWCrbzOuAe4hay4E_Unnfg,n,pm_math_VLHhnPUiNQpve5VIL9P3O9):
pm_math_lin_alg_getRankWithTol(pm_math_VWCrbzOuAe4hay4E_Unnfg,n,
pm_math_VLHhnPUiNQpve5VIL9P3O9,pm_math_Vwo26eepTNdHYHWZbLLpBj);for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_VLHhnPUiNQpve5VIL9P3O9;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math_V5gSkPeCSkdzZ1xMyj2v7U[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math_FYb3BU2fbqC_bHhRn9AW9b[pm_math_FrVYLSQleN4GYXlrWrmRui[
pm_math_kyp6uAyJE40UVuAQNEYzS1]];}}else{pm_math_V2__YrimeI4E_yWnhKofpy=
pm_math_VLHhnPUiNQpve5VIL9P3O9;memcpy(pm_math_V5gSkPeCSkdzZ1xMyj2v7U,
pm_math_FYb3BU2fbqC_bHhRn9AW9b,pm_math_VLHhnPUiNQpve5VIL9P3O9*sizeof(real_T));
}for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<
pm_math_V2__YrimeI4E_yWnhKofpy;pm_math_kwrB3ZoKf7OufTHWaHJV7a++){for(
pm_math_kyp6uAyJE40UVuAQNEYzS1=0;pm_math_kyp6uAyJE40UVuAQNEYzS1<
pm_math_kwrB3ZoKf7OufTHWaHJV7a;pm_math_kyp6uAyJE40UVuAQNEYzS1++){
pm_math_V5gSkPeCSkdzZ1xMyj2v7U[pm_math_kwrB3ZoKf7OufTHWaHJV7a]-=
pm_math_VWCrbzOuAe4hay4E_Unnfg[pm_math_kyp6uAyJE40UVuAQNEYzS1+n*
pm_math_kwrB3ZoKf7OufTHWaHJV7a]*pm_math_V5gSkPeCSkdzZ1xMyj2v7U[
pm_math_kyp6uAyJE40UVuAQNEYzS1];}pm_math_V5gSkPeCSkdzZ1xMyj2v7U[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]/=pm_math_VWCrbzOuAe4hay4E_Unnfg[
pm_math_kwrB3ZoKf7OufTHWaHJV7a+n*pm_math_kwrB3ZoKf7OufTHWaHJV7a];}for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=pm_math_V2__YrimeI4E_yWnhKofpy;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;pm_math_kwrB3ZoKf7OufTHWaHJV7a++){
pm_math_V5gSkPeCSkdzZ1xMyj2v7U[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=0.0;}
pm_math_VNXkOckvWTpeiak_RjnMAI(n,pm_math_VLHhnPUiNQpve5VIL9P3O9,
pm_math_VWCrbzOuAe4hay4E_Unnfg,pm_math_kTQbRrK9Zn49jTgjOBqDPQ,
pm_math_V5gSkPeCSkdzZ1xMyj2v7U);};return(pm_math_V2__YrimeI4E_yWnhKofpy);}
