#include "pm_std.h"
struct sm_F10Fbm8pyTOM_L3tMhj4e7{size_t sm__VyGPUhHOUGr_9l0UciNgg;size_t
sm__ZBqXvglQt_BhaAcrCFP7W;real_T*sm_VELrIEUHeudCamAhgAMD2P;};typedef struct
sm_F10Fbm8pyTOM_L3tMhj4e7 sm_VoDgoJA_OIdYZyh9aOzfzC;void
sm_core_SmRealVector_create(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw,size_t n,real_T pm_kpzAtHMD4_WnheH0UiioSE);void
sm_core_SmRealVector_copy(sm_VoDgoJA_OIdYZyh9aOzfzC*sm_kyVvGx32uvlKVTEMWrNekp,
const sm_VoDgoJA_OIdYZyh9aOzfzC*orig);void sm_core_SmRealVector_destroy(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw);int
sm_core_SmRealVector_isEmpty(const sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw);size_t sm_core_SmRealVector_size(const
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw);size_t
sm_core_SmRealVector_capacity(const sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw);real_T sm_core_SmRealVector_value(const
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw,size_t
sm__wN5g_p_uwdVVLfRtx9szc);void sm_core_SmRealVector_setValue(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw,size_t
sm__wN5g_p_uwdVVLfRtx9szc,real_T sm_kg4CyRgbu6OdeewZOel7Qx);const real_T*
sm_core_SmRealVector_values(const sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw);real_T*sm_core_SmRealVector_nonConstValues(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw);void
sm_core_SmRealVector_reserve(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw,size_t n);void sm_core_SmRealVector_clear(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw);void
sm_core_SmRealVector_pushBack(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw,real_T pm_kpzAtHMD4_WnheH0UiioSE);void
sm_core_SmRealVector_popBack(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw);
#include "string.h"
#include "pm_std.h"
#include "pm_std.h"
void sm_core_SmRealVector_create(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw,size_t n,real_T pm_kpzAtHMD4_WnheH0UiioSE){size_t
sm_kwrB3ZoKf7OufTHWaHJV7a;sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
=n;sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W=n;
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P=pmf_malloc(n*sizeof(
real_T));(void)0;;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n;
++sm_kwrB3ZoKf7OufTHWaHJV7a)sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P[sm_kwrB3ZoKf7OufTHWaHJV7a]=pm_kpzAtHMD4_WnheH0UiioSE
;}void sm_core_SmRealVector_copy(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_kyVvGx32uvlKVTEMWrNekp,const sm_VoDgoJA_OIdYZyh9aOzfzC*orig){const size_t n
=orig->sm__VyGPUhHOUGr_9l0UciNgg;sm_kyVvGx32uvlKVTEMWrNekp->
sm__VyGPUhHOUGr_9l0UciNgg=n;sm_kyVvGx32uvlKVTEMWrNekp->
sm__ZBqXvglQt_BhaAcrCFP7W=n;sm_kyVvGx32uvlKVTEMWrNekp->
sm_VELrIEUHeudCamAhgAMD2P=pmf_malloc(n*sizeof(real_T));(void)0;;memcpy(
sm_kyVvGx32uvlKVTEMWrNekp->sm_VELrIEUHeudCamAhgAMD2P,orig->
sm_VELrIEUHeudCamAhgAMD2P,n*sizeof(real_T));}void sm_core_SmRealVector_destroy
(sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw){
sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg=0;
sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W=0;pmf_free(
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P);}int
sm_core_SmRealVector_isEmpty(const sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw){return sm_VcZASgthsTt6dul8DlRWaw->
sm__VyGPUhHOUGr_9l0UciNgg==0;}size_t sm_core_SmRealVector_size(const
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw){return
sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg;}size_t
sm_core_SmRealVector_capacity(const sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw){return sm_VcZASgthsTt6dul8DlRWaw->
sm__ZBqXvglQt_BhaAcrCFP7W;}real_T sm_core_SmRealVector_value(const
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw,size_t
sm__wN5g_p_uwdVVLfRtx9szc){return sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P[sm__wN5g_p_uwdVVLfRtx9szc];}void
sm_core_SmRealVector_setValue(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw,size_t sm__wN5g_p_uwdVVLfRtx9szc,real_T
sm_kg4CyRgbu6OdeewZOel7Qx){sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P[sm__wN5g_p_uwdVVLfRtx9szc]=sm_kg4CyRgbu6OdeewZOel7Qx
;}const real_T*sm_core_SmRealVector_values(const sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw){return sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P;}real_T*sm_core_SmRealVector_nonConstValues(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw){return
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P;}void
sm_core_SmRealVector_reserve(sm_VoDgoJA_OIdYZyh9aOzfzC*
sm_VcZASgthsTt6dul8DlRWaw,size_t n){if(sm_VcZASgthsTt6dul8DlRWaw->
sm__ZBqXvglQt_BhaAcrCFP7W<n){real_T*sm_V0NIpiHDZuOreH8HYF0R1c=pmf_malloc(n*
sizeof(real_T));(void)0;;memcpy(sm_V0NIpiHDZuOreH8HYF0R1c,
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P,sm_VcZASgthsTt6dul8DlRWaw
->sm__VyGPUhHOUGr_9l0UciNgg*sizeof(real_T));pmf_free(sm_VcZASgthsTt6dul8DlRWaw
->sm_VELrIEUHeudCamAhgAMD2P);sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P=sm_V0NIpiHDZuOreH8HYF0R1c;sm_VcZASgthsTt6dul8DlRWaw
->sm__ZBqXvglQt_BhaAcrCFP7W=n;}}void sm_core_SmRealVector_clear(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw){sm_VcZASgthsTt6dul8DlRWaw
->sm__VyGPUhHOUGr_9l0UciNgg=0;}void sm_core_SmRealVector_pushBack(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw,real_T
pm_kpzAtHMD4_WnheH0UiioSE){const size_t sm_FF4ljM6P4J09VTBr_9j0_n=
sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg+1;if(
sm_FF4ljM6P4J09VTBr_9j0_n>sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W
){real_T*sm_V0NIpiHDZuOreH8HYF0R1c=NULL;size_t sm_kWBy9c3tvf0AfLsF3yPjEJ=1;
while((sm_kWBy9c3tvf0AfLsF3yPjEJ<sm_FF4ljM6P4J09VTBr_9j0_n)&&
sm_kWBy9c3tvf0AfLsF3yPjEJ)sm_kWBy9c3tvf0AfLsF3yPjEJ<<=1;(void)0;;
sm_V0NIpiHDZuOreH8HYF0R1c=pmf_malloc(sm_kWBy9c3tvf0AfLsF3yPjEJ*sizeof(real_T))
;(void)0;;memcpy(sm_V0NIpiHDZuOreH8HYF0R1c,sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P,sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
*sizeof(real_T));pmf_free(sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P
);sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P=
sm_V0NIpiHDZuOreH8HYF0R1c;sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W
=sm_kWBy9c3tvf0AfLsF3yPjEJ;}sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P[sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
]=pm_kpzAtHMD4_WnheH0UiioSE;++sm_VcZASgthsTt6dul8DlRWaw->
sm__VyGPUhHOUGr_9l0UciNgg;}void sm_core_SmRealVector_popBack(
sm_VoDgoJA_OIdYZyh9aOzfzC*sm_VcZASgthsTt6dul8DlRWaw){(void)0;;--
sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg;}
