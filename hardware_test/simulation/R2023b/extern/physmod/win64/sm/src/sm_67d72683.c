#include "pm_std.h"
struct sm_FAe_hLKRs0t1gPCNBVVtIR{size_t sm__VyGPUhHOUGr_9l0UciNgg;size_t
sm__ZBqXvglQt_BhaAcrCFP7W;int*sm_VELrIEUHeudCamAhgAMD2P;};typedef struct
sm_FAe_hLKRs0t1gPCNBVVtIR sm_V7xsSSeDhNGdeHRBws_2Zg;void
sm_core_SmIntVector_create(sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw
,size_t n,int pm_kpzAtHMD4_WnheH0UiioSE);void sm_core_SmIntVector_copy(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_kyVvGx32uvlKVTEMWrNekp,const
sm_V7xsSSeDhNGdeHRBws_2Zg*orig);void sm_core_SmIntVector_destroy(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw);int
sm_core_SmIntVector_isEmpty(const sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw);size_t sm_core_SmIntVector_size(const
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw);size_t
sm_core_SmIntVector_capacity(const sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw);int sm_core_SmIntVector_value(const
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw,size_t
sm__wN5g_p_uwdVVLfRtx9szc);void sm_core_SmIntVector_setValue(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw,size_t
sm__wN5g_p_uwdVVLfRtx9szc,int sm_kg4CyRgbu6OdeewZOel7Qx);const int*
sm_core_SmIntVector_values(const sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw);int*sm_core_SmIntVector_nonConstValues(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw);void
sm_core_SmIntVector_reserve(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,size_t n);void sm_core_SmIntVector_clear(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw);void
sm_core_SmIntVector_pushBack(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,int pm_kpzAtHMD4_WnheH0UiioSE);void
sm_core_SmIntVector_popBack(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw);
#include "string.h"
#include "pm_std.h"
#include "pm_std.h"
void sm_core_SmIntVector_create(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,size_t n,int pm_kpzAtHMD4_WnheH0UiioSE){size_t
sm_kwrB3ZoKf7OufTHWaHJV7a;sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
=n;sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W=n;
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P=pmf_malloc(n*sizeof(int))
;(void)0;;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n;++
sm_kwrB3ZoKf7OufTHWaHJV7a)sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P
[sm_kwrB3ZoKf7OufTHWaHJV7a]=pm_kpzAtHMD4_WnheH0UiioSE;}void
sm_core_SmIntVector_copy(sm_V7xsSSeDhNGdeHRBws_2Zg*sm_kyVvGx32uvlKVTEMWrNekp,
const sm_V7xsSSeDhNGdeHRBws_2Zg*orig){const size_t n=orig->
sm__VyGPUhHOUGr_9l0UciNgg;sm_kyVvGx32uvlKVTEMWrNekp->sm__VyGPUhHOUGr_9l0UciNgg
=n;sm_kyVvGx32uvlKVTEMWrNekp->sm__ZBqXvglQt_BhaAcrCFP7W=n;
sm_kyVvGx32uvlKVTEMWrNekp->sm_VELrIEUHeudCamAhgAMD2P=pmf_malloc(n*sizeof(int))
;(void)0;;memcpy(sm_kyVvGx32uvlKVTEMWrNekp->sm_VELrIEUHeudCamAhgAMD2P,orig->
sm_VELrIEUHeudCamAhgAMD2P,n*sizeof(int));}void sm_core_SmIntVector_destroy(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw){sm_VcZASgthsTt6dul8DlRWaw
->sm__VyGPUhHOUGr_9l0UciNgg=0;sm_VcZASgthsTt6dul8DlRWaw->
sm__ZBqXvglQt_BhaAcrCFP7W=0;pmf_free(sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P);}int sm_core_SmIntVector_isEmpty(const
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw){return
sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg==0;}size_t
sm_core_SmIntVector_size(const sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw){return sm_VcZASgthsTt6dul8DlRWaw->
sm__VyGPUhHOUGr_9l0UciNgg;}size_t sm_core_SmIntVector_capacity(const
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw){return
sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W;}int
sm_core_SmIntVector_value(const sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,size_t sm__wN5g_p_uwdVVLfRtx9szc){return
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P[sm__wN5g_p_uwdVVLfRtx9szc
];}void sm_core_SmIntVector_setValue(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,size_t sm__wN5g_p_uwdVVLfRtx9szc,int
sm_kg4CyRgbu6OdeewZOel7Qx){sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P[sm__wN5g_p_uwdVVLfRtx9szc]=sm_kg4CyRgbu6OdeewZOel7Qx
;}const int*sm_core_SmIntVector_values(const sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw){return sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P;}int*sm_core_SmIntVector_nonConstValues(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw){return
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P;}void
sm_core_SmIntVector_reserve(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,size_t n){if(sm_VcZASgthsTt6dul8DlRWaw->
sm__ZBqXvglQt_BhaAcrCFP7W<n){int*sm_V0NIpiHDZuOreH8HYF0R1c=pmf_malloc(n*sizeof
(int));(void)0;;memcpy(sm_V0NIpiHDZuOreH8HYF0R1c,sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P,sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
*sizeof(int));pmf_free(sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P);
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P=sm_V0NIpiHDZuOreH8HYF0R1c
;sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W=n;}}void
sm_core_SmIntVector_clear(sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw)
{sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg=0;}void
sm_core_SmIntVector_pushBack(sm_V7xsSSeDhNGdeHRBws_2Zg*
sm_VcZASgthsTt6dul8DlRWaw,int pm_kpzAtHMD4_WnheH0UiioSE){const size_t
sm_FF4ljM6P4J09VTBr_9j0_n=sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
+1;if(sm_FF4ljM6P4J09VTBr_9j0_n>sm_VcZASgthsTt6dul8DlRWaw->
sm__ZBqXvglQt_BhaAcrCFP7W){int*sm_V0NIpiHDZuOreH8HYF0R1c=NULL;size_t
sm_kWBy9c3tvf0AfLsF3yPjEJ=1;while((sm_kWBy9c3tvf0AfLsF3yPjEJ<
sm_FF4ljM6P4J09VTBr_9j0_n)&&sm_kWBy9c3tvf0AfLsF3yPjEJ)
sm_kWBy9c3tvf0AfLsF3yPjEJ<<=1;(void)0;;sm_V0NIpiHDZuOreH8HYF0R1c=pmf_malloc(
sm_kWBy9c3tvf0AfLsF3yPjEJ*sizeof(int));(void)0;;memcpy(
sm_V0NIpiHDZuOreH8HYF0R1c,sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P
,sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg*sizeof(int));pmf_free(
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P);
sm_VcZASgthsTt6dul8DlRWaw->sm_VELrIEUHeudCamAhgAMD2P=sm_V0NIpiHDZuOreH8HYF0R1c
;sm_VcZASgthsTt6dul8DlRWaw->sm__ZBqXvglQt_BhaAcrCFP7W=
sm_kWBy9c3tvf0AfLsF3yPjEJ;}sm_VcZASgthsTt6dul8DlRWaw->
sm_VELrIEUHeudCamAhgAMD2P[sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg
]=pm_kpzAtHMD4_WnheH0UiioSE;++sm_VcZASgthsTt6dul8DlRWaw->
sm__VyGPUhHOUGr_9l0UciNgg;}void sm_core_SmIntVector_popBack(
sm_V7xsSSeDhNGdeHRBws_2Zg*sm_VcZASgthsTt6dul8DlRWaw){(void)0;;--
sm_VcZASgthsTt6dul8DlRWaw->sm__VyGPUhHOUGr_9l0UciNgg;}
