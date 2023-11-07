#include "pm_std.h"
#include "pm_std.h"
void sm_core_math_computeIndicesNonzeroElements(uint32_T n,const int32_T*
sm_kEbBObcYFIxUZ5_77V3CO_,int32_T*sm_VgJW5ZqpwPpuY1inYtaofQ);boolean_T
sm_core_math_anyIsInf(uint32_T n,const real_T*sm_VgJW5ZqpwPpuY1inYtaofQ);
boolean_T sm_core_math_anyIsNaN(uint32_T n,const real_T*
sm_VgJW5ZqpwPpuY1inYtaofQ);void sm_core_math_computeIndicesNonzeroElements(
uint32_T n,const int32_T*sm_kEbBObcYFIxUZ5_77V3CO_,int32_T*
sm_VgJW5ZqpwPpuY1inYtaofQ){uint32_T sm_kwrB3ZoKf7OufTHWaHJV7a;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n;++
sm_kwrB3ZoKf7OufTHWaHJV7a)if(sm_kEbBObcYFIxUZ5_77V3CO_[
sm_kwrB3ZoKf7OufTHWaHJV7a]!=0)*sm_VgJW5ZqpwPpuY1inYtaofQ++=
sm_kwrB3ZoKf7OufTHWaHJV7a;}boolean_T sm_core_math_anyIsInf(uint32_T n,const
real_T*sm_VgJW5ZqpwPpuY1inYtaofQ){uint32_T sm_kwrB3ZoKf7OufTHWaHJV7a;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n;++
sm_kwrB3ZoKf7OufTHWaHJV7a)if(pmf_is_inf(sm_VgJW5ZqpwPpuY1inYtaofQ[
sm_kwrB3ZoKf7OufTHWaHJV7a]))return true;return false;}boolean_T
sm_core_math_anyIsNaN(uint32_T n,const real_T*sm_VgJW5ZqpwPpuY1inYtaofQ){
uint32_T sm_kwrB3ZoKf7OufTHWaHJV7a;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<n;++sm_kwrB3ZoKf7OufTHWaHJV7a)if(pmf_is_nan(
sm_VgJW5ZqpwPpuY1inYtaofQ[sm_kwrB3ZoKf7OufTHWaHJV7a]))return true;return false
;}
