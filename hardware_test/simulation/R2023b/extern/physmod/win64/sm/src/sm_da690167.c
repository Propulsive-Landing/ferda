#include "pm_std.h"
#include "math.h"
void sm_core_compiler_sortIndices(uint32_T n,const real_T*
sm_VgJW5ZqpwPpuY1inYtaofQ,int32_T*sm__VS9Y1bpgQtfX99yIDrTJ5);int32_T
sm_core_compiler_uniqueTol(uint32_T sm_VLHhnPUiNQpve5VIL9P3O9,const real_T*
sm_kEbBObcYFIxUZ5_77V3CO_,const int32_T*sm_VQxtjlFikdhGePsiYe4RiL,real_T
sm_FfDTppU8N_tOWuLK37x_08,real_T*sm_VgJW5ZqpwPpuY1inYtaofQ,int32_T*
sm__VS9Y1bpgQtfX99yIDrTJ5);void sm_core_compiler_subdividePartition(uint32_T
sm_VLHhnPUiNQpve5VIL9P3O9,const real_T*sm_kEbBObcYFIxUZ5_77V3CO_,uint32_T n,
real_T*sm_VgJW5ZqpwPpuY1inYtaofQ,uint32_T pm__lqjegyKuwStj56WZLiC_e,int32_T*
sm_VTbIrYvuMBCD_qZ2cpUD41,real_T*sm_V2reFDdIGnlm_L7ScexjoD,int32_T*
sm_kXI9_kljYIWjWX2UWUAF5L);void sm_core_compiler_sortIndices(uint32_T n,const
real_T*sm_VgJW5ZqpwPpuY1inYtaofQ,int32_T*sm__VS9Y1bpgQtfX99yIDrTJ5){int32_T
sm_kwrB3ZoKf7OufTHWaHJV7a,sm_kyp6uAyJE40UVuAQNEYzS1,sm_V2__YrimeI4E_yWnhKofpy,
sm__lO81KuDBk41W9Wd2wAkb0,pm__lqjegyKuwStj56WZLiC_e,sm__AExROh1PVWNeP7hmMWuJv;
pm__lqjegyKuwStj56WZLiC_e=n;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<pm__lqjegyKuwStj56WZLiC_e;++
sm_kwrB3ZoKf7OufTHWaHJV7a)*sm__VS9Y1bpgQtfX99yIDrTJ5++=
sm_kwrB3ZoKf7OufTHWaHJV7a;sm__VS9Y1bpgQtfX99yIDrTJ5-=pm__lqjegyKuwStj56WZLiC_e
;while(pm__lqjegyKuwStj56WZLiC_e>1){sm__AExROh1PVWNeP7hmMWuJv=0;for(
sm_kyp6uAyJE40UVuAQNEYzS1=1;sm_kyp6uAyJE40UVuAQNEYzS1<
pm__lqjegyKuwStj56WZLiC_e;++sm_kyp6uAyJE40UVuAQNEYzS1){
sm_kwrB3ZoKf7OufTHWaHJV7a=sm_kyp6uAyJE40UVuAQNEYzS1-1;
sm_V2__YrimeI4E_yWnhKofpy=sm__VS9Y1bpgQtfX99yIDrTJ5[sm_kwrB3ZoKf7OufTHWaHJV7a]
;sm__lO81KuDBk41W9Wd2wAkb0=sm__VS9Y1bpgQtfX99yIDrTJ5[sm_kyp6uAyJE40UVuAQNEYzS1
];if(sm_VgJW5ZqpwPpuY1inYtaofQ[sm_V2__YrimeI4E_yWnhKofpy]>
sm_VgJW5ZqpwPpuY1inYtaofQ[sm__lO81KuDBk41W9Wd2wAkb0]){
sm__VS9Y1bpgQtfX99yIDrTJ5[sm_kyp6uAyJE40UVuAQNEYzS1]=sm_V2__YrimeI4E_yWnhKofpy
;sm__VS9Y1bpgQtfX99yIDrTJ5[sm_kwrB3ZoKf7OufTHWaHJV7a]=
sm__lO81KuDBk41W9Wd2wAkb0;sm__AExROh1PVWNeP7hmMWuJv=sm_kyp6uAyJE40UVuAQNEYzS1;
}}pm__lqjegyKuwStj56WZLiC_e=sm__AExROh1PVWNeP7hmMWuJv;}}int32_T
sm_core_compiler_uniqueTol(uint32_T sm_VLHhnPUiNQpve5VIL9P3O9,const real_T*
sm_kEbBObcYFIxUZ5_77V3CO_,const int32_T*sm_VQxtjlFikdhGePsiYe4RiL,real_T
sm_FfDTppU8N_tOWuLK37x_08,real_T*sm_VgJW5ZqpwPpuY1inYtaofQ,int32_T*
sm__VS9Y1bpgQtfX99yIDrTJ5){uint32_T sm_kwrB3ZoKf7OufTHWaHJV7a;int32_T
sm_kyp6uAyJE40UVuAQNEYzS1,n;real_T sm__Ts24nhK4R8lgi8BOs3xFO,
sm_k_8SHLNUg0tOV5t6YAympy;if(sm_VLHhnPUiNQpve5VIL9P3O9==0)return 0;if(
sm_FfDTppU8N_tOWuLK37x_08<0.0)sm_FfDTppU8N_tOWuLK37x_08=0.0;n= -1;
sm__Ts24nhK4R8lgi8BOs3xFO=sm_kEbBObcYFIxUZ5_77V3CO_[sm_VQxtjlFikdhGePsiYe4RiL[
0]]-2.0*sm_FfDTppU8N_tOWuLK37x_08-1.0;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_VLHhnPUiNQpve5VIL9P3O9;++
sm_kwrB3ZoKf7OufTHWaHJV7a){sm_kyp6uAyJE40UVuAQNEYzS1=sm_VQxtjlFikdhGePsiYe4RiL
[sm_kwrB3ZoKf7OufTHWaHJV7a];sm_k_8SHLNUg0tOV5t6YAympy=
sm_kEbBObcYFIxUZ5_77V3CO_[sm_kyp6uAyJE40UVuAQNEYzS1];if(fabs(
sm_k_8SHLNUg0tOV5t6YAympy-sm__Ts24nhK4R8lgi8BOs3xFO)>sm_FfDTppU8N_tOWuLK37x_08
){*sm_VgJW5ZqpwPpuY1inYtaofQ++=sm_k_8SHLNUg0tOV5t6YAympy;++n;
sm__Ts24nhK4R8lgi8BOs3xFO=sm_k_8SHLNUg0tOV5t6YAympy;}sm__VS9Y1bpgQtfX99yIDrTJ5
[sm_kyp6uAyJE40UVuAQNEYzS1]=n;}++n;return n;}void
sm_core_compiler_subdividePartition(uint32_T sm_VLHhnPUiNQpve5VIL9P3O9,const
real_T*sm_kEbBObcYFIxUZ5_77V3CO_,uint32_T n,real_T*sm_VgJW5ZqpwPpuY1inYtaofQ,
uint32_T pm__lqjegyKuwStj56WZLiC_e,int32_T*sm_VTbIrYvuMBCD_qZ2cpUD41,real_T*
sm_V2reFDdIGnlm_L7ScexjoD,int32_T*sm_kXI9_kljYIWjWX2UWUAF5L){uint32_T
sm_kwrB3ZoKf7OufTHWaHJV7a;int32_T sm_kyp6uAyJE40UVuAQNEYzS1,
sm_FQferGZUKft3_i5GvYy4Oy;real_T sm_kUQBO1dSP8_IVqRAUx4R8G,x;uint32_T
sm_VcZDhkN3sT4oW9z4LTO8tH=sm_VLHhnPUiNQpve5VIL9P3O9-1;int32_T
sm__k0fc452T4WmfLlKvOB9N0=(int32_T)n-1;real_T*sm_F32Ql82vv6pW_PYIdpkFQ0=
sm_V2reFDdIGnlm_L7ScexjoD;real_T*sm_V1pxxydYEnWVVi3QKesjvf=
sm_V2reFDdIGnlm_L7ScexjoD+sm_VLHhnPUiNQpve5VIL9P3O9;int32_T*
sm__AExROh1PVWNeP7hmMWuJv=sm_kXI9_kljYIWjWX2UWUAF5L+1;int32_T*
sm_kRctYIzEpBGRZ5gYLh6fko=sm_kXI9_kljYIWjWX2UWUAF5L+sm_VLHhnPUiNQpve5VIL9P3O9;
if(n<=sm_VLHhnPUiNQpve5VIL9P3O9){for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<n;++sm_kwrB3ZoKf7OufTHWaHJV7a)*
sm_VgJW5ZqpwPpuY1inYtaofQ++= *sm_kEbBObcYFIxUZ5_77V3CO_++;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<
pm__lqjegyKuwStj56WZLiC_e;++sm_kwrB3ZoKf7OufTHWaHJV7a,++
sm_VTbIrYvuMBCD_qZ2cpUD41)if(*sm_VTbIrYvuMBCD_qZ2cpUD41>
sm__k0fc452T4WmfLlKvOB9N0)*sm_VTbIrYvuMBCD_qZ2cpUD41=sm__k0fc452T4WmfLlKvOB9N0
;return;}if(sm_VLHhnPUiNQpve5VIL9P3O9<=1){*sm_VgJW5ZqpwPpuY1inYtaofQ++=
sm_VLHhnPUiNQpve5VIL9P3O9==0?0.0:*sm_kEbBObcYFIxUZ5_77V3CO_;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=1;sm_kwrB3ZoKf7OufTHWaHJV7a<n;++
sm_kwrB3ZoKf7OufTHWaHJV7a,++sm_VgJW5ZqpwPpuY1inYtaofQ)*
sm_VgJW5ZqpwPpuY1inYtaofQ= *(sm_VgJW5ZqpwPpuY1inYtaofQ-1)+1.0;return;}
sm_kUQBO1dSP8_IVqRAUx4R8G=((real_T)sm__k0fc452T4WmfLlKvOB9N0)/(
sm_kEbBObcYFIxUZ5_77V3CO_[sm_VcZDhkN3sT4oW9z4LTO8tH]-sm_kEbBObcYFIxUZ5_77V3CO_
[0]);sm_FQferGZUKft3_i5GvYy4Oy=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_VcZDhkN3sT4oW9z4LTO8tH;++
sm_kwrB3ZoKf7OufTHWaHJV7a,++sm_kEbBObcYFIxUZ5_77V3CO_){*
sm_F32Ql82vv6pW_PYIdpkFQ0++=x= *(sm_kEbBObcYFIxUZ5_77V3CO_+1)-*
sm_kEbBObcYFIxUZ5_77V3CO_;x*=sm_kUQBO1dSP8_IVqRAUx4R8G;*
sm__AExROh1PVWNeP7hmMWuJv++=sm_kyp6uAyJE40UVuAQNEYzS1=(int32_T)ceil(x);
sm_FQferGZUKft3_i5GvYy4Oy+=sm_kyp6uAyJE40UVuAQNEYzS1;*
sm_V1pxxydYEnWVVi3QKesjvf++=x-floor(x);}sm_kEbBObcYFIxUZ5_77V3CO_-=
sm_VcZDhkN3sT4oW9z4LTO8tH;sm_F32Ql82vv6pW_PYIdpkFQ0-=sm_VcZDhkN3sT4oW9z4LTO8tH
;sm__AExROh1PVWNeP7hmMWuJv-=sm_VcZDhkN3sT4oW9z4LTO8tH;
sm_V1pxxydYEnWVVi3QKesjvf-=sm_VcZDhkN3sT4oW9z4LTO8tH;if(
sm_FQferGZUKft3_i5GvYy4Oy>sm__k0fc452T4WmfLlKvOB9N0){
sm_core_compiler_sortIndices(sm_VcZDhkN3sT4oW9z4LTO8tH,
sm_V1pxxydYEnWVVi3QKesjvf,sm_kRctYIzEpBGRZ5gYLh6fko);while(
sm_FQferGZUKft3_i5GvYy4Oy>sm__k0fc452T4WmfLlKvOB9N0){for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<
sm_VcZDhkN3sT4oW9z4LTO8tH;++sm_kwrB3ZoKf7OufTHWaHJV7a){
sm_kyp6uAyJE40UVuAQNEYzS1=sm_kRctYIzEpBGRZ5gYLh6fko[sm_kwrB3ZoKf7OufTHWaHJV7a]
;if(sm__AExROh1PVWNeP7hmMWuJv[sm_kyp6uAyJE40UVuAQNEYzS1]>1){--
sm__AExROh1PVWNeP7hmMWuJv[sm_kyp6uAyJE40UVuAQNEYzS1];--
sm_FQferGZUKft3_i5GvYy4Oy;if(sm_FQferGZUKft3_i5GvYy4Oy==
sm__k0fc452T4WmfLlKvOB9N0)break;}}}}for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_VcZDhkN3sT4oW9z4LTO8tH;++
sm_kwrB3ZoKf7OufTHWaHJV7a,++sm_F32Ql82vv6pW_PYIdpkFQ0){*
sm_VgJW5ZqpwPpuY1inYtaofQ++= *sm_kEbBObcYFIxUZ5_77V3CO_++;
sm_FQferGZUKft3_i5GvYy4Oy= *sm__AExROh1PVWNeP7hmMWuJv++;if(
sm_FQferGZUKft3_i5GvYy4Oy>1){x=(*sm_F32Ql82vv6pW_PYIdpkFQ0)/(real_T)
sm_FQferGZUKft3_i5GvYy4Oy;for(sm_kyp6uAyJE40UVuAQNEYzS1=1;
sm_kyp6uAyJE40UVuAQNEYzS1<sm_FQferGZUKft3_i5GvYy4Oy;++
sm_kyp6uAyJE40UVuAQNEYzS1,++sm_VgJW5ZqpwPpuY1inYtaofQ)*
sm_VgJW5ZqpwPpuY1inYtaofQ= *(sm_VgJW5ZqpwPpuY1inYtaofQ-1)+x;}}*
sm_VgJW5ZqpwPpuY1inYtaofQ++= *sm_kEbBObcYFIxUZ5_77V3CO_++;
sm__AExROh1PVWNeP7hmMWuJv-=sm_VcZDhkN3sT4oW9z4LTO8tH;if(
pm__lqjegyKuwStj56WZLiC_e>0){*(sm__AExROh1PVWNeP7hmMWuJv-1)=0;
sm_FQferGZUKft3_i5GvYy4Oy=0;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<sm_VcZDhkN3sT4oW9z4LTO8tH;++
sm_kwrB3ZoKf7OufTHWaHJV7a){*sm__AExROh1PVWNeP7hmMWuJv+=
sm_FQferGZUKft3_i5GvYy4Oy;sm_FQferGZUKft3_i5GvYy4Oy= *
sm__AExROh1PVWNeP7hmMWuJv++;}sm__AExROh1PVWNeP7hmMWuJv-=
sm_VLHhnPUiNQpve5VIL9P3O9;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<pm__lqjegyKuwStj56WZLiC_e;++
sm_kwrB3ZoKf7OufTHWaHJV7a,++sm_VTbIrYvuMBCD_qZ2cpUD41)*
sm_VTbIrYvuMBCD_qZ2cpUD41=sm__AExROh1PVWNeP7hmMWuJv[*sm_VTbIrYvuMBCD_qZ2cpUD41
];}}
