#include "pm_std.h"
#include "float.h"
#include "math.h"
#include "pm_std.h"
#include "pm_std.h"
#include "pm_std.h"
#include "pm_std.h"
real_T pmf_get_eps(void){return DBL_EPSILON;}real_T pmf_get_real_max(void){
return DBL_MAX;}real_T pmf_get_real_min(void){return DBL_MIN;}real_T pmf_get_pi
(void){return 3.14159265358979323846;}static real_T pm__Rh5HkseM_4siH0L7B_HYm(
real_T x){real_T pm_kUQBO1dSP8_IVqRAUx4R8G=0.0;if(x>0.0){
pm_kUQBO1dSP8_IVqRAUx4R8G=1.0;}else if(x<0.0){pm_kUQBO1dSP8_IVqRAUx4R8G= -1.0;
}return pm_kUQBO1dSP8_IVqRAUx4R8G;}boolean_T pmf_is_nan(real_T x){return(x!=x)
;}boolean_T pmf_is_inf(real_T x){double const pm_FoNeuZqlVg0LaPZFB7SrM8=
pmf_get_inf();return(x==pm_FoNeuZqlVg0LaPZFB7SrM8)||(-x==
pm_FoNeuZqlVg0LaPZFB7SrM8);}real_T pmf_get_inf(void){return 1.0/
pm__Rh5HkseM_4siH0L7B_HYm(0.0);}real_T pmf_get_nan(void){return pmf_get_inf()-
pmf_get_inf();}void pmf_warning(const char*pm_V1pxxydYEnWVVi3QKesjvf,...){
va_list args;va_start(args,pm_V1pxxydYEnWVVi3QKesjvf);pmf_printf("Warning: ");
pmf_vprintf(pm_V1pxxydYEnWVVi3QKesjvf,args);pmf_printf("\n");va_end(args);}
void pmf_preformatted_warning(const char*id,const char*
pm_kvsOBKjJwlx5dTxrw7qAwS){(void)id;pmf_printf("Warning: ");pmf_printf("%s\n",
pm_kvsOBKjJwlx5dTxrw7qAwS);}void pmf_hotlink(char*buffer,int bufferSize,const
char*obj){pmf_snprintf(buffer,bufferSize,"%s",obj);}int pmf_vsnprintf_message(
char*buffer,size_t pm__lO81KuDBk41W9Wd2wAkb0,const char*format,va_list arg){
return pmf_vsnprintf(buffer,pm__lO81KuDBk41W9Wd2wAkb0,format,arg);}int
pmf_snprintf_message(char*buffer,size_t pm__dZ3R3yisKSMd19Osaf1CO,const char*
format,...){va_list args;int pm__1Zf2IciMRCub1vvbEr1C4=0;va_start(args,format)
;pm__1Zf2IciMRCub1vvbEr1C4=pmf_vsnprintf_message(buffer,
pm__dZ3R3yisKSMd19Osaf1CO,format,args);va_end(args);return
pm__1Zf2IciMRCub1vvbEr1C4;}boolean_T pm_FdhLvQWWbUtYVTa73QP1EP(void){return
false;}real_T pmf_asin(real_T x){return asin(x<= -1.0?-1.0:(x>=1.0?1.0:x));}
real_T pmf_asin_nfp(real_T x){return asin(x);}real_T pmf_acos(real_T x){return
acos(x<= -1.0?-1.0:(x>=1.0?1.0:x));}real_T pmf_acos_nfp(real_T x){return acos(
x);}real_T pmf_log(real_T x){return log(x<=2.5e-308?2.5e-308:x);}real_T
pmf_log_nfp(real_T x){return log(x);}real_T pmf_log10(real_T x){return log10(x
<=2.5e-308?2.5e-308:x);}real_T pmf_log10_nfp(real_T x){return log10(x);}real_T
pmf_pow(real_T x,real_T pm_FzyLWRgau0pMYq2XSI3ETL){real_T
pm__Od_D9zGMgKm_HQMy_79Qq=0;if(x<0.0&&pm_FzyLWRgau0pMYq2XSI3ETL!=floor(
pm_FzyLWRgau0pMYq2XSI3ETL)){pm__Od_D9zGMgKm_HQMy_79Qq= -pow(-x,
pm_FzyLWRgau0pMYq2XSI3ETL);}else{pm__Od_D9zGMgKm_HQMy_79Qq=pow(x,
pm_FzyLWRgau0pMYq2XSI3ETL);}if(pm__Od_D9zGMgKm_HQMy_79Qq>=1.7e+308){
pm__Od_D9zGMgKm_HQMy_79Qq=1.7e+308;}else if(pm__Od_D9zGMgKm_HQMy_79Qq<= -
1.7e+308){pm__Od_D9zGMgKm_HQMy_79Qq= -1.7e+308;}return
pm__Od_D9zGMgKm_HQMy_79Qq;}real_T pmf_pow_nfp(real_T x,real_T
pm_FzyLWRgau0pMYq2XSI3ETL){return pow(x,pm_FzyLWRgau0pMYq2XSI3ETL);}real_T
pmf_sqrt(real_T x){if(x<0.0){return-sqrt(-x);}return sqrt(x);}real_T
pmf_sqrt_nfp(real_T x){return sqrt(x);}real_T pmf_exp(real_T x){const real_T
pm_kBwivO3jVcxZjm0iF26HsG=663.6751350333474;if(x>=pm_kBwivO3jVcxZjm0iF26HsG){
return exp(pm_kBwivO3jVcxZjm0iF26HsG);}return exp(x);}real_T pmf_exp_nfp(
real_T x){return exp(x);}
