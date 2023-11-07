#include "stdarg.h"
#include "stdio.h"
void pmf_vprintf(char const*format,va_list args){vprintf(format,args);}void
pmf_printf(char const*format,...){va_list args;va_start(args,format);
pmf_vprintf(format,args);va_end(args);}int pmf_vsnprintf(char*buffer,size_t
pm__lO81KuDBk41W9Wd2wAkb0,char const*format,va_list args){(void)
pm__lO81KuDBk41W9Wd2wAkb0;return vsprintf(buffer,format,args);}int pmf_snprintf
(char*buffer,size_t pm__lO81KuDBk41W9Wd2wAkb0,char const*format,...){int
pm_Fbnx9FOLhnW0X1Wd233Ai7=0;va_list args;va_start(args,format);
pm_Fbnx9FOLhnW0X1Wd233Ai7=pmf_vsnprintf(buffer,pm__lO81KuDBk41W9Wd2wAkb0,
format,args);va_end(args);return pm_Fbnx9FOLhnW0X1Wd233Ai7;}
