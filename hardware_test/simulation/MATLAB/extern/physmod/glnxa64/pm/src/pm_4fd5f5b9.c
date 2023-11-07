#include "pm_std.h"
#include "pm_std.h"
#include "pm_std.h"
PmAllocator*pm_default_allocator(void);
#include "pm_std.h"
static void*pm__W_2FzII5JlhYXMp2LFrU0(PmAllocator*a,size_t
pm_VLHhnPUiNQpve5VIL9P3O9,size_t n){(void)a;return pmf_calloc(
pm_VLHhnPUiNQpve5VIL9P3O9,n);}static void pm_VbdIR14vwnlxfuXZYqfsnb(
PmAllocator*a,void*ptr){(void)a;pmf_free(ptr);}PmAllocator*
pm_default_allocator(void){static PmAllocator pm_FKBDkHjmIDpKYeDT9WpTRH={
pm__W_2FzII5JlhYXMp2LFrU0,pm_VbdIR14vwnlxfuXZYqfsnb};return&
pm_FKBDkHjmIDpKYeDT9WpTRH;}
