#include "pm_std.h"
#include "string.h"
#include "pm_std.h"
#include "pm_std.h"
struct sm__86ZcO1AmuChiX5wQ615hE{size_t sm_VobqE_as4__3W9a6zbgq0x;unsigned char
*sm_FgUP9l3DqHWwgy6WveH3ZJ;};typedef struct sm__86ZcO1AmuChiX5wQ615hE
sm_koCjJG_z1QWpXeGI5fHIzh;void sm_core_SmBoundedSet_create(
sm_koCjJG_z1QWpXeGI5fHIzh*sm_Fnt04uhLkDWuYXj9o_cpB6,size_t
sm_VUsSkoTm1_CqXeiwqxIZO4);void sm_core_SmBoundedSet_copy(
sm_koCjJG_z1QWpXeGI5fHIzh*sm_kyVvGx32uvlKVTEMWrNekp,const
sm_koCjJG_z1QWpXeGI5fHIzh*orig);void sm_core_SmBoundedSet_assign(
sm_koCjJG_z1QWpXeGI5fHIzh*dst,const sm_koCjJG_z1QWpXeGI5fHIzh*src);void
sm_core_SmBoundedSet_destroy(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6);size_t sm_core_SmBoundedSet_bound(const
sm_koCjJG_z1QWpXeGI5fHIzh*sm_Fnt04uhLkDWuYXj9o_cpB6);size_t
sm_core_SmBoundedSet_size(const sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6);boolean_T sm_core_SmBoundedSet_isMember(const
sm_koCjJG_z1QWpXeGI5fHIzh*sm_Fnt04uhLkDWuYXj9o_cpB6,size_t
sm_FL1llpmubk_sXeD10Sbe3f);void sm_core_SmBoundedSet_clear(
sm_koCjJG_z1QWpXeGI5fHIzh*sm_Fnt04uhLkDWuYXj9o_cpB6);void
sm_core_SmBoundedSet_insert(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,size_t sm_FL1llpmubk_sXeD10Sbe3f);void
sm_core_SmBoundedSet_remove(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,size_t sm_FL1llpmubk_sXeD10Sbe3f);void
sm_core_SmBoundedSet_complement(const sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,sm_koCjJG_z1QWpXeGI5fHIzh*sm_FWjmyhJaz5WOf5x4yKSbtc)
;void sm_core_SmBoundedSet_intersection(const sm_koCjJG_z1QWpXeGI5fHIzh*a,
const sm_koCjJG_z1QWpXeGI5fHIzh*b,sm_koCjJG_z1QWpXeGI5fHIzh*
sm_FFZbGh27ya8eem_J_hUtAZ);void sm_core_SmBoundedSet_union(const
sm_koCjJG_z1QWpXeGI5fHIzh*a,const sm_koCjJG_z1QWpXeGI5fHIzh*b,
sm_koCjJG_z1QWpXeGI5fHIzh*sm_FFZbGh27ya8eem_J_hUtAZ);void
sm_core_SmBoundedSet_difference(const sm_koCjJG_z1QWpXeGI5fHIzh*a,const
sm_koCjJG_z1QWpXeGI5fHIzh*b,sm_koCjJG_z1QWpXeGI5fHIzh*
sm_FFZbGh27ya8eem_J_hUtAZ);void sm_core_SmBoundedSet_create(
sm_koCjJG_z1QWpXeGI5fHIzh*sm_Fnt04uhLkDWuYXj9o_cpB6,size_t
sm_VUsSkoTm1_CqXeiwqxIZO4){size_t sm_kwrB3ZoKf7OufTHWaHJV7a;
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_VobqE_as4__3W9a6zbgq0x=sm_VUsSkoTm1_CqXeiwqxIZO4
;sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ=pmf_malloc(
sm_VUsSkoTm1_CqXeiwqxIZO4*sizeof(unsigned char));(void)0;;for(
sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<
sm_VUsSkoTm1_CqXeiwqxIZO4;++sm_kwrB3ZoKf7OufTHWaHJV7a)
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a
]=0;}void sm_core_SmBoundedSet_copy(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_kyVvGx32uvlKVTEMWrNekp,const sm_koCjJG_z1QWpXeGI5fHIzh*orig){const size_t n
=orig->sm_VobqE_as4__3W9a6zbgq0x;sm_kyVvGx32uvlKVTEMWrNekp->
sm_VobqE_as4__3W9a6zbgq0x=n;sm_kyVvGx32uvlKVTEMWrNekp->
sm_FgUP9l3DqHWwgy6WveH3ZJ=pmf_malloc(n*sizeof(unsigned char));(void)0;;memcpy(
sm_kyVvGx32uvlKVTEMWrNekp->sm_FgUP9l3DqHWwgy6WveH3ZJ,orig->
sm_FgUP9l3DqHWwgy6WveH3ZJ,n*sizeof(unsigned char));}void
sm_core_SmBoundedSet_assign(sm_koCjJG_z1QWpXeGI5fHIzh*dst,const
sm_koCjJG_z1QWpXeGI5fHIzh*src){if(dst!=src){const size_t n=src->
sm_VobqE_as4__3W9a6zbgq0x;(void)0;;memcpy(dst->sm_FgUP9l3DqHWwgy6WveH3ZJ,src->
sm_FgUP9l3DqHWwgy6WveH3ZJ,n*sizeof(unsigned char));}}void
sm_core_SmBoundedSet_destroy(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6){pmf_free(sm_Fnt04uhLkDWuYXj9o_cpB6->
sm_FgUP9l3DqHWwgy6WveH3ZJ);sm_Fnt04uhLkDWuYXj9o_cpB6->
sm_VobqE_as4__3W9a6zbgq0x=0;}size_t sm_core_SmBoundedSet_bound(const
sm_koCjJG_z1QWpXeGI5fHIzh*sm_Fnt04uhLkDWuYXj9o_cpB6){return
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_VobqE_as4__3W9a6zbgq0x;}size_t
sm_core_SmBoundedSet_size(const sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6){size_t size=0;const unsigned char*
sm_kDScWWGnJ_StZmXNHaJBpR=sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ
,*sm_Vs6xonQX17Krc5vXv8T_zq=sm_kDScWWGnJ_StZmXNHaJBpR+
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_VobqE_as4__3W9a6zbgq0x;for(;
sm_kDScWWGnJ_StZmXNHaJBpR<sm_Vs6xonQX17Krc5vXv8T_zq;++
sm_kDScWWGnJ_StZmXNHaJBpR)if(*sm_kDScWWGnJ_StZmXNHaJBpR!=0)++size;return size;
}boolean_T sm_core_SmBoundedSet_isMember(const sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,size_t sm_FL1llpmubk_sXeD10Sbe3f){(void)0;;return
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_FL1llpmubk_sXeD10Sbe3f
]!=0;}void sm_core_SmBoundedSet_clear(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6){unsigned char*sm_kDScWWGnJ_StZmXNHaJBpR=
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ,*
sm_Vs6xonQX17Krc5vXv8T_zq=sm_kDScWWGnJ_StZmXNHaJBpR+sm_Fnt04uhLkDWuYXj9o_cpB6
->sm_VobqE_as4__3W9a6zbgq0x;while(sm_kDScWWGnJ_StZmXNHaJBpR<
sm_Vs6xonQX17Krc5vXv8T_zq)*sm_kDScWWGnJ_StZmXNHaJBpR++=0;}void
sm_core_SmBoundedSet_insert(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,size_t sm_FL1llpmubk_sXeD10Sbe3f){(void)0;;
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_FL1llpmubk_sXeD10Sbe3f
]=1;}void sm_core_SmBoundedSet_remove(sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,size_t sm_FL1llpmubk_sXeD10Sbe3f){(void)0;;
sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_FL1llpmubk_sXeD10Sbe3f
]=0;}void sm_core_SmBoundedSet_complement(const sm_koCjJG_z1QWpXeGI5fHIzh*
sm_Fnt04uhLkDWuYXj9o_cpB6,sm_koCjJG_z1QWpXeGI5fHIzh*sm_FWjmyhJaz5WOf5x4yKSbtc)
{const size_t n=sm_Fnt04uhLkDWuYXj9o_cpB6->sm_VobqE_as4__3W9a6zbgq0x;size_t
sm_kwrB3ZoKf7OufTHWaHJV7a;(void)0;;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<n;++sm_kwrB3ZoKf7OufTHWaHJV7a)
sm_FWjmyhJaz5WOf5x4yKSbtc->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a
]=(sm_Fnt04uhLkDWuYXj9o_cpB6->sm_FgUP9l3DqHWwgy6WveH3ZJ[
sm_kwrB3ZoKf7OufTHWaHJV7a]==0?1:0);}void sm_core_SmBoundedSet_intersection(
const sm_koCjJG_z1QWpXeGI5fHIzh*a,const sm_koCjJG_z1QWpXeGI5fHIzh*b,
sm_koCjJG_z1QWpXeGI5fHIzh*sm_FFZbGh27ya8eem_J_hUtAZ){const size_t n=
sm_FFZbGh27ya8eem_J_hUtAZ->sm_VobqE_as4__3W9a6zbgq0x;size_t
sm_kwrB3ZoKf7OufTHWaHJV7a;(void)0;;(void)0;;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<n;++sm_kwrB3ZoKf7OufTHWaHJV7a)
sm_FFZbGh27ya8eem_J_hUtAZ->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a
]=(a->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a]!=0&&b->
sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a]!=0)?1:0;}void
sm_core_SmBoundedSet_union(const sm_koCjJG_z1QWpXeGI5fHIzh*a,const
sm_koCjJG_z1QWpXeGI5fHIzh*b,sm_koCjJG_z1QWpXeGI5fHIzh*
sm_FFZbGh27ya8eem_J_hUtAZ){const size_t n=sm_FFZbGh27ya8eem_J_hUtAZ->
sm_VobqE_as4__3W9a6zbgq0x;size_t sm_kwrB3ZoKf7OufTHWaHJV7a;(void)0;;(void)0;;
for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;sm_kwrB3ZoKf7OufTHWaHJV7a<n;++
sm_kwrB3ZoKf7OufTHWaHJV7a)sm_FFZbGh27ya8eem_J_hUtAZ->sm_FgUP9l3DqHWwgy6WveH3ZJ
[sm_kwrB3ZoKf7OufTHWaHJV7a]=(a->sm_FgUP9l3DqHWwgy6WveH3ZJ[
sm_kwrB3ZoKf7OufTHWaHJV7a]!=0||b->sm_FgUP9l3DqHWwgy6WveH3ZJ[
sm_kwrB3ZoKf7OufTHWaHJV7a]!=0)?1:0;}void sm_core_SmBoundedSet_difference(const
sm_koCjJG_z1QWpXeGI5fHIzh*a,const sm_koCjJG_z1QWpXeGI5fHIzh*b,
sm_koCjJG_z1QWpXeGI5fHIzh*sm_FFZbGh27ya8eem_J_hUtAZ){const size_t n=
sm_FFZbGh27ya8eem_J_hUtAZ->sm_VobqE_as4__3W9a6zbgq0x;size_t
sm_kwrB3ZoKf7OufTHWaHJV7a;(void)0;;(void)0;;for(sm_kwrB3ZoKf7OufTHWaHJV7a=0;
sm_kwrB3ZoKf7OufTHWaHJV7a<n;++sm_kwrB3ZoKf7OufTHWaHJV7a)
sm_FFZbGh27ya8eem_J_hUtAZ->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a
]=(a->sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a]!=0&&b->
sm_FgUP9l3DqHWwgy6WveH3ZJ[sm_kwrB3ZoKf7OufTHWaHJV7a]==0)?1:0;}
