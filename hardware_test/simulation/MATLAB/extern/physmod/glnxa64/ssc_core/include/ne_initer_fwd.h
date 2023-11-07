#ifndef __ne_initer_fwd_h__
#define __ne_initer_fwd_h__
#include "ne_stiffness_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct SscIniterTag SscIniter;typedef enum IniterTypeTag{
INITER_TYPE_INVALID= -1,INITER_TYPE_AC,INITER_TYPE_AC_DAE,INITER_TYPE_DAE,
INITER_TYPE_COUNT}IniterType;typedef struct SscComputeStateTag SscComputeState
;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ne_initer_fwd_h__ */
