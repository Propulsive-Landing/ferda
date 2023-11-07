#ifndef __ne_std_h__
#define __ne_std_h__
#include "stddef.h"
#include "string.h"
#include "pm_inline.h"
#include "pm_std.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct SscArraySizeTag{size_t dimension[2];}SscArraySize;
PMF_DEPLOY_STATIC void ssc_array_size_set_scalar(SscArraySize*size){
SscArraySize const x={{1,1}};memcpy(size,&x,sizeof(SscArraySize));}
PMF_DEPLOY_STATIC SscArraySize ssc_make_scalar_array_size(void){SscArraySize
out;ssc_array_size_set_scalar(&out);return out;}PMF_DEPLOY_STATIC size_t
ssc_array_size_num_dimensions(SscArraySize const*size){(void)size;return 2;}
PMF_DEPLOY_STATIC size_t ssc_array_size_get_dim(SscArraySize const*size,size_t
d){(void)0;;return size->dimension[d];}PMF_DEPLOY_STATIC void
ssc_array_size_set_dim(SscArraySize*size,size_t d,size_t value){(void)0;;size
->dimension[d]=value;}PMF_DEPLOY_STATIC size_t ssc_array_size_get_m(
SscArraySize const*size){return ssc_array_size_get_dim(size,0);}
PMF_DEPLOY_STATIC size_t ssc_array_size_get_n(SscArraySize const*size){return
ssc_array_size_get_dim(size,1);}PMF_DEPLOY_STATIC void ssc_array_size_set_m(
SscArraySize*size,size_t m){ssc_array_size_set_dim(size,0,m);}
PMF_DEPLOY_STATIC void ssc_array_size_set_n(SscArraySize*size,size_t n){
ssc_array_size_set_dim(size,1,n);}PMF_DEPLOY_STATIC size_t
ssc_array_size_num_elements(SscArraySize const*size){size_t out=size->
dimension[0];size_t i=0;size_t const nDims=ssc_array_size_num_dimensions(size)
;for(i=1;i<nDims;++i){out*=size->dimension[i];}return out;}typedef struct
NeCgParamsTag NeCgParams;typedef struct NeCgResultsTag NeCgResults;typedef
struct NeuDiagnosticTreeTag NeuDiagnosticTree;typedef struct
NeuDiagnosticManagerTag NeuDiagnosticManager;typedef enum{
NEU_DIAGNOSTIC_LEVEL_INVALID= -1,NEU_DIAGNOSTIC_LEVEL_TERSE,
NEU_DIAGNOSTIC_LEVEL_VERBOSE,NEU_NUM_DIAGNOSTIC_LEVELS}NeuDiagnosticLevel;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeSystemInputTag{PmIntVector mQ;PmIntVector mM;PmRealVector mT;
PmRealVector mU;PmRealVector mV;PmRealVector mX;PmRealVector mD;PmIntVector mE
;PmRealVector mCR;PmIntVector mCI;}NeSystemInput;typedef enum
NeSystemInputIdTag{NE_INVALID_SYSTEM_INPUT_ID= -1,NE_SYSTEM_INPUT_ID_Q,
NE_SYSTEM_INPUT_ID_M,NE_SYSTEM_INPUT_ID_T,NE_SYSTEM_INPUT_ID_U,
NE_SYSTEM_INPUT_ID_V,NE_SYSTEM_INPUT_ID_X,NE_SYSTEM_INPUT_ID_D,
NE_SYSTEM_INPUT_ID_E,NE_SYSTEM_INPUT_ID_CR,NE_SYSTEM_INPUT_ID_CI,
NE_NUM_SYSTEM_INPUT_ID}NeSystemInputId;typedef struct NeSystemInputSizesTag{
size_t mSizes[NE_NUM_SYSTEM_INPUT_ID];}NeSystemInputSizes;NeSystemInputSizes
neu_get_empty_system_input_sizes(void);NeSystemInputSizes
neu_get_system_input_sizes(const NeSystemInput*in);NeSystemInput*
neu_create_system_input(NeSystemInputSizes sizes,PmAllocator*allocatorPtr);
void neu_destroy_system_input(NeSystemInput*sysInputPtr,PmAllocator*
allocatorPtr);void neu_si_equals_si(const NeSystemInput*destPtr,const
NeSystemInput*srcPtr);NeSystemInput*neu_copy_si(const NeSystemInput*in,
PmAllocator*al);boolean_T neu_si_equalequals_si(const NeSystemInput*destPtr,
const NeSystemInput*srcPtr);boolean_T neu_sis_equalequals_sis(const
NeSystemInputSizes left,const NeSystemInputSizes right);typedef struct
NeDynamicSystemInputTag{PmIntVector mQ;PmIntVector mM;PmRealVector mT;
PmRealVector mU;PmRealVector mV;PmRealVector mX;PmRealVector mD;PmIntVector mE
;PmRealVector mCR;PmIntVector mCI;PmRealVector mW;PmRealVector mS;PmIntVector
mP_L;PmIntVector mP_I;PmIntVector mP_J;PmRealVector mP_R;PmIntVector mDP_L;
PmIntVector mDP_I;PmIntVector mDP_J;PmRealVector mDP_R;}NeDynamicSystemInput;
typedef enum NeDynamicSystemInputIdTag{NE_INVALID_DYNAMIC_SYSTEM_INPUT_ID= -1,
NE_DYNAMIC_SYSTEM_INPUT_ID_Q,NE_DYNAMIC_SYSTEM_INPUT_ID_M,
NE_DYNAMIC_SYSTEM_INPUT_ID_T,NE_DYNAMIC_SYSTEM_INPUT_ID_U,
NE_DYNAMIC_SYSTEM_INPUT_ID_V,NE_DYNAMIC_SYSTEM_INPUT_ID_X,
NE_DYNAMIC_SYSTEM_INPUT_ID_D,NE_DYNAMIC_SYSTEM_INPUT_ID_E,
NE_DYNAMIC_SYSTEM_INPUT_ID_CR,NE_DYNAMIC_SYSTEM_INPUT_ID_CI,
NE_DYNAMIC_SYSTEM_INPUT_ID_W,NE_DYNAMIC_SYSTEM_INPUT_ID_S,
NE_DYNAMIC_SYSTEM_INPUT_ID_P_L,NE_DYNAMIC_SYSTEM_INPUT_ID_P_I,
NE_DYNAMIC_SYSTEM_INPUT_ID_P_J,NE_DYNAMIC_SYSTEM_INPUT_ID_P_R,
NE_DYNAMIC_SYSTEM_INPUT_ID_DP_L,NE_DYNAMIC_SYSTEM_INPUT_ID_DP_I,
NE_DYNAMIC_SYSTEM_INPUT_ID_DP_J,NE_DYNAMIC_SYSTEM_INPUT_ID_DP_R,
NE_NUM_DYNAMIC_SYSTEM_INPUT_ID}NeDynamicSystemInputId;typedef struct
NeDynamicSystemInputSizesTag{size_t mSizes[NE_NUM_DYNAMIC_SYSTEM_INPUT_ID];}
NeDynamicSystemInputSizes;NeDynamicSystemInputSizes
neu_get_empty_dynamic_system_input_sizes(void);NeDynamicSystemInputSizes
neu_get_dynamic_system_input_sizes(const NeDynamicSystemInput*in);
NeDynamicSystemInput*neu_create_dynamic_system_input(NeDynamicSystemInputSizes
sizes,PmAllocator*allocatorPtr);void neu_destroy_dynamic_system_input(
NeDynamicSystemInput*sysInputPtr,PmAllocator*allocatorPtr);void
neu_dsi_equals_dsi(const NeDynamicSystemInput*destPtr,const
NeDynamicSystemInput*srcPtr);NeDynamicSystemInput*neu_copy_dsi(const
NeDynamicSystemInput*in,PmAllocator*al);boolean_T neu_dsi_equalequals_dsi(
const NeDynamicSystemInput*destPtr,const NeDynamicSystemInput*srcPtr);
boolean_T neu_dsis_equalequals_dsis(const NeDynamicSystemInputSizes left,const
NeDynamicSystemInputSizes right);typedef enum NeEquationDomainTag{
NE_EQUATION_DOMAIN_INVALID= -1,NE_EQUATION_DOMAIN_TIME,
NE_EQUATION_DOMAIN_FREQUENCY_REAL,NE_EQUATION_DOMAIN_FREQUENCY_IMAG,
NE_EQUATION_DOMAIN_COMPLEX,NE_EQUATION_DOMAIN_DELAY,NE_EQUATION_DOMAIN_NUM}
NeEquationDomain;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef uint32_T NeChecksum[4];typedef enum NeVariabilityTypeTag{
NE_INVALID_VARIABILITY= -1,NE_CONSTANT_VARIABILITY,NE_PARAMETRIC_VARIABILITY,
NE_MODAL_VARIABILITY,NE_FULL_VARIABILITY}NeVariabilityType;typedef enum
NeInitModeTag{NE_INIT_MODE_INVALID= -1,NE_INIT_MODE_DEFAULT,NE_INIT_MODE_NONE,
NE_INIT_MODE_OPTIONAL,NE_INIT_MODE_MANDATORY,NE_INIT_MODE_NUM}NeInitMode;
typedef enum NeNominalSourceTag{NE_NOMINAL_SOURCE_INVALID= -1,
NE_NOMINAL_SOURCE_NONE,NE_NOMINAL_SOURCE_MODEL,NE_NOMINAL_SOURCE_BLOCK,
NE_NOMINAL_SOURCE_DERIVED,NE_NOMINAL_SOURCE_FIXED,NE_NOMINAL_SOURCE_NUM}
NeNominalSource;typedef enum NeFreqTimeTypeTag{NE_FREQTIME_TYPE_INVALID= -1,
NE_FREQTIME_TYPE_TIME,NE_FREQTIME_TYPE_FREQUENCY,NE_FREQTIME_TYPE_PARTFREQ,
NE_FREQTIME_TYPE_NUM}NeFreqTimeType;typedef unsigned int NeEquationFlag;
typedef unsigned int NeCERFlag;typedef unsigned int NeICRFlag;typedef unsigned
int NeVariableFlag;typedef struct NeEquationDataTag{const char*mFullPath;
size_t mIndex;NeEquationFlag mFlags;NeEquationDomain mDomain;const char*
mObject;size_t mNumRanges;size_t mStart;boolean_T mIsSwitchedLinear;real_T
mScale;const char*mUnit;}NeEquationData;typedef struct NeCERDataTag{const char
*mFullPath;size_t mIndex;NeCERFlag mFlags;const char*mObject;size_t mNumRanges
;size_t mStart;boolean_T mIsLinear;}NeCERData;typedef struct NeICRDataTag{
const char*mFullPath;size_t mIndex;NeICRFlag mFlags;const char*mObject;size_t
mNumRanges;size_t mStart;}NeICRData;typedef struct NeVariableDataTag{const char
*mFullPath;size_t mIndex;NeVariableFlag mFlags;const char*mObject;real_T mScale
;const char*mUnit;real_T mICValue;boolean_T mIsDifferential;boolean_T
mHasInitialRange;SscArraySize mSize;NeInitMode mInitMode;const char*
mDescription;}NeVariableData;typedef struct NeObservableDataTag{const char*
mFullPath;const char*mObject;SscArraySize mSize;const char*mUnit;real_T
mNominalValue;const char*mNominalUnit;NeNominalSource mNominalSource;
NeInitMode mInitMode;boolean_T mIsDetermined;boolean_T mIsDifferential;
NeFreqTimeType mFreqTimeType;boolean_T mIsExternal;boolean_T mIsVariable;const
char*mDescription;}NeObservableData;typedef struct NeModeDataTag{const char*
mFullPath;size_t mIndex;const char*mObject;int32_T mICValue;const char*
mDescription;}NeModeData;typedef enum NeRangeTypeIdTag{NE_RANGE_TYPE_INVALID= -
1,NE_RANGE_TYPE_NORMAL,NE_RANGE_TYPE_PROTECTED,NE_RANGE_TYPE_NA,
NE_RANGE_TYPE_NUM}NeRangeTypeId;typedef struct NeRangeTag{const char*mFileName
;size_t mBeginLine;size_t mBeginColumn;size_t mEndLine;size_t mEndColumn;
NeRangeTypeId mType;}NeRange;typedef enum NeZcTypeTag{NE_ZC_TYPE_INVALID= -1,
NE_ZC_TYPE_TRUE,NE_ZC_TYPE_FALSE,NE_ZC_TYPE_EITHER,NE_ZC_TYPE_NUM}NeZcType;
typedef struct NeZCDataTag{const char*mObject;size_t mNumRanges;size_t mStart;
const char*mPath;const char*mDescriptor;NeZcType mType;}NeZCData;typedef struct
NeAssertDataTag{const char*mObject;size_t mNumRanges;size_t mStart;const char*
mPath;const char*mDescriptor;boolean_T mIsWarn;const char*mMessage;const char*
mMessageID;}NeAssertData;typedef struct NeParameterDataTag{const char*
mFullPath;const char*mObject;const char*mDescription;size_t mIndex;size_t mM;
boolean_T mIsTop;}NeParameterData;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_inline.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef enum SscIoTypeTag{UNKNOWN_IO_TYPE= -1,SSC_INPUT_IO_TYPE,
SSC_OUTPUT_IO_TYPE,NUM_IO_TYPES}SscIoType;typedef struct SscIoInfoTag{const
char*mName;SscArraySize mSize;const char*mUnit;const char*mIdentifier;}
SscIoInfo;PMF_DEPLOY_STATIC size_t ssc_get_num_io_signals(const SscIoInfo*
io_info_ptr,size_t io_info_size){size_t i=0;size_t num_signals=0;for(i=0;i<
io_info_size;i++){num_signals+=ssc_array_size_num_elements(&io_info_ptr[i].
mSize);}return num_signals;}
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ne_std_h__ */
