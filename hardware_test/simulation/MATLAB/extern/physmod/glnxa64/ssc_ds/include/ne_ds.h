#ifndef __ne_ds_h__
#define __ne_ds_h__
#include "stddef.h"
#include "pm_inline.h"
#include "pm_log.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeDynamicSystemTag NeDynamicSystem;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
enum{NE_NODAL_EQ_FLAG=1<<0,NE_SIGNAL_EQ_FLAG=1<<1,NE_BRANCH_EQ_FLAG=1<<2,
NE_GENERAL_EQ_FLAG=1<<3,NE_TIME_EQ_FLAG=1<<4,NE_THROUGH_EQ_FLAG=1<<5,
NE_INTEG_EQ_FLAG=1<<6,NE_NO_EQ_FLAGS=0};enum{
NE_RESERVED_FOR_FUTURE_USE_CER_FLAG=1<<0,NE_NO_CER_FLAGS=0};enum{
NE_INITIAL_ICR_FLAG=1<<0,NE_NO_ICR_FLAGS=0};enum{NE_NODAL_VAR_FLAG=1<<0,
NE_OUTPUT_VAR_FLAG=1<<1,NE_INPUT_VAR_FLAG=1<<2,NE_SIGNAL_VAR_FLAG=1<<3,
NE_TIME_VAR_FLAG=1<<4,NE_INTEG_VAR_FLAG=1<<5,NE_NO_VAR_FLAGS=0};typedef union
NeDsMethodOutputTag{PmSparsityPattern mM_P;PmRealVector mM;PmBoolVector mVMM;
PmSparsityPattern mDXM_P;PmRealVector mDXM;PmSparsityPattern mDDM_P;
PmRealVector mDDM;PmSparsityPattern mDUM_P;PmRealVector mDUM;PmSparsityPattern
mDTM_P;PmRealVector mDTM;PmSparsityPattern mDPM_P;PmRealVector mDPM;
PmSparsityPattern mA_P;PmRealVector mA;PmSparsityPattern mB_P;PmRealVector mB;
PmSparsityPattern mC_P;PmRealVector mC;PmRealVector mF;PmBoolVector mVMF;
PmBoolVector mVPF;PmBoolVector mVSF;PmBoolVector mSLF;PmBoolVector mSLF0;
PmSparsityPattern mDXF_P;PmRealVector mDXF;PmSparsityPattern mDUF_P;
PmRealVector mDUF;PmSparsityPattern mDTF_P;PmRealVector mDTF;PmSparsityPattern
mDDF_P;PmRealVector mDDF;PmSparsityPattern mDPDXF_P;PmRealVector mDPDXF;
PmSparsityPattern mDWF_P;PmRealVector mDWF;PmSparsityPattern mTDUF_P;
PmSparsityPattern mTDXF_P;PmSparsityPattern mDNF_P;PmRealVector mDNF;
PmBoolVector mDNF_V_X;PmRealVector mCER;PmRealVector mDXCER;PmSparsityPattern
mDXCER_P;PmRealVector mDDCER;PmSparsityPattern mDDCER_P;PmRealVector mIC;
PmRealVector mICR;PmIntVector mICR_IM;PmBoolVector mICR_ID;PmBoolVector mICR_IL
;PmRealVector mDXICR;PmSparsityPattern mDXICR_P;PmRealVector mDDICR;
PmSparsityPattern mDDICR_P;PmSparsityPattern mTDUICR_P;PmSparsityPattern
mICRM_P;PmRealVector mICRM;PmSparsityPattern mDXICRM_P;PmRealVector mDXICRM;
PmSparsityPattern mDDICRM_P;PmRealVector mDDICRM;PmRealVector mFREQS;
PmRealVector mSOLVERHITS;PmSparsityPattern mMDUY_P;PmSparsityPattern mMDXY_P;
PmSparsityPattern mTDUY_P;PmSparsityPattern mTDXY_P;PmRealVector mY;
PmSparsityPattern mDXY_P;PmRealVector mDXY;PmSparsityPattern mDUY_P;
PmRealVector mDUY;PmSparsityPattern mDTY_P;PmRealVector mDTY;PmIntVector mMODE
;PmRealVector mZC;PmRealVector mCACHE_R;PmIntVector mCACHE_I;PmRealVector
mUPDATE_R;PmIntVector mUPDATE_I;PmRealVector mUPDATE2_R;PmIntVector mUPDATE2_I
;PmBoolVector mLOCK_R;PmBoolVector mLOCK_I;PmBoolVector mLOCK2_R;PmBoolVector
mLOCK2_I;PmRealVector mSFO;PmRealVector mSFP;PmRealVector mINIT_R;PmIntVector
mINIT_I;PmRealVector mLOG;PmIntVector mASSERT;PmIntVector mPASSERT;PmIntVector
mIASSERT;PmRealVector mDEL_T;PmRealVector mDEL_V;PmRealVector mDEL_V0;
PmRealVector mDEL_TMAX;PmSparsityPattern mDXDELT_P;PmRealVector mDXDELT;
PmSparsityPattern mDUDELT_P;PmRealVector mDUDELT;PmSparsityPattern mDTDELT_P;
PmRealVector mDTDELT;PmRealVector mOBS_EXP;PmRealVector mOBS_ACT;PmRealVector
mOBS_ALL;PmBoolVector mOBS_IL;PmIntVector mDP_L;PmIntVector mDP_I;PmIntVector
mDP_J;PmRealVector mDP_R;PmRealVector mQX;PmRealVector mQU;PmRealVector mQT;
PmRealVector mQ1;PmSparsityPattern mQX_P;PmSparsityPattern mQU_P;
PmSparsityPattern mQT_P;PmSparsityPattern mQ1_P;PmRealVector mVAR_TOL;
PmRealVector mEQ_TOL;PmBoolVector mLV;PmBoolVector mSLV;PmBoolVector mNLDV;
PmBoolVector mSCLV;PmRealVector mIMIN;PmRealVector mIMAX;PmRealVector mDIMIN;
PmRealVector mDIMAX;}NeDsMethodOutput;typedef enum NeDsMethodIdTag{
NE_INVALID_METHOD_ID= -1,NE_DS_METHOD_M_P,NE_DS_METHOD_M,NE_DS_METHOD_VMM,
NE_DS_METHOD_DXM_P,NE_DS_METHOD_DXM,NE_DS_METHOD_DDM_P,NE_DS_METHOD_DDM,
NE_DS_METHOD_DUM_P,NE_DS_METHOD_DUM,NE_DS_METHOD_DTM_P,NE_DS_METHOD_DTM,
NE_DS_METHOD_DPM_P,NE_DS_METHOD_DPM,NE_DS_METHOD_A_P,NE_DS_METHOD_A,
NE_DS_METHOD_B_P,NE_DS_METHOD_B,NE_DS_METHOD_C_P,NE_DS_METHOD_C,NE_DS_METHOD_F
,NE_DS_METHOD_VMF,NE_DS_METHOD_VPF,NE_DS_METHOD_VSF,NE_DS_METHOD_SLF,
NE_DS_METHOD_SLF0,NE_DS_METHOD_DXF_P,NE_DS_METHOD_DXF,NE_DS_METHOD_DUF_P,
NE_DS_METHOD_DUF,NE_DS_METHOD_DTF_P,NE_DS_METHOD_DTF,NE_DS_METHOD_DDF_P,
NE_DS_METHOD_DDF,NE_DS_METHOD_DPDXF_P,NE_DS_METHOD_DPDXF,NE_DS_METHOD_DWF_P,
NE_DS_METHOD_DWF,NE_DS_METHOD_TDUF_P,NE_DS_METHOD_TDXF_P,NE_DS_METHOD_DNF_P,
NE_DS_METHOD_DNF,NE_DS_METHOD_DNF_V_X,NE_DS_METHOD_CER,NE_DS_METHOD_DXCER,
NE_DS_METHOD_DXCER_P,NE_DS_METHOD_DDCER,NE_DS_METHOD_DDCER_P,NE_DS_METHOD_IC,
NE_DS_METHOD_ICR,NE_DS_METHOD_ICR_IM,NE_DS_METHOD_ICR_ID,NE_DS_METHOD_ICR_IL,
NE_DS_METHOD_DXICR,NE_DS_METHOD_DXICR_P,NE_DS_METHOD_DDICR,
NE_DS_METHOD_DDICR_P,NE_DS_METHOD_TDUICR_P,NE_DS_METHOD_ICRM_P,
NE_DS_METHOD_ICRM,NE_DS_METHOD_DXICRM_P,NE_DS_METHOD_DXICRM,
NE_DS_METHOD_DDICRM_P,NE_DS_METHOD_DDICRM,NE_DS_METHOD_FREQS,
NE_DS_METHOD_SOLVERHITS,NE_DS_METHOD_MDUY_P,NE_DS_METHOD_MDXY_P,
NE_DS_METHOD_TDUY_P,NE_DS_METHOD_TDXY_P,NE_DS_METHOD_Y,NE_DS_METHOD_DXY_P,
NE_DS_METHOD_DXY,NE_DS_METHOD_DUY_P,NE_DS_METHOD_DUY,NE_DS_METHOD_DTY_P,
NE_DS_METHOD_DTY,NE_DS_METHOD_MODE,NE_DS_METHOD_ZC,NE_DS_METHOD_CACHE_R,
NE_DS_METHOD_CACHE_I,NE_DS_METHOD_UPDATE_R,NE_DS_METHOD_UPDATE_I,
NE_DS_METHOD_UPDATE2_R,NE_DS_METHOD_UPDATE2_I,NE_DS_METHOD_LOCK_R,
NE_DS_METHOD_LOCK_I,NE_DS_METHOD_LOCK2_R,NE_DS_METHOD_LOCK2_I,NE_DS_METHOD_SFO
,NE_DS_METHOD_SFP,NE_DS_METHOD_INIT_R,NE_DS_METHOD_INIT_I,NE_DS_METHOD_LOG,
NE_DS_METHOD_ASSERT,NE_DS_METHOD_PASSERT,NE_DS_METHOD_IASSERT,
NE_DS_METHOD_DEL_T,NE_DS_METHOD_DEL_V,NE_DS_METHOD_DEL_V0,
NE_DS_METHOD_DEL_TMAX,NE_DS_METHOD_DXDELT_P,NE_DS_METHOD_DXDELT,
NE_DS_METHOD_DUDELT_P,NE_DS_METHOD_DUDELT,NE_DS_METHOD_DTDELT_P,
NE_DS_METHOD_DTDELT,NE_DS_METHOD_OBS_EXP,NE_DS_METHOD_OBS_ACT,
NE_DS_METHOD_OBS_ALL,NE_DS_METHOD_OBS_IL,NE_DS_METHOD_DP_L,NE_DS_METHOD_DP_I,
NE_DS_METHOD_DP_J,NE_DS_METHOD_DP_R,NE_DS_METHOD_QX,NE_DS_METHOD_QU,
NE_DS_METHOD_QT,NE_DS_METHOD_Q1,NE_DS_METHOD_QX_P,NE_DS_METHOD_QU_P,
NE_DS_METHOD_QT_P,NE_DS_METHOD_Q1_P,NE_DS_METHOD_VAR_TOL,NE_DS_METHOD_EQ_TOL,
NE_DS_METHOD_LV,NE_DS_METHOD_SLV,NE_DS_METHOD_NLDV,NE_DS_METHOD_SCLV,
NE_DS_METHOD_IMIN,NE_DS_METHOD_IMAX,NE_DS_METHOD_DIMIN,NE_DS_METHOD_DIMAX,
NE_NUM_METHODS}NeDsMethodId;typedef int32_T(*NeDsMethod)(const NeDynamicSystem
*,const NeDynamicSystemInput*,NeDsMethodOutput*);typedef NeDsMethodOutput*(*
NeDsMakeOutputFcn)(const NeDynamicSystem*,PmAllocator*);typedef void(*
NeDsManagementFcn)(NeDynamicSystem*);typedef void(*NeDsGenerateFcn)(const
NeDynamicSystem*,const NeCgParams*,NeCgResults*);typedef void(*
NeDsSetupLoggerFcn)(const NeDynamicSystem*,NeLoggerBuilder*);typedef
NeDynamicSystem*(*NeDsCreateDsFcn)(NeDynamicSystem*,boolean_T);typedef void(*
NeDsExpandFcn)(const NeDynamicSystem*,const NeDynamicSystemInput*,PmRealVector
*,boolean_T);typedef void(*NeDsExpandParamFcn)(const NeDynamicSystem*,const
NeDynamicSystemInput*,PmRealVector*,boolean_T);typedef void(*NeDsRtpMapFcn)(
const NeDynamicSystem*,const PmIntVector*,const PmIntVector*,const PmIntVector
*,const PmRealVector*,PmIntVector*,PmIntVector*,PmIntVector*,PmRealVector*);
struct NeDynamicSystemTag{size_t mNumIo[NUM_IO_TYPES];const SscIoInfo*mIo[
NUM_IO_TYPES];size_t mNumVariables;size_t mNumDiscreteRealVariables;size_t
mNumDifferentialVariables;size_t mNumEquations;size_t mNumCEResiduals;size_t
mNumICResiduals;size_t mNumFreqs;size_t mNumSolverHits;size_t mNumModes;size_t
mNumMajorModes;size_t mNumRealCache;size_t mNumIntCache;size_t mNumObservables
;size_t mNumObservableElements;size_t mNumZcs;size_t mNumAsserts;size_t
mNumAssertRanges;size_t mNumParamAsserts;size_t mNumParamAssertRanges;size_t
mNumInitialAsserts;size_t mNumInitialAssertRanges;size_t mNumRanges;size_t
mNumEquationRanges;size_t mNumCERRanges;size_t mNumICRRanges;size_t
mNumFundamentalSamples;size_t mNumDelays;size_t mNumLogicalParameters;size_t
mNumIntegerParameters;size_t mNumIndexParameters;size_t mNumRealParameters;
size_t mNumLogicalDerivedParameters;size_t mNumIntegerDerivedParameters;size_t
mNumIndexDerivedParameters;size_t mNumRealDerivedParameters;boolean_T
mIsOutputLinear;boolean_T mIsOutputSwitchedLinear;boolean_T mIsScalable;
NeDsManagementFcn mReleaseReference;NeDsManagementFcn mGetReference;NeDsMethod
mMethods[NE_NUM_METHODS];NeDsMakeOutputFcn mMakeOutput[NE_NUM_METHODS];
NeDsSetupLoggerFcn mSetupLogger;NeEquationData*mEquationData;NeCERData*
mCERData;NeICRData*mICRData;NeVariableData*mVariableData;NeVariableData*
mDiscreteData;NeModeData*mMajorModeData;NeObservableData*mObservableData;
NeZCData*mZCData;NeAssertData*mAssertData;NeRange*mAssertRanges;NeAssertData*
mParamAssertData;NeRange*mParamAssertRanges;NeAssertData*mInitialAssertData;
NeRange*mInitialAssertRanges;NeRange*mRanges;NeRange*mEquationRanges;NeRange*
mCERRanges;NeRange*mICRRanges;NeParameterData*mLogicalParameters;
NeParameterData*mIntegerParameters;NeParameterData*mIndexParameters;
NeParameterData*mRealParameters;void*mLargeArrayData;size_t mNumLargeArray;
size_t mNumIntModes;int*mIntModes;NeDsGenerateFcn mGenerateFcn;NeDsCreateDsFcn
mDiagnosticsDsFcn;NeDsExpandFcn mExpandFcn;NeDsExpandParamFcn mExpandParamFcn;
NeDsRtpMapFcn mRtpMapFcn;NeChecksum mChecksum;};PMF_DEPLOY_STATIC void
ne_ds_release_reference(NeDynamicSystem*dynSys){if(dynSys){dynSys->
mReleaseReference(dynSys);}}PMF_DEPLOY_STATIC void ne_ds_get_reference(
NeDynamicSystem*dynSys){if(dynSys){dynSys->mGetReference(dynSys);}}
PMF_DEPLOY_STATIC int32_T ne_ds_call_method(const NeDynamicSystem*dynSys,
NeDsMethodId id,const NeDynamicSystemInput*input,NeDsMethodOutput*output){
return dynSys->mMethods[id](dynSys,input,output);}PMF_DEPLOY_STATIC
NeDsMethodOutput*ne_ds_default_output(const NeDynamicSystem*dynSys,
NeDsMethodId id,PmAllocator*allocator){return dynSys->mMakeOutput[id](dynSys,
allocator);}PMF_DEPLOY_STATIC NeDynamicSystemInputSizes
ne_ds_get_system_input_sizes(const NeDynamicSystem*ds){
NeDynamicSystemInputSizes sizes;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_Q]=ds
->mNumMajorModes;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_M]=ds->mNumModes;
sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_T]=1;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_U]=ssc_get_num_io_signals(ds->mIo[SSC_INPUT_IO_TYPE
],ds->mNumIo[SSC_INPUT_IO_TYPE]);sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_V]=
ssc_get_num_io_signals(ds->mIo[SSC_INPUT_IO_TYPE],ds->mNumIo[SSC_INPUT_IO_TYPE
]);sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_X]=ds->mNumVariables;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_D]=ds->mNumDiscreteRealVariables;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_E]=ds->mNumFundamentalSamples;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_CR]=ds->mNumRealCache;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_CI]=ds->mNumIntCache;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_W]=ds->mNumDelays;sizes.mSizes[
NE_DYNAMIC_SYSTEM_INPUT_ID_S]=1;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_P_L]=
ds->mNumLogicalParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_P_I]=ds->
mNumIntegerParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_P_J]=ds->
mNumIndexParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_P_R]=ds->
mNumRealParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_DP_L]=ds->
mNumLogicalDerivedParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_DP_I]=ds
->mNumIntegerDerivedParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_DP_J]=
ds->mNumIndexDerivedParameters;sizes.mSizes[NE_DYNAMIC_SYSTEM_INPUT_ID_DP_R]=
ds->mNumRealDerivedParameters;return sizes;}PMF_DEPLOY_STATIC void
ne_ds_setup_logger(const NeDynamicSystem*dynSys,NeLoggerBuilder*builder){
dynSys->mSetupLogger(dynSys,builder);}
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ne_ds_h__ */
