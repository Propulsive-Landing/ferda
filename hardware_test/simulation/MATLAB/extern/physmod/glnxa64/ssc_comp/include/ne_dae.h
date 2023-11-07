#ifndef __ne_dae_h__
#define __ne_dae_h__
#include "pm_inline.h"
#include "pm_std.h"
#include "pm_std.h"
#include "pm_log.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct IterationsLogCoreTag{int32_T mNumIters;boolean_T mConvergence;
boolean_T mPrecisionMet;}IterationsLogCore;typedef struct NeCustomDataSizeTag
NeCustomDataSize;typedef struct NeCustomDataTag NeCustomData;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_dae_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeRealFunctorDataTag NeRealFunctorData;typedef struct
NeRealFunctorTag NeRealFunctor;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef enum SscDiagnosticSettingTag{SSC_DIAGNOSTIC_INVALID= -1,
SSC_DIAGNOSTIC_ERROR,SSC_DIAGNOSTIC_WARNING,SSC_DIAGNOSTIC_NONE,
SSC_DIAGNOSTIC_NUM}SscDiagnosticSetting;typedef enum SscLoggingSettingTag{
SSC_LOGGING_INVALID= -1,SSC_LOGGING_OFF,SSC_LOGGING_ON,SSC_LOGGING_LOCAL,
SSC_LOGGING_NUM}SscLoggingSetting;typedef struct NeModelParametersTag
NeModelParameters;typedef enum NeSolverTypeTag{NE_SOLVER_TYPE_INVALID= -1,
NE_SOLVER_TYPE_DAE,NE_SOLVER_TYPE_ODE,NE_SOLVER_TYPE_DISCRETE,
NE_SOLVER_TYPE_NUM}NeSolverType;typedef enum NeModifyAbsTolTag{
NE_MODIFY_ABS_TOL_INVALID= -1,NE_MODIFY_ABS_TOL_NO,NE_MODIFY_ABS_TOL_YES,
NE_MODIFY_ABS_TOL_MAYBE,NE_MODIFY_ABS_TOL_NUM}NeModifyAbsTol;struct
NeModelParametersTag{NeSolverType mSolverType;real_T mSolverTolerance;real_T
mSolverAbsTol;real_T mSolverRelTol;boolean_T mVariableStepSolver;boolean_T
mIsUsingODEN;NeModifyAbsTol mSolverModifyAbsTol;real_T mFixedStepSize;real_T
mStartTime;boolean_T mLoadInitialState;boolean_T mUseSimState;boolean_T
mLinTrimCompile;SscLoggingSetting mLoggingMode;real_T mRTWModifiedTimeStamp;
boolean_T mZcDisabled;boolean_T mUseModelRefSolver;boolean_T mTargetFPGAHIL;};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeParameterBundleTag{PmIntVector mLogicalParameters;PmIntVector
mIntegerParameters;PmIntVector mIndexParameters;PmRealVector mRealParameters;}
NeParameterBundle;typedef struct NeDerivedParameterBundleTag{PmIntVector
mLogicalDerivedParameters;PmIntVector mIntegerDerivedParameters;PmIntVector
mIndexDerivedParameters;PmRealVector mRealDerivedParameters;}
NeDerivedParameterBundle;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeParameterVectorTag{size_t mN;NeParameterData*mX;}
NeParameterVector;typedef struct NeParameterInfoTag{NeParameterVector mLogicals
;NeParameterVector mIntegers;NeParameterVector mIndices;NeParameterVector
mReals;}NeParameterInfo;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_profiler_fwd.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct SlSparseMatrixTag{PmSparsityPattern*mPattern;PmRealVector*mPr;}
SlSparseMatrix;typedef struct SLMatricesTag{boolean_T mIsYAffineTermUpdated;
boolean_T mIsXP0AffineTermUpdated;size_t mOffsetX0InD;size_t mOffsetU0InD;
size_t mSlDaeNumDiscreteMajor;size_t mSlDaeNumCache;PmRealVector mAffineY;
PmRealVector mAffineXP0;PmRealVector mYX0;PmRealVector mYU0;SlSparseMatrix
mDxddotDxd;SlSparseMatrix mDxddotDu;SlSparseMatrix mDyDxd;SlSparseMatrix mDyDu
;SlSparseMatrix mDxaDxd;SlSparseMatrix mDxaDu;PmIntVector mMode;PmIntVector
mMajor;PmIntVector mCache;}SLMatrices;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef union NeDaeMethodOutputTag{PmSparsityPattern mM_P;PmRealVector mM;
PmSparsityPattern mDXM_P;PmRealVector mDXM;PmSparsityPattern mDUM_P;
PmRealVector mDUM;PmRealVector mF;PmSparsityPattern mDXF_P;PmRealVector mDXF;
PmSparsityPattern mDUF_P;PmRealVector mDUF;PmSparsityPattern mDTF_P;
PmRealVector mDTF;PmRealVector mY;PmSparsityPattern mDXY_P;PmRealVector mDXY;
PmRealVector mFREQS;PmRealVector mSOLVERHITS;PmSparsityPattern mDUY_P;
PmSparsityPattern mTDUY_P;PmRealVector mDUY;PmRealVector mXP0;PmRealVector mZC
;PmIntVector mASSERT;PmIntVector mPASSERT;PmIntVector mIASSERT;PmRealVector mSO
;PmRealVector mSP;PmIntVector mMODE;PmIntVector mUDOT_REQ;PmRealVector mDELAYS
;PmBoolVector mDXF_V_X;PmBoolVector mDUF_V_X;PmRealVector mOBS_EXP;
PmRealVector mOBS_ACT;PmRealVector mEQ_TOL;PmRealVector mNUMJAC_DX_LO;
PmRealVector mNUMJAC_DX_HI;}NeDaeMethodOutput;typedef enum NeDaeMethodIdTag{
NE_INVALID_DAE_METHOD_ID= -1,NE_DAE_METHOD_M_P,NE_DAE_METHOD_M,
NE_DAE_METHOD_DXM_P,NE_DAE_METHOD_DXM,NE_DAE_METHOD_DUM_P,NE_DAE_METHOD_DUM,
NE_DAE_METHOD_F,NE_DAE_METHOD_DXF_P,NE_DAE_METHOD_DXF,NE_DAE_METHOD_DUF_P,
NE_DAE_METHOD_DUF,NE_DAE_METHOD_DTF_P,NE_DAE_METHOD_DTF,NE_DAE_METHOD_Y,
NE_DAE_METHOD_DXY_P,NE_DAE_METHOD_DXY,NE_DAE_METHOD_FREQS,
NE_DAE_METHOD_SOLVERHITS,NE_DAE_METHOD_DUY_P,NE_DAE_METHOD_TDUY_P,
NE_DAE_METHOD_DUY,NE_DAE_METHOD_XP0,NE_DAE_METHOD_ZC,NE_DAE_METHOD_ASSERT,
NE_DAE_METHOD_PASSERT,NE_DAE_METHOD_IASSERT,NE_DAE_METHOD_SO,NE_DAE_METHOD_SP,
NE_DAE_METHOD_MODE,NE_DAE_METHOD_UDOT_REQ,NE_DAE_METHOD_DELAYS,
NE_DAE_METHOD_DXF_V_X,NE_DAE_METHOD_DUF_V_X,NE_DAE_METHOD_OBS_EXP,
NE_DAE_METHOD_OBS_ACT,NE_DAE_METHOD_EQ_TOL,NE_DAE_METHOD_NUMJAC_DX_LO,
NE_DAE_METHOD_NUMJAC_DX_HI,NE_NUM_DAE_METHODS}NeDaeMethodId;typedef enum
NeDaeSolveIdTag{NE_DAE_SOLVE_INVALID= -1,NE_DAE_SOLVE_IC_MODE,
NE_DAE_SOLVE_IC_LIS,NE_DAE_SOLVE_IC_LSS,NE_DAE_SOLVE_CIC_MODE,
NE_DAE_SOLVE_CONSAT,NE_DAE_SOLVE_CIC,NE_DAE_SOLVE_RESET,NE_DAE_SOLVE_NUDGE,
NE_DAE_SOLVE_NUM}NeDaeSolveId;typedef PmfMessageId(*NeDaeMethod)(const NeDae*,
const NeSystemInput*,NeDaeMethodOutput*,NeuDiagnosticManager*);typedef
PmfMessageId(*NeDaeSolve)(const NeDae*,const NeSystemInput*,
NeuDiagnosticManager*);typedef PmfMessageId(*NeDaeFunction)(const NeDae*,const
NeSystemInput*);typedef void(*NeDaeReportFcn)(const NeDae*);typedef
PmfMessageId(*NeDaeLinearizeFcn)(const NeDae*,const NeSystemInput*,
PmRealVector*,NeuDiagnosticManager*);typedef NeDaeMethodOutput*(*
NeDaeMakeOutputFcn)(const NeDae*,PmAllocator*);typedef void(*
NeDaeFreeOutputFcn)(NeDaeMethodOutput*,PmAllocator*);typedef NeRealFunctor*(*
NeRealFunctorFcn)(const NeDae*dae,NeDaeMethodId id,PmAllocator*allocator);
typedef void(*NeDaeSetParameters)(const NeDae*,const NeParameterBundle*);
typedef struct StateTracerTag StateTracer;typedef struct NeDaePrivateDataTag
NeDaePrivateData;struct NeDaeTag{size_t mNumStates;size_t
mNumDifferentialStates;size_t mNumDiscreteStates;size_t mNumInputs;size_t
mInputOrder;size_t mNumOutputs;size_t mNumMassMatrixNzMax;size_t
mNumTrimResiduals;size_t mNumLinJacobianNzMax;size_t mNumModes;size_t
mNumMajorModes;size_t mNumRealCache;size_t mNumIntCache;size_t mNumZcs;size_t
mNumAsserts;size_t mNumParamAsserts;size_t mNumInitialAsserts;size_t mNumRanges
;size_t mNumEquationRanges;size_t mNumAssertRanges;size_t mNumParamAssertRanges
;size_t mNumInitialAssertRanges;size_t mNumSamples;size_t mNumSolverHits;
size_t mNumFreqs;size_t mNumDelays;size_t mNumDxfNzMax;size_t mNumDufNzMax;
size_t mNumDtfNzMax;size_t mNumDxyNzMax;size_t mNumDuyNzMax;size_t
mNumTDuyNzMax;size_t mNumDtyNzMax;size_t mNumDxmNzMax;size_t mNumDumNzMax;
size_t mNumObservables;size_t mNumObservableElements;boolean_T mIsMConstant;
boolean_T mIsMPwConstant;boolean_T mIsDxfConstant;boolean_T mIsDufConstant;
boolean_T mIsYLinear;boolean_T mHasConstraints;boolean_T
mHasConstraintEquations;boolean_T mSupportsLinearization;boolean_T mIsDae;
boolean_T mIsContinuousLti;boolean_T mIsSwitchedLinear;boolean_T mIsDfAnalytic
;boolean_T mIsDmAnalytic;boolean_T mIsDyAnalytic;real_T mFundamentalSampleTime
;size_t mRefCount;boolean_T mIsNormalizable;NeEquationData*mEquationData;
NeVariableData*mVariableData;NeVariableData*mDiscreteData;NeModeData*
mMajorModeData;NeZCData*mZCData;NeAssertData*mAssertData;NeAssertData*
mParamAssertData;NeAssertData*mInitialAssertData;NeRange*mRanges;NeRange*
mEquationRanges;NeRange*mAssertRanges;NeRange*mParamAssertRanges;NeRange*
mInitialAssertRanges;NeObservableData*mObservableData;NeParameterInfo
mParameterInfo;const PmBoolVector*mDirectFeedthrough;const PmIntVector*
mOutputFunctionMap;size_t mNumIo[NUM_IO_TYPES];const SscIoInfo*mIo[
NUM_IO_TYPES];NeChecksum*mChecksum;NeProfiler*mProfiler;NeDaePrivateData*
mPrivateData;void(*mDestroy)(NeDae*);NeDae*(*mCloneFcn)(const NeDae*);
NeDaeMethod mMethods[NE_NUM_DAE_METHODS];NeDaeSolve mSolves[NE_DAE_SOLVE_NUM];
NeDaeSolve mCheck;NeDaeSetParameters mSetParameters;NeDaeFunction
mPreTrDiagnoseFcn;NeDaeFunction mPreDcDiagnoseFcn;NeDaeReportFcn mPushInfo;
NeDaeSolve mAllInputsWorkFcn;NeDaeLinearizeFcn mResidualsFcn;NeDaeLinearizeFcn
mLinearizeFcn;NeDaeLinearizeFcn mLinearizeDssFcn;PmfMessageId(*mInitializeFcn)
(NeDae*dae,NeModelParameters mp,NeuDiagnosticManager*mgr);PmfMessageId(*
mStiffDiagnosticFcn)(const NeDae*,const NeSystemInput*,void*,
NeuDiagnosticManager*);void(*mLogIterationsFcn)(const NeDae*,IterationsLogCore
*);void(*mSetStateTracer)(NeDae*,StateTracer*);void(*mSolverFailureNotify)(
NeDae*dae,size_t num_consecutive_failures);void(*mSetupLoggerFcn)(const NeDae*
,NeLoggerBuilder*);PmfMessageId(*mLogFcn)(const NeDae*,const NeSystemInput*,
PmRealVector*,NeuDiagnosticManager*);void(*mGenerateFcn)(const NeDae*,const
NeCgParams*,NeCgResults*);const SLMatrices**(*mSwitchedLinearDataFcn)(const
NeDae*);NeDaeMakeOutputFcn mMakeOutput[NE_NUM_DAE_METHODS];NeDaeFreeOutputFcn
mFreeOutput[NE_NUM_DAE_METHODS];NeRealFunctorFcn mRealFunctorFcn;void(*
mSetOutputFunction)(const NeDae*,size_t outputFunctionIndex);void(*
mStartOutputLoop)(const NeDae*);NeCustomData*(*mGetCustomDataFcn)(const NeDae*
);PmfMessageId(*mSetCustomDataFcn)(const NeCustomData*,const NeDae*,
NeuDiagnosticManager*mgr);boolean_T mIsScalable;};PMF_DEPLOY_STATIC
PmfMessageId ne_dae_linearize(const NeDae*dae,const NeSystemInput*input,
PmRealVector*LinJacobian,NeuDiagnosticManager*mgr){return dae->mLinearizeFcn(
dae,input,LinJacobian,mgr);}PMF_DEPLOY_STATIC PmfMessageId ne_dae_linearize_dss
(const NeDae*dae,const NeSystemInput*input,PmRealVector*LinJacobian,
NeuDiagnosticManager*mgr){return dae->mLinearizeDssFcn(dae,input,LinJacobian,
mgr);}PMF_DEPLOY_STATIC PmfMessageId ne_dae_residuals(const NeDae*daePtr,const
NeSystemInput*inputPtr,PmRealVector*residualsRvPtr,NeuDiagnosticManager*mgr){
return daePtr->mResidualsFcn(daePtr,inputPtr,residualsRvPtr,mgr);}
PMF_DEPLOY_STATIC PmfMessageId ne_dae_call_method(const NeDae*dae,
NeDaeMethodId id,const NeSystemInput*input,NeDaeMethodOutput*output,
NeuDiagnosticManager*mgr){return dae->mMethods[id](dae,input,output,mgr);}
PMF_DEPLOY_STATIC NeDaeMethodOutput*ne_dae_default_output(const NeDae*dae,
NeDaeMethodId id,PmAllocator*allocator){return dae->mMakeOutput[id](dae,
allocator);}PMF_DEPLOY_STATIC NeDaeMethodOutput*ne_dae_method_output(const
NeDae*dae,NeDaeMethodId id,const NeSystemInput*input,NeuDiagnosticManager*mgr,
PmAllocator*allocator){NeDaeMethodOutput*daeOutput=ne_dae_default_output(dae,
id,allocator);ne_dae_call_method(dae,id,input,daeOutput,mgr);return daeOutput;
}PMF_DEPLOY_STATIC PmfMessageId ne_dae_solve(const NeDae*dae,NeDaeSolveId id,
const NeSystemInput*input,NeuDiagnosticManager*mgr){return dae->mSolves[id](
dae,input,mgr);}PMF_DEPLOY_STATIC NeSystemInputSizes
ne_dae_get_system_input_sizes(const NeDae*dae){NeSystemInputSizes sizes;sizes.
mSizes[NE_SYSTEM_INPUT_ID_Q]=dae->mNumMajorModes;sizes.mSizes[
NE_SYSTEM_INPUT_ID_M]=dae->mNumModes;sizes.mSizes[NE_SYSTEM_INPUT_ID_T]=1;
sizes.mSizes[NE_SYSTEM_INPUT_ID_U]=dae->mNumInputs;sizes.mSizes[
NE_SYSTEM_INPUT_ID_V]=dae->mNumInputs;sizes.mSizes[NE_SYSTEM_INPUT_ID_X]=dae->
mNumStates;sizes.mSizes[NE_SYSTEM_INPUT_ID_D]=dae->mNumDiscreteStates;sizes.
mSizes[NE_SYSTEM_INPUT_ID_E]=dae->mNumSamples;sizes.mSizes[
NE_SYSTEM_INPUT_ID_CR]=dae->mNumRealCache;sizes.mSizes[NE_SYSTEM_INPUT_ID_CI]=
dae->mNumIntCache;return sizes;}PMF_DEPLOY_STATIC void
ne_dae_default_output_free(const NeDae*dae,NeDaeMethodOutput*output,
NeDaeMethodId id,PmAllocator*allocator){dae->mFreeOutput[id](output,allocator)
;}PMF_DEPLOY_STATIC void ne_dae_get_reference(NeDae*dae){++(dae->mRefCount);}
PMF_DEPLOY_STATIC void ne_dae_release_reference(NeDae*dae){(void)0;;if(--(dae
->mRefCount)==0){dae->mDestroy(dae);}}PMF_DEPLOY_STATIC void ne_dae_push_info(
const NeDae*dae){if(dae->mPushInfo!=NULL){dae->mPushInfo(dae);}}
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ne_dae_h__ */
