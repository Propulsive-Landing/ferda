#ifndef __sm_ssci_NeDaePrivateData_h__
#define __sm_ssci_NeDaePrivateData_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef enum StateModelInputsModeTag{STATE_MODEL_INPUTS_UNSET= -1,
STATE_MODEL_INPUTS_NOINPUTS,STATE_MODEL_INPUTS_INPUTS,STATE_MODEL_INPUTS_NUM}
StateModelInputsMode;const char*state_model_inputs_mode_to_string(
StateModelInputsMode);StateModelInputsMode to_state_model_inputs_mode(const
char*);typedef enum StateModelModeTag{STATE_MODEL_MODE_UNSET= -1,
STATE_MODEL_MODE_NORMAL,STATE_MODEL_MODE_IMPORTED,STATE_MODEL_MODE_NUM}
StateModelMode;const char*state_model_mode_to_string(StateModelMode);
StateModelMode to_state_model_mode(const char*);typedef struct CStateManagerTag
CStateManager;typedef struct CStateManagerDataTag CStateManagerData;struct
CStateManagerTag{const char*(*mModel)(const CStateManager*);size_t(*mSize)(
const CStateManager*,int);void(*mAllocate)(const CStateManager*);int32_T*(*
mInteger)(const CStateManager*);size_t*(*mIndex)(const CStateManager*);double*
(*mReal)(const CStateManager*);double*(*mRealTarget)(const CStateManager*);
void(*mEliminated)(const CStateManager*,const char*);boolean_T(*mIsEliminated)
(const CStateManager*,const char*);void(*mSatisfied)(const CStateManager*,
const char*,boolean_T);boolean_T(*mIsSatisfied)(const CStateManager*,const char
*);void(*mInputDependent)(const CStateManager*,const char*);boolean_T(*
mIsInputDependent)(const CStateManager*,const char*);void(*mAlgebraic)(const
CStateManager*,const char*);boolean_T(*mIsAlgebraic)(const CStateManager*,
const char*);void(*mErrorAll)(const CStateManager*,boolean_T);void(*mError)(
const CStateManager*,const char*,boolean_T);boolean_T(*mHasError)(const
CStateManager*,const char*);boolean_T(*mIsHideValue)(const CStateManager*,
const char*);void(*mSteadyStateAll)(const CStateManager*);void(*mSteadyState)(
const CStateManager*,const char*);boolean_T(*mIsSteadyState)(const
CStateManager*,const char*);StateModelMode(*mGetModelMode)(const CStateManager
*);void(*mSetModelMode)(const CStateManager*,StateModelMode);
StateModelInputsMode(*mGetModelInputsMode)(const CStateManager*);void(*
mSetModelInputsMode)(const CStateManager*,StateModelInputsMode);int(*
mGetModelModifiedTimeStamp)(const CStateManager*);void(*
mSetModelModifiedTimeStamp)(const CStateManager*,int);const char*(*
mGetModelLastUpdatedDateTime)(const CStateManager*);void(*
mSetModelLastUpdatedDateTime)(const CStateManager*,const char*);void(*mUpdate)
(const CStateManager*);void(*mUpdateMode)(const CStateManager*);void(*
mUpdateViewer)(const CStateManager*);void(*mUpdateTarget)(const CStateManager*
);void(*mUpdateISI)(const CStateManager*,void*);void(*mUpdateModeISI)(const
CStateManager*,void*);void(*mUpdateViewerISI)(const CStateManager*,void*);void
(*mUpdateTargetISI)(const CStateManager*,void*);void(*mApplyISI)(const
CStateManager*,const void*);CStateManager*(*mClone)(const CStateManager*);void
(*mDestroy)(CStateManager*);CStateManagerData*mData;};struct SmBoundedSetTag{
size_t mBound;unsigned char*mFlags;};typedef struct SmBoundedSetTag
SmBoundedSet;void sm_core_SmBoundedSet_create(SmBoundedSet*set,size_t bound);
void sm_core_SmBoundedSet_copy(SmBoundedSet*copy,const SmBoundedSet*orig);void
sm_core_SmBoundedSet_assign(SmBoundedSet*dst,const SmBoundedSet*src);void
sm_core_SmBoundedSet_destroy(SmBoundedSet*set);size_t
sm_core_SmBoundedSet_bound(const SmBoundedSet*set);size_t
sm_core_SmBoundedSet_size(const SmBoundedSet*set);boolean_T
sm_core_SmBoundedSet_isMember(const SmBoundedSet*set,size_t elt);void
sm_core_SmBoundedSet_clear(SmBoundedSet*set);void sm_core_SmBoundedSet_insert(
SmBoundedSet*set,size_t elt);void sm_core_SmBoundedSet_remove(SmBoundedSet*set
,size_t elt);void sm_core_SmBoundedSet_complement(const SmBoundedSet*set,
SmBoundedSet*comp);void sm_core_SmBoundedSet_intersection(const SmBoundedSet*a
,const SmBoundedSet*b,SmBoundedSet*c);void sm_core_SmBoundedSet_union(const
SmBoundedSet*a,const SmBoundedSet*b,SmBoundedSet*c);void
sm_core_SmBoundedSet_difference(const SmBoundedSet*a,const SmBoundedSet*b,
SmBoundedSet*c);struct SmIntVectorTag{size_t mSize;size_t mCap;int*mValues;};
typedef struct SmIntVectorTag SmIntVector;void sm_core_SmIntVector_create(
SmIntVector*vector,size_t n,int val);void sm_core_SmIntVector_copy(SmIntVector
*copy,const SmIntVector*orig);void sm_core_SmIntVector_destroy(SmIntVector*
vector);int sm_core_SmIntVector_isEmpty(const SmIntVector*vector);size_t
sm_core_SmIntVector_size(const SmIntVector*vector);size_t
sm_core_SmIntVector_capacity(const SmIntVector*vector);int
sm_core_SmIntVector_value(const SmIntVector*vector,size_t index);void
sm_core_SmIntVector_setValue(SmIntVector*vector,size_t index,int value);const
int*sm_core_SmIntVector_values(const SmIntVector*vector);int*
sm_core_SmIntVector_nonConstValues(SmIntVector*vector);void
sm_core_SmIntVector_reserve(SmIntVector*vector,size_t n);void
sm_core_SmIntVector_clear(SmIntVector*vector);void sm_core_SmIntVector_pushBack
(SmIntVector*vector,int val);void sm_core_SmIntVector_popBack(SmIntVector*
vector);struct SmBoundedSetTag;struct SmSizeTVectorTag{size_t mSize;size_t mCap
;size_t*mValues;};typedef struct SmSizeTVectorTag SmSizeTVector;void
sm_core_SmSizeTVector_create(SmSizeTVector*vector,size_t n,size_t val);void
sm_core_SmSizeTVector_copy(SmSizeTVector*copy,const SmSizeTVector*orig);void
sm_core_SmSizeTVector_destroy(SmSizeTVector*vector);int
sm_core_SmSizeTVector_isEmpty(const SmSizeTVector*vector);size_t
sm_core_SmSizeTVector_size(const SmSizeTVector*vector);size_t
sm_core_SmSizeTVector_capacity(const SmSizeTVector*vector);size_t
sm_core_SmSizeTVector_value(const SmSizeTVector*vector,size_t index);void
sm_core_SmSizeTVector_setValue(SmSizeTVector*vector,size_t index,size_t value)
;const size_t*sm_core_SmSizeTVector_values(const SmSizeTVector*vector);size_t*
sm_core_SmSizeTVector_nonConstValues(SmSizeTVector*vector);void
sm_core_SmSizeTVector_reserve(SmSizeTVector*vector,size_t n);void
sm_core_SmSizeTVector_clear(SmSizeTVector*vector);void
sm_core_SmSizeTVector_pushBack(SmSizeTVector*vector,size_t val);void
sm_core_SmSizeTVector_popBack(SmSizeTVector*vector);void
sm_core_SmSizeTVector_assignFromBoundedSet(SmSizeTVector*vector,const struct
SmBoundedSetTag*set);struct SmSizePairTag{size_t mFirst;size_t mSecond;};
typedef struct SmSizePairTag SmSizePair;struct SmSizePairVectorTag{size_t mSize
;size_t mCap;SmSizePair*mValues;};typedef struct SmSizePairVectorTag
SmSizePairVector;void sm_core_SmSizePairVector_create(SmSizePairVector*vector,
size_t n,const SmSizePair*val);void sm_core_SmSizePairVector_copy(
SmSizePairVector*copy,const SmSizePairVector*orig);void
sm_core_SmSizePairVector_destroy(SmSizePairVector*vector);int
sm_core_SmSizePairVector_isEmpty(const SmSizePairVector*vector);size_t
sm_core_SmSizePairVector_size(const SmSizePairVector*vector);size_t
sm_core_SmSizePairVector_capacity(const SmSizePairVector*vector);const
SmSizePair*sm_core_SmSizePairVector_value(const SmSizePairVector*vector,size_t
index);void sm_core_SmSizePairVector_setValue(SmSizePairVector*vector,size_t
index,const SmSizePair*value);const SmSizePair*sm_core_SmSizePairVector_values
(const SmSizePairVector*vector);SmSizePair*
sm_core_SmSizePairVector_nonConstValues(SmSizePairVector*vector);void
sm_core_SmSizePairVector_reserve(SmSizePairVector*vector,size_t n);void
sm_core_SmSizePairVector_clear(SmSizePairVector*vector);void
sm_core_SmSizePairVector_pushBack(SmSizePairVector*vector,const SmSizePair*val
);void sm_core_SmSizePairVector_popBack(SmSizePairVector*vector);size_t
sm_core_SmSizePairVector_find(const SmSizePairVector*map,size_t key);struct
SmRealVectorTag{size_t mSize;size_t mCap;real_T*mValues;};typedef struct
SmRealVectorTag SmRealVector;void sm_core_SmRealVector_create(SmRealVector*
vector,size_t n,real_T val);void sm_core_SmRealVector_copy(SmRealVector*copy,
const SmRealVector*orig);void sm_core_SmRealVector_destroy(SmRealVector*vector
);int sm_core_SmRealVector_isEmpty(const SmRealVector*vector);size_t
sm_core_SmRealVector_size(const SmRealVector*vector);size_t
sm_core_SmRealVector_capacity(const SmRealVector*vector);real_T
sm_core_SmRealVector_value(const SmRealVector*vector,size_t index);void
sm_core_SmRealVector_setValue(SmRealVector*vector,size_t index,real_T value);
const real_T*sm_core_SmRealVector_values(const SmRealVector*vector);real_T*
sm_core_SmRealVector_nonConstValues(SmRealVector*vector);void
sm_core_SmRealVector_reserve(SmRealVector*vector,size_t n);void
sm_core_SmRealVector_clear(SmRealVector*vector);void
sm_core_SmRealVector_pushBack(SmRealVector*vector,real_T val);void
sm_core_SmRealVector_popBack(SmRealVector*vector);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct CTargetTag;struct RuntimeDerivedValuesBundleTag;struct
SmMechanismDelegateScratchpadTag;typedef struct
SmMechanismDelegateScratchpadTag SmMechanismDelegateScratchpad;struct
SmMechanismDelegateTag{unsigned int mTargetStrengthFree;unsigned int
mTargetStrengthSuggested;unsigned int mTargetStrengthDesired;unsigned int
mTargetStrengthRequired;double mConsistencyTol;size_t mTreeJointDof;size_t mDof
;size_t mStateSize;size_t mContinuousStateSize;size_t mModeVectorSize;size_t
mNumStages;size_t mNumConstraints;size_t mNumAllConstraintEquations;
SmSizePairVector mJointToStageIdx;SmSizeTVector mPrimitiveIndices;
SmSizePairVector mStateOffsets;SmSizePairVector mDofOffsets;SmSizeTVector
mFlexibleStages;SmSizeTVector mRemodIndices;SmSizeTVector
mEquationsPerConstraint;SmIntVector mHasAllVelocityDisabledEquations;
SmIntVector mRunTimeEnabledEquations;SmSizeTVector mDofToVelSlot;SmSizeTVector
mConstraintDofs;SmSizeTVector mConstraintDofOffsets;
SmMechanismDelegateScratchpad*mScratchpad;void(*mSetTargets)(const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,struct CTargetTag*
targets);void(*mResetStateVector)(const void*mech,double*stateVector);void(*
mInitializeTrackedAngleState)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const int*modeVector,
const double*motionData,double*stateVector);void(*mComputeDiscreteState)(const
void*mech,const struct RuntimeDerivedValuesBundleTag*
runtimeDerivedValuesBundle,double*stateVector);void(*mAdjustPosition)(const
void*mech,const double*dofDeltas,double*stateVector);void(*
mPerturbJointPrimitiveState)(const void*mech,size_t stageIdx,size_t
primitiveIdx,double magnitude,boolean_T doPerturbVelocity,double*stateVector);
void(*mPerturbFlexibleBodyState)(const void*mech,size_t stageIdx,double
magnitude,boolean_T doPerturbVelocity,double*stateVector);void(*
mComputePosDofBlendMatrix)(const void*mech,size_t stageIdx,size_t primitiveIdx
,const double*stateVector,int partialType,double*matrix);void(*
mComputeVelDofBlendMatrix)(const void*mech,size_t stageIdx,size_t primitiveIdx
,const double*stateVector,int partialType,double*matrix);void(*
mProjectPartiallyTargetedPos)(const void*mech,size_t stageIdx,size_t
primitiveIdx,const double*origStateVector,int partialType,double*stateVector);
void(*mPropagateMotion)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const double*
stateVector,double*motionData);size_t(*mComputeAssemblyError)(const void*mech,
const struct RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,size_t
constraintIdx,const int*modeVector,const double*motionData,double*error);
size_t(*mComputeAssemblyJacobian)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,size_t constraintIdx,
boolean_T forVelocitySatisfaction,const double*stateVector,const int*
modeVector,const double*motionData,double*J);size_t(*
mComputeFullAssemblyJacobian)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const double*
stateVector,const int*modeVector,const double*motionData,double*J);boolean_T(*
mIsInKinematicSingularity)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,size_t constraintIdx,
const int*modeVector,const double*motionData);void(*mConvertStateVector)(const
void*asmMech,const struct RuntimeDerivedValuesBundleTag*
asmRuntimeDerivedValuesBundle,const void*simMech,const double*asmStateVector,
const int*asmModeVector,const int*simModeVector,double*simStateVector);void(*
mConstructStateVector)(const void*mech,const double*solverStateVector,const
double*u,const double*uDot,const double*discreteStateVector,double*
fullStateVector);void(*mExtractSolverStateVector)(const void*mech,const double
*fullStateVector,double*solverStateVector);boolean_T(*mIsPositionViolation)(
const void*mech,const struct RuntimeDerivedValuesBundleTag*
runtimeDerivedValuesBundle,const int*constraintEqnEnableFlags,const double*
stateVector,const int*modeVector);boolean_T(*mIsVelocityViolation)(const void*
mech,const struct RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,
const int*constraintEqnEnableFlags,const double*stateVector,const int*
modeVector);PmfMessageId(*mProjectStateSim)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const int*
constraintEqnEnableFlags,const int*modeVector,double*stateVector,void*
neDiagMgr);void(*mComputeConstraintError)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const double*
stateVector,const int*modeVector,double*error);void(*mResetModeVector)(const
void*mech,int*modeVector);boolean_T(*mHasJointDisToNormModeChange)(const void*
mech,const int*prevModeVector,const int*modeVector);PmfMessageId(*
mPerformJointDisToNormModeChange)(const void*mech,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const int*
constraintEqnEnableFlags,const int*prevModeVector,const int*modeVector,const
double*inputVector,double*stateVector,void*neDiagMgr);void(*
mOnModeChangedCutJoints)(const void*mech,const int*prevModeVector,const int*
modeVector,double*stateVector);const void*mMech;};typedef struct
SmMechanismDelegateTag SmMechanismDelegate;struct
SmMechanismDelegateScratchpadTag{SmBoundedSet mPosRequired,mPosDesired,
mPosSuggested,mPosFree,mPosNonRequired,mPosSuggAndFree;SmBoundedSet
mVelRequired,mVelDesired,mVelSuggested,mVelFree,mVelNonRequired,
mVelSuggAndFree;SmBoundedSet mConstraintFilter;SmBoundedSet mActiveConstraints
,mActiveDofs0,mActiveDofs;SmBoundedSet mNewConstraints,mNewDofs;SmBoundedSet
mUnsatisfiedConstraints;SmSizeTVector mActiveConstraintsVect,mActiveDofsVect;
SmSizeTVector mFullDofToActiveDof;SmSizePairVector mPartiallyPosTargetedPrims;
SmSizePairVector mPartiallyVelTargetedPrims;SmSizeTVector mPosPartialTypes;
SmSizeTVector mVelPartialTypes;SmSizeTVector mPartiallyActivePrims;
SmSizePairVector mBaseFrameVelOffsets;SmSizePairVector mCvQuaternionVelOffsets
;SmRealVector mCvQuaternionAzimuthValues;SmRealVector mInitialState;
SmRealVector mStartState;SmRealVector mTestState;SmRealVector mFullStateVector
;SmIntVector mModeVector;SmRealVector mJacobianRowMaj;SmRealVector mJacobian;
SmRealVector mJacobianPrimSubmatrix;SmRealVector mConstraintNonhomoTerms;
SmRealVector mConstraintError;SmRealVector mBestConstraintError;SmRealVector
mDeltas;SmRealVector mSvdWork;SmRealVector mLineSearchScaledDeltaVect;
SmRealVector mLineSearchTestStateVect;SmRealVector mLineSearchErrorVect;
SmRealVector mActiveDofVelsVect;SmRealVector mVelSystemRhs;SmRealVector
mMotionData;};void sm_core_MechanismDelegate_allocScratchpad(
SmMechanismDelegate*delegate);void sm_core_MechanismDelegate_create(
SmMechanismDelegate*delegate);void sm_core_MechanismDelegate_copy(
SmMechanismDelegate*copy,const SmMechanismDelegate*orig);void
sm_core_MechanismDelegate_destroy(SmMechanismDelegate*delegate);void
sm_core_computeStateVector(const SmMechanismDelegate*mechDelegate,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const int32_T*
modeVector,size_t numTargets,const struct CTargetTag*cTargets,double*
stateVector);void sm_core_computeStateVectorCustom(const SmMechanismDelegate*
mechDelegate,const struct RuntimeDerivedValuesBundleTag*
runtimeDerivedValuesBundle,const int32_T*modeVector,size_t numTargets,const
struct CTargetTag*cTargets,boolean_T doPosSatPhase1,boolean_T doPosSatPhase3,
boolean_T doVelSat,int32_T maxIters,double*stateVector);unsigned int
sm_core_checkAssembly(const SmMechanismDelegate*mechDelegate,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const double*
stateVector,const int32_T*modeVector,SmSizeTVector*
posViolationConstraintIndices,SmSizeTVector*velViolationConstraintIndices,
SmSizeTVector*singViolationConstraintIndices);PmfMessageId sm_core_projectState
(boolean_T doCheckFirst,const SmMechanismDelegate*mechDelegate,const struct
RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const int*modeVector,
const double*inputVector,const double*inputDotVector,const double*
discreteStateVector,double*solverStateVector,void*neDiagMgr);void
sm_core_computeRedundantConstraintEquations(SmMechanismDelegate*mechDelegate,
const struct RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle);
PmfMessageId sm_core_onModeChanged(const SmMechanismDelegate*mechDelegate,
const struct RuntimeDerivedValuesBundleTag*runtimeDerivedValuesBundle,const
double*inputVector,const double*inputDotVector,const double*
discreteStateVector,const int*prevModeVector,const int*modeVector,double*
solverStateVector,void*neDiagMgr);struct RuntimeDerivedValuesBundleTag{
SmRealVector mDoubles;SmIntVector mInts;};typedef struct
RuntimeDerivedValuesBundleTag RuntimeDerivedValuesBundle;void
sm_core_RuntimeDerivedValuesBundle_create(RuntimeDerivedValuesBundle*bundle,
size_t nDoubles,size_t nInts);void sm_core_RuntimeDerivedValuesBundle_copy(
const RuntimeDerivedValuesBundle*orig,RuntimeDerivedValuesBundle*copy);void
sm_core_RuntimeDerivedValuesBundle_destroy(RuntimeDerivedValuesBundle*bundle);
struct LiveSmLink;struct CTargetTag;struct NeDaePrivateDataTag{size_t
mStateVectorSize;size_t mFullStateVectorSize;size_t mDiscreteStateSize;size_t
mModeVectorSize;size_t mNumZeroCrossings;size_t mInputVectorSize;size_t
mOutputVectorSize;size_t mNumConstraintEqns;real_T mFundamentalSampleTime;
NeChecksum mChecksum;PmRealVector mAssemblyFullStateVector;PmRealVector
mSimulationFullStateVector;size_t mNumInternalTargets;size_t
mNumInputMotionPrimitives;PmIntVector mStateVectorMap;struct CTargetTag*
mTargets;CStateManager*mStateManager;size_t mNumParamAsserts;PmCharVector*
mParamAssertObjects;PmCharVector*mParamAssertPaths;PmCharVector*
mParamAssertDescriptors;PmCharVector*mParamAssertMessages;PmCharVector*
mParamAssertMessageIds;PmBoolVector mParamAssertIsWarnings;PmIntVector
mAssemblyModeVector;PmIntVector mCachedModeVector;PmCharVector*
mZeroCrossingObjects;PmCharVector*mZeroCrossingPaths;PmCharVector*
mZeroCrossingDescriptors;PmIntVector mZeroCrossingTypes;size_t mNumVarScalars;
PmCharVector*mVarFullPaths;PmCharVector*mVarObjects;size_t mNumRtpRootVars;
PmIntVector mRtpRootVarRows;PmIntVector mRtpRootVarCols;PmCharVector*
mRtpFullPaths;size_t mNumRuntimeRootVarScalars;PmRealVector
mRuntimeParameterScalars;struct RuntimeDerivedValuesBundleTag
mAsmRuntimeDerivedValuesBundle,mSimRuntimeDerivedValuesBundle;size_t mNumInputs
;PmCharVector*mInputPortPaths;PmCharVector*mInputUnits;SscIoInfo*mInputInfos;
size_t mNumOutputs;PmCharVector*mOutputPortPaths;PmCharVector*mOutputUnits;
SscIoInfo*mOutputInfos;PmIntVector mNumInputDerivs;size_t mInputOrder;
PmBoolVector mDirectFeedthroughVector;PmBoolVector mDirectFeedthroughMatrix;
PmIntVector*mOutputFunctionMap;unsigned int mNumOutputClasses;boolean_T
mHasKinematicOutputs;boolean_T mHasDynamicOutputs;boolean_T
mIsOutputClass0Dynamic;boolean_T mDoComputeDynamicOutputs;boolean_T
mCachedDerivativesAvailable;PmRealVector mCachedDerivatives;struct
SmMechanismDelegateTag mAssemblyDelegate,mSimulationDelegate;PmIntVector
mMotionInputOffsets;NeDaeSetParameters mSetParametersFcn;NeDaeMethod
mPAssertFcn;NeDaeMethod mDerivativeFcn;NeDaeMethod mNumJacPerturbLoBoundsFcn;
NeDaeMethod mNumJacPerturbHiBoundsFcn;NeDaeMethod mOutputFcn;NeDaeMethod
mModeFcn;NeDaeMethod mZeroCrossingFcn;NeDaeSolve mProjectionFcn;NeDaeSolve
mCIC_MODE_Fcn;NeDaeSolve mCheckFcn;NeDaeSolve mAssemblyFcn;void(*
mSetupLoggerFcn)(const NeDae*,NeLoggerBuilder*);PmfMessageId(*mLogFcn)(const
NeDae*,const NeSystemInput*,PmRealVector*,NeuDiagnosticManager*);
NeDaeLinearizeFcn mResidualsFcn;NeDaeLinearizeFcn mLinearizeFcn;void(*
mGenerateFcn)(const NeDae*,const NeCgParams*,NeCgResults*);struct LiveSmLink*
mLiveSmLink;void(*mLiveSmLink_destroy)(struct LiveSmLink*link);struct
LiveSmLink*(*mLiveSmLink_copy)(const struct LiveSmLink*orig);};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_ssci_NeDaePrivateData_h__ */
