#ifndef ___nesl_rtw_h__
#define ___nesl_rtw_h__
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
int_T pm_create_sparsity_pattern_fields(PmSparsityPattern*patternPtr,size_t
nNzMax,size_t nRow,size_t nCol,PmAllocator*allocator);PmSparsityPattern*
pm_create_sparsity_pattern(size_t nNzMax,size_t nRow,size_t nCol,PmAllocator*
allocatorPtr);void pm_sp_equals_sp(PmSparsityPattern*Ap,const PmSparsityPattern
*Bp);boolean_T pm_sp_equalequals_sp(const PmSparsityPattern*Ap,const
PmSparsityPattern*Bp);PmSparsityPattern*pm_copy_sparsity_pattern(const
PmSparsityPattern*input_pattern,PmAllocator*allocatorPtr);void
pm_destroy_sparsity_pattern_fields(PmSparsityPattern*patternPtr,PmAllocator*
allocator);void pm_destroy_sparsity_pattern(PmSparsityPattern*patternPtr,
PmAllocator*allocatorPtr);PmSparsityPattern*pm_create_full_sparsity_pattern(
size_t nrows,size_t ncols,PmAllocator*allocatorPtr);PmSparsityPattern*
pm_create_empty_sparsity_pattern(size_t nrows,size_t ncols,PmAllocator*
allocatorPtr);PmSparsityPattern*pm_create_partial_identity_sparsity_pattern(
size_t p,size_t n,PmAllocator*allocatorPtr);PmSparsityPattern*
pm_create_identity_sparsity_pattern(size_t n,PmAllocator*allocatorPtr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void pm_rv_equals_rv(const PmRealVector*xPtr,const PmRealVector*yPtr);void
pm_iv_equals_iv(const PmIntVector*xPtr,const PmIntVector*yPtr);void
pm_bv_equals_bv(const PmBoolVector*xPtr,const PmBoolVector*yPtr);void
pm_cv_equals_cv(const PmCharVector*xPtr,const PmCharVector*yPtr);boolean_T
pm_rv_equalequals_rv(const PmRealVector*xPtr,const PmRealVector*yPtr);
boolean_T pm_iv_equalequals_iv(const PmIntVector*xPtr,const PmIntVector*yPtr);
boolean_T pm_bv_equalequals_bv(const PmBoolVector*xPtr,const PmBoolVector*yPtr
);int_T pm_create_real_vector_fields(PmRealVector*vecPtr,size_t size,
PmAllocator*allocatorPtr);PmRealVector*pm_create_real_vector(size_t numElements
,PmAllocator*allocatorPtr);PmRealVector*pm_copy_real_vector(const PmRealVector
*vecPtr,PmAllocator*allocatorPtr);void pm_destroy_real_vector_fields(
PmRealVector*vecPtr,PmAllocator*allocatorPtr);void pm_destroy_real_vector(
PmRealVector*vecPtr,PmAllocator*allocatorPtr);int_T pm_create_int_vector_fields
(PmIntVector*vecPtr,size_t size,PmAllocator*allocatorPtr);PmIntVector*
pm_create_int_vector(size_t numElements,PmAllocator*allocatorPtr);PmIntVector*
pm_copy_int_vector(const PmIntVector*vecPtr,PmAllocator*allocatorPtr);void
pm_destroy_int_vector_fields(PmIntVector*vecPtr,PmAllocator*allocatorPtr);void
pm_destroy_int_vector(PmIntVector*vecPtr,PmAllocator*allocatorPtr);int_T
pm_create_bool_vector_fields(PmBoolVector*vecPtr,size_t size,PmAllocator*
allocatorPtr);PmBoolVector*pm_create_bool_vector(size_t numElements,
PmAllocator*allocatorPtr);void pm_destroy_bool_vector_fields(PmBoolVector*
vecPtr,PmAllocator*allocatorPtr);void pm_destroy_bool_vector(PmBoolVector*
vecPtr,PmAllocator*allocatorPtr);PmBoolVector*pm_copy_bool_vector(const
PmBoolVector*vecPtr,PmAllocator*allocatorPtr);int_T
pm_create_char_vector_fields(PmCharVector*vecPtr,size_t size,PmAllocator*
allocatorPtr);PmCharVector*pm_create_char_vector(size_t numElements,
PmAllocator*allocatorPtr);void pm_destroy_char_vector_fields(PmCharVector*
vecPtr,PmAllocator*allocatorPtr);void pm_destroy_char_vector(PmCharVector*
vecPtr,PmAllocator*allocatorPtr);int_T pm_create_size_vector_fields(
PmSizeVector*vecPtr,size_t size,PmAllocator*allocatorPtr);PmSizeVector*
pm_create_size_vector(size_t numElements,PmAllocator*allocatorPtr);void
pm_destroy_size_vector_fields(PmSizeVector*vecPtr,PmAllocator*allocatorPtr);
void pm_destroy_size_vector(PmSizeVector*vecPtr,PmAllocator*allocatorPtr);void
pm_sv_equals_sv(const PmSizeVector*xPtr,const PmSizeVector*yPtr);boolean_T
pm_sv_equalequals_sv(const PmSizeVector*xPtr,const PmSizeVector*yPtr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "mc_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
const McLinearAlgebraFactory*mc_get_csparse_linear_algebra(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "mc_std.h"
#include "ne_dae_fwd.h"
#include "ne_profiler_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeSolverSystemTag NeSolverSystem;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "ne_dae_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void nes_dae_get_base_methods(NeDae*dae_ptr);PmIntVector*
nes_compute_output_function_map(const PmSparsityPattern*duy,uint32_T inputOrder
);
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
typedef struct NeSolverParametersTag NeSolverParameters;typedef enum
NeLocalSolverChoiceTag{NE_INVALID_ADVANCER_CHOICE= -1,
NE_BACKWARD_EULER_ADVANCER,NE_TRAPEZOIDAL_ADVANCER,NE_NDF2_ADVANCER,
NE_VARIABLE_STEP_ADVANCER,NE_PARTITIONING_ADVANCER,NE_NUM_ADVANCER_CHOICES}
NeLocalSolverChoice;typedef enum NeIndexReductionMethodTag{
NE_INVALID_IR_METHOD= -1,NE_NONE_IR,NE_DERIVATIVE_REPLACEMENT_IR,
NE_PROJECTION_IR,NE_NUM_IR_METHOD}NeIndexReductionMethod;typedef enum
NeLinearAlgebraChoiceTag{NE_INVALID_LA_CHOICE= -1,NE_AUTO_LA,NE_SPARSE_LA,
NE_FULL_LA,NE_NUM_LA_CHOICES}NeLinearAlgebraChoice;typedef enum
NeEquationFormulationChoiceTag{NE_INVALID_EF_CHOICE= -1,NE_TIME_EF,
NE_FREQUENCY_TIME_EF,NE_NUM_EF_CHOICES}NeEquationFormulationChoice;typedef enum
NePartitionStorageMethodTag{NE_PARTITIONING_INVALID= -1,
NE_PARTITIONING_AS_NEEDED,NE_PARTITIONING_EXHAUSTIVE,NE_PARTITIONING_NUM}
NePartitionStorageMethod;typedef enum NePartitionMethodTag{
NE_PARTITIONING_METHOD_INVALID= -1,NE_PARTITIONING_METHOD_ROBUST,
NE_PARTITIONING_METHOD_FAST,NE_PARTITIONING_METHOD_ALG_EQS,
NE_PARTITIONING_METHOD_ALL_EQS,NE_PARTITIONING_METHOD_MERGE,
NE_PARTITIONING_METHOD_MULTICORE,NE_PARTITIONING_METHOD_NUM}NePartitionMethod;
typedef enum NeConsistencySolverTag{NE_CONSISTENCY_SOLVER_INVALID= -1,
NE_CS_NEWTON_FTOL,NE_CS_NEWTON_XTOL,NE_CS_NEWTON_XTOL_AFTER_FTOL,
NE_CONSISTENCY_SOLVER_NUM}NeConsistencySolver;typedef enum NeToleranceSourceTag
{NE_TOLERANCE_SOURCE_INVALID= -1,NE_TOLERANCE_SOURCE_LOCAL,
NE_TOLERANCE_SOURCE_GLOBAL,NE_TOLERANCE_SOURCE_NUM}NeToleranceSource;typedef
enum NeMultibodyLocalSolverChoiceTag{NE_MULTIBODY_LOCAL_SOLVER_INVALID= -1,
NE_MULTIBODY_LOCAL_SOLVER_ODE1,NE_MULTIBODY_LOCAL_SOLVER_ODE2,
NE_MULTIBODY_LOCAL_SOLVER_ODE3,NE_MULTIBODY_LOCAL_SOLVER_ODE4,
NE_MULTIBODY_LOCAL_SOLVER_ODE5,NE_MULTIBODY_LOCAL_SOLVER_ODE8,
NE_MULTIBODY_LOCAL_SOLVER_NUM}NeMultibodyLocalSolverChoice;struct
NeSolverParametersTag{boolean_T mProfile;boolean_T mUseLocalSampling;boolean_T
mEnableSwitchedLinearOptims;boolean_T mFrequencyDomain;boolean_T mUseCCode;
real_T mRelTol;real_T mAbsTol;real_T mMinStep;boolean_T mToWorkspace;boolean_T
mRevertToSquareIcSolve;size_t mNumHomotopyIterations;boolean_T mPhasorMode;
size_t mPhasorModeNumHarmonics;NeConsistencySolver mConsistencySolver;
NeIndexReductionMethod mIndexReductionMethod;boolean_T mDoDC;real_T
mResidualTolerance;NeToleranceSource mConsistencyTolSource;real_T
mConsistencyAbsTol;real_T mConsistencyRelTol;real_T mConsistencyTolFactor;
boolean_T mUseLocalSolver;NeLocalSolverChoice mLocalSolverChoice;boolean_T
mResolveIndetEquations;size_t mFunctionEvalNumThread;real_T
mLocalSolverSampleTime;boolean_T mDoFixedCost;size_t mMaxNonlinIter;size_t
mMaxModeIter;boolean_T mComputeImpulses;size_t mImpulseIterations;
NeLinearAlgebraChoice mLinearAlgebra;size_t mLinearAlgebraNumThread;
NeEquationFormulationChoice mEquationFormulation;size_t mDelaysMemoryBudget;
boolean_T mAutomaticFiltering;real_T mFilteringTimeConstant;
NePartitionStorageMethod mPartitionStorageMethod;size_t mPartitionMemoryBudget
;NePartitionMethod mPartitionMethod;boolean_T mMultibodyUseLocalSolver;
NeMultibodyLocalSolverChoice mMultibodyLocalSolverChoice;real_T
mMultibodyLocalSolverSampleTime;};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_ds.h"
#include "pm_std.h"
#include "pm_std.h"
#include "mc_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef enum{MC_SOLVER_STATUS_INVALID= -1,MC_SOLVER_SUCCESS,MC_SOLVER_FAILURE,
MC_SOLVER_SINGULAR_MATRIX,MC_SOLVER_NO_CONVERGENCE,MC_SOLVER_TOLERANCE_UNMET,
MC_SOLVER_LINEAR_INCONSISTENT,MC_SOLVER_STATUS_COUNT}McSolverStatus;typedef
struct McRealFunctionPrivateDataTag McRealFunctionPrivateData;struct
McRealFunctionTag{McRealFunctionPrivateData*mPrivateDataPtr;McSolverStatus(*
mFunctionPtr)(const void*input,const PmRealVector*output,
McRealFunctionPrivateData*data);const void*(*mGetDefaultInput)(const
McRealFunction*function);void(*mDestroy)(McRealFunction*function);};typedef
struct McIntFunctionPrivateDataTag McIntFunctionPrivateData;struct
McIntFunctionTag{McIntFunctionPrivateData*mPrivateDataPtr;void(*mFunctionPtr)(
const void*input,const PmIntVector*output,McIntFunctionPrivateData*data);const
void*(*mGetDefaultInput)(const McIntFunction*function);void(*mDestroy)(
McIntFunction*function);};typedef struct McMatrixFunctionPrivateDataTag
McMatrixFunctionPrivateData;struct McMatrixFunctionTag{
McMatrixFunctionPrivateData*mPrivateDataPtr;const PmSparsityPattern*
mPatternPtr;void(*mFunctionPtr)(const void*input,const PmRealVector*output,
McMatrixFunctionPrivateData*data);const void*(*mGetDefaultInput)(const
McMatrixFunction*function);void(*mDestroy)(McMatrixFunction*function);};
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
typedef struct DDIWrapperPrivateDataTag DDIWrapperPrivateData;typedef struct
DaeDynamicInfoWrapperTag DaeDynamicInfoWrapper;struct DaeDynamicInfoWrapperTag
{DDIWrapperPrivateData*mPrivateData;void(*mReportIC)(const
DaeDynamicInfoWrapper*,const NeSystemInput*,const NeParameterBundle*,const
NeDerivedParameterBundle*,size_t,boolean_T,boolean_T);void(*mReportMode)(const
DaeDynamicInfoWrapper*,const NeSystemInput*,boolean_T,boolean_T,boolean_T,
int16_T);void(*mPushInfo)(const DaeDynamicInfoWrapper*,size_t);void(*
mInitializeState)(DaeDynamicInfoWrapper*,McMatrixFunction*,McMatrixFunction*,
McMatrixFunction*,McMatrixFunction*,McMatrixFunction*,McRealFunction*,
McRealFunction*,size_t,size_t,size_t,size_t);void(*mDestroy)(
DaeDynamicInfoWrapper*);};DaeDynamicInfoWrapper*ddi_wrapper_create(void);
typedef enum{DAEMON_CHOICE_INVALID= -1,DAEMON_CHOICE_NONE,
DAEMON_CHOICE_LOG_NUM_ITERATIONS,DAEMON_CHOICE_SSC2HDL,NUM_DAEMON_CHOICES}
DaemonChoice;NeSolverSystem*nes_create_crude_solver_system(NeDynamicSystem*
dynamicsystemPtr,NeModelParameters modelParameters,NeSolverParameters
solverParameters,PmAllocator*allocatorPtr);NeSolverSystem*
nes_create_crude_solver_system_daemon(NeDynamicSystem*dynamicsystemPtr,
NeModelParameters modelParameters,NeSolverParameters solverParameters,
PmAllocator*allocatorPtr,DaeDynamicInfoWrapper*ddiWrapper);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeConstraintStatTag NeConstraintStat;struct NeConstraintStatTag
{void(*mVariable)(const NeConstraintStat*,const NeVariableData*);struct
NeConstraintStatData*mData;};typedef struct NeCompStatsTag NeCompStats;struct
NeCompStatsTag{const NeConstraintStat*(*mConstraint)(const NeCompStats*);
struct NeCompStatsData*mData;};NeSolverSystem*
nes_create_index_reduced_solver_system(NeSolverSystem*basePtr,
NeSolverParameters sp,const NeCompStats*compStats,PmAllocator*allocatorPtr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
NeSolverSystem*nes_create_optimized_solver_system(NeSolverSystem*
solverSystemPtr,PmAllocator*allocatorPtr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
NeSolverSystem*nes_create_regularized_solver_system(NeSolverSystem*ss,
NeSolverParameters sp,PmAllocator*allocator);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_profiler_fwd.h"
#include "ne_ds.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeProfilingServiceTag{NeDynamicSystem*(*mProfileDs)(const
NeProfiler*,NeDynamicSystem*);NeSolverSystem*(*mProfileSs)(const NeProfiler*,
NeSolverSystem*,const char*);NeDae*(*mProfileDae)(const NeProfiler*,NeDae*);}
NeProfilingService;boolean_T nes_set_profiling_service(NeProfilingService*);
NeDynamicSystem*nes_profile_dynamic_system(const NeProfiler*,NeDynamicSystem*)
;NeSolverSystem*nes_profile_solver_system(const NeProfiler*,NeSolverSystem*,
const char*);NeDae*nes_profile_dae(const NeProfiler*,NeDae*);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "mc_std.h"
#include "pm_std.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct McStateTracerInfoTag McStateTracerInfo;struct
McStateTracerInfoTag{void*mPrivateData;void(*mLogSolveInfo)(McStateTracerInfo*
,const char*solveId,const char*label);void(*mLogSolveInfoR)(McStateTracerInfo*
,const char*solveId,const char*label,real_T data);void(*mLogSolveInfoRv)(
McStateTracerInfo*,const char*solveId,const char*label,const PmRealVector*data
);void(*mLogSolveInfoPr)(McStateTracerInfo*,const char*solveId,const char*
label,const PmSparsityPattern*pattern,const PmRealVector*data);};typedef struct
McNonlinearEquationTag McNonlinearEquation;typedef struct
McNonlinearEquationPrivateDataTag McNonlinearEquationPrivateData;typedef struct
McNonlinearStateTracerTag McNonlinearStateTracer;struct
McNonlinearStateTracerTag{McStateTracerInfo*mInfoTracer;void(*mLogSolveValues)
(const McNonlinearEquation*eq,const PmRealVector*in,const char*solveId);};
struct McNonlinearEquationTag{McNonlinearEquationPrivateData*mPrivateData;
const PmSparsityPattern*mPattern;McNonlinearStateTracer mStateTracer;void(*
mEvaluateFunction)(const McNonlinearEquation*eq,const PmRealVector*in,
PmRealVector*out);void(*mEvaluateJacobian)(const McNonlinearEquation*eq,const
PmRealVector*in,PmRealVector*out);void(*mEvaluateTolerances)(const
McNonlinearEquation*eq,const PmRealVector*in,PmRealVector*out);void(*mDestroy)
(McNonlinearEquation*eq);};typedef struct McNonlinearSolverTag
McNonlinearSolver;typedef struct McNonlinearSolverPrivateDataTag
McNonlinearSolverPrivateData;typedef struct McSolverDiagnosticTag
McSolverDiagnostic;struct McSolverDiagnosticTag{int32_T mBadEquations[5];};
PMF_DEPLOY_STATIC McSolverDiagnostic mc_get_default_diagnostic(void){int32_T i
=0;McSolverDiagnostic result;for(i=0;i<5;i++){result.mBadEquations[i]= -1;}
return result;}struct McNonlinearSolverTag{McNonlinearSolverPrivateData*
mPrivateData;McSolverDiagnostic(*mGetDiagnostic)(const McNonlinearSolver*nls);
McSolverStatus(*mSolve)(const McNonlinearSolver*nls,PmRealVector*rv);void(*
mSetFactor)(const McNonlinearSolver*nls,real_T factor);void(*mDestroy)(
McNonlinearSolver*nls);};typedef struct McNonlinearSolverFactoryTag
McNonlinearSolverFactory;typedef struct McNonlinearSolverFactoryPrivateDataTag
McNonlinearSolverFactoryPrivateData;struct McNonlinearSolverFactoryTag{
McNonlinearSolverFactoryPrivateData*mPrivateData;McNonlinearSolver*(*
mCreateSolver)(const McNonlinearSolverFactory*nlsf,const McNonlinearEquation*
eq);void(*mDestroy)(McNonlinearSolverFactory*nlsf);};typedef struct
McHybridEquationTag McHybridEquation;typedef struct
McHybridEquationPrivateDataTag McHybridEquationPrivateData;typedef struct
McHybridStateTracerTag McHybridStateTracer;struct McHybridStateTracerTag{
McStateTracerInfo*mInfoTracer;void(*mLogSolveValues)(const McHybridEquation*eq
,const PmIntVector*modes,const PmRealVector*in,const char*solveId);};struct
McHybridEquationTag{McHybridEquationPrivateData*mPrivateData;const
PmSparsityPattern*mPattern;size_t mNumModes;McHybridStateTracer mStateTracer;
void(*mEvaluateFunction)(const McHybridEquation*eq,const PmIntVector*modes,
const PmRealVector*in,PmRealVector*out);void(*mEvaluateJacobian)(const
McHybridEquation*eq,const PmIntVector*modes,const PmRealVector*in,PmRealVector
*out);void(*mEvaluateTolerances)(const McHybridEquation*eq,const PmIntVector*
modes,const PmRealVector*in,PmRealVector*out);void(*mComputeModes)(const
McHybridEquation*eq,const PmIntVector*modes,const PmRealVector*in,PmIntVector*
out);void(*mDestroy)(McHybridEquation*eq);};typedef struct McHybridSolverTag
McHybridSolver;typedef struct McHybridSolverPrivateDataTag
McHybridSolverPrivateData;struct McHybridSolverTag{McHybridSolverPrivateData*
mPrivateData;McSolverStatus(*mSolve)(const McHybridSolver*,PmIntVector*,
PmRealVector*);McSolverDiagnostic(*mGetDiagnostic)(const McHybridSolver*nls);
void(*mSetFactor)(const McHybridSolver*nls,real_T factor);void(*mDestroy)(
McHybridSolver*nls);};typedef struct McHybridSolverFactoryTag
McHybridSolverFactory;typedef struct McHybridSolverFactoryPrivateDataTag
McHybridSolverFactoryPrivateData;struct McHybridSolverFactoryTag{
McHybridSolverFactoryPrivateData*mPrivateData;McHybridSolver*(*mCreateSolver)(
const McHybridSolverFactory*nlsf,const McHybridEquation*eq);void(*mDestroy)(
McHybridSolverFactory*nlsf);};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeuSystemEquationPrivateDataTag NeuSystemEquationPrivateData;
typedef struct NeuSystemEquationTag NeuSystemEquation;struct
NeuSystemEquationTag{const McHybridEquation*mGenericEquation;const PmBoolVector
*mLinearEquations;const PmRealVector*mMin;const PmRealVector*mMax;
NeuSystemEquationPrivateData*mPrivateData;void(*mInitialize)(const
NeuSystemEquation*eq,const NeSystemInput*si,PmRealVector*state);void(*
mTerminate)(const NeuSystemEquation*eq,const NeSystemInput*si,const
PmRealVector*state,PmRealVector*out);NeSystemInputSizes(*mSizes)(const
NeuSystemEquation*eq);McSolverDiagnostic(*mConvertDiagnostic)(const
NeuSystemEquation*eq,McSolverDiagnostic genericEqnDiag);void(*mDestroy)(
NeuSystemEquation*);};typedef struct NeuSystemSolverPrivateDataTag
NeuSystemSolverPrivateData;typedef struct NeuSystemSolverTag NeuSystemSolver;
struct NeuSystemSolverTag{NeuSystemSolverPrivateData*mPrivateData;
McSolverStatus(*mSolve)(const NeuSystemSolver*sol,const NeSystemInput*si);
McSolverDiagnostic(*mGetDiagnostic)(const NeuSystemSolver*sol);void(*
mSetFactor)(const NeuSystemSolver*sol,real_T factor);void(*mDestroy)(
NeuSystemSolver*sol);};typedef struct NeuAdvanceableTag NeuAdvanceable;typedef
struct NeuAdvanceablePrivateDataTag NeuAdvanceablePrivateData;struct
NeuAdvanceableTag{McRealFunction*mFF;McMatrixFunction*mMM;McMatrixFunction*
mDxf;McRealFunction*mTol;McRealFunction*mXdDot;McIntFunction*mModes;
McHybridSolverFactory*mSolverFactory;McHybridSolverFactory*mRobustFactory;
real_T mH;};typedef struct NeuAdvancingSolverTag NeuAdvancingSolver;typedef
struct NeuAdvancingSolverPrivateDataTag NeuAdvancingSolverPrivateData;struct
NeuAdvancingSolverTag{NeuAdvancingSolverPrivateData*mPrivateData;
NeuSystemSolver*mStep;boolean_T mNeedsEventNotify;McSolverStatus(*
mInitSolverState)(const NeuAdvancingSolver*solver,const NeSystemInput*si);
size_t(*mGetSolverStateSize)(const NeuAdvancingSolver*solver);void(*mReset)(
const NeuAdvancingSolver*solver);void(*mPutSolverState)(const
NeuAdvancingSolver*solver,const PmRealVector*state);void(*mGetSolverState)(
const NeuAdvancingSolver*solver,const PmRealVector*state);void(*mDestroy)(
NeuAdvancingSolver*solver);};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef int32_T(*DiagnosisSupportMethod)(const NeSolverSystem*topSys,const
NeSystemInput*topInput,real_T,PmCharVector,boolean_T);typedef PmfMessageId(*
StiffDiagnosticMethod)(const NeSolverSystem*sys,const NeSystemInput*input,void
*st);typedef PmfMessageId(*LinearizationSupportMethod)(const NeSolverSystem*,
const NeSystemInput*,PmRealVector*,const boolean_T);typedef PmfMessageId(*
VslsSupportMethod)(const PmRealVector*);typedef void(*EventIterationDiagMethod
)(const NeSolverSystem*,const NeSystemInput*,const PmRealVector*d,const
PmIntVector*q,PmCharVector);typedef struct NeSolverSupportMethodsTag{
DiagnosisSupportMethod mDcDiagnosisSupportMethod;DiagnosisSupportMethod
mTrDiagnosisSupportMethod;DiagnosisSupportMethod mAdvSolDiagnosisSupportMethod
;DiagnosisSupportMethod mInconsistentIcMethod;DiagnosisSupportMethod
mIcJacobianDiagnosticsMethod;DiagnosisSupportMethod
mMassMatrixDiagnosticsMethod;LinearizationSupportMethod
mLinearizationSupportMethod;StiffDiagnosticMethod mStiffDiagnosticMethod;
VslsSupportMethod mVslsSupportMethod;EventIterationDiagMethod
mEventIterationDiagMethod;}NeSolverSupportMethods;void
nes_register_solver_support(const NeSolverSupportMethods*methods);const
NeSolverSupportMethods*nes_get_support_methods(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "ne_std.h"
#include "pm_std.h"
#include "stdarg.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef NeuDiagnosticTree*NeuDiagnosticId;typedef struct
NeuDiagnosticManagerPrivateDataTag NeuDiagnosticManagerPrivateData;struct
NeuDiagnosticManagerTag{NeuDiagnosticManagerPrivateData*mPrivateData;
NeuDiagnosticId(*mStartSubtree)(const NeuDiagnosticManager*mgr);void(*
mFinishSubtree)(const NeuDiagnosticManager*mgr,NeuDiagnosticId diagId,
NeuDiagnosticLevel verbosity,PmfMessageId msgId,va_list args);void(*
mFinishSubtreeUser)(const NeuDiagnosticManager*mgr,NeuDiagnosticId diagId,
NeuDiagnosticLevel verbosity,PmfMessageId msgId,va_list args);void(*
mClearSubtree)(const NeuDiagnosticManager*mgr,NeuDiagnosticId diagId);void(*
mTransferTree)(const NeuDiagnosticManager*dest,const NeuDiagnosticManager*src)
;const NeuDiagnosticTree*(*mGetInitialTree)(const NeuDiagnosticManager*mgr);
void(*mDestroy)(NeuDiagnosticManager*mgr);};PmfMessageId
neu_diagnostic_finish_subtree(const NeuDiagnosticManager*mgr,NeuDiagnosticId
diagId,NeuDiagnosticLevel verbosity,PmfMessageId msgId,...);PmfMessageId
neu_diagnostic_finish_subtree_user(const NeuDiagnosticManager*mgr,
NeuDiagnosticId diagId,NeuDiagnosticLevel verbosity,PmfMessageId msgId,...);
PmfMessageId neu_diagnostic_one_node_subtree(const NeuDiagnosticManager*mgr,
NeuDiagnosticLevel verbosity,PmfMessageId msgId,...);PmfMessageId
neu_diagnostic_finish_subtree_preformatted(const NeuDiagnosticManager*mgr,
NeuDiagnosticId diagId,NeuDiagnosticLevel verbosity,PmfMessageId msgId,const
char*msg);PmfMessageId neu_diagnostic_one_node_subtree_preformatted(const
NeuDiagnosticManager*mgr,NeuDiagnosticLevel verbosity,PmfMessageId msgId,const
char*msg);NeuDiagnosticManager*neu_create_diagnostic_manager(PmAllocator*alloc
);void neu_destroy_diagnostic_manager(NeuDiagnosticManager*mgr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeuDiagnosticTreePrinterTag NeuDiagnosticTreePrinter;typedef
struct NeuDiagnosticTreePrinterPrivateDataTag
NeuDiagnosticTreePrinterPrivateData;struct NeuDiagnosticTreePrinterTag{
NeuDiagnosticTreePrinterPrivateData*mPrivateData;const char*(*mPrint)(
NeuDiagnosticTreePrinter*printer,const NeuDiagnosticTree*tree);void(*mDestroy)
(NeuDiagnosticTreePrinter*printer);};NeuDiagnosticTreePrinter*
neu_create_diagnostic_tree_printer(PmAllocator*alloc);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
const char*neu_tree_viewer_get_id(const NeuDiagnosticTree*tree);const char*
neu_tree_viewer_get_msg(const NeuDiagnosticTree*tree);NeuDiagnosticLevel
neu_tree_viewer_get_level(const NeuDiagnosticTree*tree);size_t
neu_tree_viewer_get_num_causes(const NeuDiagnosticTree*tree);const
NeuDiagnosticTree*neu_tree_viewer_get_cause_k(const NeuDiagnosticTree*tree,
size_t k);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void neu_diagnostic_tree_warning(const NeuDiagnosticTree*tree);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_dae_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct SscRTWLogFcnManagerTag SscRTWLogFcnManager;typedef struct
SscRTWLogFcnManagerObjectTag SscRTWLogFcnManagerObject;struct
SscRTWLogFcnManagerTag{void(*mRtwSetupLogFcn)(SscRTWLogFcnManager*);void(*
mRtwLogFcn)(SscRTWLogFcnManager*,double timeStep,double const*voidInputs,
double*buffer);void(*mRtwDestroy)(SscRTWLogFcnManager*);
SscRTWLogFcnManagerObject*mRtwObject;};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_profiler_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeOutputParametersTag{size_t mDaeIndex;size_t
mOutputFunctionIndex;}NeOutputParameters;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeParameterVectorTag{size_t mN;NeParameterData*mX;}
NeParameterVector;typedef struct NeParameterInfoTag{NeParameterVector mLogicals
;NeParameterVector mIntegers;NeParameterVector mIndices;NeParameterVector
mReals;}NeParameterInfo;typedef struct NeslRtpManagerTag NeslRtpManager;
typedef struct NeslRtpManagerUserDataTag NeslRtpManagerUserData;struct
NeslRtpManagerTag{NeslRtpManagerUserData*mUserData;void(*mDestroy)(
NeslRtpManager*);NeParameterInfo(*mParameterInfo)(const NeslRtpManager*);
boolean_T(*mSetParameters)(const NeslRtpManager*,double,const NeParameterBundle
*,NeuDiagnosticManager*);};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeslSolverHitManagerTag NeslSolverHitManager;typedef struct
NeslSolverHitManagerUserDataTag NeslSolverHitManagerUserData;struct
NeslSolverHitManagerTag{NeslSolverHitManagerUserData*mUserData;void(*mDestroy)
(NeslSolverHitManager*);void(*mGetSolverHits)(const NeslSolverHitManager*,
NeuDiagnosticManager*,PmRealVector*);size_t(*mSolverHitsSize)(const
NeslSolverHitManager*);};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_inline.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct IterationsLogCoreTag{int32_T mNumIters;boolean_T mConvergence;
boolean_T mPrecisionMet;}IterationsLogCore;
#ifdef __cplusplus
}
#endif /* __cplusplus */
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
mMajor;PmIntVector mCache;}SLMatrices;typedef struct NeCustomDataSizeTag
NeCustomDataSize;typedef struct NeCustomDataTag NeCustomData;struct
NeCustomDataSizeTag{size_t mDoubleSize;size_t mIntegerSize;};struct
NeCustomDataTag{double*mDouble;int32_T*mInteger;NeCustomDataSize mSizes;
NeCustomData*mChildData;};NeCustomDataSize ne_get_zero_custom_data_size(void);
NeCustomData*ne_allocate_custom_data(NeCustomDataSize sizes);NeCustomData*
ne_allocate_empty_custom_data(void);void ne_free_custom_data(NeCustomData*ncd)
;size_t ne_measure_packed_custom_data(const NeCustomData*ncd);void
ne_pack_custom_data(const NeCustomData*ncd,char*store);NeCustomData*
ne_recreate_custom_data(const char*store,size_t len);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_stiffness_fwd.h"
#include "ne_std.h"
#include "ne_std.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct McSparseMatrixTag McSparseMatrix;struct McSparseMatrixTag{
PmSparsityPattern*mPattern;PmRealVector*mPr;};McSparseMatrix*
mc_assemble_sparse_matrix(const PmSparsityPattern*patternPtr,const PmRealVector
*prPtr,PmAllocator*allocatorPtr);void mc_disassemble_sparse_matrix(
McSparseMatrix*matrix,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_sparse_matrix(size_t nNzMax,size_t nRow,size_t nCol,PmAllocator*
allocatorPtr);McSparseMatrix*mc_copy_sparse_matrix(const McSparseMatrix*
matrixPtr,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_identity_sparse_matrix(size_t n,PmAllocator*allocatorPtr);
McSparseMatrix*mc_create_hcat_sparse_matrix(const McSparseMatrix*a,const
McSparseMatrix*b,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_vcat_sparse_matrix(const McSparseMatrix*a,const McSparseMatrix*b,
PmAllocator*allocatorPtr);McSparseMatrix*mc_create_section_sparse_matrix(const
McSparseMatrix*matrix,size_t rowStart,size_t rowEndPlusOne,size_t colStart,
size_t colEndPlusOne,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_transpose_sparse_matrix(const McSparseMatrix*matrix,PmAllocator*
allocatorPtr);McSparseMatrix*mc_create_colperm_sparse_matrix(const
McSparseMatrix*matrix,const PmIntVector*perm,PmAllocator*allocatorPtr);
McSparseMatrix*mc_create_rowperm_sparse_matrix(const McSparseMatrix*matrix,
const PmIntVector*perm,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_right_padded_sparse_matrix(const McSparseMatrix*matrix,size_t padding
,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_bottom_padded_sparse_matrix(const McSparseMatrix*matrix,size_t
padding,PmAllocator*allocatorPtr);McSparseMatrix*
mc_create_product_sparse_matrix(const McSparseMatrix*A,const McSparseMatrix*B,
PmAllocator*allocatorPtr);McSparseMatrix*mc_create_sum_sparse_matrix(const
McSparseMatrix*A,const McSparseMatrix*B,PmAllocator*allocatorPtr);
McSparseMatrix*mc_create_sparse_submatrix(const McSparseMatrix*matrix,const
PmBoolVector*selectedRows,const PmBoolVector*selectedCols,PmAllocator*
allocatorPtr);McSparseMatrix*mc_create_sparse_supermatrix(const McSparseMatrix
*matrix,const PmBoolVector*rows,const PmBoolVector*cols,PmAllocator*al);
McSparseMatrix*mc_create_sparse_restriction(const McSparseMatrix*matrix,const
PmBoolVector*selectedRows,const PmBoolVector*selectedCols,PmAllocator*
allocatorPtr);McSparseMatrix*mc_extract_col(const McSparseMatrix*matrix,size_t
col,PmAllocator*allocatorPtr);McSparseMatrix*mc_extract_row(const
McSparseMatrix*matrix,size_t row,PmAllocator*allocatorPtr);McSparseMatrix*
mc_remove_col(const McSparseMatrix*matrix,size_t col,PmAllocator*allocatorPtr)
;McSparseMatrix*mc_remove_row(const McSparseMatrix*matrix,size_t row,
PmAllocator*allocatorPtr);void mc_destroy_sparse_matrix(McSparseMatrix*
matrixPtr,PmAllocator*allocatorPtr);McSparseMatrix*mc_remove_element(const
McSparseMatrix*matrix,size_t element,PmAllocator*allocator);McSparseMatrix*
mc_clean_sparse_matrix(const McSparseMatrix*in,PmAllocator*allocator);typedef
struct NeslContLtiDataTag NeslContLtiData;struct NeslContLtiDataTag{
PmRealVector*mLtiFlatU;McSparseMatrix mDx;McSparseMatrix mDu;PmRealVector mC0;
boolean_T mEmptyDx;boolean_T mEmptyDu;};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_inline.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeslSimulationDataTag NeslSimulationData;typedef struct
NeslSimulationDataDataTag NeslSimulationDataData;struct NeslSimulationDataTag{
PmRealVector*(*mTime)(const NeslSimulationData*);PmRealVector*(*mContStates)(
const NeslSimulationData*);PmIntVector*(*mModeVector)(const NeslSimulationData
*);PmRealVector*(*mDiscStates)(const NeslSimulationData*);PmIntVector*(*
mSampleHits)(const NeslSimulationData*);PmRealVector*(*mOutputs)(const
NeslSimulationData*);PmRealVector*(*mDx)(const NeslSimulationData*);
PmRealVector*(*mNonSampledZCs)(const NeslSimulationData*);PmRealVector*(*
mResiduals)(const NeslSimulationData*);PmRealVector*(*mTolerances)(const
NeslSimulationData*);PmSparsityPattern*(*mM_P)(const NeslSimulationData*);
PmRealVector*(*mMassMatrixPr)(const NeslSimulationData*);PmSparsityPattern*(*
mLinJacobianPattern)(const NeslSimulationData*);PmRealVector*(*mLinJacobian)(
const NeslSimulationData*);PmSparsityPattern*(*mSolJacobianPattern)(const
NeslSimulationData*);PmRealVector*(*mSolJacobianPr)(const NeslSimulationData*)
;PmRealVector*(*mNumjacDxLo)(const NeslSimulationData*);PmRealVector*(*
mNumjacDxHi)(const NeslSimulationData*);PmRealVector*(*mInputValues)(const
NeslSimulationData*);PmIntVector*(*mInputOffsets)(const NeslSimulationData*);
boolean_T*(*mFoundZcEvents)(const NeslSimulationData*);boolean_T*(*
mIsMajorTimeStep)(const NeslSimulationData*);boolean_T*(*mIsSolverAssertCheck)
(const NeslSimulationData*);boolean_T*(*mIsSolverCheckingCIC)(const
NeslSimulationData*);boolean_T*(*mIsComputingJacobian)(const NeslSimulationData
*);boolean_T*(*mIsEvaluatingF0)(const NeslSimulationData*);boolean_T*(*
mIsSolverRequestingReset)(const NeslSimulationData*);boolean_T*(*
mIsFundamentalSampleHit)(const NeslSimulationData*);boolean_T*(*
mIsModeUpdateTimeStep)(const NeslSimulationData*);boolean_T*(*
mCstateHasChanged)(const NeslSimulationData*);void(*mDestroy)(
NeslSimulationData*);NeslSimulationDataData*mData;};PMF_DEPLOY_STATIC
PmRealVector nesl_get_time(const NeslSimulationData*sd){return*(sd->mTime(sd))
;}PMF_DEPLOY_STATIC PmRealVector nesl_get_cont_states(const NeslSimulationData
*sd){return*(sd->mContStates(sd));}PMF_DEPLOY_STATIC PmIntVector
nesl_get_mode_vector(const NeslSimulationData*sd){return*(sd->mModeVector(sd))
;}PMF_DEPLOY_STATIC PmRealVector nesl_get_disc_states(const NeslSimulationData
*sd){return*(sd->mDiscStates(sd));}PMF_DEPLOY_STATIC PmIntVector
nesl_get_sample_hits(const NeslSimulationData*sd){return*(sd->mSampleHits(sd))
;}PMF_DEPLOY_STATIC PmRealVector nesl_get_outputs(const NeslSimulationData*sd)
{return*(sd->mOutputs(sd));}PMF_DEPLOY_STATIC PmRealVector nesl_get_dx(const
NeslSimulationData*sd){return*(sd->mDx(sd));}PMF_DEPLOY_STATIC PmRealVector
nesl_get_nonsampled_zcs(const NeslSimulationData*sd){return*(sd->
mNonSampledZCs(sd));}PMF_DEPLOY_STATIC PmRealVector nesl_get_residuals(const
NeslSimulationData*sd){return*(sd->mResiduals(sd));}PMF_DEPLOY_STATIC
PmRealVector nesl_get_tolerances(const NeslSimulationData*sd){return*(sd->
mTolerances(sd));}PMF_DEPLOY_STATIC PmSparsityPattern
nesl_get_mass_matrix_pattern(const NeslSimulationData*sd){return*(sd->mM_P(sd)
);}PMF_DEPLOY_STATIC PmRealVector nesl_get_mass_matrix_pr(const
NeslSimulationData*sd){return*(sd->mMassMatrixPr(sd));}PMF_DEPLOY_STATIC
PmSparsityPattern nesl_get_lin_jacobian_pattern(const NeslSimulationData*sd){
return*(sd->mLinJacobianPattern(sd));}PMF_DEPLOY_STATIC PmRealVector
nesl_get_lin_jacobian(const NeslSimulationData*sd){return*(sd->mLinJacobian(sd
));}PMF_DEPLOY_STATIC PmSparsityPattern nesl_get_sol_jacobian_pattern(const
NeslSimulationData*sd){return*(sd->mSolJacobianPattern(sd));}PMF_DEPLOY_STATIC
PmRealVector nesl_get_sol_jacobian_pr(const NeslSimulationData*sd){return*(sd
->mSolJacobianPr(sd));}PMF_DEPLOY_STATIC PmRealVector nesl_get_numjac_dx_lo(
const NeslSimulationData*sd){return*(sd->mNumjacDxLo(sd));}PMF_DEPLOY_STATIC
PmRealVector nesl_get_numjac_dx_hi(const NeslSimulationData*sd){return*(sd->
mNumjacDxHi(sd));}PMF_DEPLOY_STATIC PmRealVector nesl_get_input_values(const
NeslSimulationData*sd){return*(sd->mInputValues(sd));}PMF_DEPLOY_STATIC
PmIntVector nesl_get_input_offsets(const NeslSimulationData*sd){return*(sd->
mInputOffsets(sd));}PMF_DEPLOY_STATIC boolean_T nesl_found_zcs(const
NeslSimulationData*sd){return*(sd->mFoundZcEvents(sd));}PMF_DEPLOY_STATIC
boolean_T nesl_is_major_time_step(const NeslSimulationData*sd){return*(sd->
mIsMajorTimeStep(sd));}PMF_DEPLOY_STATIC boolean_T nesl_is_solver_assert_check
(const NeslSimulationData*sd){return*(sd->mIsSolverAssertCheck(sd));}
PMF_DEPLOY_STATIC boolean_T nesl_is_solver_checking_cic(const
NeslSimulationData*sd){return*(sd->mIsSolverCheckingCIC(sd));}
PMF_DEPLOY_STATIC boolean_T nesl_is_computing_jacobian(const NeslSimulationData
*sd){return*(sd->mIsComputingJacobian(sd));}PMF_DEPLOY_STATIC boolean_T
nesl_is_evaluating_f0(const NeslSimulationData*sd){return*(sd->mIsEvaluatingF0
(sd));}PMF_DEPLOY_STATIC boolean_T nesl_is_solver_requesting_reset(const
NeslSimulationData*sd){return*(sd->mIsSolverRequestingReset(sd));}
PMF_DEPLOY_STATIC boolean_T nesl_is_fundamental_sample_hit(const
NeslSimulationData*sd){return*(sd->mIsFundamentalSampleHit(sd));}
PMF_DEPLOY_STATIC boolean_T nesl_is_mode_update_time_step(const
NeslSimulationData*sd){return*(sd->mIsModeUpdateTimeStep(sd));}
PMF_DEPLOY_STATIC void nesl_set_cstate_has_changed(const NeslSimulationData*sd
,boolean_T value){*(sd->mCstateHasChanged(sd))=value;}PMF_DEPLOY_STATIC
boolean_T nesl_get_cstate_has_changed(const NeslSimulationData*sd){return*(sd
->mCstateHasChanged(sd));}PMF_DEPLOY_STATIC boolean_T nesl_has_sample_hits(
const NeslSimulationData*sd){size_t i;PmIntVector sample_hits=
nesl_get_sample_hits(sd);for(i=0;i<sample_hits.mN;i++){if(sample_hits.mX[i]){
return true;}}return false;}typedef struct NeslSimulationSizesTag{size_t
mNumContStates;size_t mNumDiscStates;size_t mNumModes;size_t mNumOutputs;
size_t mNumZCs;size_t mNumRanges;size_t mNumSamples;size_t mNumMassMatrixNzMax
;size_t mNumLinJacobianNzMax;size_t mNumSolJacobianNzMax;size_t
mNumTrimResiduals;boolean_T mDisableProjection;boolean_T mDefaultLinJacobian;
boolean_T mDisableSolJacobian;boolean_T mIsMConstant;boolean_T
mUpdateJacobianAtReset;boolean_T mIsDae;boolean_T mRegistersStateNames;
boolean_T mHasUpdate;const PmIntVector*mNumInputs;const PmBoolVector*
mDirectFeedthrough;}NeslSimulationSizes;typedef struct NeslStateNameDataTag{
size_t mWidth;const char*mStateName;const char*mBlockName;}NeslStateNameData;
typedef struct NeslStateNameVectorTag{NeslStateNameData*mStateNames;size_t
mNumStates;}NeslStateNameVector;typedef enum NeslSimulatorMethodIdTag{
NESL_SIM_INVALID= -1,NESL_SIM_INITIALIZEONCE,NESL_SIM_INITSYSTEMMATRICES,
NESL_SIM_OUTPUTS,NESL_SIM_UPDATE,NESL_SIM_PROJECTION,NESL_SIM_MASSMATRIX,
NESL_SIM_DERIVATIVES,NESL_SIM_FORCINGFUNCTION,NESL_SIM_ZEROCROSSINGS,
NESL_SIM_LINJACOBIAN,NESL_SIM_LINJACOBIAN_DSS,NESL_SIM_SOLJACOBIAN,
NESL_SIM_LINJACOBIANIRJC,NESL_SIM_SOLJACOBIANIRJC,NESL_SIM_RESIDUALS,
NESL_SIM_TOLERANCES,NESL_SIM_REINIT,NESL_SIM_NUMJAC_DX_BOUNDS,
NESL_SIM_PUSHINFO,NESL_NUM_SIMULATOR_METHODS}NeslSimulatorMethodId;typedef
struct NeslSimulatorTag NeslSimulator;typedef struct NeslSimulatorUserDataTag
NeslSimulatorUserData;typedef enum NeslSimulatorStatusTag{NESL_SIM_ERROR= -1,
NESL_SIM_OK}NeslSimulatorStatus;typedef NeslSimulatorStatus(*
NeslSimulatorMethod)(const NeslSimulator*,const NeslSimulationData*,
NeuDiagnosticManager*);typedef void(*NeslSimulatorInitializeStartStatus)(void*
);typedef void(*NeslSimulatorInitializeEndStatus)(void*);struct
NeslSimulatorTag{NeslSimulatorUserData*mUserData;NeslSimulatorMethod mMethods[
NESL_NUM_SIMULATOR_METHODS];NeslSimulationSizes(*mSizes)(const NeslSimulator*s
);void(*mStateNames)(const NeslSimulator*s,const NeslStateNameVector*snv);void
(*mSampleTimes)(const NeslSimulator*s,PmRealVector*periods,PmRealVector*
offsets);NeslSimulatorStatus(*mInitialize)(const NeslSimulator*s,const
NeModelParameters*mp,NeuDiagnosticManager*mgr,
NeslSimulatorInitializeStartStatus start,NeslSimulatorInitializeEndStatus end,
void*rb);const NeslContLtiData*(*mGetContLtiData)(const NeslSimulator*s);const
SLMatrices**(*mGetSLMatrices)(const NeslSimulator*s);NeslSimulatorStatus(*
mGetStiffness)(const NeslSimulator*s,const NeslSimulationData*sim_data,void*st
,NeuDiagnosticManager*);NeStiffness*mStiffness;void(*mLogIterations)(const
NeslSimulator*s,const NeslSimulationData*sim_data,IterationsLogCore*il);
NeslSimulatorStatus(*mGetMaxFreq)(const NeslSimulator*simulator_ptr,double*out
);void(*mDestroy)(NeslSimulator*);NeCustomData*(*mGetCustomData)(const
NeslSimulator*);NeslSimulatorStatus(*mSetCustomData)(const NeCustomData*,const
NeslSimulator*,NeuDiagnosticManager*);const NeZCData*mZCData;const NeRange*
mRanges;NeParameterInfo mParameterInfo;void(*mSetParameters)(const
NeslSimulator*,const NeParameterBundle*);};PMF_DEPLOY_STATIC
NeslSimulatorStatus ne_simulator_method(const NeslSimulator*simulator_ptr,
NeslSimulatorMethodId method_id,const NeslSimulationData*sim_data,
NeuDiagnosticManager*mgr){return simulator_ptr->mMethods[method_id](
simulator_ptr,sim_data,mgr);}typedef enum NeslSimulatorCategoryTag{
NESL_SIMULATOR_CATEGORY_INVALID= -1,NESL_SIMULATOR_CATEGORY_STATE,
NESL_SIMULATOR_CATEGORY_OUTPUT,NESL_SIMULATOR_CATEGORY_NUM}
NeslSimulatorCategory;typedef struct NeslSimulatorGroupTag NeslSimulatorGroup;
typedef struct NeslSimulatorGroupDataTag NeslSimulatorGroupData;struct
NeslSimulatorGroupTag{const NeslSimulator*(*mSimulator)(const
NeslSimulatorGroup*g,NeslSimulatorCategory c,size_t i);const NeslRtpManager*(*
mRtpManager)(const NeslSimulatorGroup*g,size_t i);const SscRTWLogFcnManager*(*
mRtwLogFcnManager)(const NeslSimulatorGroup*g,size_t i);const
NeslSolverHitManager*(*mSolverHitManager)(const NeslSimulatorGroup*g,size_t i)
;NeslSimulatorGroup*(*mClone)(const NeslSimulatorGroup*g);void(*mGetReference)
(NeslSimulatorGroup*g);void(*mReleaseReference)(NeslSimulatorGroup*g);const
NeProfiler*mProfiler;NeslSimulatorGroupData*mData;size_t mNumSimulators[
NESL_SIMULATOR_CATEGORY_NUM];size_t mNumRtpManagers;size_t
mNumRtwLogFcnManagers;size_t mNumSolverHitManagers;};PmfMessageId
nesl_register_simulator_group(const char*key,const size_t numDae,NeDae*const*
dae,const NeSolverParameters*sp,const NeModelParameters*mp,const size_t
numOutputs,const NeOutputParameters*op,const size_t numRtpDaes,const int*
rtpDaes,const size_t numRtwLoggingDaes,const int*rtwLoggingDaes,const size_t
numSolverHitDaes,const int*solverhitDaes);typedef struct
NeslSimulatorGroupRegistryTag NeslSimulatorGroupRegistry;typedef struct
NeslSimulatorGroupRegistryDataTag NeslSimulatorGroupRegistryData;struct
NeslSimulatorGroupRegistryTag{void(*mInsert)(const NeslSimulatorGroupRegistry*
registry,const char*key,NeslSimulatorGroup*group);void(*mErase)(const
NeslSimulatorGroupRegistry*registry,const char*key);void(*mEraseModel)(const
NeslSimulatorGroupRegistry*registry,const char*model);NeslSimulatorGroup*(*
mLookup)(const NeslSimulatorGroupRegistry*registry,const char*key);const
NeslSimulator*(*mLeaseSimulator)(const NeslSimulatorGroupRegistry*registry,
const char*key,NeslSimulatorCategory cat,size_t idx);const NeslRtpManager*(*
mLeaseRtpManager)(const NeslSimulatorGroupRegistry*registry,const char*key,
size_t idx);const SscRTWLogFcnManager*(*mLeaseRtwLogFcnManager)(const
NeslSimulatorGroupRegistry*registry,const char*key,size_t idx);const
NeslSolverHitManager*(*mLeaseSolverHitManager)(const NeslSimulatorGroupRegistry
*registry,const char*key,size_t idx);boolean_T(*mHasBeenCloned)(const
NeslSimulatorGroupRegistry*registry,const char*key);void(*mClear)(const
NeslSimulatorGroupRegistry*registry);void(*mDestroy)(
NeslSimulatorGroupRegistry*registry);NeslSimulatorGroupRegistryData*mData;};
const NeslSimulatorGroupRegistry*nesl_get_registry(void);void
nesl_destroy_current_registry(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct LtiIcDataTag{PmSparsityPattern mStatesPattern;PmRealVector mStatesPr;
PmSparsityPattern mInputsPattern;PmRealVector mInputsPr;PmRealVector mConsts;}
;typedef struct LtiIcDataTag LtiIcData;struct LtiDcDataTag{PmSparsityPattern
mAPattern;PmRealVector mAPr;PmSparsityPattern mBPattern;PmRealVector mBPr;
PmRealVector mFa;};typedef struct LtiDcDataTag LtiDcData;struct LtiIcParamsTag
{LtiIcData mFirstSolve,mSecondSolve;LtiDcData mDcSolve;PmRealVector mGuess;
real_T mResTol;boolean_T mDoDC;};typedef struct LtiIcParamsTag LtiIcParams;
typedef void(*LtiDiagnosisMethod)(void*,PmCharVector);typedef struct
LtiSupportMethodsTag{void*mLtiDiagnosisData;LtiDiagnosisMethod
mIcDiagnosisMethod;LtiDiagnosisMethod mDcDiagnosisMethod;}LtiSupportMethods;
boolean_T ic_solve_impl(PmRealVector*x,const PmRealVector*u,const LtiIcParams*
ltiIcParams,const LtiSupportMethods*methods,NeuDiagnosticManager*mgr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct LtiRtwLogDataTag{PmSparsityPattern mCPattern;PmRealVector mCPr;
PmSparsityPattern mDPattern;PmRealVector mDPr;PmRealVector mYa;};typedef struct
LtiRtwLogDataTag LtiRtwLogData;struct LtiRtwLogParamsTag{LtiRtwLogData mLogData
;};typedef struct LtiRtwLogParamsTag LtiRtwLogParams;void rtw_log_obs_solve(
const LtiRtwLogParams*ltiLogParams,const NeDynamicSystemInput*input,
PmRealVector*output,NeuDiagnosticManager*mgr);SscRTWLogFcnManager*
ssc_lti_create_rtw_log_fcn_manager(const LtiRtwLogParams*logParams,real_T
startTime,unsigned int numInputs,unsigned int numStates,unsigned int
inputPortWidth,unsigned int bufferWidth,NeuDiagnosticManager*diagMgr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "mc_std.h"
#include "pm_inline.h"
#include "pm_std.h"
#include "pm_std.h"
#include "pm_log.h"
#include "ne_dae_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeRealFunctorDataTag NeRealFunctorData;typedef struct
NeRealFunctorTag NeRealFunctor;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_profiler_fwd.h"
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
#include "pm_std.h"
#include "pm_std.h"
#include "mc_std.h"
#include "pm_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct NeRealFunctorTag{PmfMessageId(*mEvaluate)(const NeRealFunctor*functor,
const NeSystemInput*input,const PmRealVector*output,NeuDiagnosticManager*
manager);NeSystemInputSizes(*mGetInputSizes)(const NeRealFunctor*functor);void
(*mDestroy)(NeRealFunctor*functor);NeRealFunctorData*mData;};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_initer_fwd.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct SscIniterDataTag SscIniterData;typedef struct
SscComputeSimDataTag SscComputeSimData;typedef struct SscComputeSimTag
SscComputeSim;struct SscComputeSimTag{SscComputeSimData*mData;void(*
mComputeSim)(const SscComputeSim*cs,const NeSystemInput*simSI,const
NeRealFunctor*obsIniter,const NeSystemInput*initerSI);void(*mDestroy)(
SscComputeSim*cs);};struct SscComputeStateTag{SscComputeSimData*mData;void(*
mComputeState)(const SscComputeState*cs,const NeSystemInput*initerSI,const
NeSystemInput*simSI);void(*mDestroy)(SscComputeState*cs);};struct SscIniterTag
{PmfMessageId(*mSolve)(const SscIniter*initer,const SscComputeSim*cs,const
NeSystemInput*simSI,NeuDiagnosticManager*mgr);void(*mSetParameters)(const
SscIniter*initer,const NeParameterBundle*bundle);void(*mGenerate)(const
SscIniter*initer,const NeCgParams*cgParams,NeCgResults*cgResults);void(*
mSetDiagPtr)(const SscIniter*initer,void*st);const NeObservableData*
mObservableData;size_t mNumObservables;SscIniter*(*mCloneFcn)(const SscIniter*
initer);void(*mDestroy)(SscIniter*initer);size_t mRefCount;SscIniterData*mData
;};PMF_DEPLOY_STATIC void ssc_initer_get_reference(SscIniter*initer){if(initer
){++(initer->mRefCount);}}PMF_DEPLOY_STATIC void ssc_initer_release_reference(
SscIniter*initer){if(initer){(void)0;;if(--(initer->mRefCount)==0){initer->
mDestroy(initer);}}}
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_stiffness_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct NeStiffnessDataTag NeStiffnessData;struct NeStiffnessTag{
NeStiffnessData*mData;void(*mSetTimes)(NeStiffness*s,const PmRealVector*ts);
PmRealVector*(*mGetTimes)(NeStiffness*s);void(*mStart)(NeStiffness*s);void(*
mSetState)(NeStiffness*s,NeSystemInput*input);size_t(*mGetNumStates)(
NeStiffness*s);PmfMessageId(*mComputeStiffness)(NeStiffness*s,void*stiff,
size_t k,NeuDiagnosticManager*mgr);void(*mDestroy)(NeStiffness*s);};
NeStiffness*stiffness_create(NeDae*dae,SscComputeState*cs,NeSystemInputSizes*
sizes);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "pm_std.h"
#include "pm_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct _McNDTree{int32_T st,sz,id;struct _McNDTree*lt,*rt;}McNDTree;
typedef struct _McNDPermData{McNDTree*mNd;PmIntVector*mP;PmIntVector*mQ;}
McNDPermData;typedef struct _McNDPermDataFlat{size_t m,n;int32_T*st,*sz,*lt,*
rt;int32_T*p,*q;}McNDPermDataFlat;PmIntVector*mc_basker_column_permute(const
McNDTree*bt,const PmSparsityPattern*pat,PmAllocator*alloc);void
mc_basker_tree_free(McNDTree*nd);McNDTree*mc_basker_tree_copy(McNDTree*nd);
void mc_basker_tree_shift(McNDTree*nd,int32_T st);int32_T mc_basker_tree_level
(McNDTree*bt);void mc_free_perm_data(McNDPermData*pd);McNDPermDataFlat*
mc_perm_data_to_flat(const McNDPermData*pd);McNDPermData*
mc_perm_data_from_flat(const McNDPermDataFlat*pdf);void mc_free_perm_data_flat
(McNDPermDataFlat*pdf);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_initer_fwd.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef enum{INVALID_CARDINALITY= -1,SWL_FINITE,SWL_COUNTABLE,SWL_UNCOUNTABLE,
SWL_NONLINEAR,NUM_CARDINALITY}ModeCardinality;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#include "ne_std.h"
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct GlobalMethodTableTag GlobalMethodTable;typedef union
GmtMethodOutputTag{PmIntVector mMODE;PmRealVector mY;PmRealVector mOBS;
PmIntVector mASSERT;PmIntVector mIASSERT;PmRealVector mINIT_R;PmIntVector
mINIT_I;PmRealVector mCACHE_R;PmIntVector mCACHE_I;PmRealVector mUPDATE_R;
PmIntVector mUPDATE_I;PmBoolVector mLOCK_R;PmBoolVector mLOCK_I;PmRealVector
mUPDATE2_R;PmIntVector mUPDATE2_I;PmBoolVector mLOCK2_R;PmBoolVector mLOCK2_I;
PmIntVector mDP_L;PmIntVector mDP_I;PmIntVector mDP_J;PmRealVector mDP_R;
PmRealVector mINIT_DIFF;PmRealVector mLOG;}GmtMethodOutput;typedef enum
GmtMethodIdTag{GMT_INVALID_METHOD_ID= -1,GMT_METHOD_MODE,GMT_METHOD_Y,
GMT_METHOD_OBS,GMT_METHOD_ASSERT,GMT_METHOD_IASSERT,GMT_METHOD_INIT_R,
GMT_METHOD_INIT_I,GMT_METHOD_CACHE_R,GMT_METHOD_CACHE_I,GMT_METHOD_UPDATE_R,
GMT_METHOD_UPDATE_I,GMT_METHOD_LOCK_R,GMT_METHOD_LOCK_I,GMT_METHOD_UPDATE2_R,
GMT_METHOD_UPDATE2_I,GMT_METHOD_LOCK2_R,GMT_METHOD_LOCK2_I,GMT_METHOD_DP_L,
GMT_METHOD_DP_I,GMT_METHOD_DP_J,GMT_METHOD_DP_R,GMT_METHOD_INIT_DIFF,
GMT_METHOD_LOG,GMT_NUM_METHODS}GmtMethodId;typedef int32_T(*GmtMethod)(const
GlobalMethodTable*,const NeDynamicSystemInput*,GmtMethodOutput*);struct
GlobalMethodTableTag{GmtMethod mMethods[GMT_NUM_METHODS];size_t mNumY;size_t
mNumObsElts;size_t mNumAsserts;const NeAssertData*mAssertData;size_t
mNumAssertRanges;const NeRange*mAssertRanges;size_t mNumInitialAsserts;const
NeAssertData*mInitialAssertData;size_t mNumInitialAssertRanges;const NeRange*
mInitialAssertRanges;NeParameterInfo mParameterInfo;PmIntVector*mModeIndices;
PmIntVector*mObsIsLinear;};typedef struct SwitchedLinearClumpTag
SwitchedLinearClump;typedef union SlcMethodOutputTag{PmSparsityPattern mM_P;
PmRealVector mM;PmSparsityPattern mJ_P;PmRealVector mJ;PmRealVector mFX;
PmRealVector mF;PmRealVector mXNPLUS1;PmIntVector mMODE;}SlcMethodOutput;
typedef enum SlcMethodIdTag{SLC_INVALID_METHOD_ID= -1,SLC_METHOD_M_P,
SLC_METHOD_M,SLC_METHOD_J_P,SLC_METHOD_J,SLC_METHOD_FX,SLC_METHOD_F,
SLC_METHOD_XNPLUS1,SLC_METHOD_MODE,SLC_NUM_METHODS}SlcMethodId;typedef int32_T
(*SlcMethod)(const SwitchedLinearClump*,const NeDynamicSystemInput*,
SlcMethodOutput*);typedef PmSizeVector*SizeVectorPtr;struct
SwitchedLinearClumpTag{SlcMethod mMethods[SLC_NUM_METHODS];size_t mMNnz;size_t
mJNnz;size_t mNEqs;boolean_T mIsLti;NeDynamicSystemInputSizes mSizes;
SizeVectorPtr mSelectors[NE_NUM_DYNAMIC_SYSTEM_INPUT_ID];PmIntVector*
mStateIndices;PmIntVector*mStateIndicesOut;PmIntVector*mModeIndices;
PmIntVector*mMRefIndices;PmIntVector*mQRefIndices;boolean_T mIsModeBoolean;
ModeCardinality mModeCardinality;};typedef struct DifferentialClumpTag
DifferentialClump;typedef union DcMethodOutputTag{PmSparsityPattern mM_P;
PmRealVector mM;PmRealVector mF;}DcMethodOutput;typedef enum DcMethodIdTag{
DC_INVALID_METHOD_ID= -1,DC_METHOD_M_P,DC_METHOD_M,DC_METHOD_F,DC_NUM_METHODS}
DcMethodId;typedef int32_T(*DcMethod)(const DifferentialClump*,const
NeDynamicSystemInput*,DcMethodOutput*);struct DifferentialClumpTag{DcMethod
mMethods[DC_NUM_METHODS];size_t mMNnz;PmIntVector*mStateIndices;PmIntVector*
mMRefIndices;PmIntVector*mQRefIndices;ModeCardinality mModeCardinality;};
typedef struct SwitchedLinearSystemTag SwitchedLinearSystem;struct
SwitchedLinearSystemTag{NeDynamicSystemInputSizes mSizes;PmIntVector*
mDiffStateIndices;GlobalMethodTable*mGlobalMethodTable;size_t mNumClumps;
SwitchedLinearClump**mClumps;DifferentialClump*mDiffClump;SscIniter*mIniter;
SscIniter*mDaeIniter;McNDPermData*(*mGetNDPermData)(const PmSparsityPattern*
pat,size_t nLev);void(*mDestroy)(SwitchedLinearSystem*sls);};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
typedef struct SimulatorTag Simulator;typedef struct SimulatorDataTag
SimulatorData;typedef void(*SimulatorInitializeStartStatus)(void*);typedef void
(*SimulatorInitializeEndStatus)(void*);struct SimulatorTag{size_t mNumInputs;
size_t mNumStates;size_t mNumOutputs;double mStepSize;const NeParameterInfo*
mParameterInfo;SimulatorData*mData;const PmSizeVector*mMemEstimates;void(*
mSetReInit)(const Simulator*solver,boolean_T reInit);boolean_T(*mGetReInit)(
const Simulator*solver);boolean_T(*mSetParameters)(const Simulator*solver,
const NeParameterBundle*bundle);boolean_T(*mStart)(const Simulator*solver,
const NeuDiagnosticManager*mgr);NeDynamicSystemInput(*mGetDsi)(const Simulator
*simulator,const PmRealVector*inputs,const PmRealVector*states,real_T time);
void(*mRtwLogFcn)(const Simulator*simulator,const NeDynamicSystemInput*input,
PmRealVector*output);boolean_T(*mSolve)(const Simulator*solver,const
PmRealVector*inputs,const PmRealVector*states,const PmRealVector*outputs,
real_T time,void*st,const NeuDiagnosticManager*mgr,boolean_T firstOutput);
boolean_T(*mCheck)(const Simulator*solver,const PmRealVector*inputs,const
PmRealVector*states,real_T time,const NeuDiagnosticManager*mgr);void(*
mInitialize)(const Simulator*s,SimulatorInitializeStartStatus start,
SimulatorInitializeEndStatus end,void*rb);void(*mPushInfo)(const Simulator*s,
IterationsLogCore*il);const McNDPermData*(*mGetNDPermData)(const Simulator*s);
NeStiffness*(*mGetStiffness)(const Simulator*s);void(*mDestroy)(Simulator*
solver);};Simulator*simulator_create(SwitchedLinearSystem*sls,const
NeSolverParameters param,const McLinearAlgebraFactory*la,const NeVariableData*
vardata,const NeObservableData*obsdata,const size_t numObservables,const
DaemonChoice dc);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
NeslRtpManager*rtp_manager_create(const Simulator*simulator);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#include "ne_std.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
SscRTWLogFcnManager*ssc_swl_create_rtw_log_fcn_manager(const Simulator*
simulator,unsigned int inputPortWidth,unsigned int bufferWidth,
NeuDiagnosticManager*diagMgr);typedef struct NeslInputHelperVectorsTag
NeslInputHelperVectors;struct NeslInputHelperVectorsTag{PmRealVector*
mTimeVector;PmRealVector*mInputValuesVector;PmIntVector*mInputOffsetsVector;
void(*mDestroy)(NeslInputHelperVectors*helper);};typedef struct
NeslInputHelperTag NeslInputHelper;typedef struct NeslInputHelperDataTag
NeslInputHelperData;struct NeslInputHelperTag{NeslInputHelperVectors*(*
mCreateVectors)(const NeslInputHelper*helper,size_t inputPortWidth);void(*
mUpdate)(const NeslInputHelper*helper,const NeslInputHelperVectors*vectors);
void(*mDestroy)(NeslInputHelper*helper);const NeSystemInput*mI;
NeslInputHelperData*mData;};NeslInputHelper*nesl_create_input_helper(
NeSystemInputSizes sizes,boolean_T overlap);struct SscRTWLogFcnManagerObjectTag
{NeDae*mDaePtr;PmAllocator*mAllocatorPtr;boolean_T*mDirectFeedThrough;unsigned
int mNumInputPorts;unsigned int mInputPortWidth;unsigned int mBufferWidth;
NeslInputHelperVectors*mInputHelperVectors;NeslInputHelper*mInputHelper;
NeuDiagnosticManager*mDiagnosticManager;};
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* ___nesl_rtw_h__ */
