/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: simulation.c
 *
 * Code generated for Simulink model 'simulation'.
 *
 * Model version                  : 4.4
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Tue Nov  7 17:36:31 2023
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulation.h"
#include "simulation_types.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "simulation_private.h"
#include <stddef.h>
#include <emmintrin.h>
#include "simulation_capi.h"
#include "rt_defines.h"

/* Named constants for Chart: '<S5>/Discretizer' */
#define simulation_IN_Detached         ((uint8_T)1U)
#define simulation_IN_Idle             ((uint8_T)2U)
#define simulation_IN_NO_ACTIVE_CHILD  ((uint8_T)0U)
#define simulation_IN_S0               ((uint8_T)3U)
#define simulation_IN_S1_Ignite        ((uint8_T)1U)
#define simulation_IN_S1_engaged       ((uint8_T)2U)
#define simulation_IN_S1_fire          ((uint8_T)4U)
#define simulation_IN_S2_Fire          ((uint8_T)5U)

/* Block signals (default storage) */
B_simulation_T simulation_B;

/* Continuous states */
X_simulation_T simulation_X;

/* Disabled State Vector */
XDis_simulation_T simulation_XDis;

/* Block states (default storage) */
DW_simulation_T simulation_DW;

/* External inputs (root inport signals with default storage) */
ExtU_simulation_T simulation_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_simulation_T simulation_Y;

/* Real-time model */
static RT_MODEL_simulation_T simulation_M_;
RT_MODEL_simulation_T *const simulation_M = &simulation_M_;

/* Forward declaration for local functions */
static boolean_T simulation_isequal_o(const real_T varargin_1[3], const real_T
  varargin_2[3]);
static void imuSensor_set_MagneticFieldNED(fusion_simulink_imuSensor_sim_T *obj,
  const real_T val[3]);
static boolean_T simulation_isequal_oi(const real_T varargin_1[9], const real_T
  varargin_2[9]);
static boolean_T simulation_isequal(const real_T varargin_1[2], const real_T
  varargin_2[2]);
static void IMUSensorParameters_updateSyste(real_T obj_MeasurementRange, real_T
  obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], g_fusion_internal_Acceleromet_T
  *sobj);
static void IMUSensorParameters_updateSys_o(real_T obj_MeasurementRange, real_T
  obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], const real_T obj_AccelerationBias
  [3], h_fusion_internal_GyroscopeSi_T *sobj);
static void IMUSensorParameters_updateSy_oi(real_T obj_MeasurementRange, real_T
  obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], i_fusion_internal_Magnetomete_T
  *sobj);
static boolean_T simulation_vectorAny(const boolean_T x_data[], const int32_T
  x_size[2]);
static void simulat_genrand_uint32_vector_o(uint32_T mt[625], uint32_T u[2]);
static real_T simulation_genrandu_o(uint32_T mt[625]);
static void simulation_filter(real_T b, real_T a[2], const real_T x[3], const
  real_T zi[3], real_T y[3], real_T zf[3]);
static void simulation_SystemCore_step_o(g_fusion_internal_Acceleromet_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[9], const real_T
  varargin_3[9], real_T varargout_1[3]);
static void simulation_SystemCore_step_oi(h_fusion_internal_GyroscopeSi_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[3], const real_T
  varargin_3[9], const real_T varargin_4[9], real_T varargout_1[3]);
static void simulation_imuSensor_stepImpl(fusion_simulink_imuSensor_sim_T *obj,
  const real_T la[3], const real_T av[3], const real_T o[4], real_T a[3], real_T
  g[3], real_T m[3]);
static void simulation_SystemCore_step(fusion_simulink_imuSensor_sim_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[3], const real_T
  varargin_3[4], real_T varargout_1[3], real_T varargout_2[3], real_T
  varargout_3[3]);
static void simulation_SystemCore_setup(fusion_simulink_imuSensor_sim_T *obj);
static void simulat_IMUSensorBase_resetImpl(fusion_simulink_imuSensor_sim_T *obj);
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (simulation_M->Timing.TaskCounters.TID[2])++;
  if ((simulation_M->Timing.TaskCounters.TID[2]) > 49) {/* Sample time: [0.05s, 0.0s] */
    simulation_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/* Projection for root system: '<Root>' */
void simulation_projection(void)
{
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  char *msg;
  real_T tmp_0[20];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[6];
  boolean_T tmp;

  /* Projection for SimscapeExecutionBlock: '<S69>/STATE_1' */
  simulationData = (NeslSimulationData *)simulation_DW.STATE_1_SimData;
  time = simulation_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 26;
  simulationData->mData->mContStates.mX =
    &simulation_X.simulationPlantRocket_BodyFree_[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &simulation_DW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 1;
  simulationData->mData->mModeVector.mX = &simulation_DW.STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(simulation_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&simulation_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
    (&simulation_M->solverInfo);
  tmp_1[0] = 0;
  tmp_0[0] = simulation_B.INPUT_4_1_1[0];
  tmp_0[1] = simulation_B.INPUT_4_1_1[1];
  tmp_0[2] = simulation_B.INPUT_4_1_1[2];
  tmp_0[3] = simulation_B.INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = simulation_B.INPUT_5_1_1[0];
  tmp_0[5] = simulation_B.INPUT_5_1_1[1];
  tmp_0[6] = simulation_B.INPUT_5_1_1[2];
  tmp_0[7] = simulation_B.INPUT_5_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = simulation_B.INPUT_1_1_1[0];
  tmp_0[9] = simulation_B.INPUT_1_1_1[1];
  tmp_0[10] = simulation_B.INPUT_1_1_1[2];
  tmp_0[11] = simulation_B.INPUT_1_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = simulation_B.INPUT_2_1_1[0];
  tmp_0[13] = simulation_B.INPUT_2_1_1[1];
  tmp_0[14] = simulation_B.INPUT_2_1_1[2];
  tmp_0[15] = simulation_B.INPUT_2_1_1[3];
  tmp_1[4] = 16;
  tmp_0[16] = simulation_B.INPUT_3_1_1[0];
  tmp_0[17] = simulation_B.INPUT_3_1_1[1];
  tmp_0[18] = simulation_B.INPUT_3_1_1[2];
  tmp_0[19] = simulation_B.INPUT_3_1_1[3];
  tmp_1[5] = 20;
  simulationData->mData->mInputValues.mN = 20;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 6;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  diagnosticManager = (NeuDiagnosticManager *)simulation_DW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)simulation_DW.STATE_1_Simulator,
    NESL_SIM_PROJECTION, simulationData, diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(simulation_M, msg);
    }
  }

  /* End of Projection for SimscapeExecutionBlock: '<S69>/STATE_1' */
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 30;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  simulation_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  simulation_step();
  simulation_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  simulation_step();
  simulation_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  simulation_step();
  simulation_projection();
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static boolean_T simulation_isequal_o(const real_T varargin_1[3], const real_T
  varargin_2[3])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 3)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

static void imuSensor_set_MagneticFieldNED(fusion_simulink_imuSensor_sim_T *obj,
  const real_T val[3])
{
  /* Start for MATLABSystem: '<S17>/IMU' */
  if (obj->isInitialized == 1) {
    obj->TunablePropsChanged = true;
    obj->tunablePropertyChanged[2] = true;
  }

  obj->MagneticField[0] = val[0];
  obj->MagneticField[1] = val[1];
  obj->MagneticField[2] = val[2];

  /* End of Start for MATLABSystem: '<S17>/IMU' */
}

static boolean_T simulation_isequal_oi(const real_T varargin_1[9], const real_T
  varargin_2[9])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 9)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

static boolean_T simulation_isequal(const real_T varargin_1[2], const real_T
  varargin_2[2])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

static void IMUSensorParameters_updateSyste(real_T obj_MeasurementRange, real_T
  obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], g_fusion_internal_Acceleromet_T
  *sobj)
{
  int32_T i;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[0] = true;
  }

  sobj->MeasurementRange = obj_MeasurementRange;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[1] = true;
  }

  sobj->Resolution = obj_Resolution;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[2] = true;
  }

  sobj->ConstantBias[0] = obj_ConstantBias[0];
  sobj->ConstantBias[1] = obj_ConstantBias[1];
  sobj->ConstantBias[2] = obj_ConstantBias[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[3] = true;
  }

  memcpy(&sobj->AxesMisalignment[0], &obj_AxesMisalignment[0], 9U * sizeof
         (real_T));

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[4] = true;
  }

  sobj->NoiseDensity[0] = obj_NoiseDensity[0];
  sobj->NoiseDensity[1] = obj_NoiseDensity[1];
  sobj->NoiseDensity[2] = obj_NoiseDensity[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[5] = true;
  }

  sobj->BiasInstability[0] = obj_BiasInstability[0];
  sobj->BiasInstability[1] = obj_BiasInstability[1];
  sobj->BiasInstability[2] = obj_BiasInstability[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[6] = true;
  }

  sobj->RandomWalk[0] = obj_RandomWalk[0];
  sobj->RandomWalk[1] = obj_RandomWalk[1];
  sobj->RandomWalk[2] = obj_RandomWalk[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[7] = true;
  }

  sobj->BiasInstabilityCoefficients.Numerator = obj_BiasInstabilityCoefficients;
  sobj->BiasInstabilityCoefficients.Denominator[0] =
    obj_BiasInstabilityCoefficien_0[0];
  sobj->BiasInstabilityCoefficients.Denominator[1] =
    obj_BiasInstabilityCoefficien_0[1];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[8] = true;
  }

  for (i = 0; i < 12; i++) {
    sobj->NoiseType.Value[i] = obj_NoiseType_Value[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[9] = true;
  }

  sobj->TemperatureBias[0] = obj_TemperatureBias[0];
  sobj->TemperatureBias[1] = obj_TemperatureBias[1];
  sobj->TemperatureBias[2] = obj_TemperatureBias[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[10] = true;
  }

  sobj->TemperatureScaleFactor[0] = obj_TemperatureScaleFactor[0];
  sobj->TemperatureScaleFactor[1] = obj_TemperatureScaleFactor[1];
  sobj->TemperatureScaleFactor[2] = obj_TemperatureScaleFactor[2];
}

static void IMUSensorParameters_updateSys_o(real_T obj_MeasurementRange, real_T
  obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], const real_T obj_AccelerationBias
  [3], h_fusion_internal_GyroscopeSi_T *sobj)
{
  int32_T i;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[0] = true;
  }

  sobj->AccelerationBias[0] = obj_AccelerationBias[0];
  sobj->AccelerationBias[1] = obj_AccelerationBias[1];
  sobj->AccelerationBias[2] = obj_AccelerationBias[2];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[1] = true;
  }

  sobj->MeasurementRange = obj_MeasurementRange;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[2] = true;
  }

  sobj->Resolution = obj_Resolution;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[3] = true;
  }

  sobj->ConstantBias[0] = obj_ConstantBias[0];
  sobj->ConstantBias[1] = obj_ConstantBias[1];
  sobj->ConstantBias[2] = obj_ConstantBias[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[4] = true;
  }

  memcpy(&sobj->AxesMisalignment[0], &obj_AxesMisalignment[0], 9U * sizeof
         (real_T));

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[5] = true;
  }

  sobj->NoiseDensity[0] = obj_NoiseDensity[0];
  sobj->NoiseDensity[1] = obj_NoiseDensity[1];
  sobj->NoiseDensity[2] = obj_NoiseDensity[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[6] = true;
  }

  sobj->BiasInstability[0] = obj_BiasInstability[0];
  sobj->BiasInstability[1] = obj_BiasInstability[1];
  sobj->BiasInstability[2] = obj_BiasInstability[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[7] = true;
  }

  sobj->RandomWalk[0] = obj_RandomWalk[0];
  sobj->RandomWalk[1] = obj_RandomWalk[1];
  sobj->RandomWalk[2] = obj_RandomWalk[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[8] = true;
  }

  sobj->BiasInstabilityCoefficients.Numerator = obj_BiasInstabilityCoefficients;
  sobj->BiasInstabilityCoefficients.Denominator[0] =
    obj_BiasInstabilityCoefficien_0[0];
  sobj->BiasInstabilityCoefficients.Denominator[1] =
    obj_BiasInstabilityCoefficien_0[1];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[9] = true;
  }

  for (i = 0; i < 12; i++) {
    sobj->NoiseType.Value[i] = obj_NoiseType_Value[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[10] = true;
  }

  sobj->TemperatureBias[0] = obj_TemperatureBias[0];
  sobj->TemperatureBias[1] = obj_TemperatureBias[1];
  sobj->TemperatureBias[2] = obj_TemperatureBias[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[11] = true;
  }

  sobj->TemperatureScaleFactor[0] = obj_TemperatureScaleFactor[0];
  sobj->TemperatureScaleFactor[1] = obj_TemperatureScaleFactor[1];
  sobj->TemperatureScaleFactor[2] = obj_TemperatureScaleFactor[2];
}

static void IMUSensorParameters_updateSy_oi(real_T obj_MeasurementRange, real_T
  obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], i_fusion_internal_Magnetomete_T
  *sobj)
{
  int32_T i;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[0] = true;
  }

  sobj->MeasurementRange = obj_MeasurementRange;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[1] = true;
  }

  sobj->Resolution = obj_Resolution;

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[2] = true;
  }

  sobj->ConstantBias[0] = obj_ConstantBias[0];
  sobj->ConstantBias[1] = obj_ConstantBias[1];
  sobj->ConstantBias[2] = obj_ConstantBias[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[3] = true;
  }

  memcpy(&sobj->AxesMisalignment[0], &obj_AxesMisalignment[0], 9U * sizeof
         (real_T));

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[4] = true;
  }

  sobj->NoiseDensity[0] = obj_NoiseDensity[0];
  sobj->NoiseDensity[1] = obj_NoiseDensity[1];
  sobj->NoiseDensity[2] = obj_NoiseDensity[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[5] = true;
  }

  sobj->BiasInstability[0] = obj_BiasInstability[0];
  sobj->BiasInstability[1] = obj_BiasInstability[1];
  sobj->BiasInstability[2] = obj_BiasInstability[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[6] = true;
  }

  sobj->RandomWalk[0] = obj_RandomWalk[0];
  sobj->RandomWalk[1] = obj_RandomWalk[1];
  sobj->RandomWalk[2] = obj_RandomWalk[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[7] = true;
  }

  sobj->BiasInstabilityCoefficients.Numerator = obj_BiasInstabilityCoefficients;
  sobj->BiasInstabilityCoefficients.Denominator[0] =
    obj_BiasInstabilityCoefficien_0[0];
  sobj->BiasInstabilityCoefficients.Denominator[1] =
    obj_BiasInstabilityCoefficien_0[1];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[8] = true;
  }

  for (i = 0; i < 12; i++) {
    sobj->NoiseType.Value[i] = obj_NoiseType_Value[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[9] = true;
  }

  sobj->TemperatureBias[0] = obj_TemperatureBias[0];
  sobj->TemperatureBias[1] = obj_TemperatureBias[1];
  sobj->TemperatureBias[2] = obj_TemperatureBias[2];

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[10] = true;
  }

  sobj->TemperatureScaleFactor[0] = obj_TemperatureScaleFactor[0];
  sobj->TemperatureScaleFactor[1] = obj_TemperatureScaleFactor[1];
  sobj->TemperatureScaleFactor[2] = obj_TemperatureScaleFactor[2];
}

static boolean_T simulation_vectorAny(const boolean_T x_data[], const int32_T
  x_size[2])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k <= x_size[1] - 1)) {
    if (x_data[b_k]) {
      y = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  return y;
}

static void simulat_genrand_uint32_vector_o(uint32_T mt[625], uint32_T u[2])
{
  int32_T b_j;
  int32_T b_kk;
  for (b_j = 0; b_j < 2; b_j++) {
    uint32_T mti;
    uint32_T y;
    mti = mt[624] + 1U;
    if (mt[624] + 1U >= 625U) {
      for (b_kk = 0; b_kk < 227; b_kk++) {
        /* Start for MATLABSystem: '<S17>/IMU' */
        y = (mt[b_kk + 1] & 2147483647U) | (mt[b_kk] & 2147483648U);
        if ((y & 1U) == 0U) {
          mti = y >> 1U;
        } else {
          mti = y >> 1U ^ 2567483615U;
        }

        mt[b_kk] = mt[b_kk + 397] ^ mti;
      }

      for (b_kk = 0; b_kk < 396; b_kk++) {
        /* Start for MATLABSystem: '<S17>/IMU' */
        y = (mt[b_kk + 227] & 2147483648U) | (mt[b_kk + 228] & 2147483647U);
        if ((y & 1U) == 0U) {
          mti = y >> 1U;
        } else {
          mti = y >> 1U ^ 2567483615U;
        }

        mt[b_kk + 227] = mt[b_kk] ^ mti;
      }

      y = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);

      /* Start for MATLABSystem: '<S17>/IMU' */
      if ((y & 1U) == 0U) {
        mti = y >> 1U;
      } else {
        mti = y >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ mti;
      mti = 1U;
    }

    y = mt[(int32_T)mti - 1];
    mt[624] = mti;
    y ^= y >> 11U;
    y ^= y << 7U & 2636928640U;
    y ^= y << 15U & 4022730752U;

    /* Start for MATLABSystem: '<S17>/IMU' */
    u[b_j] = y >> 18U ^ y;
  }
}

static real_T simulation_genrandu_o(uint32_T mt[625])
{
  real_T r;
  int32_T exitg1;
  int32_T k;
  uint32_T b_u[2];
  uint32_T r_0;
  boolean_T b_isvalid;
  boolean_T exitg2;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    simulat_genrand_uint32_vector_o(mt, b_u);
    r = ((real_T)(b_u[0] >> 5U) * 6.7108864E+7 + (real_T)(b_u[1] >> 6U)) *
      1.1102230246251565E-16;
    if (r == 0.0) {
      if ((mt[624] >= 1U) && (mt[624] < 625U)) {
        b_isvalid = true;
      } else {
        b_isvalid = false;
      }

      if (b_isvalid) {
        b_isvalid = false;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k + 1 < 625)) {
          if (mt[k] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        r_0 = 67U;
        mt[0] = 67U;
        for (k = 0; k < 623; k++) {
          r_0 = ((r_0 >> 30U ^ r_0) * 1812433253U + (uint32_T)k) + 1U;
          mt[k + 1] = r_0;
        }

        mt[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static void simulation_filter(real_T b, real_T a[2], const real_T x[3], const
  real_T zi[3], real_T y[3], real_T zf[3])
{
  /* Start for MATLABSystem: '<S17>/IMU' */
  if ((!rtIsInf(a[0])) && (!rtIsNaN(a[0])) && (!(a[0] == 0.0)) && (a[0] != 1.0))
  {
    b /= a[0];
    a[1] /= a[0];
  }

  y[0] = x[0] * b + zi[0];
  zf[0] = -y[0] * a[1];
  y[1] = x[1] * b + zi[1];
  zf[1] = -y[1] * a[1];
  y[2] = x[2] * b + zi[2];
  zf[2] = -y[2] * a[1];

  /* End of Start for MATLABSystem: '<S17>/IMU' */
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

static void simulation_SystemCore_step_o(g_fusion_internal_Acceleromet_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[9], const real_T
  varargin_3[9], real_T varargout_1[3])
{
  real_T B[3];
  real_T c[3];
  real_T y[3];
  real_T b_x;
  real_T b_x_0;
  real_T b_x_1;
  int32_T ret;
  int8_T b_data[3];
  static const char_T b[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i', 'd',
    'e', 'd' };

  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T obj_0[3];
  real_T obj_1[2];
  real_T x_idx_5;
  int32_T tmp_size_idx_1;
  int32_T trueCount;
  int8_T tmp_data[3];
  if (obj->isInitialized != 1) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->isInitialized = 1;
    for (ret = 0; ret <= 6; ret += 2) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      tmp_2 = _mm_loadu_pd(&obj->AxesMisalignment[ret]);
      _mm_storeu_pd(&obj->pGain[ret], _mm_div_pd(tmp_2, _mm_set1_pd(100.0)));
    }

    for (ret = 8; ret < 9; ret++) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj->pGain[ret] = obj->AxesMisalignment[ret] / 100.0;
    }

    /* Start for MATLABSystem: '<S17>/IMU' */
    ret = memcmp(&obj->NoiseType.Value[0], &b[0], 12);
    if (ret == 0) {
      obj->pBandwidth = 500.0;
    } else {
      obj->pBandwidth = 1000.0;
    }

    obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
    obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator[0];
    obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator[1];
    obj->pCorrelationTime = 0.002;
    b_x = sqrt(2.0 / (1000.0 * obj->pCorrelationTime));
    b_x_0 = sqrt(obj->pBandwidth);
    b_x_1 = sqrt(obj->pBandwidth);
    obj->TunablePropsChanged = false;
    obj->pStdDevBiasInst[0] = b_x * obj->BiasInstability[0];
    obj->pStdDevWhiteNoise[0] = b_x_0 * obj->NoiseDensity[0];
    obj->pStdDevRandWalk[0] = obj->RandomWalk[0] / b_x_1;
    obj->pBiasInstFilterStates[0] = 0.0;
    obj->pRandWalkFilterStates[0] = 0.0;
    obj->pStdDevBiasInst[1] = b_x * obj->BiasInstability[1];
    obj->pStdDevWhiteNoise[1] = b_x_0 * obj->NoiseDensity[1];
    obj->pStdDevRandWalk[1] = obj->RandomWalk[1] / b_x_1;
    obj->pBiasInstFilterStates[1] = 0.0;
    obj->pRandWalkFilterStates[1] = 0.0;
    obj->pStdDevBiasInst[2] = b_x * obj->BiasInstability[2];
    obj->pStdDevWhiteNoise[2] = b_x_0 * obj->NoiseDensity[2];
    obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / b_x_1;
    obj->pBiasInstFilterStates[2] = 0.0;
    obj->pRandWalkFilterStates[2] = 0.0;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    if (obj->tunablePropertyChanged[3]) {
      for (ret = 0; ret <= 6; ret += 2) {
        tmp_2 = _mm_loadu_pd(&obj->AxesMisalignment[ret]);
        _mm_storeu_pd(&obj->pGain[ret], _mm_div_pd(tmp_2, _mm_set1_pd(100.0)));
      }

      for (ret = 8; ret < 9; ret++) {
        obj->pGain[ret] = obj->AxesMisalignment[ret] / 100.0;
      }
    }

    if (obj->tunablePropertyChanged[4] || obj->tunablePropertyChanged[8]) {
      ret = memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (ret == 0) {
        obj->pBandwidth = 500.0;
      } else {
        obj->pBandwidth = 1000.0;
      }

      b_x = sqrt(obj->pBandwidth);
      obj->pStdDevWhiteNoise[0] = b_x * obj->NoiseDensity[0];
      obj->pStdDevWhiteNoise[1] = b_x * obj->NoiseDensity[1];
      obj->pStdDevWhiteNoise[2] = b_x * obj->NoiseDensity[2];
    }

    if (obj->tunablePropertyChanged[5]) {
      b_x = sqrt(2.0 / (1000.0 * obj->pCorrelationTime));
      obj->pStdDevBiasInst[0] = b_x * obj->BiasInstability[0];
      obj->pStdDevBiasInst[1] = b_x * obj->BiasInstability[1];
      obj->pStdDevBiasInst[2] = b_x * obj->BiasInstability[2];
    }

    if (obj->tunablePropertyChanged[6] || obj->tunablePropertyChanged[8]) {
      ret = memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (ret == 0) {
        obj->pBandwidth = 500.0;
      } else {
        obj->pBandwidth = 1000.0;
      }

      b_x = sqrt(obj->pBandwidth);
      obj->pStdDevRandWalk[0] = obj->RandomWalk[0] / b_x;
      obj->pStdDevRandWalk[1] = obj->RandomWalk[1] / b_x;
      obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / b_x;
    }

    if (obj->tunablePropertyChanged[7]) {
      obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
      obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator
        [0];
      obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator
        [1];
    }

    for (ret = 0; ret < 12; ret++) {
      obj->tunablePropertyChanged[ret] = false;
    }
  }

  b_x = -varargin_1[0];
  b_x_0 = -varargin_1[1];
  b_x_1 = -varargin_1[2] + 9.81;
  for (ret = 0; ret <= 0; ret += 2) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    tmp_2 = _mm_loadu_pd(&varargin_2[ret + 3]);
    tmp_0 = _mm_loadu_pd(&varargin_2[ret]);
    tmp_1 = _mm_loadu_pd(&varargin_2[ret + 6]);
    _mm_storeu_pd(&B[ret], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd
      (b_x_0)), _mm_mul_pd(tmp_0, _mm_set1_pd(b_x))), _mm_mul_pd(tmp_1,
      _mm_set1_pd(b_x_1))));
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  for (ret = 2; ret < 3; ret++) {
    B[ret] = (varargin_2[ret + 3] * b_x_0 + varargin_2[ret] * b_x) +
      varargin_2[ret + 6] * b_x_1;
  }

  b_x = B[1];
  b_x_0 = B[0];
  b_x_1 = B[2];
  for (ret = 0; ret <= 0; ret += 2) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    tmp_2 = _mm_loadu_pd(&obj->pGain[ret + 3]);
    tmp_0 = _mm_loadu_pd(&obj->pGain[ret]);
    tmp_1 = _mm_loadu_pd(&obj->pGain[ret + 6]);
    tmp = _mm_loadu_pd(&obj->ConstantBias[ret]);
    _mm_storeu_pd(&B[ret], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_2,
      _mm_set1_pd(b_x)), _mm_mul_pd(tmp_0, _mm_set1_pd(b_x_0))), _mm_mul_pd
      (tmp_1, _mm_set1_pd(b_x_1))), tmp));
    tmp_2 = _mm_loadu_pd(&varargin_3[ret]);
    tmp_0 = _mm_loadu_pd(&obj->pStdDevBiasInst[ret]);
    _mm_storeu_pd(&c[ret], _mm_mul_pd(tmp_2, tmp_0));
  }

  for (ret = 2; ret < 3; ret++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    B[ret] = ((obj->pGain[ret + 3] * b_x + obj->pGain[ret] * b_x_0) + obj->
              pGain[ret + 6] * b_x_1) + obj->ConstantBias[ret];
    c[ret] = varargin_3[ret] * obj->pStdDevBiasInst[ret];
  }

  for (ret = 0; ret < 3; ret++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj_0[ret] = obj->pBiasInstFilterStates[ret];
  }

  for (ret = 0; ret < 2; ret++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj_1[ret] = obj->pBiasInstFilterDen[ret];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  simulation_filter(obj->pBiasInstFilterNum, obj_1, c, obj_0, y,
                    obj->pBiasInstFilterStates);
  b_x_0 = obj->pStdDevRandWalk[0] * varargin_3[6] + obj->pRandWalkFilterStates[0];
  b_x_1 = obj->pStdDevRandWalk[1] * varargin_3[7] + obj->pRandWalkFilterStates[1];
  x_idx_5 = obj->pStdDevRandWalk[2] * varargin_3[8] + obj->
    pRandWalkFilterStates[2];
  obj->pRandWalkFilterStates[0] = b_x_0;
  obj->pRandWalkFilterStates[1] = b_x_1;
  obj->pRandWalkFilterStates[2] = x_idx_5;
  b_x = (obj->Temperature - 25.0) * 0.01;
  varargout_1[0] = ((((obj->pStdDevWhiteNoise[0] * varargin_3[3] + y[0]) + b_x_0)
                     + (obj->Temperature - 25.0) * obj->TemperatureBias[0]) + B
                    [0]) * (b_x * obj->TemperatureScaleFactor[0] + 1.0);
  varargout_1[1] = ((((obj->pStdDevWhiteNoise[1] * varargin_3[4] + y[1]) + b_x_1)
                     + (obj->Temperature - 25.0) * obj->TemperatureBias[1]) + B
                    [1]) * (b_x * obj->TemperatureScaleFactor[1] + 1.0);
  varargout_1[2] = ((((obj->pStdDevWhiteNoise[2] * varargin_3[5] + y[2]) +
                      x_idx_5) + (obj->Temperature - 25.0) *
                     obj->TemperatureBias[2]) + B[2]) * (b_x *
    obj->TemperatureScaleFactor[2] + 1.0);
  if (!rtIsInf(obj->MeasurementRange)) {
    b_x = fabs(varargout_1[0]);
    y[0] = b_x;
    b_x_0 = fabs(varargout_1[1]);
    y[1] = b_x_0;
    b_x_1 = fabs(varargout_1[2]);
    y[2] = b_x_1;
    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (y[ret] > obj->MeasurementRange) {
        b_data[trueCount] = (int8_T)ret;
        trueCount++;
      }
    }

    y[0] = b_x;
    y[1] = b_x_0;
    y[2] = b_x_1;
    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (y[ret] > obj->MeasurementRange) {
        trueCount++;
      }
    }

    tmp_size_idx_1 = trueCount;
    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (y[ret] > obj->MeasurementRange) {
        tmp_data[trueCount] = (int8_T)ret;
        trueCount++;
      }
    }

    for (ret = 0; ret < tmp_size_idx_1; ret++) {
      B[ret] = varargout_1[tmp_data[ret]];
    }

    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (y[ret] > obj->MeasurementRange) {
        trueCount++;
      }
    }

    trueCount--;
    for (ret = 0; ret <= trueCount; ret++) {
      b_x = B[ret];
      if (rtIsNaN(b_x)) {
        B[ret] = (rtNaN);
      } else if (b_x < 0.0) {
        B[ret] = -1.0;
      } else {
        B[ret] = (b_x > 0.0);
      }
    }

    for (ret = 0; ret < tmp_size_idx_1; ret++) {
      varargout_1[b_data[ret]] = B[ret] * obj->MeasurementRange;
    }
  }

  if (obj->Resolution != 0.0) {
    varargout_1[0] /= obj->Resolution;
    varargout_1[1] /= obj->Resolution;
    varargout_1[2] /= obj->Resolution;
    varargout_1[0] = rt_roundd_snf(varargout_1[0]);
    varargout_1[1] = rt_roundd_snf(varargout_1[1]);
    varargout_1[2] = rt_roundd_snf(varargout_1[2]);
    varargout_1[0] *= obj->Resolution;
    varargout_1[1] *= obj->Resolution;
    varargout_1[2] *= obj->Resolution;
  }
}

static void simulation_SystemCore_step_oi(h_fusion_internal_GyroscopeSi_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[3], const real_T
  varargin_3[9], const real_T varargin_4[9], real_T varargout_1[3])
{
  real_T B[3];
  real_T c[3];
  real_T b_x;
  real_T b_x_0;
  real_T b_x_1;
  int32_T ret;
  int8_T b_data[3];
  static const char_T b[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i', 'd',
    'e', 'd' };

  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T obj_0[3];
  real_T varargin_3_0[3];
  real_T obj_1[2];
  real_T x_idx_5;
  int32_T tmp_size_idx_1;
  int32_T trueCount;
  int8_T tmp_data[3];
  if (obj->isInitialized != 1) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->isInitialized = 1;
    for (ret = 0; ret <= 6; ret += 2) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      tmp_2 = _mm_loadu_pd(&obj->AxesMisalignment[ret]);
      _mm_storeu_pd(&obj->pGain[ret], _mm_div_pd(tmp_2, _mm_set1_pd(100.0)));
    }

    for (ret = 8; ret < 9; ret++) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj->pGain[ret] = obj->AxesMisalignment[ret] / 100.0;
    }

    /* Start for MATLABSystem: '<S17>/IMU' */
    ret = memcmp(&obj->NoiseType.Value[0], &b[0], 12);
    if (ret == 0) {
      obj->pBandwidth = 500.0;
    } else {
      obj->pBandwidth = 1000.0;
    }

    obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
    obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator[0];
    obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator[1];
    obj->pCorrelationTime = 0.002;
    b_x = sqrt(2.0 / (1000.0 * obj->pCorrelationTime));
    b_x_0 = sqrt(obj->pBandwidth);
    b_x_1 = sqrt(obj->pBandwidth);
    obj->TunablePropsChanged = false;
    obj->pStdDevBiasInst[0] = b_x * obj->BiasInstability[0];
    obj->pStdDevWhiteNoise[0] = b_x_0 * obj->NoiseDensity[0];
    obj->pStdDevRandWalk[0] = obj->RandomWalk[0] / b_x_1;
    obj->pBiasInstFilterStates[0] = 0.0;
    obj->pRandWalkFilterStates[0] = 0.0;
    obj->pStdDevBiasInst[1] = b_x * obj->BiasInstability[1];
    obj->pStdDevWhiteNoise[1] = b_x_0 * obj->NoiseDensity[1];
    obj->pStdDevRandWalk[1] = obj->RandomWalk[1] / b_x_1;
    obj->pBiasInstFilterStates[1] = 0.0;
    obj->pRandWalkFilterStates[1] = 0.0;
    obj->pStdDevBiasInst[2] = b_x * obj->BiasInstability[2];
    obj->pStdDevWhiteNoise[2] = b_x_0 * obj->NoiseDensity[2];
    obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / b_x_1;
    obj->pBiasInstFilterStates[2] = 0.0;
    obj->pRandWalkFilterStates[2] = 0.0;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    if (obj->tunablePropertyChanged[4]) {
      for (ret = 0; ret <= 6; ret += 2) {
        tmp_2 = _mm_loadu_pd(&obj->AxesMisalignment[ret]);
        _mm_storeu_pd(&obj->pGain[ret], _mm_div_pd(tmp_2, _mm_set1_pd(100.0)));
      }

      for (ret = 8; ret < 9; ret++) {
        obj->pGain[ret] = obj->AxesMisalignment[ret] / 100.0;
      }
    }

    if (obj->tunablePropertyChanged[5] || obj->tunablePropertyChanged[9]) {
      ret = memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (ret == 0) {
        obj->pBandwidth = 500.0;
      } else {
        obj->pBandwidth = 1000.0;
      }

      b_x = sqrt(obj->pBandwidth);
      obj->pStdDevWhiteNoise[0] = b_x * obj->NoiseDensity[0];
      obj->pStdDevWhiteNoise[1] = b_x * obj->NoiseDensity[1];
      obj->pStdDevWhiteNoise[2] = b_x * obj->NoiseDensity[2];
    }

    if (obj->tunablePropertyChanged[6]) {
      b_x = sqrt(2.0 / (1000.0 * obj->pCorrelationTime));
      obj->pStdDevBiasInst[0] = b_x * obj->BiasInstability[0];
      obj->pStdDevBiasInst[1] = b_x * obj->BiasInstability[1];
      obj->pStdDevBiasInst[2] = b_x * obj->BiasInstability[2];
    }

    if (obj->tunablePropertyChanged[7] || obj->tunablePropertyChanged[9]) {
      ret = memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (ret == 0) {
        obj->pBandwidth = 500.0;
      } else {
        obj->pBandwidth = 1000.0;
      }

      b_x = sqrt(obj->pBandwidth);
      obj->pStdDevRandWalk[0] = obj->RandomWalk[0] / b_x;
      obj->pStdDevRandWalk[1] = obj->RandomWalk[1] / b_x;
      obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / b_x;
    }

    if (obj->tunablePropertyChanged[8]) {
      obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
      obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator
        [0];
      obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator
        [1];
    }

    for (ret = 0; ret < 13; ret++) {
      obj->tunablePropertyChanged[ret] = false;
    }
  }

  b_x = varargin_1[1];
  b_x_0 = varargin_1[0];
  b_x_1 = varargin_1[2];
  for (ret = 0; ret <= 0; ret += 2) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    tmp_2 = _mm_loadu_pd(&varargin_2[ret]);
    _mm_storeu_pd(&obj->pAcceleration[ret], tmp_2);
    tmp_2 = _mm_loadu_pd(&varargin_3[ret + 3]);
    tmp_0 = _mm_loadu_pd(&varargin_3[ret]);
    tmp_1 = _mm_loadu_pd(&varargin_3[ret + 6]);
    _mm_storeu_pd(&varargin_3_0[ret], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_2,
      _mm_set1_pd(b_x)), _mm_mul_pd(tmp_0, _mm_set1_pd(b_x_0))), _mm_mul_pd
      (tmp_1, _mm_set1_pd(b_x_1))));
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  for (ret = 2; ret < 3; ret++) {
    obj->pAcceleration[ret] = varargin_2[ret];
    varargin_3_0[ret] = (varargin_3[ret + 3] * b_x + varargin_3[ret] * b_x_0) +
      varargin_3[ret + 6] * b_x_1;
  }

  b_x = varargin_3_0[1];
  b_x_0 = varargin_3_0[0];
  b_x_1 = varargin_3_0[2];
  for (ret = 0; ret <= 0; ret += 2) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    tmp_2 = _mm_loadu_pd(&obj->pGain[ret + 3]);
    tmp_0 = _mm_loadu_pd(&obj->pGain[ret]);
    tmp_1 = _mm_loadu_pd(&obj->pGain[ret + 6]);
    tmp = _mm_loadu_pd(&obj->ConstantBias[ret]);
    _mm_storeu_pd(&B[ret], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_2,
      _mm_set1_pd(b_x)), _mm_mul_pd(tmp_0, _mm_set1_pd(b_x_0))), _mm_mul_pd
      (tmp_1, _mm_set1_pd(b_x_1))), tmp));
    tmp_2 = _mm_loadu_pd(&varargin_4[ret]);
    tmp_0 = _mm_loadu_pd(&obj->pStdDevBiasInst[ret]);
    _mm_storeu_pd(&c[ret], _mm_mul_pd(tmp_2, tmp_0));
  }

  for (ret = 2; ret < 3; ret++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    B[ret] = ((obj->pGain[ret + 3] * b_x + obj->pGain[ret] * b_x_0) + obj->
              pGain[ret + 6] * b_x_1) + obj->ConstantBias[ret];
    c[ret] = varargin_4[ret] * obj->pStdDevBiasInst[ret];
  }

  for (ret = 0; ret < 3; ret++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj_0[ret] = obj->pBiasInstFilterStates[ret];
  }

  for (ret = 0; ret < 2; ret++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj_1[ret] = obj->pBiasInstFilterDen[ret];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  simulation_filter(obj->pBiasInstFilterNum, obj_1, c, obj_0, varargin_3_0,
                    obj->pBiasInstFilterStates);
  b_x_0 = obj->pStdDevRandWalk[0] * varargin_4[6] + obj->pRandWalkFilterStates[0];
  b_x_1 = obj->pStdDevRandWalk[1] * varargin_4[7] + obj->pRandWalkFilterStates[1];
  x_idx_5 = obj->pStdDevRandWalk[2] * varargin_4[8] + obj->
    pRandWalkFilterStates[2];
  obj->pRandWalkFilterStates[0] = b_x_0;
  obj->pRandWalkFilterStates[1] = b_x_1;
  obj->pRandWalkFilterStates[2] = x_idx_5;
  b_x = (obj->Temperature - 25.0) * 0.01;
  varargout_1[0] = ((((obj->pStdDevWhiteNoise[0] * varargin_4[3] + varargin_3_0
                       [0]) + b_x_0) + ((obj->Temperature - 25.0) *
    obj->TemperatureBias[0] + obj->pAcceleration[0] * obj->AccelerationBias[0]))
                    + B[0]) * (b_x * obj->TemperatureScaleFactor[0] + 1.0);
  varargout_1[1] = ((((obj->pStdDevWhiteNoise[1] * varargin_4[4] + varargin_3_0
                       [1]) + b_x_1) + ((obj->Temperature - 25.0) *
    obj->TemperatureBias[1] + obj->pAcceleration[1] * obj->AccelerationBias[1]))
                    + B[1]) * (b_x * obj->TemperatureScaleFactor[1] + 1.0);
  varargout_1[2] = ((((obj->pStdDevWhiteNoise[2] * varargin_4[5] + varargin_3_0
                       [2]) + x_idx_5) + ((obj->Temperature - 25.0) *
    obj->TemperatureBias[2] + obj->pAcceleration[2] * obj->AccelerationBias[2]))
                    + B[2]) * (b_x * obj->TemperatureScaleFactor[2] + 1.0);
  if (!rtIsInf(obj->MeasurementRange)) {
    b_x = fabs(varargout_1[0]);
    varargin_3_0[0] = b_x;
    b_x_0 = fabs(varargout_1[1]);
    varargin_3_0[1] = b_x_0;
    b_x_1 = fabs(varargout_1[2]);
    varargin_3_0[2] = b_x_1;
    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (varargin_3_0[ret] > obj->MeasurementRange) {
        b_data[trueCount] = (int8_T)ret;
        trueCount++;
      }
    }

    varargin_3_0[0] = b_x;
    varargin_3_0[1] = b_x_0;
    varargin_3_0[2] = b_x_1;
    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (varargin_3_0[ret] > obj->MeasurementRange) {
        trueCount++;
      }
    }

    tmp_size_idx_1 = trueCount;
    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (varargin_3_0[ret] > obj->MeasurementRange) {
        tmp_data[trueCount] = (int8_T)ret;
        trueCount++;
      }
    }

    for (ret = 0; ret < tmp_size_idx_1; ret++) {
      B[ret] = varargout_1[tmp_data[ret]];
    }

    trueCount = 0;
    for (ret = 0; ret < 3; ret++) {
      if (varargin_3_0[ret] > obj->MeasurementRange) {
        trueCount++;
      }
    }

    trueCount--;
    for (ret = 0; ret <= trueCount; ret++) {
      b_x = B[ret];
      if (rtIsNaN(b_x)) {
        B[ret] = (rtNaN);
      } else if (b_x < 0.0) {
        B[ret] = -1.0;
      } else {
        B[ret] = (b_x > 0.0);
      }
    }

    for (ret = 0; ret < tmp_size_idx_1; ret++) {
      varargout_1[b_data[ret]] = B[ret] * obj->MeasurementRange;
    }
  }

  if (obj->Resolution != 0.0) {
    varargout_1[0] /= obj->Resolution;
    varargout_1[1] /= obj->Resolution;
    varargout_1[2] /= obj->Resolution;
    varargout_1[0] = rt_roundd_snf(varargout_1[0]);
    varargout_1[1] = rt_roundd_snf(varargout_1[1]);
    varargout_1[2] = rt_roundd_snf(varargout_1[2]);
    varargout_1[0] *= obj->Resolution;
    varargout_1[1] *= obj->Resolution;
    varargout_1[2] *= obj->Resolution;
  }
}

static void simulation_imuSensor_stepImpl(fusion_simulink_imuSensor_sim_T *obj,
  const real_T la[3], const real_T av[3], const real_T o[4], real_T a[3], real_T
  g[3], real_T m[3])
{
  i_fusion_internal_Magnetomete_T *obj_0;
  real_T allRandData[27];
  real_T a_1[9];
  real_T rmat[9];
  real_T a_0[3];
  real_T magneticfield[3];
  real_T temperatureDrift[3];
  real_T y[3];
  real_T aasq;
  real_T ab2;
  real_T ac2;
  real_T ad2;
  real_T bc2;
  real_T bd2;
  real_T cd2;
  real_T maximum;
  real_T n;
  real_T x;
  int32_T b_colIdx;
  uint32_T c_mt[625];
  uint32_T u32[2];
  char_T obj1_Value[12];
  int8_T b_data[3];
  boolean_T flag;
  static const char_T b[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i', 'd',
    'e', 'd' };

  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  int32_T i;
  int8_T tmp_data[3];
  static const real_T tmp_2[257] = { 0.0, 0.215241895984875, 0.286174591792068,
    0.335737519214422, 0.375121332878378, 0.408389134611989, 0.43751840220787,
    0.46363433679088, 0.487443966139235, 0.50942332960209, 0.529909720661557,
    0.549151702327164, 0.567338257053817, 0.584616766106378, 0.601104617755991,
    0.61689699000775, 0.63207223638606, 0.646695714894993, 0.660822574244419,
    0.674499822837293, 0.687767892795788, 0.700661841106814, 0.713212285190975,
    0.725446140909999, 0.737387211434295, 0.749056662017815, 0.760473406430107,
    0.771654424224568, 0.782615023307232, 0.793369058840623, 0.80392911698997,
    0.814306670135215, 0.824512208752291, 0.834555354086381, 0.844444954909153,
    0.854189171008163, 0.863795545553308, 0.87327106808886, 0.882622229585165,
    0.891855070732941, 0.900975224461221, 0.909987953496718, 0.91889818364959,
    0.927710533401999, 0.936429340286575, 0.945058684468165, 0.953602409881086,
    0.96206414322304, 0.970447311064224, 0.978755155294224, 0.986990747099062,
    0.99515699963509, 1.00325667954467, 1.01129241744, 1.01926671746548,
    1.02718196603564, 1.03504043983344, 1.04284431314415, 1.05059566459093,
    1.05829648333067, 1.06594867476212, 1.07355406579244, 1.0811144097034,
    1.08863139065398, 1.09610662785202, 1.10354167942464, 1.11093804601357,
    1.11829717411934, 1.12562045921553, 1.13290924865253, 1.14016484436815,
    1.14738850542085, 1.15458145035993, 1.16174485944561, 1.16887987673083,
    1.17598761201545, 1.18306914268269, 1.19012551542669, 1.19715774787944,
    1.20416683014438, 1.2111537262437, 1.21811937548548, 1.22506469375653,
    1.23199057474614, 1.23889789110569, 1.24578749554863, 1.2526602218949,
    1.25951688606371, 1.26635828701823, 1.27318520766536, 1.27999841571382,
    1.28679866449324, 1.29358669373695, 1.30036323033084, 1.30712898903073,
    1.31388467315022, 1.32063097522106, 1.32736857762793, 1.33409815321936,
    1.3408203658964, 1.34753587118059, 1.35424531676263, 1.36094934303328,
    1.36764858359748, 1.37434366577317, 1.38103521107586, 1.38772383568998,
    1.39441015092814, 1.40109476367925, 1.4077782768464, 1.41446128977547,
    1.42114439867531, 1.42782819703026, 1.43451327600589, 1.44120022484872,
    1.44788963128058, 1.45458208188841, 1.46127816251028, 1.46797845861808,
    1.47468355569786, 1.48139403962819, 1.48811049705745, 1.49483351578049,
    1.50156368511546, 1.50830159628131, 1.51504784277671, 1.521803020761,
    1.52856772943771, 1.53534257144151, 1.542128153229, 1.54892508547417,
    1.55573398346918, 1.56255546753104, 1.56939016341512, 1.57623870273591,
    1.58310172339603, 1.58997987002419, 1.59687379442279, 1.60378415602609,
    1.61071162236983, 1.61765686957301, 1.62462058283303, 1.63160345693487,
    1.63860619677555, 1.64562951790478, 1.65267414708306, 1.65974082285818,
    1.66683029616166, 1.67394333092612, 1.68108070472517, 1.68824320943719,
    1.69543165193456, 1.70264685479992, 1.7098896570713, 1.71716091501782,
    1.72446150294804, 1.73179231405296, 1.73915426128591, 1.74654827828172,
    1.75397532031767, 1.76143636531891, 1.76893241491127, 1.77646449552452,
    1.78403365954944, 1.79164098655216, 1.79928758454972, 1.80697459135082,
    1.81470317596628, 1.82247454009388, 1.83028991968276, 1.83815058658281,
    1.84605785028518, 1.8540130597602, 1.86201760539967, 1.87007292107127,
    1.878180486293, 1.88634182853678, 1.8945585256707, 1.90283220855043,
    1.91116456377125, 1.91955733659319, 1.92801233405266, 1.93653142827569,
    1.94511656000868, 1.95376974238465, 1.96249306494436, 1.97128869793366,
    1.98015889690048, 1.98910600761744, 1.99813247135842, 2.00724083056053,
    2.0164337349062, 2.02571394786385, 2.03508435372962, 2.04454796521753,
    2.05410793165065, 2.06376754781173, 2.07353026351874, 2.0833996939983,
    2.09337963113879, 2.10347405571488, 2.11368715068665, 2.12402331568952,
    2.13448718284602, 2.14508363404789, 2.15581781987674, 2.16669518035431,
    2.17772146774029, 2.18890277162636, 2.20024554661128, 2.21175664288416,
    2.22344334009251, 2.23531338492992, 2.24737503294739, 2.25963709517379,
    2.27210899022838, 2.28480080272449, 2.29772334890286, 2.31088825060137,
    2.32430801887113, 2.33799614879653, 2.35196722737914, 2.36623705671729,
    2.38082279517208, 2.39574311978193, 2.41101841390112, 2.42667098493715,
    2.44272531820036, 2.4592083743347, 2.47614993967052, 2.49358304127105,
    2.51154444162669, 2.53007523215985, 2.54922155032478, 2.56903545268184,
    2.58957598670829, 2.61091051848882, 2.63311639363158, 2.65628303757674,
    2.68051464328574, 2.70593365612306, 2.73268535904401, 2.76094400527999,
    2.79092117400193, 2.82287739682644, 2.85713873087322, 2.89412105361341,
    2.93436686720889, 2.97860327988184, 3.02783779176959, 3.08352613200214,
    3.147889289518, 3.2245750520478, 3.32024473383983, 3.44927829856143,
    3.65415288536101, 3.91075795952492 };

  static const real_T tmp_3[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  real_T obj_1[2];
  int32_T exitg1;
  int32_T tmp_size_idx_1;
  boolean_T guard1;

  /* Start for MATLABSystem: '<S17>/IMU' */
  n = sqrt(((o[0] * o[0] + o[1] * o[1]) + o[2] * o[2]) + o[3] * o[3]);
  aasq = o[0] / n;
  maximum = o[1] / n;
  x = o[2] / n;
  n = o[3] / n;
  ab2 = aasq * maximum * 2.0;
  ac2 = aasq * x * 2.0;
  ad2 = aasq * n * 2.0;
  bc2 = maximum * x * 2.0;
  bd2 = maximum * n * 2.0;
  cd2 = x * n * 2.0;

  /* Start for MATLABSystem: '<S17>/IMU' */
  aasq = aasq * aasq * 2.0 - 1.0;
  rmat[0] = maximum * maximum * 2.0 + aasq;
  rmat[3] = bc2 + ad2;
  rmat[6] = bd2 - ac2;
  rmat[1] = bc2 - ad2;

  /* Start for MATLABSystem: '<S17>/IMU' */
  rmat[4] = x * x * 2.0 + aasq;
  rmat[7] = cd2 + ab2;
  rmat[2] = bd2 + ac2;
  rmat[5] = cd2 - ab2;

  /* Start for MATLABSystem: '<S17>/IMU' */
  rmat[8] = n * n * 2.0 + aasq;
  for (i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    c_mt[i] = obj->pStreamState[i];
  }

  for (b_colIdx = 0; b_colIdx < 27; b_colIdx++) {
    do {
      exitg1 = 0;
      simulat_genrand_uint32_vector_o(c_mt, u32);
      i = (int32_T)((u32[1] >> 24U) + 1U);
      maximum = (((real_T)(u32[0] >> 3U) * 1.6777216E+7 + (real_T)((int32_T)u32
        [1] & 16777215)) * 2.2204460492503131E-16 - 1.0) * tmp_2[i];
      if (fabs(maximum) <= tmp_2[i - 1]) {
        exitg1 = 1;
      } else if (i < 256) {
        x = simulation_genrandu_o(c_mt);
        if ((tmp_3[i - 1] - tmp_3[i]) * x + tmp_3[i] < exp(-0.5 * maximum *
             maximum)) {
          exitg1 = 1;
        }
      } else {
        do {
          x = simulation_genrandu_o(c_mt);
          x = log(x) * 0.273661237329758;
          aasq = simulation_genrandu_o(c_mt);
        } while (!(-2.0 * log(aasq) > x * x));

        if (maximum < 0.0) {
          maximum = x - 3.65415288536101;
        } else {
          maximum = 3.65415288536101 - x;
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S17>/IMU' */
    allRandData[b_colIdx] = maximum;
  }

  for (i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->pStreamState[i] = c_mt[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  simulation_SystemCore_step_o(obj->pAccel, la, rmat, &allRandData[0], a);
  simulation_SystemCore_step_oi(obj->pGyro, av, la, rmat, &allRandData[9], g);
  a_0[0] = obj->MagneticField[0];
  a_0[1] = obj->MagneticField[1];
  a_0[2] = obj->MagneticField[2];
  obj_0 = obj->pMag;
  if (obj_0->isInitialized != 1) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj_0->isInitialized = 1;
    for (i = 0; i < 9; i++) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj_0->pGain[i] = obj_0->AxesMisalignment[i] / 100.0;
    }

    for (i = 0; i < 12; i++) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj1_Value[i] = obj_0->NoiseType.Value[i];
    }

    /* Start for MATLABSystem: '<S17>/IMU' */
    i = memcmp(&obj1_Value[0], &b[0], 12);
    if (i == 0) {
      obj_0->pBandwidth = 500.0;
    } else {
      obj_0->pBandwidth = 1000.0;
    }

    maximum = obj_0->BiasInstabilityCoefficients.Numerator;
    obj_0->pBiasInstFilterNum = maximum;
    maximum = obj_0->BiasInstabilityCoefficients.Denominator[0];
    obj_0->pBiasInstFilterDen[0] = maximum;
    maximum = obj_0->BiasInstabilityCoefficients.Denominator[1];
    obj_0->pBiasInstFilterDen[1] = maximum;
    obj_0->pCorrelationTime = 0.002;
    x = 2.0 / (1000.0 * obj_0->pCorrelationTime);
    maximum = sqrt(x);
    x = obj_0->pBandwidth;
    aasq = sqrt(x);
    x = obj_0->pBandwidth;
    x = sqrt(x);
    obj_0->TunablePropsChanged = false;
    obj_0->pBiasInstFilterStates[0] = 0.0;
    obj_0->pStdDevBiasInst[0] = maximum * obj_0->BiasInstability[0];
    obj_0->pStdDevWhiteNoise[0] = aasq * obj_0->NoiseDensity[0];
    obj_0->pRandWalkFilterStates[0] = 0.0;
    obj_0->pStdDevRandWalk[0] = obj_0->RandomWalk[0] / x;
    obj_0->pBiasInstFilterStates[0] = 0.0;
    obj_0->pRandWalkFilterStates[0] = 0.0;
    obj_0->pBiasInstFilterStates[1] = 0.0;
    obj_0->pStdDevBiasInst[1] = maximum * obj_0->BiasInstability[1];
    obj_0->pStdDevWhiteNoise[1] = aasq * obj_0->NoiseDensity[1];
    obj_0->pRandWalkFilterStates[1] = 0.0;
    obj_0->pStdDevRandWalk[1] = obj_0->RandomWalk[1] / x;
    obj_0->pBiasInstFilterStates[1] = 0.0;
    obj_0->pRandWalkFilterStates[1] = 0.0;
    obj_0->pBiasInstFilterStates[2] = 0.0;
    obj_0->pStdDevBiasInst[2] = maximum * obj_0->BiasInstability[2];
    obj_0->pStdDevWhiteNoise[2] = aasq * obj_0->NoiseDensity[2];
    obj_0->pRandWalkFilterStates[2] = 0.0;
    obj_0->pStdDevRandWalk[2] = obj_0->RandomWalk[2] / x;
    obj_0->pBiasInstFilterStates[2] = 0.0;
    obj_0->pRandWalkFilterStates[2] = 0.0;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (obj_0->TunablePropsChanged) {
    obj_0->TunablePropsChanged = false;
    flag = obj_0->tunablePropertyChanged[3];
    if (flag) {
      for (i = 0; i < 9; i++) {
        obj_0->pGain[i] = obj_0->AxesMisalignment[i] / 100.0;
      }
    }

    flag = obj_0->tunablePropertyChanged[4];
    guard1 = false;
    if (flag) {
      guard1 = true;
    } else {
      flag = obj_0->tunablePropertyChanged[8];
      if (flag) {
        guard1 = true;
      }
    }

    if (guard1) {
      for (i = 0; i < 12; i++) {
        obj1_Value[i] = obj_0->NoiseType.Value[i];
      }

      i = memcmp(&obj1_Value[0], &b[0], 12);
      if (i == 0) {
        obj_0->pBandwidth = 500.0;
      } else {
        obj_0->pBandwidth = 1000.0;
      }

      x = obj_0->pBandwidth;
      maximum = sqrt(x);
      obj_0->pStdDevWhiteNoise[0] = maximum * obj_0->NoiseDensity[0];
      obj_0->pStdDevWhiteNoise[1] = maximum * obj_0->NoiseDensity[1];
      obj_0->pStdDevWhiteNoise[2] = maximum * obj_0->NoiseDensity[2];
    }

    flag = obj_0->tunablePropertyChanged[5];
    if (flag) {
      x = 2.0 / (1000.0 * obj_0->pCorrelationTime);
      maximum = sqrt(x);
      obj_0->pStdDevBiasInst[0] = maximum * obj_0->BiasInstability[0];
      obj_0->pStdDevBiasInst[1] = maximum * obj_0->BiasInstability[1];
      obj_0->pStdDevBiasInst[2] = maximum * obj_0->BiasInstability[2];
    }

    flag = obj_0->tunablePropertyChanged[6];
    guard1 = false;
    if (flag) {
      guard1 = true;
    } else {
      flag = obj_0->tunablePropertyChanged[8];
      if (flag) {
        guard1 = true;
      }
    }

    if (guard1) {
      for (i = 0; i < 12; i++) {
        obj1_Value[i] = obj_0->NoiseType.Value[i];
      }

      i = memcmp(&obj1_Value[0], &b[0], 12);
      if (i == 0) {
        obj_0->pBandwidth = 500.0;
      } else {
        obj_0->pBandwidth = 1000.0;
      }

      x = obj_0->pBandwidth;
      maximum = sqrt(x);
      obj_0->pStdDevRandWalk[0] = obj_0->RandomWalk[0] / maximum;
      obj_0->pStdDevRandWalk[1] = obj_0->RandomWalk[1] / maximum;
      obj_0->pStdDevRandWalk[2] = obj_0->RandomWalk[2] / maximum;
    }

    flag = obj_0->tunablePropertyChanged[7];
    if (flag) {
      maximum = obj_0->BiasInstabilityCoefficients.Numerator;
      obj_0->pBiasInstFilterNum = maximum;
      maximum = obj_0->BiasInstabilityCoefficients.Denominator[0];
      obj_0->pBiasInstFilterDen[0] = maximum;
      maximum = obj_0->BiasInstabilityCoefficients.Denominator[1];
      obj_0->pBiasInstFilterDen[1] = maximum;
    }

    for (i = 0; i < 12; i++) {
      obj_0->tunablePropertyChanged[i] = false;
    }
  }

  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    a_1[i] = obj_0->pGain[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  maximum = a_0[1];
  x = a_0[0];
  aasq = a_0[2];
  for (i = 0; i <= 0; i += 2) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    tmp = _mm_loadu_pd(&rmat[i + 3]);
    tmp_0 = _mm_loadu_pd(&rmat[i]);
    tmp_1 = _mm_loadu_pd(&rmat[i + 6]);
    _mm_storeu_pd(&magneticfield[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp,
      _mm_set1_pd(maximum)), _mm_mul_pd(tmp_0, _mm_set1_pd(x))), _mm_mul_pd
      (tmp_1, _mm_set1_pd(aasq))));
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  for (i = 2; i < 3; i++) {
    magneticfield[i] = (rmat[i + 3] * maximum + rmat[i] * x) + rmat[i + 6] *
      aasq;
  }

  x = magneticfield[1];
  aasq = magneticfield[0];
  n = magneticfield[2];
  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    maximum = obj_0->ConstantBias[i];
    magneticfield[i] = ((a_1[i + 3] * x + a_1[i] * aasq) + a_1[i + 6] * n) +
      maximum;
    maximum = obj_0->pStdDevBiasInst[i];
    temperatureDrift[i] = allRandData[i + 18] * maximum;
  }

  for (i = 0; i < 2; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj_1[i] = obj_0->pBiasInstFilterDen[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  simulation_filter(obj_0->pBiasInstFilterNum, obj_1, temperatureDrift,
                    obj_0->pBiasInstFilterStates, y, a_0);
  obj_0->pBiasInstFilterStates[0] = a_0[0];
  maximum = obj_0->pStdDevWhiteNoise[0];
  x = allRandData[21] * maximum;
  maximum = obj_0->pStdDevRandWalk[0];
  ad2 = obj_0->pRandWalkFilterStates[0];
  ab2 = allRandData[24] * maximum;
  obj_0->pBiasInstFilterStates[1] = a_0[1];
  maximum = obj_0->pStdDevWhiteNoise[1];
  aasq = allRandData[22] * maximum;
  maximum = obj_0->pStdDevRandWalk[1];
  bc2 = obj_0->pRandWalkFilterStates[1];
  ac2 = allRandData[25] * maximum;
  obj_0->pBiasInstFilterStates[2] = a_0[2];
  maximum = obj_0->pStdDevWhiteNoise[2];
  n = allRandData[23] * maximum;
  maximum = obj_0->pStdDevRandWalk[2];
  bd2 = obj_0->pRandWalkFilterStates[2];
  ab2 += ad2;
  ac2 += bc2;

  /* Start for MATLABSystem: '<S17>/IMU' */
  ad2 = allRandData[26] * maximum + bd2;
  maximum = obj_0->Temperature - 25.0;
  obj_0->pRandWalkFilterStates[0] = ab2;
  a_0[0] = maximum * obj_0->TemperatureBias[0];
  obj_0->pRandWalkFilterStates[1] = ac2;
  a_0[1] = maximum * obj_0->TemperatureBias[1];
  obj_0->pRandWalkFilterStates[2] = ad2;
  a_0[2] = maximum * obj_0->TemperatureBias[2];
  temperatureDrift[0] = a_0[0];
  temperatureDrift[1] = a_0[1];
  temperatureDrift[2] = a_0[2];
  maximum = (obj_0->Temperature - 25.0) * 0.01;
  a_0[0] = maximum * obj_0->TemperatureScaleFactor[0] + 1.0;
  a_0[1] = maximum * obj_0->TemperatureScaleFactor[1] + 1.0;
  a_0[2] = maximum * obj_0->TemperatureScaleFactor[2] + 1.0;
  m[0] = ((((x + y[0]) + ab2) + temperatureDrift[0]) + magneticfield[0]) * a_0[0];
  m[1] = ((((aasq + y[1]) + ac2) + temperatureDrift[1]) + magneticfield[1]) *
    a_0[1];
  m[2] = ((((n + y[2]) + ad2) + temperatureDrift[2]) + magneticfield[2]) * a_0[2];
  x = obj_0->MeasurementRange;
  if (!rtIsInf(x)) {
    maximum = obj_0->MeasurementRange;
    x = fabs(m[0]);
    y[0] = x;
    aasq = fabs(m[1]);
    y[1] = aasq;
    n = fabs(m[2]);
    y[2] = n;
    b_colIdx = 0;
    for (i = 0; i < 3; i++) {
      if (y[i] > maximum) {
        b_data[b_colIdx] = (int8_T)i;
        b_colIdx++;
      }
    }

    y[0] = x;
    y[1] = aasq;
    y[2] = n;
    b_colIdx = 0;
    for (i = 0; i < 3; i++) {
      if (y[i] > maximum) {
        b_colIdx++;
      }
    }

    tmp_size_idx_1 = b_colIdx;
    b_colIdx = 0;
    for (i = 0; i < 3; i++) {
      if (y[i] > maximum) {
        tmp_data[b_colIdx] = (int8_T)i;
        b_colIdx++;
      }
    }

    for (i = 0; i < tmp_size_idx_1; i++) {
      a_0[i] = m[tmp_data[i]];
    }

    b_colIdx = 0;
    for (i = 0; i < 3; i++) {
      if (y[i] > maximum) {
        b_colIdx++;
      }
    }

    b_colIdx--;
    for (i = 0; i <= b_colIdx; i++) {
      x = a_0[i];
      if (rtIsNaN(x)) {
        a_0[i] = (rtNaN);
      } else if (x < 0.0) {
        a_0[i] = -1.0;
      } else {
        a_0[i] = (x > 0.0);
      }
    }

    for (i = 0; i < tmp_size_idx_1; i++) {
      m[b_data[i]] = a_0[i] * maximum;
    }
  }

  if (obj_0->Resolution != 0.0) {
    maximum = obj_0->Resolution;
    m[0] /= maximum;
    m[1] /= maximum;
    m[2] /= maximum;
    m[0] = rt_roundd_snf(m[0]);
    m[1] = rt_roundd_snf(m[1]);
    m[2] = rt_roundd_snf(m[2]);
    m[0] *= maximum;
    m[1] *= maximum;
    m[2] *= maximum;
  }
}

static void simulation_SystemCore_step(fusion_simulink_imuSensor_sim_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[3], const real_T
  varargin_3[4], real_T varargout_1[3], real_T varargout_2[3], real_T
  varargout_3[3])
{
  g_fusion_internal_Acceleromet_T *obj_0;
  h_fusion_internal_GyroscopeSi_T *obj_1;
  i_fusion_internal_Magnetomete_T *obj_2;
  real_T valIn[9];
  real_T valIn_0[9];
  real_T valIn_1[9];
  real_T val_1[3];
  real_T val_2[3];
  real_T val_3[3];
  real_T val_4[3];
  real_T val_5[3];
  real_T val_6[3];
  real_T val_9[3];
  real_T val_a[3];
  real_T val_b[3];
  real_T val_c[3];
  real_T val_d[3];
  real_T val_e[3];
  real_T val_f[3];
  real_T val_i[3];
  real_T val_j[3];
  real_T val_k[3];
  real_T val_l[3];
  real_T val_m[3];
  real_T val_n[3];
  real_T b_Denominator[2];
  real_T b_Denominator_0[2];
  real_T b_Denominator_1[2];
  real_T b_Numerator;
  real_T b_Numerator_0;
  real_T b_Numerator_1;
  real_T val;
  real_T val_0;
  real_T val_7;
  real_T val_8;
  real_T val_g;
  real_T val_h;
  int32_T i;
  boolean_T flag_1[12];
  boolean_T flag_0[11];
  boolean_T flag;
  static const char_T ap_NoiseType_Value[12] = { 'd', 'o', 'u', 'b', 'l', 'e',
    '-', 's', 'i', 'd', 'e', 'd' };

  static const int32_T tmp[2] = { 1, 11 };

  static const int32_T tmp_0[2] = { 1, 12 };

  /* Start for MATLABSystem: '<S17>/IMU' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    val = obj->AccelParamsMeasurementRange;
    val_0 = obj->AccelParamsResolution;
    val_1[0] = obj->AccelParamsConstantBias[0];
    val_1[1] = obj->AccelParamsConstantBias[1];
    val_1[2] = obj->AccelParamsConstantBias[2];
    for (i = 0; i < 9; i++) {
      valIn[i] = obj->AccelParamsAxesMisalignment[i];
    }

    val_2[0] = obj->AccelParamsNoiseDensity[0];
    val_2[1] = obj->AccelParamsNoiseDensity[1];
    val_2[2] = obj->AccelParamsNoiseDensity[2];
    val_3[0] = obj->AccelParamsBiasInstability[0];
    val_3[1] = obj->AccelParamsBiasInstability[1];
    val_3[2] = obj->AccelParamsBiasInstability[2];
    val_4[0] = obj->AccelParamsRandomWalk[0];
    val_4[1] = obj->AccelParamsRandomWalk[1];
    val_4[2] = obj->AccelParamsRandomWalk[2];
    val_5[0] = obj->AccelParamsTemperatureBias[0];
    val_5[1] = obj->AccelParamsTemperatureBias[1];
    val_5[2] = obj->AccelParamsTemperatureBias[2];
    val_6[0] = obj->AccelParamsTemperatureScaleFactor[0];
    val_6[1] = obj->AccelParamsTemperatureScaleFactor[1];
    val_6[2] = obj->AccelParamsTemperatureScaleFactor[2];
    b_Numerator = obj->AccelParamsBiasInstabilityNumerator;
    b_Denominator[0] = obj->AccelParamsBiasInstabilityDenominator[0];
    b_Denominator[1] = obj->AccelParamsBiasInstabilityDenominator[1];
    val_7 = obj->GyroParamsMeasurementRange;
    val_8 = obj->GyroParamsResolution;
    val_9[0] = obj->GyroParamsConstantBias[0];
    val_9[1] = obj->GyroParamsConstantBias[1];
    val_9[2] = obj->GyroParamsConstantBias[2];
    for (i = 0; i < 9; i++) {
      valIn_0[i] = obj->GyroParamsAxesMisalignment[i];
    }

    val_a[0] = obj->GyroParamsNoiseDensity[0];
    val_a[1] = obj->GyroParamsNoiseDensity[1];
    val_a[2] = obj->GyroParamsNoiseDensity[2];
    val_b[0] = obj->GyroParamsBiasInstability[0];
    val_b[1] = obj->GyroParamsBiasInstability[1];
    val_b[2] = obj->GyroParamsBiasInstability[2];
    val_c[0] = obj->GyroParamsRandomWalk[0];
    val_c[1] = obj->GyroParamsRandomWalk[1];
    val_c[2] = obj->GyroParamsRandomWalk[2];
    val_d[0] = obj->GyroParamsTemperatureBias[0];
    val_d[1] = obj->GyroParamsTemperatureBias[1];
    val_d[2] = obj->GyroParamsTemperatureBias[2];
    val_e[0] = obj->GyroParamsTemperatureScaleFactor[0];
    val_e[1] = obj->GyroParamsTemperatureScaleFactor[1];
    val_e[2] = obj->GyroParamsTemperatureScaleFactor[2];
    val_f[0] = obj->GyroParamsAccelerationBias[0];
    val_f[1] = obj->GyroParamsAccelerationBias[1];
    val_f[2] = obj->GyroParamsAccelerationBias[2];
    b_Numerator_0 = obj->GyroParamsBiasInstabilityNumerator;
    b_Denominator_0[0] = obj->GyroParamsBiasInstabilityDenominator[0];
    b_Denominator_0[1] = obj->GyroParamsBiasInstabilityDenominator[1];
    val_g = obj->MagParamsMeasurementRange;
    val_h = obj->MagParamsResolution;
    val_i[0] = obj->MagParamsConstantBias[0];
    val_i[1] = obj->MagParamsConstantBias[1];
    val_i[2] = obj->MagParamsConstantBias[2];
    for (i = 0; i < 9; i++) {
      valIn_1[i] = obj->MagParamsAxesMisalignment[i];
    }

    val_j[0] = obj->MagParamsNoiseDensity[0];
    val_j[1] = obj->MagParamsNoiseDensity[1];
    val_j[2] = obj->MagParamsNoiseDensity[2];
    val_k[0] = obj->MagParamsBiasInstability[0];
    val_k[1] = obj->MagParamsBiasInstability[1];
    val_k[2] = obj->MagParamsBiasInstability[2];
    val_l[0] = obj->MagParamsRandomWalk[0];
    val_l[1] = obj->MagParamsRandomWalk[1];
    val_l[2] = obj->MagParamsRandomWalk[2];
    val_m[0] = obj->MagParamsTemperatureBias[0];
    val_m[1] = obj->MagParamsTemperatureBias[1];
    val_m[2] = obj->MagParamsTemperatureBias[2];
    val_n[0] = obj->MagParamsTemperatureScaleFactor[0];
    val_n[1] = obj->MagParamsTemperatureScaleFactor[1];
    val_n[2] = obj->MagParamsTemperatureScaleFactor[2];
    b_Numerator_1 = obj->MagParamsBiasInstabilityNumerator;
    b_Denominator_1[0] = obj->MagParamsBiasInstabilityDenominator[0];
    b_Denominator_1[1] = obj->MagParamsBiasInstabilityDenominator[1];
    flag = obj->tunablePropertyChanged[37];
    if (flag) {
      obj_0 = obj->pAccel;
      flag = (obj_0->isInitialized == 1);
      if (flag) {
        obj_0->TunablePropsChanged = true;
        obj_0->tunablePropertyChanged[11] = true;
      }

      obj->pAccel->Temperature = obj->Temperature;
      IMUSensorParameters_updateSyste(val, val_0, val_1, valIn, val_2, val_3,
        val_4, b_Numerator, b_Denominator, ap_NoiseType_Value, val_5, val_6,
        obj->pAccel);
      obj_1 = obj->pGyro;
      flag = (obj_1->isInitialized == 1);
      if (flag) {
        obj_1->TunablePropsChanged = true;
        obj_1->tunablePropertyChanged[12] = true;
      }

      obj->pGyro->Temperature = obj->Temperature;
      IMUSensorParameters_updateSys_o(val_7, val_8, val_9, valIn_0, val_a, val_b,
        val_c, b_Numerator_0, b_Denominator_0, ap_NoiseType_Value, val_d, val_e,
        val_f, obj->pGyro);
      obj_2 = obj->pMag;
      flag = (obj_2->isInitialized == 1);
      if (flag) {
        obj_2->TunablePropsChanged = true;
        obj_2->tunablePropertyChanged[11] = true;
      }

      obj->pMag->Temperature = obj->Temperature;
      IMUSensorParameters_updateSy_oi(val_g, val_h, val_i, valIn_1, val_j, val_k,
        val_l, b_Numerator_1, b_Denominator_1, ap_NoiseType_Value, val_m, val_n,
        obj->pMag);
    }

    flag_0[0] = obj->tunablePropertyChanged[3];
    flag_0[1] = obj->tunablePropertyChanged[4];
    flag_0[2] = obj->tunablePropertyChanged[5];
    flag_0[3] = obj->tunablePropertyChanged[6];
    flag_0[4] = obj->tunablePropertyChanged[7];
    flag_0[5] = obj->tunablePropertyChanged[8];
    flag_0[6] = obj->tunablePropertyChanged[9];
    flag_0[7] = obj->tunablePropertyChanged[10];
    flag_0[8] = obj->tunablePropertyChanged[11];
    flag_0[9] = obj->tunablePropertyChanged[12];
    flag_0[10] = obj->tunablePropertyChanged[13];
    if (simulation_vectorAny(flag_0, tmp)) {
      IMUSensorParameters_updateSyste(val, val_0, val_1, valIn, val_2, val_3,
        val_4, b_Numerator, b_Denominator, ap_NoiseType_Value, val_5, val_6,
        obj->pAccel);
    }

    flag_1[0] = obj->tunablePropertyChanged[14];
    flag_1[1] = obj->tunablePropertyChanged[15];
    flag_1[2] = obj->tunablePropertyChanged[16];
    flag_1[3] = obj->tunablePropertyChanged[17];
    flag_1[4] = obj->tunablePropertyChanged[18];
    flag_1[5] = obj->tunablePropertyChanged[19];
    flag_1[6] = obj->tunablePropertyChanged[20];
    flag_1[7] = obj->tunablePropertyChanged[21];
    flag_1[8] = obj->tunablePropertyChanged[22];
    flag_1[9] = obj->tunablePropertyChanged[23];
    flag_1[10] = obj->tunablePropertyChanged[24];
    flag_1[11] = obj->tunablePropertyChanged[25];
    if (simulation_vectorAny(flag_1, tmp_0)) {
      IMUSensorParameters_updateSys_o(val_7, val_8, val_9, valIn_0, val_a, val_b,
        val_c, b_Numerator_0, b_Denominator_0, ap_NoiseType_Value, val_d, val_e,
        val_f, obj->pGyro);
    }

    flag_0[0] = obj->tunablePropertyChanged[26];
    flag_0[1] = obj->tunablePropertyChanged[27];
    flag_0[2] = obj->tunablePropertyChanged[28];
    flag_0[3] = obj->tunablePropertyChanged[29];
    flag_0[4] = obj->tunablePropertyChanged[30];
    flag_0[5] = obj->tunablePropertyChanged[31];
    flag_0[6] = obj->tunablePropertyChanged[32];
    flag_0[7] = obj->tunablePropertyChanged[33];
    flag_0[8] = obj->tunablePropertyChanged[34];
    flag_0[9] = obj->tunablePropertyChanged[35];
    flag_0[10] = obj->tunablePropertyChanged[36];
    if (simulation_vectorAny(flag_0, tmp)) {
      IMUSensorParameters_updateSy_oi(val_g, val_h, val_i, valIn_1, val_j, val_k,
        val_l, b_Numerator_1, b_Denominator_1, ap_NoiseType_Value, val_m, val_n,
        obj->pMag);
    }

    for (i = 0; i < 38; i++) {
      obj->tunablePropertyChanged[i] = false;
    }
  }

  /* End of Start for MATLABSystem: '<S17>/IMU' */
  simulation_imuSensor_stepImpl(obj, varargin_1, varargin_2, varargin_3,
    varargout_1, varargout_2, varargout_3);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

static void simulation_SystemCore_setup(fusion_simulink_imuSensor_sim_T *obj)
{
  g_fusion_internal_Acceleromet_T *obj_0;
  h_fusion_internal_GyroscopeSi_T *obj_1;
  i_fusion_internal_Magnetomete_T *obj_2;
  real_T valIn[9];
  real_T val_1[3];
  real_T val_2[3];
  real_T val_3[3];
  real_T val_4[3];
  real_T val_5[3];
  real_T val_6[3];
  real_T val_7[3];
  real_T b_Denominator[2];
  real_T b_Numerator;
  real_T val;
  real_T val_0;
  int32_T i;
  boolean_T flag;
  static const char_T ap_NoiseType_Value[12] = { 'd', 'o', 'u', 'b', 'l', 'e',
    '-', 's', 'i', 'd', 'e', 'd' };

  obj->isInitialized = 1;

  /* Start for MATLABSystem: '<S17>/IMU' */
  val = obj->AccelParamsMeasurementRange;
  val_0 = obj->AccelParamsResolution;
  val_1[0] = obj->AccelParamsConstantBias[0];
  val_1[1] = obj->AccelParamsConstantBias[1];
  val_1[2] = obj->AccelParamsConstantBias[2];
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    valIn[i] = obj->AccelParamsAxesMisalignment[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  val_2[0] = obj->AccelParamsNoiseDensity[0];
  val_2[1] = obj->AccelParamsNoiseDensity[1];
  val_2[2] = obj->AccelParamsNoiseDensity[2];
  val_3[0] = obj->AccelParamsBiasInstability[0];
  val_3[1] = obj->AccelParamsBiasInstability[1];
  val_3[2] = obj->AccelParamsBiasInstability[2];
  val_4[0] = obj->AccelParamsRandomWalk[0];
  val_4[1] = obj->AccelParamsRandomWalk[1];
  val_4[2] = obj->AccelParamsRandomWalk[2];
  val_5[0] = obj->AccelParamsTemperatureBias[0];
  val_5[1] = obj->AccelParamsTemperatureBias[1];
  val_5[2] = obj->AccelParamsTemperatureBias[2];
  val_6[0] = obj->AccelParamsTemperatureScaleFactor[0];
  val_6[1] = obj->AccelParamsTemperatureScaleFactor[1];
  val_6[2] = obj->AccelParamsTemperatureScaleFactor[2];
  b_Numerator = obj->AccelParamsBiasInstabilityNumerator;
  b_Denominator[0] = obj->AccelParamsBiasInstabilityDenominator[0];
  b_Denominator[1] = obj->AccelParamsBiasInstabilityDenominator[1];
  obj->_pobj2.isInitialized = 0;
  for (i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->_pobj2.tunablePropertyChanged[i] = false;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  IMUSensorParameters_updateSyste(val, val_0, val_1, valIn, val_2, val_3, val_4,
    b_Numerator, b_Denominator, ap_NoiseType_Value, val_5, val_6, &obj->_pobj2);
  obj->pAccel = &obj->_pobj2;
  obj_0 = obj->pAccel;
  flag = (obj_0->isInitialized == 1);
  if (flag) {
    obj_0->TunablePropsChanged = true;
    obj_0->tunablePropertyChanged[11] = true;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  obj->pAccel->Temperature = obj->Temperature;
  val = obj->GyroParamsMeasurementRange;
  val_0 = obj->GyroParamsResolution;
  val_1[0] = obj->GyroParamsConstantBias[0];
  val_1[1] = obj->GyroParamsConstantBias[1];
  val_1[2] = obj->GyroParamsConstantBias[2];
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    valIn[i] = obj->GyroParamsAxesMisalignment[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  val_2[0] = obj->GyroParamsNoiseDensity[0];
  val_2[1] = obj->GyroParamsNoiseDensity[1];
  val_2[2] = obj->GyroParamsNoiseDensity[2];
  val_3[0] = obj->GyroParamsBiasInstability[0];
  val_3[1] = obj->GyroParamsBiasInstability[1];
  val_3[2] = obj->GyroParamsBiasInstability[2];
  val_4[0] = obj->GyroParamsRandomWalk[0];
  val_4[1] = obj->GyroParamsRandomWalk[1];
  val_4[2] = obj->GyroParamsRandomWalk[2];
  val_5[0] = obj->GyroParamsTemperatureBias[0];
  val_5[1] = obj->GyroParamsTemperatureBias[1];
  val_5[2] = obj->GyroParamsTemperatureBias[2];
  val_6[0] = obj->GyroParamsTemperatureScaleFactor[0];
  val_6[1] = obj->GyroParamsTemperatureScaleFactor[1];
  val_6[2] = obj->GyroParamsTemperatureScaleFactor[2];
  val_7[0] = obj->GyroParamsAccelerationBias[0];
  val_7[1] = obj->GyroParamsAccelerationBias[1];
  val_7[2] = obj->GyroParamsAccelerationBias[2];
  b_Numerator = obj->GyroParamsBiasInstabilityNumerator;
  b_Denominator[0] = obj->GyroParamsBiasInstabilityDenominator[0];
  b_Denominator[1] = obj->GyroParamsBiasInstabilityDenominator[1];
  obj->_pobj1.isInitialized = 0;
  for (i = 0; i < 13; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->_pobj1.tunablePropertyChanged[i] = false;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  IMUSensorParameters_updateSys_o(val, val_0, val_1, valIn, val_2, val_3, val_4,
    b_Numerator, b_Denominator, ap_NoiseType_Value, val_5, val_6, val_7,
    &obj->_pobj1);
  obj->pGyro = &obj->_pobj1;
  obj_1 = obj->pGyro;
  flag = (obj_1->isInitialized == 1);
  if (flag) {
    obj_1->TunablePropsChanged = true;
    obj_1->tunablePropertyChanged[12] = true;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  obj->pGyro->Temperature = obj->Temperature;
  val = obj->MagParamsMeasurementRange;
  val_0 = obj->MagParamsResolution;
  val_1[0] = obj->MagParamsConstantBias[0];
  val_1[1] = obj->MagParamsConstantBias[1];
  val_1[2] = obj->MagParamsConstantBias[2];
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    valIn[i] = obj->MagParamsAxesMisalignment[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  val_2[0] = obj->MagParamsNoiseDensity[0];
  val_2[1] = obj->MagParamsNoiseDensity[1];
  val_2[2] = obj->MagParamsNoiseDensity[2];
  val_3[0] = obj->MagParamsBiasInstability[0];
  val_3[1] = obj->MagParamsBiasInstability[1];
  val_3[2] = obj->MagParamsBiasInstability[2];
  val_4[0] = obj->MagParamsRandomWalk[0];
  val_4[1] = obj->MagParamsRandomWalk[1];
  val_4[2] = obj->MagParamsRandomWalk[2];
  val_5[0] = obj->MagParamsTemperatureBias[0];
  val_5[1] = obj->MagParamsTemperatureBias[1];
  val_5[2] = obj->MagParamsTemperatureBias[2];
  val_6[0] = obj->MagParamsTemperatureScaleFactor[0];
  val_6[1] = obj->MagParamsTemperatureScaleFactor[1];
  val_6[2] = obj->MagParamsTemperatureScaleFactor[2];
  b_Numerator = obj->MagParamsBiasInstabilityNumerator;
  b_Denominator[0] = obj->MagParamsBiasInstabilityDenominator[0];
  b_Denominator[1] = obj->MagParamsBiasInstabilityDenominator[1];
  obj->_pobj0.isInitialized = 0;
  for (i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->_pobj0.tunablePropertyChanged[i] = false;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  IMUSensorParameters_updateSy_oi(val, val_0, val_1, valIn, val_2, val_3, val_4,
    b_Numerator, b_Denominator, ap_NoiseType_Value, val_5, val_6, &obj->_pobj0);
  obj->pMag = &obj->_pobj0;
  obj_2 = obj->pMag;
  flag = (obj_2->isInitialized == 1);
  if (flag) {
    obj_2->TunablePropsChanged = true;
    obj_2->tunablePropertyChanged[11] = true;
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  obj->pMag->Temperature = obj->Temperature;
  obj->TunablePropsChanged = false;
}

static void simulat_IMUSensorBase_resetImpl(fusion_simulink_imuSensor_sim_T *obj)
{
  g_fusion_internal_Acceleromet_T *obj_0;
  h_fusion_internal_GyroscopeSi_T *obj_1;
  i_fusion_internal_Magnetomete_T *obj_2;
  int32_T i;
  uint32_T b_state[625];
  uint32_T r;
  boolean_T flag;
  for (i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->pStreamState[i] = 0U;
  }

  for (i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    b_state[i] = obj->pStreamState[i];
  }

  r = 67U;
  b_state[0] = 67U;
  for (i = 0; i < 623; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    r = ((r >> 30U ^ r) * 1812433253U + (uint32_T)i) + 1U;
    b_state[i + 1] = r;
  }

  b_state[624] = 624U;
  for (i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<S17>/IMU' */
    obj->pStreamState[i] = b_state[i];
  }

  /* Start for MATLABSystem: '<S17>/IMU' */
  flag = (obj->isInitialized == 1);
  if (flag) {
    obj_0 = obj->pAccel;
    if (obj_0->isInitialized == 1) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj_0->pBiasInstFilterStates[0] = 0.0;
      obj_0->pRandWalkFilterStates[0] = 0.0;
      obj_0->pBiasInstFilterStates[1] = 0.0;
      obj_0->pRandWalkFilterStates[1] = 0.0;
      obj_0->pBiasInstFilterStates[2] = 0.0;
      obj_0->pRandWalkFilterStates[2] = 0.0;
    }

    obj_1 = obj->pGyro;
    if (obj_1->isInitialized == 1) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj_1->pBiasInstFilterStates[0] = 0.0;
      obj_1->pRandWalkFilterStates[0] = 0.0;
      obj_1->pBiasInstFilterStates[1] = 0.0;
      obj_1->pRandWalkFilterStates[1] = 0.0;
      obj_1->pBiasInstFilterStates[2] = 0.0;
      obj_1->pRandWalkFilterStates[2] = 0.0;
    }

    obj_2 = obj->pMag;
    if (obj_2->isInitialized == 1) {
      /* Start for MATLABSystem: '<S17>/IMU' */
      obj_2->pBiasInstFilterStates[0] = 0.0;
      obj_2->pRandWalkFilterStates[0] = 0.0;
      obj_2->pBiasInstFilterStates[1] = 0.0;
      obj_2->pRandWalkFilterStates[1] = 0.0;
      obj_2->pBiasInstFilterStates[2] = 0.0;
      obj_2->pRandWalkFilterStates[2] = 0.0;
    }
  }
}

/* Model step function */
void simulation_step(void)
{
  if (rtmIsMajorTimeStep(simulation_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&simulation_M->solverInfo,
                          ((simulation_M->Timing.clockTick0+1)*
      simulation_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(simulation_M)) {
    simulation_M->Timing.t[0] = rtsiGetT(&simulation_M->solverInfo);
  }

  {
    NeslSimulationData *simulationData;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    NeuDiagnosticTree *diagnosticTree_0;
    NeuDiagnosticTree *diagnosticTree_1;
    char *msg;
    char *msg_0;
    char *msg_1;
    real_T tmp_3[47];
    real_T tmp_5[47];
    real_T tmp_0[20];
    real_T x[6];
    real_T y[6];
    real_T rtb_ZeroOrderHold[4];
    real_T rtb_ZeroOrderHold1[3];
    real_T rtb_ZeroOrderHold2[3];
    real_T r;
    real_T rtb_Product2;
    real_T rtb_Product3;
    real_T rtb_fcn3;
    real_T rtb_fcn5;
    real_T time;
    real_T time_0;
    real_T time_1;
    real_T time_2;
    real_T time_3;
    real_T time_4;
    int32_T exitg1;
    int32_T high_i;
    int32_T low_i;
    int32_T low_ip1;
    int32_T mid_i;
    int_T tmp_4[7];
    int_T tmp_6[7];
    int_T tmp_1[6];
    boolean_T flag;
    boolean_T tmp;
    boolean_T tmp_2;
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* UnitDelay: '<S62>/Unit Delay' */
      simulation_B.UnitDelay = simulation_DW.UnitDelay_DSTATE;
    }

    /* Switch: '<S62>/Switch' incorporates:
     *  Abs: '<S62>/Abs'
     *  Inport: '<Root>/tvc x angle'
     */
    if (fabs(simulation_U.tvcxangle) > 0.15) {
      /* Switch: '<S62>/Switch' */
      simulation_B.Switch = simulation_B.UnitDelay;
    } else {
      /* Switch: '<S62>/Switch' */
      simulation_B.Switch = simulation_U.tvcxangle;
    }

    /* End of Switch: '<S62>/Switch' */

    /* SimscapeInputBlock: '<S69>/INPUT_4_1_1' */
    if (simulation_DW.INPUT_4_1_1_FirstOutput == 0.0) {
      simulation_DW.INPUT_4_1_1_FirstOutput = 1.0;
      simulation_X.simulationPlantTVC_PhysicsSimul[0] = simulation_B.Switch;
      simulation_X.simulationPlantTVC_PhysicsSimul[1] = 0.0;
    }

    simulation_B.INPUT_4_1_1[0] = simulation_X.simulationPlantTVC_PhysicsSimul[0];
    simulation_B.INPUT_4_1_1[1] = simulation_X.simulationPlantTVC_PhysicsSimul[1];
    simulation_B.INPUT_4_1_1[2] = ((simulation_B.Switch -
      simulation_X.simulationPlantTVC_PhysicsSimul[0]) * 20.0 - 2.0 *
      simulation_X.simulationPlantTVC_PhysicsSimul[1]) * 20.0;
    simulation_B.INPUT_4_1_1[3] = 0.0;

    /* End of SimscapeInputBlock: '<S69>/INPUT_4_1_1' */
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* UnitDelay: '<S62>/Unit Delay1' */
      simulation_B.UnitDelay1 = simulation_DW.UnitDelay1_DSTATE;
    }

    /* Switch: '<S62>/Switch1' incorporates:
     *  Abs: '<S62>/Abs1'
     *  Inport: '<Root>/tvc y angle'
     */
    if (fabs(simulation_U.tvcyangle) > 0.15) {
      /* Switch: '<S62>/Switch1' */
      simulation_B.Switch1 = simulation_B.UnitDelay1;
    } else {
      /* Switch: '<S62>/Switch1' */
      simulation_B.Switch1 = simulation_U.tvcyangle;
    }

    /* End of Switch: '<S62>/Switch1' */

    /* SimscapeInputBlock: '<S69>/INPUT_5_1_1' */
    if (simulation_DW.INPUT_5_1_1_FirstOutput == 0.0) {
      simulation_DW.INPUT_5_1_1_FirstOutput = 1.0;
      simulation_X.simulationPlantTVC_PhysicsSim_a[0] = simulation_B.Switch1;
      simulation_X.simulationPlantTVC_PhysicsSim_a[1] = 0.0;
    }

    simulation_B.INPUT_5_1_1[0] = simulation_X.simulationPlantTVC_PhysicsSim_a[0];
    simulation_B.INPUT_5_1_1[1] = simulation_X.simulationPlantTVC_PhysicsSim_a[1];
    simulation_B.INPUT_5_1_1[2] = ((simulation_B.Switch1 -
      simulation_X.simulationPlantTVC_PhysicsSim_a[0]) * 20.0 - 2.0 *
      simulation_X.simulationPlantTVC_PhysicsSim_a[1]) * 20.0;
    simulation_B.INPUT_5_1_1[3] = 0.0;

    /* End of SimscapeInputBlock: '<S69>/INPUT_5_1_1' */
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* Chart: '<S5>/Discretizer' incorporates:
       *  Inport: '<Root>/ignite_s1'
       *  Inport: '<Root>/ignite_s2'
       *  UnitDelay: '<S5>/Unit Delay1'
       */
      if (simulation_DW.temporalCounter_i1 < MAX_uint32_T) {
        simulation_DW.temporalCounter_i1++;
      }

      if (simulation_DW.is_active_c3_simulation == 0U) {
        simulation_DW.is_active_c3_simulation = 1U;
        simulation_DW.is_c3_simulation = simulation_IN_S0;
        simulation_B.attached = 0.0;
        simulation_B.s1_et = 0.0;
        simulation_B.s2_et = 0.0;
      } else {
        switch (simulation_DW.is_c3_simulation) {
         case simulation_IN_Detached:
          simulation_B.attached = -1.0;
          if (simulation_U.ignite_s2 != 0.0) {
            simulation_DW.temporalCounter_i1 = 0U;
            simulation_DW.is_c3_simulation = simulation_IN_S2_Fire;
          }
          break;

         case simulation_IN_Idle:
          break;

         case simulation_IN_S0:
          simulation_B.attached = 0.0;
          if (simulation_U.ignite_s1 != 0.0) {
            simulation_DW.temporalCounter_i1 = 0U;
            simulation_DW.is_c3_simulation = simulation_IN_S1_fire;
            simulation_DW.is_S1_fire = simulation_IN_S1_Ignite;
          }
          break;

         case simulation_IN_S1_fire:
          simulation_B.s1_et = (real_T)simulation_DW.temporalCounter_i1 * 0.001;
          if (simulation_DW.is_S1_fire == simulation_IN_S1_Ignite) {
            if (simulation_DW.UnitDelay_DSTATE_m > 0.0) {
              simulation_DW.is_S1_fire = simulation_IN_S1_engaged;
            }

            /* case IN_S1_engaged: */
          } else if (simulation_DW.UnitDelay_DSTATE_m <= 0.0) {
            simulation_DW.is_S1_fire = simulation_IN_NO_ACTIVE_CHILD;
            simulation_DW.is_c3_simulation = simulation_IN_Detached;
            simulation_B.attached = -1.0;
          }
          break;

         default:
          /* case IN_S2_Fire: */
          if (simulation_DW.UnitDelay1_DSTATE_j < 0.0) {
            simulation_DW.is_c3_simulation = simulation_IN_Idle;
          } else {
            simulation_B.s2_et = (real_T)simulation_DW.temporalCounter_i1 *
              0.001;
          }
          break;
        }
      }

      /* End of Chart: '<S5>/Discretizer' */
    }

    /* SimscapeInputBlock: '<S69>/INPUT_1_1_1' */
    simulation_B.INPUT_1_1_1[0] = simulation_B.attached;
    simulation_B.INPUT_1_1_1[1] = 0.0;
    simulation_B.INPUT_1_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(simulation_M)) {
      simulation_DW.INPUT_1_1_1_Discrete[0] = !(simulation_B.INPUT_1_1_1[0] ==
        simulation_DW.INPUT_1_1_1_Discrete[1]);
      simulation_DW.INPUT_1_1_1_Discrete[1] = simulation_B.INPUT_1_1_1[0];
    }

    simulation_B.INPUT_1_1_1[0] = simulation_DW.INPUT_1_1_1_Discrete[1];
    simulation_B.INPUT_1_1_1[3] = simulation_DW.INPUT_1_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S69>/INPUT_1_1_1' */

    /* SimscapeExecutionBlock: '<S69>/STATE_1' incorporates:
     *  SimscapeExecutionBlock: '<S69>/OUTPUT_1_0'
     *  SimscapeExecutionBlock: '<S69>/OUTPUT_1_1'
     */
    simulationData = (NeslSimulationData *)simulation_DW.STATE_1_SimData;
    rtb_Product3 = simulation_M->Timing.t[0];
    time = rtb_Product3;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 26;
    simulationData->mData->mContStates.mX =
      &simulation_X.simulationPlantRocket_BodyFree_[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &simulation_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 1;
    simulationData->mData->mModeVector.mX = &simulation_DW.STATE_1_Modes;
    flag = false;
    simulationData->mData->mFoundZcEvents = flag;
    flag = rtmIsMajorTimeStep(simulation_M);
    simulationData->mData->mIsMajorTimeStep = flag;
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp = rtsiIsSolverComputingJacobian(&simulation_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    tmp = rtsiIsModeUpdateTimeStep(&simulation_M->solverInfo);
    simulationData->mData->mIsModeUpdateTimeStep = tmp;
    tmp_1[0] = 0;
    tmp_0[0] = simulation_B.INPUT_4_1_1[0];
    tmp_0[1] = simulation_B.INPUT_4_1_1[1];
    tmp_0[2] = simulation_B.INPUT_4_1_1[2];
    tmp_0[3] = simulation_B.INPUT_4_1_1[3];
    tmp_1[1] = 4;
    tmp_0[4] = simulation_B.INPUT_5_1_1[0];
    tmp_0[5] = simulation_B.INPUT_5_1_1[1];
    tmp_0[6] = simulation_B.INPUT_5_1_1[2];
    tmp_0[7] = simulation_B.INPUT_5_1_1[3];
    tmp_1[2] = 8;
    tmp_0[8] = simulation_B.INPUT_1_1_1[0];
    tmp_0[9] = simulation_B.INPUT_1_1_1[1];
    tmp_0[10] = simulation_B.INPUT_1_1_1[2];
    tmp_0[11] = simulation_B.INPUT_1_1_1[3];
    tmp_1[3] = 12;
    tmp_0[12] = simulation_B.INPUT_2_1_1[0];
    tmp_0[13] = simulation_B.INPUT_2_1_1[1];
    tmp_0[14] = simulation_B.INPUT_2_1_1[2];
    tmp_0[15] = simulation_B.INPUT_2_1_1[3];
    tmp_1[4] = 16;
    tmp_0[16] = simulation_B.INPUT_3_1_1[0];
    tmp_0[17] = simulation_B.INPUT_3_1_1[1];
    tmp_0[18] = simulation_B.INPUT_3_1_1[2];
    tmp_0[19] = simulation_B.INPUT_3_1_1[3];
    tmp_1[5] = 20;
    simulationData->mData->mInputValues.mN = 20;
    simulationData->mData->mInputValues.mX = &tmp_0[0];
    simulationData->mData->mInputOffsets.mN = 6;
    simulationData->mData->mInputOffsets.mX = &tmp_1[0];
    simulationData->mData->mOutputs.mN = 27;
    simulationData->mData->mOutputs.mX = &simulation_B.STATE_1[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    rtb_Product2 = simulation_M->Timing.t[0];
    time_0 = rtb_Product2;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_0;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    high_i = ne_simulator_method((NeslSimulator *)
      simulation_DW.STATE_1_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (high_i != 0) {
      tmp_2 = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (tmp_2) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(simulation_M, msg);
      }
    }

    /* End of SimscapeExecutionBlock: '<S69>/STATE_1' */
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* MATLAB Function: '<S5>/stage1_thrust_calculate' incorporates:
       *  Constant: '<S5>/series'
       */
      for (high_i = 0; high_i < 6; high_i++) {
        low_i = high_i << 1;
        x[high_i] = simulation_ConstP.pooled7[low_i];
        y[high_i] = simulation_ConstP.pooled7[low_i + 1];
      }

      simulation_B.thrust_j = (rtNaN);
      high_i = 0;
      do {
        exitg1 = 0;
        if (high_i < 6) {
          if (rtIsNaN(simulation_ConstP.pooled7[high_i << 1])) {
            exitg1 = 1;
          } else {
            high_i++;
          }
        } else {
          simulation_B.thrust_j = (rtNaN);
          if (rtIsNaN(simulation_B.s1_et)) {
            simulation_B.thrust_j = (rtNaN);
          } else if ((!(simulation_B.s1_et > x[5])) && (!(simulation_B.s1_et <
                       x[0]))) {
            high_i = 6;
            low_i = 1;
            low_ip1 = 2;
            while (high_i > low_ip1) {
              mid_i = (low_i + high_i) >> 1;
              if (simulation_B.s1_et >= x[mid_i - 1]) {
                low_i = mid_i;
                low_ip1 = mid_i + 1;
              } else {
                high_i = mid_i;
              }
            }

            r = x[low_i - 1];
            r = (simulation_B.s1_et - r) / (x[low_i] - r);
            if (r == 0.0) {
              simulation_B.thrust_j = y[low_i - 1];
            } else if (r == 1.0) {
              simulation_B.thrust_j = y[low_i];
            } else {
              simulation_B.thrust_j = y[low_i - 1];
              if (!(simulation_B.thrust_j == y[low_i])) {
                simulation_B.thrust_j = (1.0 - r) * simulation_B.thrust_j + r *
                  y[low_i];
              }
            }
          }

          exitg1 = 1;
        }
      } while (exitg1 == 0);

      /* End of MATLAB Function: '<S5>/stage1_thrust_calculate' */
    }

    /* SimscapeInputBlock: '<S69>/INPUT_2_1_1' */
    simulation_B.INPUT_2_1_1[0] = simulation_B.thrust_j;
    simulation_B.INPUT_2_1_1[1] = 0.0;
    simulation_B.INPUT_2_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(simulation_M)) {
      simulation_DW.INPUT_2_1_1_Discrete[0] = !(simulation_B.INPUT_2_1_1[0] ==
        simulation_DW.INPUT_2_1_1_Discrete[1]);
      simulation_DW.INPUT_2_1_1_Discrete[1] = simulation_B.INPUT_2_1_1[0];
    }

    simulation_B.INPUT_2_1_1[0] = simulation_DW.INPUT_2_1_1_Discrete[1];
    simulation_B.INPUT_2_1_1[3] = simulation_DW.INPUT_2_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S69>/INPUT_2_1_1' */
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* MATLAB Function: '<S5>/stage2_thrust_calculate' incorporates:
       *  Constant: '<S5>/series1'
       */
      for (high_i = 0; high_i < 6; high_i++) {
        low_i = high_i << 1;
        x[high_i] = simulation_ConstP.pooled7[low_i];
        y[high_i] = simulation_ConstP.pooled7[low_i + 1];
      }

      simulation_B.thrust = (rtNaN);
      high_i = 0;
      do {
        exitg1 = 0;
        if (high_i < 6) {
          if (rtIsNaN(simulation_ConstP.pooled7[high_i << 1])) {
            exitg1 = 1;
          } else {
            high_i++;
          }
        } else {
          simulation_B.thrust = (rtNaN);
          if (rtIsNaN(simulation_B.s2_et)) {
            simulation_B.thrust = (rtNaN);
          } else if ((!(simulation_B.s2_et > x[5])) && (!(simulation_B.s2_et <
                       x[0]))) {
            high_i = 6;
            low_i = 1;
            low_ip1 = 2;
            while (high_i > low_ip1) {
              mid_i = (low_i + high_i) >> 1;
              if (simulation_B.s2_et >= x[mid_i - 1]) {
                low_i = mid_i;
                low_ip1 = mid_i + 1;
              } else {
                high_i = mid_i;
              }
            }

            r = x[low_i - 1];
            r = (simulation_B.s2_et - r) / (x[low_i] - r);
            if (r == 0.0) {
              simulation_B.thrust = y[low_i - 1];
            } else if (r == 1.0) {
              simulation_B.thrust = y[low_i];
            } else {
              simulation_B.thrust = y[low_i - 1];
              if (!(simulation_B.thrust == y[low_i])) {
                simulation_B.thrust = (1.0 - r) * simulation_B.thrust + r *
                  y[low_i];
              }
            }
          }

          exitg1 = 1;
        }
      } while (exitg1 == 0);

      /* End of MATLAB Function: '<S5>/stage2_thrust_calculate' */
    }

    /* SimscapeInputBlock: '<S69>/INPUT_3_1_1' */
    simulation_B.INPUT_3_1_1[0] = simulation_B.thrust;
    simulation_B.INPUT_3_1_1[1] = 0.0;
    simulation_B.INPUT_3_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(simulation_M)) {
      simulation_DW.INPUT_3_1_1_Discrete[0] = !(simulation_B.INPUT_3_1_1[0] ==
        simulation_DW.INPUT_3_1_1_Discrete[1]);
      simulation_DW.INPUT_3_1_1_Discrete[1] = simulation_B.INPUT_3_1_1[0];
    }

    simulation_B.INPUT_3_1_1[0] = simulation_DW.INPUT_3_1_1_Discrete[1];
    simulation_B.INPUT_3_1_1[3] = simulation_DW.INPUT_3_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S69>/INPUT_3_1_1' */

    /* SimscapeExecutionBlock: '<S69>/OUTPUT_1_0' */
    simulationData = (NeslSimulationData *)simulation_DW.OUTPUT_1_0_SimData;
    time_1 = rtb_Product3;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_1;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &simulation_DW.OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &simulation_DW.OUTPUT_1_0_Modes;
    tmp_2 = false;
    simulationData->mData->mFoundZcEvents = tmp_2;
    simulationData->mData->mIsMajorTimeStep = flag;
    tmp_2 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_2;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = tmp;
    tmp_4[0] = 0;
    tmp_3[0] = simulation_B.INPUT_4_1_1[0];
    tmp_3[1] = simulation_B.INPUT_4_1_1[1];
    tmp_3[2] = simulation_B.INPUT_4_1_1[2];
    tmp_3[3] = simulation_B.INPUT_4_1_1[3];
    tmp_4[1] = 4;
    tmp_3[4] = simulation_B.INPUT_5_1_1[0];
    tmp_3[5] = simulation_B.INPUT_5_1_1[1];
    tmp_3[6] = simulation_B.INPUT_5_1_1[2];
    tmp_3[7] = simulation_B.INPUT_5_1_1[3];
    tmp_4[2] = 8;
    tmp_3[8] = simulation_B.INPUT_1_1_1[0];
    tmp_3[9] = simulation_B.INPUT_1_1_1[1];
    tmp_3[10] = simulation_B.INPUT_1_1_1[2];
    tmp_3[11] = simulation_B.INPUT_1_1_1[3];
    tmp_4[3] = 12;
    tmp_3[12] = simulation_B.INPUT_2_1_1[0];
    tmp_3[13] = simulation_B.INPUT_2_1_1[1];
    tmp_3[14] = simulation_B.INPUT_2_1_1[2];
    tmp_3[15] = simulation_B.INPUT_2_1_1[3];
    tmp_4[4] = 16;
    tmp_3[16] = simulation_B.INPUT_3_1_1[0];
    tmp_3[17] = simulation_B.INPUT_3_1_1[1];
    tmp_3[18] = simulation_B.INPUT_3_1_1[2];
    tmp_3[19] = simulation_B.INPUT_3_1_1[3];
    tmp_4[5] = 20;
    memcpy(&tmp_3[20], &simulation_B.STATE_1[0], 27U * sizeof(real_T));
    tmp_4[6] = 47;
    simulationData->mData->mInputValues.mN = 47;
    simulationData->mData->mInputValues.mX = &tmp_3[0];
    simulationData->mData->mInputOffsets.mN = 7;
    simulationData->mData->mInputOffsets.mX = &tmp_4[0];
    simulationData->mData->mOutputs.mN = 18;
    simulationData->mData->mOutputs.mX = &simulation_B.OUTPUT_1_0[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    time_2 = rtb_Product2;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_2;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    high_i = ne_simulator_method((NeslSimulator *)
      simulation_DW.OUTPUT_1_0_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (high_i != 0) {
      tmp_2 = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (tmp_2) {
        msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
        rtmSetErrorStatus(simulation_M, msg_0);
      }
    }

    /* SimscapeExecutionBlock: '<S69>/OUTPUT_1_1' */
    simulationData = (NeslSimulationData *)simulation_DW.OUTPUT_1_1_SimData;
    time_3 = rtb_Product3;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_3;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &simulation_DW.OUTPUT_1_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &simulation_DW.OUTPUT_1_1_Modes;
    tmp_2 = false;
    simulationData->mData->mFoundZcEvents = tmp_2;
    simulationData->mData->mIsMajorTimeStep = flag;
    flag = false;
    simulationData->mData->mIsSolverAssertCheck = flag;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = tmp;
    tmp_6[0] = 0;
    tmp_5[0] = simulation_B.INPUT_4_1_1[0];
    tmp_5[1] = simulation_B.INPUT_4_1_1[1];
    tmp_5[2] = simulation_B.INPUT_4_1_1[2];
    tmp_5[3] = simulation_B.INPUT_4_1_1[3];
    tmp_6[1] = 4;
    tmp_5[4] = simulation_B.INPUT_5_1_1[0];
    tmp_5[5] = simulation_B.INPUT_5_1_1[1];
    tmp_5[6] = simulation_B.INPUT_5_1_1[2];
    tmp_5[7] = simulation_B.INPUT_5_1_1[3];
    tmp_6[2] = 8;
    tmp_5[8] = simulation_B.INPUT_1_1_1[0];
    tmp_5[9] = simulation_B.INPUT_1_1_1[1];
    tmp_5[10] = simulation_B.INPUT_1_1_1[2];
    tmp_5[11] = simulation_B.INPUT_1_1_1[3];
    tmp_6[3] = 12;
    tmp_5[12] = simulation_B.INPUT_2_1_1[0];
    tmp_5[13] = simulation_B.INPUT_2_1_1[1];
    tmp_5[14] = simulation_B.INPUT_2_1_1[2];
    tmp_5[15] = simulation_B.INPUT_2_1_1[3];
    tmp_6[4] = 16;
    tmp_5[16] = simulation_B.INPUT_3_1_1[0];
    tmp_5[17] = simulation_B.INPUT_3_1_1[1];
    tmp_5[18] = simulation_B.INPUT_3_1_1[2];
    tmp_5[19] = simulation_B.INPUT_3_1_1[3];
    tmp_6[5] = 20;
    memcpy(&tmp_5[20], &simulation_B.STATE_1[0], 27U * sizeof(real_T));
    tmp_6[6] = 47;
    simulationData->mData->mInputValues.mN = 47;
    simulationData->mData->mInputValues.mX = &tmp_5[0];
    simulationData->mData->mInputOffsets.mN = 7;
    simulationData->mData->mInputOffsets.mX = &tmp_6[0];
    simulationData->mData->mOutputs.mN = 6;
    simulationData->mData->mOutputs.mX = &simulation_B.OUTPUT_1_1[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    time_4 = rtb_Product2;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_4;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.OUTPUT_1_1_DiagMgr;
    diagnosticTree_1 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    high_i = ne_simulator_method((NeslSimulator *)
      simulation_DW.OUTPUT_1_1_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (high_i != 0) {
      flag = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (flag) {
        msg_1 = rtw_diagnostics_msg(diagnosticTree_1);
        rtmSetErrorStatus(simulation_M, msg_1);
      }
    }

    /* Outport: '<Root>/true state' */
    memcpy(&simulation_Y.truestate[0], &simulation_B.OUTPUT_1_0[0], 10U * sizeof
           (real_T));
    simulation_Y.truestate[10] = simulation_B.OUTPUT_1_1[0];
    simulation_Y.truestate[11] = simulation_B.OUTPUT_1_1[1];
    simulation_Y.truestate[12] = simulation_B.OUTPUT_1_1[2];
    simulation_Y.truestate[13] = simulation_B.OUTPUT_1_0[10];
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* ZeroOrderHold: '<S17>/Zero-Order Hold' */
      rtb_ZeroOrderHold[0] = simulation_B.OUTPUT_1_0[11];
      rtb_ZeroOrderHold[1] = simulation_B.OUTPUT_1_0[12];
      rtb_ZeroOrderHold[2] = simulation_B.OUTPUT_1_0[13];
      rtb_ZeroOrderHold[3] = simulation_B.OUTPUT_1_0[14];

      /* ZeroOrderHold: '<S17>/Zero-Order Hold1' */
      rtb_ZeroOrderHold1[0] = simulation_B.OUTPUT_1_1[3];

      /* ZeroOrderHold: '<S17>/Zero-Order Hold2' */
      rtb_ZeroOrderHold2[0] = simulation_B.OUTPUT_1_0[15];

      /* ZeroOrderHold: '<S17>/Zero-Order Hold1' */
      rtb_ZeroOrderHold1[1] = simulation_B.OUTPUT_1_1[4];

      /* ZeroOrderHold: '<S17>/Zero-Order Hold2' */
      rtb_ZeroOrderHold2[1] = simulation_B.OUTPUT_1_0[16];

      /* ZeroOrderHold: '<S17>/Zero-Order Hold1' */
      rtb_ZeroOrderHold1[2] = simulation_B.OUTPUT_1_1[5];

      /* ZeroOrderHold: '<S17>/Zero-Order Hold2' */
      rtb_ZeroOrderHold2[2] = simulation_B.OUTPUT_1_0[17];

      /* MATLABSystem: '<S17>/IMU' incorporates:
       *  ZeroOrderHold: '<S17>/Zero-Order Hold'
       *  ZeroOrderHold: '<S17>/Zero-Order Hold1'
       *  ZeroOrderHold: '<S17>/Zero-Order Hold2'
       */
      if (simulation_DW.obj.Temperature != 25.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[37] = true;
        }

        simulation_DW.obj.Temperature = 25.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.MagneticFieldNED,
           simulation_ConstP.IMU_MagneticFieldNED)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[0] = true;
        }

        imuSensor_set_MagneticFieldNED(&simulation_DW.obj,
          simulation_ConstP.IMU_MagneticFieldNED);
      }

      if (simulation_DW.obj.AccelParamsMeasurementRange != (rtInf)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[3] = true;
        }

        simulation_DW.obj.AccelParamsMeasurementRange = (rtInf);
      }

      if (simulation_DW.obj.AccelParamsResolution != 0.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[4] = true;
        }

        simulation_DW.obj.AccelParamsResolution = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.AccelParamsConstantBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[5] = true;
        }

        simulation_DW.obj.AccelParamsConstantBias[0] = 0.0;
        simulation_DW.obj.AccelParamsConstantBias[1] = 0.0;
        simulation_DW.obj.AccelParamsConstantBias[2] = 0.0;
      }

      if (!simulation_isequal_oi(simulation_DW.obj.AccelParamsAxesMisalignment,
           simulation_ConstP.pooled4)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[6] = true;
        }

        memcpy(&simulation_DW.obj.AccelParamsAxesMisalignment[0],
               &simulation_ConstP.pooled4[0], 9U * sizeof(real_T));
      }

      if (!simulation_isequal_o(simulation_DW.obj.AccelParamsNoiseDensity,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[7] = true;
        }

        simulation_DW.obj.AccelParamsNoiseDensity[0] = 0.0;
        simulation_DW.obj.AccelParamsNoiseDensity[1] = 0.0;
        simulation_DW.obj.AccelParamsNoiseDensity[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.AccelParamsBiasInstability,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[8] = true;
        }

        simulation_DW.obj.AccelParamsBiasInstability[0] = 0.0;
        simulation_DW.obj.AccelParamsBiasInstability[1] = 0.0;
        simulation_DW.obj.AccelParamsBiasInstability[2] = 0.0;
      }

      if (simulation_DW.obj.AccelParamsBiasInstabilityNumerator != 1.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[9] = true;
        }

        simulation_DW.obj.AccelParamsBiasInstabilityNumerator = 1.0;
      }

      if (!simulation_isequal
          (simulation_DW.obj.AccelParamsBiasInstabilityDenominator,
           simulation_ConstP.pooled6)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[10] = true;
        }

        simulation_DW.obj.AccelParamsBiasInstabilityDenominator[0] = 1.0;
        simulation_DW.obj.AccelParamsBiasInstabilityDenominator[1] = -0.5;
      }

      if (!simulation_isequal_o(simulation_DW.obj.AccelParamsRandomWalk,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[11] = true;
        }

        simulation_DW.obj.AccelParamsRandomWalk[0] = 0.0;
        simulation_DW.obj.AccelParamsRandomWalk[1] = 0.0;
        simulation_DW.obj.AccelParamsRandomWalk[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.AccelParamsTemperatureBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[12] = true;
        }

        simulation_DW.obj.AccelParamsTemperatureBias[0] = 0.0;
        simulation_DW.obj.AccelParamsTemperatureBias[1] = 0.0;
        simulation_DW.obj.AccelParamsTemperatureBias[2] = 0.0;
      }

      if (!simulation_isequal_o
          (simulation_DW.obj.AccelParamsTemperatureScaleFactor,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[13] = true;
        }

        simulation_DW.obj.AccelParamsTemperatureScaleFactor[0] = 0.0;
        simulation_DW.obj.AccelParamsTemperatureScaleFactor[1] = 0.0;
        simulation_DW.obj.AccelParamsTemperatureScaleFactor[2] = 0.0;
      }

      if (simulation_DW.obj.GyroParamsMeasurementRange != (rtInf)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[14] = true;
        }

        simulation_DW.obj.GyroParamsMeasurementRange = (rtInf);
      }

      if (simulation_DW.obj.GyroParamsResolution != 0.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[15] = true;
        }

        simulation_DW.obj.GyroParamsResolution = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.GyroParamsConstantBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[16] = true;
        }

        simulation_DW.obj.GyroParamsConstantBias[0] = 0.0;
        simulation_DW.obj.GyroParamsConstantBias[1] = 0.0;
        simulation_DW.obj.GyroParamsConstantBias[2] = 0.0;
      }

      if (!simulation_isequal_oi(simulation_DW.obj.GyroParamsAxesMisalignment,
           simulation_ConstP.pooled4)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[17] = true;
        }

        memcpy(&simulation_DW.obj.GyroParamsAxesMisalignment[0],
               &simulation_ConstP.pooled4[0], 9U * sizeof(real_T));
      }

      if (!simulation_isequal_o(simulation_DW.obj.GyroParamsAccelerationBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[25] = true;
        }

        simulation_DW.obj.GyroParamsAccelerationBias[0] = 0.0;
        simulation_DW.obj.GyroParamsAccelerationBias[1] = 0.0;
        simulation_DW.obj.GyroParamsAccelerationBias[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.GyroParamsNoiseDensity,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[18] = true;
        }

        simulation_DW.obj.GyroParamsNoiseDensity[0] = 0.0;
        simulation_DW.obj.GyroParamsNoiseDensity[1] = 0.0;
        simulation_DW.obj.GyroParamsNoiseDensity[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.GyroParamsBiasInstability,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[19] = true;
        }

        simulation_DW.obj.GyroParamsBiasInstability[0] = 0.0;
        simulation_DW.obj.GyroParamsBiasInstability[1] = 0.0;
        simulation_DW.obj.GyroParamsBiasInstability[2] = 0.0;
      }

      if (simulation_DW.obj.GyroParamsBiasInstabilityNumerator != 1.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[20] = true;
        }

        simulation_DW.obj.GyroParamsBiasInstabilityNumerator = 1.0;
      }

      if (!simulation_isequal
          (simulation_DW.obj.GyroParamsBiasInstabilityDenominator,
           simulation_ConstP.pooled6)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[21] = true;
        }

        simulation_DW.obj.GyroParamsBiasInstabilityDenominator[0] = 1.0;
        simulation_DW.obj.GyroParamsBiasInstabilityDenominator[1] = -0.5;
      }

      if (!simulation_isequal_o(simulation_DW.obj.GyroParamsRandomWalk,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[22] = true;
        }

        simulation_DW.obj.GyroParamsRandomWalk[0] = 0.0;
        simulation_DW.obj.GyroParamsRandomWalk[1] = 0.0;
        simulation_DW.obj.GyroParamsRandomWalk[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.GyroParamsTemperatureBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[23] = true;
        }

        simulation_DW.obj.GyroParamsTemperatureBias[0] = 0.0;
        simulation_DW.obj.GyroParamsTemperatureBias[1] = 0.0;
        simulation_DW.obj.GyroParamsTemperatureBias[2] = 0.0;
      }

      if (!simulation_isequal_o
          (simulation_DW.obj.GyroParamsTemperatureScaleFactor,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[24] = true;
        }

        simulation_DW.obj.GyroParamsTemperatureScaleFactor[0] = 0.0;
        simulation_DW.obj.GyroParamsTemperatureScaleFactor[1] = 0.0;
        simulation_DW.obj.GyroParamsTemperatureScaleFactor[2] = 0.0;
      }

      if (simulation_DW.obj.MagParamsMeasurementRange != (rtInf)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[26] = true;
        }

        simulation_DW.obj.MagParamsMeasurementRange = (rtInf);
      }

      if (simulation_DW.obj.MagParamsResolution != 0.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[27] = true;
        }

        simulation_DW.obj.MagParamsResolution = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.MagParamsConstantBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[28] = true;
        }

        simulation_DW.obj.MagParamsConstantBias[0] = 0.0;
        simulation_DW.obj.MagParamsConstantBias[1] = 0.0;
        simulation_DW.obj.MagParamsConstantBias[2] = 0.0;
      }

      if (!simulation_isequal_oi(simulation_DW.obj.MagParamsAxesMisalignment,
           simulation_ConstP.pooled4)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[29] = true;
        }

        memcpy(&simulation_DW.obj.MagParamsAxesMisalignment[0],
               &simulation_ConstP.pooled4[0], 9U * sizeof(real_T));
      }

      if (!simulation_isequal_o(simulation_DW.obj.MagParamsNoiseDensity,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[30] = true;
        }

        simulation_DW.obj.MagParamsNoiseDensity[0] = 0.0;
        simulation_DW.obj.MagParamsNoiseDensity[1] = 0.0;
        simulation_DW.obj.MagParamsNoiseDensity[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.MagParamsBiasInstability,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[31] = true;
        }

        simulation_DW.obj.MagParamsBiasInstability[0] = 0.0;
        simulation_DW.obj.MagParamsBiasInstability[1] = 0.0;
        simulation_DW.obj.MagParamsBiasInstability[2] = 0.0;
      }

      if (simulation_DW.obj.MagParamsBiasInstabilityNumerator != 1.0) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[32] = true;
        }

        simulation_DW.obj.MagParamsBiasInstabilityNumerator = 1.0;
      }

      if (!simulation_isequal
          (simulation_DW.obj.MagParamsBiasInstabilityDenominator,
           simulation_ConstP.pooled6)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[33] = true;
        }

        simulation_DW.obj.MagParamsBiasInstabilityDenominator[0] = 1.0;
        simulation_DW.obj.MagParamsBiasInstabilityDenominator[1] = -0.5;
      }

      if (!simulation_isequal_o(simulation_DW.obj.MagParamsRandomWalk,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[34] = true;
        }

        simulation_DW.obj.MagParamsRandomWalk[0] = 0.0;
        simulation_DW.obj.MagParamsRandomWalk[1] = 0.0;
        simulation_DW.obj.MagParamsRandomWalk[2] = 0.0;
      }

      if (!simulation_isequal_o(simulation_DW.obj.MagParamsTemperatureBias,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[35] = true;
        }

        simulation_DW.obj.MagParamsTemperatureBias[0] = 0.0;
        simulation_DW.obj.MagParamsTemperatureBias[1] = 0.0;
        simulation_DW.obj.MagParamsTemperatureBias[2] = 0.0;
      }

      if (!simulation_isequal_o
          (simulation_DW.obj.MagParamsTemperatureScaleFactor,
           simulation_ConstP.pooled3)) {
        flag = (simulation_DW.obj.isInitialized == 1);
        if (flag) {
          simulation_DW.obj.TunablePropsChanged = true;
          simulation_DW.obj.tunablePropertyChanged[36] = true;
        }

        simulation_DW.obj.MagParamsTemperatureScaleFactor[0] = 0.0;
        simulation_DW.obj.MagParamsTemperatureScaleFactor[1] = 0.0;
        simulation_DW.obj.MagParamsTemperatureScaleFactor[2] = 0.0;
      }

      simulation_SystemCore_step(&simulation_DW.obj, rtb_ZeroOrderHold1,
        rtb_ZeroOrderHold2, rtb_ZeroOrderHold, simulation_Y.acceleration,
        simulation_Y.omega, simulation_Y.mag);

      /* End of MATLABSystem: '<S17>/IMU' */
    }

    /* Product: '<S56>/Product' */
    rtb_Product3 = simulation_B.OUTPUT_1_0[6] * simulation_B.OUTPUT_1_0[6];

    /* Product: '<S56>/Product1' */
    rtb_Product2 = simulation_B.OUTPUT_1_0[7] * simulation_B.OUTPUT_1_0[7];

    /* Product: '<S56>/Product2' */
    r = simulation_B.OUTPUT_1_0[8] * simulation_B.OUTPUT_1_0[8];

    /* Product: '<S56>/Product3' */
    rtb_fcn5 = simulation_B.OUTPUT_1_0[9] * simulation_B.OUTPUT_1_0[9];

    /* Sqrt: '<S55>/sqrt' incorporates:
     *  Sum: '<S56>/Sum'
     */
    rtb_Product3 = sqrt(((rtb_Product3 + rtb_Product2) + r) + rtb_fcn5);

    /* Product: '<S50>/Product' */
    rtb_fcn5 = simulation_B.OUTPUT_1_0[6] / rtb_Product3;

    /* Product: '<S50>/Product1' */
    r = simulation_B.OUTPUT_1_0[7] / rtb_Product3;

    /* Product: '<S50>/Product2' */
    rtb_Product2 = simulation_B.OUTPUT_1_0[8] / rtb_Product3;

    /* Product: '<S50>/Product3' */
    rtb_Product3 = simulation_B.OUTPUT_1_0[9] / rtb_Product3;

    /* Fcn: '<S46>/fcn3' */
    rtb_fcn3 = (r * rtb_Product3 + rtb_fcn5 * rtb_Product2) * 2.0;

    /* If: '<S51>/If' */
    if (rtsiIsModeUpdateTimeStep(&simulation_M->solverInfo)) {
      if (rtb_fcn3 > 1.0) {
        simulation_DW.If_ActiveSubsystem = 0;
      } else if (rtb_fcn3 < -1.0) {
        simulation_DW.If_ActiveSubsystem = 1;
      } else {
        simulation_DW.If_ActiveSubsystem = 2;
      }
    }

    switch (simulation_DW.If_ActiveSubsystem) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem' incorporates:
       *  ActionPort: '<S52>/Action Port'
       */
      if (rtmIsMajorTimeStep(simulation_M) &&
          simulation_M->Timing.TaskCounters.TID[1] == 0) {
        /* Merge: '<S51>/Merge' incorporates:
         *  Constant: '<S52>/Constant'
         */
        simulation_B.Merge = 1.0;
      }

      /* End of Outputs for SubSystem: '<S51>/If Action Subsystem' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S53>/Action Port'
       */
      if (rtmIsMajorTimeStep(simulation_M) &&
          simulation_M->Timing.TaskCounters.TID[1] == 0) {
        /* Merge: '<S51>/Merge' incorporates:
         *  Constant: '<S53>/Constant'
         */
        simulation_B.Merge = 1.0;
      }

      /* End of Outputs for SubSystem: '<S51>/If Action Subsystem1' */
      break;

     default:
      /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem2' incorporates:
       *  ActionPort: '<S54>/Action Port'
       */
      /* Merge: '<S51>/Merge' incorporates:
       *  SignalConversion generated from: '<S54>/In'
       */
      simulation_B.Merge = rtb_fcn3;

      /* End of Outputs for SubSystem: '<S51>/If Action Subsystem2' */
      break;
    }

    /* End of If: '<S51>/If' */

    /* Trigonometry: '<S49>/trigFcn' */
    if (simulation_B.Merge > 1.0) {
      rtb_fcn3 = 1.0;
    } else if (simulation_B.Merge < -1.0) {
      rtb_fcn3 = -1.0;
    } else {
      rtb_fcn3 = simulation_B.Merge;
    }

    /* Product: '<S36>/Divide' incorporates:
     *  Abs: '<S36>/Abs'
     *  Abs: '<S36>/Abs1'
     *  Fcn: '<S46>/fcn1'
     *  Fcn: '<S46>/fcn2'
     *  Product: '<S36>/multiply'
     *  Trigonometry: '<S36>/Cos'
     *  Trigonometry: '<S36>/Cos1'
     *  Trigonometry: '<S49>/Trigonometric Function1'
     *  Trigonometry: '<S49>/trigFcn'
     */
    rtb_Product3 = simulation_B.OUTPUT_1_0[4] / (cos(fabs(rt_atan2d_snf
      ((rtb_Product2 * rtb_Product3 - rtb_fcn5 * r) * -2.0, ((rtb_fcn5 *
      rtb_fcn5 - r * r) - rtb_Product2 * rtb_Product2) + rtb_Product3 *
       rtb_Product3))) * cos(fabs(asin(rtb_fcn3))));
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[2] == 0) {
      /* RandomNumber: '<S36>/Random Number' */
      simulation_B.RandomNumber = simulation_DW.NextOutput;
    }

    /* Outport: '<Root>/lidar' incorporates:
     *  Sum: '<S36>/Plus'
     */
    simulation_Y.lidar = simulation_B.RandomNumber + rtb_Product3;
  }

  if (rtmIsMajorTimeStep(simulation_M)) {
    NeslSimulationData *simulationData;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    char *msg;
    real_T tmp_0[20];
    real_T time;
    int32_T tmp_2;
    int_T tmp_1[6];
    boolean_T tmp;
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[1] == 0) {
      /* Update for UnitDelay: '<S62>/Unit Delay' */
      simulation_DW.UnitDelay_DSTATE = simulation_B.Switch;

      /* Update for UnitDelay: '<S62>/Unit Delay1' */
      simulation_DW.UnitDelay1_DSTATE = simulation_B.Switch1;

      /* Update for UnitDelay: '<S5>/Unit Delay' */
      simulation_DW.UnitDelay_DSTATE_m = simulation_B.thrust_j;

      /* Update for UnitDelay: '<S5>/Unit Delay1' */
      simulation_DW.UnitDelay1_DSTATE_j = simulation_B.thrust;
    }

    /* Update for SimscapeExecutionBlock: '<S69>/STATE_1' */
    simulationData = (NeslSimulationData *)simulation_DW.STATE_1_SimData;
    time = simulation_M->Timing.t[0];
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 26;
    simulationData->mData->mContStates.mX =
      &simulation_X.simulationPlantRocket_BodyFree_[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &simulation_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 1;
    simulationData->mData->mModeVector.mX = &simulation_DW.STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(simulation_M);
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp = rtsiIsSolverComputingJacobian(&simulation_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
      (&simulation_M->solverInfo);
    tmp_1[0] = 0;
    tmp_0[0] = simulation_B.INPUT_4_1_1[0];
    tmp_0[1] = simulation_B.INPUT_4_1_1[1];
    tmp_0[2] = simulation_B.INPUT_4_1_1[2];
    tmp_0[3] = simulation_B.INPUT_4_1_1[3];
    tmp_1[1] = 4;
    tmp_0[4] = simulation_B.INPUT_5_1_1[0];
    tmp_0[5] = simulation_B.INPUT_5_1_1[1];
    tmp_0[6] = simulation_B.INPUT_5_1_1[2];
    tmp_0[7] = simulation_B.INPUT_5_1_1[3];
    tmp_1[2] = 8;
    tmp_0[8] = simulation_B.INPUT_1_1_1[0];
    tmp_0[9] = simulation_B.INPUT_1_1_1[1];
    tmp_0[10] = simulation_B.INPUT_1_1_1[2];
    tmp_0[11] = simulation_B.INPUT_1_1_1[3];
    tmp_1[3] = 12;
    tmp_0[12] = simulation_B.INPUT_2_1_1[0];
    tmp_0[13] = simulation_B.INPUT_2_1_1[1];
    tmp_0[14] = simulation_B.INPUT_2_1_1[2];
    tmp_0[15] = simulation_B.INPUT_2_1_1[3];
    tmp_1[4] = 16;
    tmp_0[16] = simulation_B.INPUT_3_1_1[0];
    tmp_0[17] = simulation_B.INPUT_3_1_1[1];
    tmp_0[18] = simulation_B.INPUT_3_1_1[2];
    tmp_0[19] = simulation_B.INPUT_3_1_1[3];
    tmp_1[5] = 20;
    simulationData->mData->mInputValues.mN = 20;
    simulationData->mData->mInputValues.mX = &tmp_0[0];
    simulationData->mData->mInputOffsets.mN = 6;
    simulationData->mData->mInputOffsets.mX = &tmp_1[0];
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_2 = ne_simulator_method((NeslSimulator *)simulation_DW.STATE_1_Simulator,
      NESL_SIM_UPDATE, simulationData, diagnosticManager);
    if (tmp_2 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(simulation_M, msg);
      }
    }

    /* End of Update for SimscapeExecutionBlock: '<S69>/STATE_1' */
    if (rtmIsMajorTimeStep(simulation_M) &&
        simulation_M->Timing.TaskCounters.TID[2] == 0) {
      /* Update for RandomNumber: '<S36>/Random Number' */
      simulation_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
        (&simulation_DW.RandSeed) * 0.0;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(simulation_M)) {
    rt_ertODEUpdateContinuousStates(&simulation_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++simulation_M->Timing.clockTick0;
    simulation_M->Timing.t[0] = rtsiGetSolverStopTime(&simulation_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      simulation_M->Timing.clockTick1++;
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void simulation_derivatives(void)
{
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  XDot_simulation_T *_rtXdot;
  char *msg;
  real_T tmp_0[20];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[6];
  boolean_T tmp;
  _rtXdot = ((XDot_simulation_T *) simulation_M->derivs);

  /* Derivatives for SimscapeInputBlock: '<S69>/INPUT_4_1_1' */
  _rtXdot->simulationPlantTVC_PhysicsSimul[0] =
    simulation_X.simulationPlantTVC_PhysicsSimul[1];
  _rtXdot->simulationPlantTVC_PhysicsSimul[1] = ((simulation_B.Switch -
    simulation_X.simulationPlantTVC_PhysicsSimul[0]) * 20.0 - 2.0 *
    simulation_X.simulationPlantTVC_PhysicsSimul[1]) * 20.0;

  /* Derivatives for SimscapeInputBlock: '<S69>/INPUT_5_1_1' */
  _rtXdot->simulationPlantTVC_PhysicsSim_a[0] =
    simulation_X.simulationPlantTVC_PhysicsSim_a[1];
  _rtXdot->simulationPlantTVC_PhysicsSim_a[1] = ((simulation_B.Switch1 -
    simulation_X.simulationPlantTVC_PhysicsSim_a[0]) * 20.0 - 2.0 *
    simulation_X.simulationPlantTVC_PhysicsSim_a[1]) * 20.0;

  /* Derivatives for SimscapeExecutionBlock: '<S69>/STATE_1' */
  simulationData = (NeslSimulationData *)simulation_DW.STATE_1_SimData;
  time = simulation_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 26;
  simulationData->mData->mContStates.mX =
    &simulation_X.simulationPlantRocket_BodyFree_[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &simulation_DW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 1;
  simulationData->mData->mModeVector.mX = &simulation_DW.STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(simulation_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&simulation_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
    (&simulation_M->solverInfo);
  tmp_1[0] = 0;
  tmp_0[0] = simulation_B.INPUT_4_1_1[0];
  tmp_0[1] = simulation_B.INPUT_4_1_1[1];
  tmp_0[2] = simulation_B.INPUT_4_1_1[2];
  tmp_0[3] = simulation_B.INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = simulation_B.INPUT_5_1_1[0];
  tmp_0[5] = simulation_B.INPUT_5_1_1[1];
  tmp_0[6] = simulation_B.INPUT_5_1_1[2];
  tmp_0[7] = simulation_B.INPUT_5_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = simulation_B.INPUT_1_1_1[0];
  tmp_0[9] = simulation_B.INPUT_1_1_1[1];
  tmp_0[10] = simulation_B.INPUT_1_1_1[2];
  tmp_0[11] = simulation_B.INPUT_1_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = simulation_B.INPUT_2_1_1[0];
  tmp_0[13] = simulation_B.INPUT_2_1_1[1];
  tmp_0[14] = simulation_B.INPUT_2_1_1[2];
  tmp_0[15] = simulation_B.INPUT_2_1_1[3];
  tmp_1[4] = 16;
  tmp_0[16] = simulation_B.INPUT_3_1_1[0];
  tmp_0[17] = simulation_B.INPUT_3_1_1[1];
  tmp_0[18] = simulation_B.INPUT_3_1_1[2];
  tmp_0[19] = simulation_B.INPUT_3_1_1[3];
  tmp_1[5] = 20;
  simulationData->mData->mInputValues.mN = 20;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 6;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  simulationData->mData->mDx.mN = 26;
  simulationData->mData->mDx.mX = &_rtXdot->simulationPlantRocket_BodyFree_[0];
  diagnosticManager = (NeuDiagnosticManager *)simulation_DW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)simulation_DW.STATE_1_Simulator,
    NESL_SIM_DERIVATIVES, simulationData, diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(simulation_M, msg);
    }
  }

  /* End of Derivatives for SimscapeExecutionBlock: '<S69>/STATE_1' */
}

/* Model initialize function */
void simulation_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&simulation_M->solverInfo,
                          &simulation_M->Timing.simTimeStep);
    rtsiSetTPtr(&simulation_M->solverInfo, &rtmGetTPtr(simulation_M));
    rtsiSetStepSizePtr(&simulation_M->solverInfo,
                       &simulation_M->Timing.stepSize0);
    rtsiSetdXPtr(&simulation_M->solverInfo, &simulation_M->derivs);
    rtsiSetContStatesPtr(&simulation_M->solverInfo, (real_T **)
                         &simulation_M->contStates);
    rtsiSetNumContStatesPtr(&simulation_M->solverInfo,
      &simulation_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&simulation_M->solverInfo,
      &simulation_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&simulation_M->solverInfo,
      &simulation_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&simulation_M->solverInfo,
      &simulation_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&simulation_M->solverInfo, (boolean_T**)
      &simulation_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&simulation_M->solverInfo, (&rtmGetErrorStatus
      (simulation_M)));
    rtsiSetRTModelPtr(&simulation_M->solverInfo, simulation_M);
  }

  rtsiSetSimTimeStep(&simulation_M->solverInfo, MAJOR_TIME_STEP);
  simulation_M->intgData.y = simulation_M->odeY;
  simulation_M->intgData.f[0] = simulation_M->odeF[0];
  simulation_M->intgData.f[1] = simulation_M->odeF[1];
  simulation_M->intgData.f[2] = simulation_M->odeF[2];
  simulation_M->contStates = ((X_simulation_T *) &simulation_X);
  simulation_M->contStateDisabled = ((XDis_simulation_T *) &simulation_XDis);
  simulation_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&simulation_M->solverInfo, (void *)&simulation_M->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&simulation_M->solverInfo, false);
  rtsiSetSolverName(&simulation_M->solverInfo,"ode3");
  rtmSetTPtr(simulation_M, &simulation_M->Timing.tArray[0]);
  simulation_M->Timing.stepSize0 = 0.001;

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  simulation_InitializeDataMapInfo();

  {
    NeModelParameters modelParameters;
    NeModelParameters modelParameters_0;
    NeModelParameters modelParameters_1;
    NeslSimulationData *tmp_0;
    NeslSimulator *tmp;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    NeuDiagnosticTree *diagnosticTree_0;
    NeuDiagnosticTree *diagnosticTree_1;
    char *msg;
    char *msg_0;
    char *msg_1;
    real_T tmp_1;
    int32_T i;
    boolean_T flag;

    /* Start for SimscapeExecutionBlock: '<S69>/STATE_1' */
    tmp = nesl_lease_simulator(
      "simulation/Plant/ World Setup/Solver Configuration_1", 0, 0);
    simulation_DW.STATE_1_Simulator = (void *)tmp;
    flag = pointer_is_null(simulation_DW.STATE_1_Simulator);
    if (flag) {
      simulation_b048d748_1_gateway();
      tmp = nesl_lease_simulator(
        "simulation/Plant/ World Setup/Solver Configuration_1", 0, 0);
      simulation_DW.STATE_1_Simulator = (void *)tmp;
    }

    tmp_0 = nesl_create_simulation_data();
    simulation_DW.STATE_1_SimData = (void *)tmp_0;
    diagnosticManager = rtw_create_diagnostics();
    simulation_DW.STATE_1_DiagMgr = (void *)diagnosticManager;
    modelParameters.mSolverType = NE_SOLVER_TYPE_DAE;
    modelParameters.mSolverAbsTol = 0.001;
    modelParameters.mSolverRelTol = 0.001;
    modelParameters.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters.mStartTime = 0.0;
    modelParameters.mLoadInitialState = false;
    modelParameters.mUseSimState = false;
    modelParameters.mLinTrimCompile = false;
    modelParameters.mLoggingMode = SSC_LOGGING_OFF;
    modelParameters.mRTWModifiedTimeStamp = 6.21297144E+8;
    modelParameters.mUseModelRefSolver = false;
    modelParameters.mTargetFPGAHIL = false;
    tmp_1 = 0.001;
    modelParameters.mSolverTolerance = tmp_1;
    tmp_1 = 0.001;
    modelParameters.mFixedStepSize = tmp_1;
    flag = false;
    modelParameters.mVariableStepSolver = flag;
    flag = false;
    modelParameters.mIsUsingODEN = flag;
    modelParameters.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      simulation_DW.STATE_1_Simulator, &modelParameters, diagnosticManager);
    if (i != 0) {
      flag = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (flag) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(simulation_M, msg);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S69>/STATE_1' */

    /* Start for SimscapeExecutionBlock: '<S69>/OUTPUT_1_0' */
    tmp = nesl_lease_simulator(
      "simulation/Plant/ World Setup/Solver Configuration_1", 1, 0);
    simulation_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    flag = pointer_is_null(simulation_DW.OUTPUT_1_0_Simulator);
    if (flag) {
      simulation_b048d748_1_gateway();
      tmp = nesl_lease_simulator(
        "simulation/Plant/ World Setup/Solver Configuration_1", 1, 0);
      simulation_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    }

    tmp_0 = nesl_create_simulation_data();
    simulation_DW.OUTPUT_1_0_SimData = (void *)tmp_0;
    diagnosticManager = rtw_create_diagnostics();
    simulation_DW.OUTPUT_1_0_DiagMgr = (void *)diagnosticManager;
    modelParameters_0.mSolverType = NE_SOLVER_TYPE_DAE;
    modelParameters_0.mSolverAbsTol = 0.001;
    modelParameters_0.mSolverRelTol = 0.001;
    modelParameters_0.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters_0.mStartTime = 0.0;
    modelParameters_0.mLoadInitialState = false;
    modelParameters_0.mUseSimState = false;
    modelParameters_0.mLinTrimCompile = false;
    modelParameters_0.mLoggingMode = SSC_LOGGING_OFF;
    modelParameters_0.mRTWModifiedTimeStamp = 6.21297144E+8;
    modelParameters_0.mUseModelRefSolver = false;
    modelParameters_0.mTargetFPGAHIL = false;
    tmp_1 = 0.001;
    modelParameters_0.mSolverTolerance = tmp_1;
    tmp_1 = 0.001;
    modelParameters_0.mFixedStepSize = tmp_1;
    flag = false;
    modelParameters_0.mVariableStepSolver = flag;
    flag = false;
    modelParameters_0.mIsUsingODEN = flag;
    modelParameters_0.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      simulation_DW.OUTPUT_1_0_Simulator, &modelParameters_0, diagnosticManager);
    if (i != 0) {
      flag = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (flag) {
        msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
        rtmSetErrorStatus(simulation_M, msg_0);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S69>/OUTPUT_1_0' */

    /* Start for SimscapeExecutionBlock: '<S69>/OUTPUT_1_1' */
    tmp = nesl_lease_simulator(
      "simulation/Plant/ World Setup/Solver Configuration_1", 1, 1);
    simulation_DW.OUTPUT_1_1_Simulator = (void *)tmp;
    flag = pointer_is_null(simulation_DW.OUTPUT_1_1_Simulator);
    if (flag) {
      simulation_b048d748_1_gateway();
      tmp = nesl_lease_simulator(
        "simulation/Plant/ World Setup/Solver Configuration_1", 1, 1);
      simulation_DW.OUTPUT_1_1_Simulator = (void *)tmp;
    }

    tmp_0 = nesl_create_simulation_data();
    simulation_DW.OUTPUT_1_1_SimData = (void *)tmp_0;
    diagnosticManager = rtw_create_diagnostics();
    simulation_DW.OUTPUT_1_1_DiagMgr = (void *)diagnosticManager;
    modelParameters_1.mSolverType = NE_SOLVER_TYPE_DAE;
    modelParameters_1.mSolverAbsTol = 0.001;
    modelParameters_1.mSolverRelTol = 0.001;
    modelParameters_1.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters_1.mStartTime = 0.0;
    modelParameters_1.mLoadInitialState = false;
    modelParameters_1.mUseSimState = false;
    modelParameters_1.mLinTrimCompile = false;
    modelParameters_1.mLoggingMode = SSC_LOGGING_OFF;
    modelParameters_1.mRTWModifiedTimeStamp = 6.21297144E+8;
    modelParameters_1.mUseModelRefSolver = false;
    modelParameters_1.mTargetFPGAHIL = false;
    tmp_1 = 0.001;
    modelParameters_1.mSolverTolerance = tmp_1;
    tmp_1 = 0.001;
    modelParameters_1.mFixedStepSize = tmp_1;
    flag = false;
    modelParameters_1.mVariableStepSolver = flag;
    flag = false;
    modelParameters_1.mIsUsingODEN = flag;
    modelParameters_1.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)simulation_DW.OUTPUT_1_1_DiagMgr;
    diagnosticTree_1 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      simulation_DW.OUTPUT_1_1_Simulator, &modelParameters_1, diagnosticManager);
    if (i != 0) {
      flag = error_buffer_is_empty(rtmGetErrorStatus(simulation_M));
      if (flag) {
        msg_1 = rtw_diagnostics_msg(diagnosticTree_1);
        rtmSetErrorStatus(simulation_M, msg_1);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S69>/OUTPUT_1_1' */

    /* Start for If: '<S51>/If' */
    simulation_DW.If_ActiveSubsystem = -1;

    /* InitializeConditions for RandomNumber: '<S36>/Random Number' */
    simulation_DW.RandSeed = 597884928U;
    simulation_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&simulation_DW.RandSeed) * 0.0;

    /* Start for MATLABSystem: '<S17>/IMU' */
    simulation_DW.obj.MagneticFieldNED[0] = 27.555;
    simulation_DW.obj.MagneticFieldNED[1] = -2.4169;
    simulation_DW.obj.MagneticFieldNED[2] = -16.0849;
    simulation_DW.obj.isInitialized = 0;
    for (i = 0; i < 38; i++) {
      simulation_DW.obj.tunablePropertyChanged[i] = false;
    }

    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[2] = true;
    }

    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[37] = true;
    }

    simulation_DW.obj.Temperature = 25.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[0] = true;
    }

    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[2] = true;
    }

    simulation_DW.obj.MagneticField[0] = 27.555;
    simulation_DW.obj.MagneticField[1] = -2.4169;
    simulation_DW.obj.MagneticField[2] = -16.0849;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[3] = true;
    }

    simulation_DW.obj.AccelParamsMeasurementRange = (rtInf);
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[4] = true;
    }

    simulation_DW.obj.AccelParamsResolution = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[5] = true;
    }

    simulation_DW.obj.AccelParamsConstantBias[0] = 0.0;
    simulation_DW.obj.AccelParamsConstantBias[1] = 0.0;
    simulation_DW.obj.AccelParamsConstantBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[6] = true;
    }

    memcpy(&simulation_DW.obj.AccelParamsAxesMisalignment[0],
           &simulation_ConstP.pooled4[0], 9U * sizeof(real_T));
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[7] = true;
    }

    simulation_DW.obj.AccelParamsNoiseDensity[0] = 0.0;
    simulation_DW.obj.AccelParamsNoiseDensity[1] = 0.0;
    simulation_DW.obj.AccelParamsNoiseDensity[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[8] = true;
    }

    simulation_DW.obj.AccelParamsBiasInstability[0] = 0.0;
    simulation_DW.obj.AccelParamsBiasInstability[1] = 0.0;
    simulation_DW.obj.AccelParamsBiasInstability[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[9] = true;
    }

    simulation_DW.obj.AccelParamsBiasInstabilityNumerator = 1.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[10] = true;
    }

    simulation_DW.obj.AccelParamsBiasInstabilityDenominator[0] = 1.0;
    simulation_DW.obj.AccelParamsBiasInstabilityDenominator[1] = -0.5;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[11] = true;
    }

    simulation_DW.obj.AccelParamsRandomWalk[0] = 0.0;
    simulation_DW.obj.AccelParamsRandomWalk[1] = 0.0;
    simulation_DW.obj.AccelParamsRandomWalk[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[12] = true;
    }

    simulation_DW.obj.AccelParamsTemperatureBias[0] = 0.0;
    simulation_DW.obj.AccelParamsTemperatureBias[1] = 0.0;
    simulation_DW.obj.AccelParamsTemperatureBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[13] = true;
    }

    simulation_DW.obj.AccelParamsTemperatureScaleFactor[0] = 0.0;
    simulation_DW.obj.AccelParamsTemperatureScaleFactor[1] = 0.0;
    simulation_DW.obj.AccelParamsTemperatureScaleFactor[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[14] = true;
    }

    simulation_DW.obj.GyroParamsMeasurementRange = (rtInf);
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[15] = true;
    }

    simulation_DW.obj.GyroParamsResolution = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[16] = true;
    }

    simulation_DW.obj.GyroParamsConstantBias[0] = 0.0;
    simulation_DW.obj.GyroParamsConstantBias[1] = 0.0;
    simulation_DW.obj.GyroParamsConstantBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[17] = true;
    }

    memcpy(&simulation_DW.obj.GyroParamsAxesMisalignment[0],
           &simulation_ConstP.pooled4[0], 9U * sizeof(real_T));
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[25] = true;
    }

    simulation_DW.obj.GyroParamsAccelerationBias[0] = 0.0;
    simulation_DW.obj.GyroParamsAccelerationBias[1] = 0.0;
    simulation_DW.obj.GyroParamsAccelerationBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[18] = true;
    }

    simulation_DW.obj.GyroParamsNoiseDensity[0] = 0.0;
    simulation_DW.obj.GyroParamsNoiseDensity[1] = 0.0;
    simulation_DW.obj.GyroParamsNoiseDensity[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[19] = true;
    }

    simulation_DW.obj.GyroParamsBiasInstability[0] = 0.0;
    simulation_DW.obj.GyroParamsBiasInstability[1] = 0.0;
    simulation_DW.obj.GyroParamsBiasInstability[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[20] = true;
    }

    simulation_DW.obj.GyroParamsBiasInstabilityNumerator = 1.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[21] = true;
    }

    simulation_DW.obj.GyroParamsBiasInstabilityDenominator[0] = 1.0;
    simulation_DW.obj.GyroParamsBiasInstabilityDenominator[1] = -0.5;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[22] = true;
    }

    simulation_DW.obj.GyroParamsRandomWalk[0] = 0.0;
    simulation_DW.obj.GyroParamsRandomWalk[1] = 0.0;
    simulation_DW.obj.GyroParamsRandomWalk[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[23] = true;
    }

    simulation_DW.obj.GyroParamsTemperatureBias[0] = 0.0;
    simulation_DW.obj.GyroParamsTemperatureBias[1] = 0.0;
    simulation_DW.obj.GyroParamsTemperatureBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[24] = true;
    }

    simulation_DW.obj.GyroParamsTemperatureScaleFactor[0] = 0.0;
    simulation_DW.obj.GyroParamsTemperatureScaleFactor[1] = 0.0;
    simulation_DW.obj.GyroParamsTemperatureScaleFactor[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[26] = true;
    }

    simulation_DW.obj.MagParamsMeasurementRange = (rtInf);
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[27] = true;
    }

    simulation_DW.obj.MagParamsResolution = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[28] = true;
    }

    simulation_DW.obj.MagParamsConstantBias[0] = 0.0;
    simulation_DW.obj.MagParamsConstantBias[1] = 0.0;
    simulation_DW.obj.MagParamsConstantBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[29] = true;
    }

    memcpy(&simulation_DW.obj.MagParamsAxesMisalignment[0],
           &simulation_ConstP.pooled4[0], 9U * sizeof(real_T));
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[30] = true;
    }

    simulation_DW.obj.MagParamsNoiseDensity[0] = 0.0;
    simulation_DW.obj.MagParamsNoiseDensity[1] = 0.0;
    simulation_DW.obj.MagParamsNoiseDensity[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[31] = true;
    }

    simulation_DW.obj.MagParamsBiasInstability[0] = 0.0;
    simulation_DW.obj.MagParamsBiasInstability[1] = 0.0;
    simulation_DW.obj.MagParamsBiasInstability[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[32] = true;
    }

    simulation_DW.obj.MagParamsBiasInstabilityNumerator = 1.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[33] = true;
    }

    simulation_DW.obj.MagParamsBiasInstabilityDenominator[0] = 1.0;
    simulation_DW.obj.MagParamsBiasInstabilityDenominator[1] = -0.5;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[34] = true;
    }

    simulation_DW.obj.MagParamsRandomWalk[0] = 0.0;
    simulation_DW.obj.MagParamsRandomWalk[1] = 0.0;
    simulation_DW.obj.MagParamsRandomWalk[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[35] = true;
    }

    simulation_DW.obj.MagParamsTemperatureBias[0] = 0.0;
    simulation_DW.obj.MagParamsTemperatureBias[1] = 0.0;
    simulation_DW.obj.MagParamsTemperatureBias[2] = 0.0;
    flag = (simulation_DW.obj.isInitialized == 1);
    if (flag) {
      simulation_DW.obj.TunablePropsChanged = true;
      simulation_DW.obj.tunablePropertyChanged[36] = true;
    }

    simulation_DW.obj.MagParamsTemperatureScaleFactor[0] = 0.0;
    simulation_DW.obj.MagParamsTemperatureScaleFactor[1] = 0.0;
    simulation_DW.obj.MagParamsTemperatureScaleFactor[2] = 0.0;
    simulation_SystemCore_setup(&simulation_DW.obj);

    /* InitializeConditions for MATLABSystem: '<S17>/IMU' */
    simulat_IMUSensorBase_resetImpl(&simulation_DW.obj);
  }
}

/* Model terminate function */
void simulation_terminate(void)
{
  /* Terminate for SimscapeExecutionBlock: '<S69>/STATE_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    simulation_DW.STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    simulation_DW.STATE_1_SimData);
  nesl_erase_simulator("simulation/Plant/ World Setup/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for SimscapeExecutionBlock: '<S69>/OUTPUT_1_0' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    simulation_DW.OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    simulation_DW.OUTPUT_1_0_SimData);
  nesl_erase_simulator("simulation/Plant/ World Setup/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for SimscapeExecutionBlock: '<S69>/OUTPUT_1_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    simulation_DW.OUTPUT_1_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    simulation_DW.OUTPUT_1_1_SimData);
  nesl_erase_simulator("simulation/Plant/ World Setup/Solver Configuration_1");
  nesl_destroy_registry();
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
