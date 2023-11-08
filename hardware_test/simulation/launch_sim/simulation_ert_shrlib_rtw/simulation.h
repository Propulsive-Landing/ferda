/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: simulation.h
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

#ifndef RTW_HEADER_simulation_h_
#define RTW_HEADER_simulation_h_
#ifndef simulation_COMMON_INCLUDES_
#define simulation_COMMON_INCLUDES_
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "nesl_rtw.h"
#include "simulation_b048d748_1_gateway.h"
#endif                                 /* simulation_COMMON_INCLUDES_ */

#include "simulation_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rtw_modelmap.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm)         ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val)    ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T UnitDelay;                    /* '<S62>/Unit Delay' */
  real_T Switch;                       /* '<S62>/Switch' */
  real_T INPUT_4_1_1[4];               /* '<S69>/INPUT_4_1_1' */
  real_T UnitDelay1;                   /* '<S62>/Unit Delay1' */
  real_T Switch1;                      /* '<S62>/Switch1' */
  real_T INPUT_5_1_1[4];               /* '<S69>/INPUT_5_1_1' */
  real_T INPUT_1_1_1[4];               /* '<S69>/INPUT_1_1_1' */
  real_T STATE_1[27];                  /* '<S69>/STATE_1' */
  real_T INPUT_2_1_1[4];               /* '<S69>/INPUT_2_1_1' */
  real_T INPUT_3_1_1[4];               /* '<S69>/INPUT_3_1_1' */
  real_T OUTPUT_1_0[18];               /* '<S69>/OUTPUT_1_0' */
  real_T OUTPUT_1_1[6];                /* '<S69>/OUTPUT_1_1' */
  real_T RandomNumber;                 /* '<S36>/Random Number' */
  real_T thrust;                       /* '<S5>/stage2_thrust_calculate' */
  real_T thrust_j;                     /* '<S5>/stage1_thrust_calculate' */
  real_T attached;                     /* '<S5>/Discretizer' */
  real_T s1_et;                        /* '<S5>/Discretizer' */
  real_T s2_et;                        /* '<S5>/Discretizer' */
  real_T Merge;                        /* '<S51>/Merge' */
} B_simulation_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  fusion_simulink_imuSensor_sim_T obj; /* '<S17>/IMU' */
  real_T UnitDelay_DSTATE;             /* '<S62>/Unit Delay' */
  real_T INPUT_4_1_1_Discrete;         /* '<S69>/INPUT_4_1_1' */
  real_T INPUT_4_1_1_FirstOutput;      /* '<S69>/INPUT_4_1_1' */
  real_T UnitDelay1_DSTATE;            /* '<S62>/Unit Delay1' */
  real_T INPUT_5_1_1_Discrete;         /* '<S69>/INPUT_5_1_1' */
  real_T INPUT_5_1_1_FirstOutput;      /* '<S69>/INPUT_5_1_1' */
  real_T UnitDelay_DSTATE_m;           /* '<S5>/Unit Delay' */
  real_T UnitDelay1_DSTATE_j;          /* '<S5>/Unit Delay1' */
  real_T INPUT_1_1_1_Discrete[2];      /* '<S69>/INPUT_1_1_1' */
  real_T INPUT_2_1_1_Discrete[2];      /* '<S69>/INPUT_2_1_1' */
  real_T INPUT_3_1_1_Discrete[2];      /* '<S69>/INPUT_3_1_1' */
  real_T STATE_1_Discrete;             /* '<S69>/STATE_1' */
  real_T OUTPUT_1_0_Discrete;          /* '<S69>/OUTPUT_1_0' */
  real_T OUTPUT_1_1_Discrete;          /* '<S69>/OUTPUT_1_1' */
  real_T NextOutput;                   /* '<S36>/Random Number' */
  void* STATE_1_Simulator;             /* '<S69>/STATE_1' */
  void* STATE_1_SimData;               /* '<S69>/STATE_1' */
  void* STATE_1_DiagMgr;               /* '<S69>/STATE_1' */
  void* STATE_1_ZcLogger;              /* '<S69>/STATE_1' */
  void* STATE_1_TsInfo;                /* '<S69>/STATE_1' */
  void* SINK_1_RtwLogger;              /* '<S69>/SINK_1' */
  void* SINK_1_RtwLogBuffer;           /* '<S69>/SINK_1' */
  void* SINK_1_RtwLogFcnManager;       /* '<S69>/SINK_1' */
  void* OUTPUT_1_0_Simulator;          /* '<S69>/OUTPUT_1_0' */
  void* OUTPUT_1_0_SimData;            /* '<S69>/OUTPUT_1_0' */
  void* OUTPUT_1_0_DiagMgr;            /* '<S69>/OUTPUT_1_0' */
  void* OUTPUT_1_0_ZcLogger;           /* '<S69>/OUTPUT_1_0' */
  void* OUTPUT_1_0_TsInfo;             /* '<S69>/OUTPUT_1_0' */
  void* OUTPUT_1_1_Simulator;          /* '<S69>/OUTPUT_1_1' */
  void* OUTPUT_1_1_SimData;            /* '<S69>/OUTPUT_1_1' */
  void* OUTPUT_1_1_DiagMgr;            /* '<S69>/OUTPUT_1_1' */
  void* OUTPUT_1_1_ZcLogger;           /* '<S69>/OUTPUT_1_1' */
  void* OUTPUT_1_1_TsInfo;             /* '<S69>/OUTPUT_1_1' */
  uint32_T RandSeed;                   /* '<S36>/Random Number' */
  uint32_T temporalCounter_i1;         /* '<S5>/Discretizer' */
  int_T STATE_1_Modes;                 /* '<S69>/STATE_1' */
  int_T OUTPUT_1_0_Modes;              /* '<S69>/OUTPUT_1_0' */
  int_T OUTPUT_1_1_Modes;              /* '<S69>/OUTPUT_1_1' */
  int8_T If_ActiveSubsystem;           /* '<S51>/If' */
  uint8_T is_active_c3_simulation;     /* '<S5>/Discretizer' */
  uint8_T is_c3_simulation;            /* '<S5>/Discretizer' */
  uint8_T is_S1_fire;                  /* '<S5>/Discretizer' */
  boolean_T STATE_1_FirstOutput;       /* '<S69>/STATE_1' */
  boolean_T OUTPUT_1_0_FirstOutput;    /* '<S69>/OUTPUT_1_0' */
  boolean_T OUTPUT_1_1_FirstOutput;    /* '<S69>/OUTPUT_1_1' */
} DW_simulation_T;

/* Continuous states (default storage) */
typedef struct {
  real_T simulationPlantTVC_PhysicsSimul[2];/* '<S69>/INPUT_4_1_1' */
  real_T simulationPlantTVC_PhysicsSim_a[2];/* '<S69>/INPUT_5_1_1' */
  real_T simulationPlantRocket_BodyFree_[26];/* '<S69>/STATE_1' */
} X_simulation_T;

/* State derivatives (default storage) */
typedef struct {
  real_T simulationPlantTVC_PhysicsSimul[2];/* '<S69>/INPUT_4_1_1' */
  real_T simulationPlantTVC_PhysicsSim_a[2];/* '<S69>/INPUT_5_1_1' */
  real_T simulationPlantRocket_BodyFree_[26];/* '<S69>/STATE_1' */
} XDot_simulation_T;

/* State disabled  */
typedef struct {
  boolean_T simulationPlantTVC_PhysicsSimul[2];/* '<S69>/INPUT_4_1_1' */
  boolean_T simulationPlantTVC_PhysicsSim_a[2];/* '<S69>/INPUT_5_1_1' */
  boolean_T simulationPlantRocket_BodyFree_[26];/* '<S69>/STATE_1' */
} XDis_simulation_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [ 27.5550, -2.4169, -16.0849 ]
   * Referenced by: '<S17>/IMU'
   */
  real_T IMU_MagneticFieldNED[3];

  /* Pooled Parameter (Expression: [ 0, 0, 0 ])
   * Referenced by: '<S17>/IMU'
   */
  real_T pooled3[3];

  /* Pooled Parameter (Expression: [ 100, 0, 0; 0, 100, 0; 0, 0, 100 ])
   * Referenced by: '<S17>/IMU'
   */
  real_T pooled4[9];

  /* Pooled Parameter (Expression: fractalcoef().Denominator)
   * Referenced by: '<S17>/IMU'
   */
  real_T pooled6[2];

  /* Pooled Parameter (Expression: stage1_series)
   * Referenced by:
   *   '<S5>/series'
   *   '<S5>/series1'
   */
  real_T pooled7[12];
} ConstP_simulation_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T tvcxangle;                    /* '<Root>/tvc x angle' */
  real_T tvcyangle;                    /* '<Root>/tvc y angle' */
  real_T ignite_s1;                    /* '<Root>/ignite_s1' */
  real_T ignite_s2;                    /* '<Root>/ignite_s2' */
} ExtU_simulation_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T lidar;                        /* '<Root>/lidar' */
  real_T omega[3];                     /* '<Root>/omega' */
  real_T acceleration[3];              /* '<Root>/acceleration' */
  real_T mag[3];                       /* '<Root>/mag' */
  real_T truestate[14];                /* '<Root>/true state' */
} ExtY_simulation_T;

/* Real-time Model Data Structure */
struct tag_RTM_simulation_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_simulation_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_simulation_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[30];
  real_T odeF[3][30];
  ODE3_IntgData intgData;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
  } DataMapInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Block signals (default storage) */
extern B_simulation_T simulation_B;

/* Continuous states (default storage) */
extern X_simulation_T simulation_X;

/* Disabled states (default storage) */
extern XDis_simulation_T simulation_XDis;

/* Block states (default storage) */
extern DW_simulation_T simulation_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_simulation_T simulation_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_simulation_T simulation_Y;

/* Constant parameters (default storage) */
extern const ConstP_simulation_T simulation_ConstP;

/* Model entry point functions */
extern void simulation_initialize(void);
extern void simulation_step(void);
extern void simulation_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  simulation_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_simulation_T *const simulation_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S25>/RESHAPE' : Reshape block reduction
 * Block '<S43>/RESHAPE' : Reshape block reduction
 * Block '<S17>/Reshape' : Reshape block reduction
 * Block '<S17>/Reshape1' : Reshape block reduction
 * Block '<S17>/Reshape2' : Reshape block reduction
 * Block '<S47>/RESHAPE' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'simulation'
 * '<S1>'   : 'simulation/Plant'
 * '<S2>'   : 'simulation/Plant/ Rocket Body'
 * '<S3>'   : 'simulation/Plant/ TVC Physics'
 * '<S4>'   : 'simulation/Plant/ World Setup'
 * '<S5>'   : 'simulation/Plant/Thrust Management'
 * '<S6>'   : 'simulation/Plant/ Rocket Body/PS-Simulink Converter'
 * '<S7>'   : 'simulation/Plant/ Rocket Body/PS-Simulink Converter1'
 * '<S8>'   : 'simulation/Plant/ Rocket Body/PS-Simulink Converter10'
 * '<S9>'   : 'simulation/Plant/ Rocket Body/PS-Simulink Converter2'
 * '<S10>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter3'
 * '<S11>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter4'
 * '<S12>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter5'
 * '<S13>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter6'
 * '<S14>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter7'
 * '<S15>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter8'
 * '<S16>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter9'
 * '<S17>'  : 'simulation/Plant/ Rocket Body/Subsystem'
 * '<S18>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter/EVAL_KEY'
 * '<S19>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter1/EVAL_KEY'
 * '<S20>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter10/EVAL_KEY'
 * '<S21>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter2/EVAL_KEY'
 * '<S22>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter3/EVAL_KEY'
 * '<S23>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter4/EVAL_KEY'
 * '<S24>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter5/EVAL_KEY'
 * '<S25>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter6/EVAL_KEY'
 * '<S26>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter7/EVAL_KEY'
 * '<S27>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter8/EVAL_KEY'
 * '<S28>'  : 'simulation/Plant/ Rocket Body/PS-Simulink Converter9/EVAL_KEY'
 * '<S29>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter'
 * '<S30>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter1'
 * '<S31>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter2'
 * '<S32>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter3'
 * '<S33>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter4'
 * '<S34>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter5'
 * '<S35>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter6'
 * '<S36>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation'
 * '<S37>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter/EVAL_KEY'
 * '<S38>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter1/EVAL_KEY'
 * '<S39>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter2/EVAL_KEY'
 * '<S40>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter3/EVAL_KEY'
 * '<S41>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter4/EVAL_KEY'
 * '<S42>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter5/EVAL_KEY'
 * '<S43>'  : 'simulation/Plant/ Rocket Body/Subsystem/PS-Simulink Converter6/EVAL_KEY'
 * '<S44>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/PS-Simulink Converter'
 * '<S45>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/PS-Simulink Converter1'
 * '<S46>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles'
 * '<S47>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/PS-Simulink Converter/EVAL_KEY'
 * '<S48>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/PS-Simulink Converter1/EVAL_KEY'
 * '<S49>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation'
 * '<S50>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Quaternion Normalize'
 * '<S51>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
 * '<S52>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
 * '<S53>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
 * '<S54>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
 * '<S55>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
 * '<S56>'  : 'simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S57>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter'
 * '<S58>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter1'
 * '<S59>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter2'
 * '<S60>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter4'
 * '<S61>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter5'
 * '<S62>'  : 'simulation/Plant/ TVC Physics/rotation limiter'
 * '<S63>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter/EVAL_KEY'
 * '<S64>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter1/EVAL_KEY'
 * '<S65>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter2/EVAL_KEY'
 * '<S66>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter4/EVAL_KEY'
 * '<S67>'  : 'simulation/Plant/ TVC Physics/Simulink-PS Converter5/EVAL_KEY'
 * '<S68>'  : 'simulation/Plant/ World Setup/Solver Configuration'
 * '<S69>'  : 'simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY'
 * '<S70>'  : 'simulation/Plant/Thrust Management/Discretizer'
 * '<S71>'  : 'simulation/Plant/Thrust Management/stage1_thrust_calculate'
 * '<S72>'  : 'simulation/Plant/Thrust Management/stage2_thrust_calculate'
 */
#endif                                 /* RTW_HEADER_simulation_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
