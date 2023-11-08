/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: simulation_capi.c
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

#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "simulation_capi_host.h"
#define sizeof(s)                      ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#ifndef SS_UINT64
#define SS_UINT64                      18
#endif

#ifndef SS_INT64
#define SS_INT64                       19
#endif

#else                                  /* HOST_CAPI_BUILD */
#include "builtin_typeid_types.h"
#include "simulation.h"
#include "simulation_capi.h"
#include "simulation_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif                                 /* HOST_CAPI_BUILD */

/* Block output signal information */
static const rtwCAPI_Signals rtBlockSignals[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 0, 5, TARGET_STRING("simulation/Plant/Thrust Management/Discretizer"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 1, 5, TARGET_STRING("simulation/Plant/Thrust Management/Discretizer"),
    TARGET_STRING(""), 1, 0, 0, 0, 0 },

  { 2, 5, TARGET_STRING("simulation/Plant/Thrust Management/Discretizer"),
    TARGET_STRING(""), 2, 0, 0, 0, 0 },

  { 3, 6, TARGET_STRING("simulation/Plant/Thrust Management/stage1_thrust_calculate"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 4, 7, TARGET_STRING("simulation/Plant/Thrust Management/stage2_thrust_calculate"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 5, 0, TARGET_STRING("simulation/Plant/ TVC Physics/rotation limiter/Switch"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 6, 0, TARGET_STRING("simulation/Plant/ TVC Physics/rotation limiter/Switch1"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 7, 0, TARGET_STRING("simulation/Plant/ TVC Physics/rotation limiter/Unit Delay"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 8, 0, TARGET_STRING("simulation/Plant/ TVC Physics/rotation limiter/Unit Delay1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 9, 0, TARGET_STRING("simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Random Number"),
    TARGET_STRING(""), 0, 0, 0, 0, 2 },

  { 10, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/INPUT_1_1_1"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 11, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/INPUT_2_1_1"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 12, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/INPUT_3_1_1"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 13, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/INPUT_4_1_1"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 14, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/INPUT_5_1_1"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 15, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/OUTPUT_1_0"),
    TARGET_STRING(""), 0, 0, 2, 0, 1 },

  { 16, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/OUTPUT_1_1"),
    TARGET_STRING(""), 0, 0, 3, 0, 1 },

  { 17, 0, TARGET_STRING(
    "simulation/Plant/ World Setup/Solver Configuration/EVAL_KEY/STATE_1"),
    TARGET_STRING(""), 0, 0, 4, 0, 1 },

  { 18, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 19, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 20, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 21, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/Merge"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 22, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem/Constant"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 23, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1/Constant"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 24, 0, TARGET_STRING(
    "simulation/Plant/ Rocket Body/Subsystem/lidar calculation/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2/In"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  {
    0, 0, (NULL), (NULL), 0, 0, 0, 0, 0
  }
};

/* Individual block tuning is not valid when inline parameters is *
 * selected. An empty map is produced to provide a consistent     *
 * interface independent  of inlining parameters.                 *
 */
static const rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  {
    0, (NULL), (NULL), 0, 0, 0
  }
};

/* Block states information */
static const rtwCAPI_States rtBlockStates[] = {
  /* addrMapIndex, contStateStartIndex, blockPath,
   * stateName, pathAlias, dWorkIndex, dataTypeIndex, dimIndex,
   * fixPtIdx, sTimeIndex, isContinuous, hierInfoIdx, flatElemIdx
   */
  { 25, -1, TARGET_STRING("simulation/Plant/Thrust Management/Unit Delay"),
    TARGET_STRING("DSTATE"), "", 0, 0, 0, 0, 0, 0, -1, 0 },

  { 26, -1, TARGET_STRING("simulation/Plant/Thrust Management/Unit Delay1"),
    TARGET_STRING("DSTATE"), "", 0, 0, 0, 0, 0, 0, -1, 0 },

  { 27, -1, TARGET_STRING(
    "simulation/Plant/\nTVC Physics/rotation limiter/Unit Delay"),
    TARGET_STRING("DSTATE"), "", 0, 0, 0, 0, 0, 0, -1, 0 },

  { 28, -1, TARGET_STRING(
    "simulation/Plant/\nTVC Physics/rotation limiter/Unit Delay1"),
    TARGET_STRING("DSTATE"), "", 0, 0, 0, 0, 0, 0, -1, 0 },

  { 29, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_1_1_1"),
    TARGET_STRING("Discrete"), "", 0, 0, 5, 0, 1, 0, -1, 0 },

  { 30, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_2_1_1"),
    TARGET_STRING("Discrete"), "", 0, 0, 5, 0, 1, 0, -1, 0 },

  { 31, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_3_1_1"),
    TARGET_STRING("Discrete"), "", 0, 0, 5, 0, 1, 0, -1, 0 },

  { 32, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_4_1_1"),
    TARGET_STRING("Discrete"), "", 0, 0, 0, 0, 1, 0, -1, 0 },

  { 33, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_4_1_1"),
    TARGET_STRING("FirstOutput"), "", 0, 0, 0, 0, 1, 0, -1, 0 },

  { 34, 0, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_4_1_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Simulink_PS_Converter4.outputFiltered_116065211_0"),
    TARGET_STRING("simulation/Plant/\nTVC Physics/Simulink-PS\nConverter4"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 35, 1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_4_1_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Simulink_PS_Converter4.outputFiltered_116065211_1"),
    TARGET_STRING("simulation/Plant/\nTVC Physics/Simulink-PS\nConverter4"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 36, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_5_1_1"),
    TARGET_STRING("Discrete"), "", 0, 0, 0, 0, 1, 0, -1, 0 },

  { 37, -1, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_5_1_1"),
    TARGET_STRING("FirstOutput"), "", 0, 0, 0, 0, 1, 0, -1, 0 },

  { 38, 2, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_5_1_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Simulink_PS_Converter5.outputFiltered_998976011_0"),
    TARGET_STRING("simulation/Plant/\nTVC Physics/Simulink-PS\nConverter5"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 39, 3, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/INPUT_5_1_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Simulink_PS_Converter5.outputFiltered_998976011_1"),
    TARGET_STRING("simulation/Plant/\nTVC Physics/Simulink-PS\nConverter5"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 40, 4, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.Px.p"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 41, 5, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.Py.p"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 42, 6, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.Pz.p"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 43, 7, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.S.Q"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 1, 0, 1, 1, -1, 0 },

  { 44, 11, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.Px.v"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 45, 12, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.Py.v"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 46, 13, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.Pz.v"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 47, 14, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.Rocket_Body.Free_Movement.S.w"),
    TARGET_STRING("simulation/Plant/ Rocket Body/Free Movement"),
    0, 0, 7, 0, 1, 1, -1, 0 },

  { 48, 17, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.Px.p"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 49, 18, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.Py.p"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 50, 19, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.Pz.p"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 51, 20, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.S.Q"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 1, 0, 1, 1, -1, 0 },

  { 52, 24, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.Px.v"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 53, 25, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.Py.v"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 54, 26, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.Pz.v"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 6, 0, 1, 1, -1, 0 },

  { 55, 27, TARGET_STRING(
    "simulation/Plant/\nWorld Setup/Solver\nConfiguration/EVAL_KEY/STATE_1"),
    TARGET_STRING("simulation.Plant.TVC_Physics.Rigid_Transform2.implicit6dof1.S.w"),
    TARGET_STRING("simulation/Plant/ TVC Physics/Rigid Transform2"),
    0, 0, 7, 0, 1, 1, -1, 0 },

  {
    0, -1, (NULL), (NULL), (NULL), 0, 0, 0, 0, 0, 0, -1, 0
  }
};

/* Root Inputs information */
static const rtwCAPI_Signals rtRootInputs[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 56, 0, TARGET_STRING("simulation/tvc x angle"),
    TARGET_STRING(""), 1, 0, 0, 0, 1 },

  { 57, 0, TARGET_STRING("simulation/tvc y angle"),
    TARGET_STRING(""), 2, 0, 0, 0, 1 },

  { 58, 0, TARGET_STRING("simulation/ignite_s1"),
    TARGET_STRING(""), 3, 0, 0, 0, 1 },

  { 59, 0, TARGET_STRING("simulation/ignite_s2"),
    TARGET_STRING(""), 4, 0, 0, 0, 1 },

  {
    0, 0, (NULL), (NULL), 0, 0, 0, 0, 0
  }
};

/* Root Outputs information */
static const rtwCAPI_Signals rtRootOutputs[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 60, 0, TARGET_STRING("simulation/lidar"),
    TARGET_STRING(""), 1, 0, 0, 0, 1 },

  { 61, 0, TARGET_STRING("simulation/omega"),
    TARGET_STRING(""), 2, 0, 8, 0, 0 },

  { 62, 0, TARGET_STRING("simulation/acceleration"),
    TARGET_STRING(""), 3, 0, 8, 0, 0 },

  { 63, 0, TARGET_STRING("simulation/mag"),
    TARGET_STRING(""), 4, 0, 8, 0, 0 },

  { 64, 0, TARGET_STRING("simulation/true state"),
    TARGET_STRING(""), 5, 0, 9, 0, 1 },

  {
    0, 0, (NULL), (NULL), 0, 0, 0, 0, 0
  }
};

/* Tunable variable parameters */
static const rtwCAPI_ModelParameters rtModelParameters[] = {
  /* addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex */
  { 0, (NULL), 0, 0, 0 }
};

#ifndef HOST_CAPI_BUILD

/* Declare Data Addresses statically */
static void* rtDataAddrMap[] = {
  &simulation_B.attached,              /* 0: Signal */
  &simulation_B.s1_et,                 /* 1: Signal */
  &simulation_B.s2_et,                 /* 2: Signal */
  &simulation_B.thrust_j,              /* 3: Signal */
  &simulation_B.thrust,                /* 4: Signal */
  &simulation_B.Switch,                /* 5: Signal */
  &simulation_B.Switch1,               /* 6: Signal */
  &simulation_B.UnitDelay,             /* 7: Signal */
  &simulation_B.UnitDelay1,            /* 8: Signal */
  &simulation_B.RandomNumber,          /* 9: Signal */
  &simulation_B.INPUT_1_1_1[0],        /* 10: Signal */
  &simulation_B.INPUT_2_1_1[0],        /* 11: Signal */
  &simulation_B.INPUT_3_1_1[0],        /* 12: Signal */
  &simulation_B.INPUT_4_1_1[0],        /* 13: Signal */
  &simulation_B.INPUT_5_1_1[0],        /* 14: Signal */
  &simulation_B.OUTPUT_1_0[0],         /* 15: Signal */
  &simulation_B.OUTPUT_1_1[0],         /* 16: Signal */
  &simulation_B.STATE_1[0],            /* 17: Signal */
  &simulation_B.Merge,                 /* 18: Signal */
  &simulation_B.Merge,                 /* 19: Signal */
  &simulation_B.Merge,                 /* 20: Signal */
  &simulation_B.Merge,                 /* 21: Signal */
  &simulation_B.Merge,                 /* 22: Signal */
  &simulation_B.Merge,                 /* 23: Signal */
  &simulation_B.Merge,                 /* 24: Signal */
  &simulation_DW.UnitDelay_DSTATE_m,   /* 25: Discrete State */
  &simulation_DW.UnitDelay1_DSTATE_j,  /* 26: Discrete State */
  &simulation_DW.UnitDelay_DSTATE,     /* 27: Discrete State */
  &simulation_DW.UnitDelay1_DSTATE,    /* 28: Discrete State */
  &simulation_DW.INPUT_1_1_1_Discrete[0],/* 29: Discrete State */
  &simulation_DW.INPUT_2_1_1_Discrete[0],/* 30: Discrete State */
  &simulation_DW.INPUT_3_1_1_Discrete[0],/* 31: Discrete State */
  &simulation_DW.INPUT_4_1_1_Discrete, /* 32: Discrete State */
  &simulation_DW.INPUT_4_1_1_FirstOutput,/* 33: Discrete State */
  &simulation_X.simulationPlantTVC_PhysicsSimul[0],/* 34: Continuous State */
  &simulation_X.simulationPlantTVC_PhysicsSimul[1],/* 35: Continuous State */
  &simulation_DW.INPUT_5_1_1_Discrete, /* 36: Discrete State */
  &simulation_DW.INPUT_5_1_1_FirstOutput,/* 37: Discrete State */
  &simulation_X.simulationPlantTVC_PhysicsSim_a[0],/* 38: Continuous State */
  &simulation_X.simulationPlantTVC_PhysicsSim_a[1],/* 39: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[0],/* 40: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[1],/* 41: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[2],/* 42: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[3],/* 43: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[7],/* 44: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[8],/* 45: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[9],/* 46: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[10],/* 47: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[13],/* 48: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[14],/* 49: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[15],/* 50: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[16],/* 51: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[20],/* 52: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[21],/* 53: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[22],/* 54: Continuous State */
  &simulation_X.simulationPlantRocket_BodyFree_[23],/* 55: Continuous State */
  &simulation_U.tvcxangle,             /* 56: Root Input */
  &simulation_U.tvcyangle,             /* 57: Root Input */
  &simulation_U.ignite_s1,             /* 58: Root Input */
  &simulation_U.ignite_s2,             /* 59: Root Input */
  &simulation_Y.lidar,                 /* 60: Root Output */
  &simulation_Y.omega[0],              /* 61: Root Output */
  &simulation_Y.acceleration[0],       /* 62: Root Output */
  &simulation_Y.mag[0],                /* 63: Root Output */
  &simulation_Y.truestate[0],          /* 64: Root Output */
};

/* Declare Data Run-Time Dimension Buffer Addresses statically */
static int32_T* rtVarDimsAddrMap[] = {
  (NULL)
};

#endif

/* Data Type Map - use dataTypeMapIndex to access this structure */
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[] = {
  /* cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
   * isComplex, isPointer, enumStorageType */
  { "double", "real_T", 0, 0, sizeof(real_T), (uint8_T)SS_DOUBLE, 0, 0, 0 }
};

#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif

/* Structure Element Map - use elemMapIndex to access this structure */
static TARGET_CONST rtwCAPI_ElementMap rtElementMap[] = {
  /* elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex */
  { (NULL), 0, 0, 0, 0 },
};

/* Dimension Map - use dimensionMapIndex to access elements of ths structure*/
static const rtwCAPI_DimensionMap rtDimensionMap[] = {
  /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */
  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 2, 2, 0 },

  { rtwCAPI_VECTOR, 4, 2, 0 },

  { rtwCAPI_VECTOR, 6, 2, 0 },

  { rtwCAPI_VECTOR, 8, 2, 0 },

  { rtwCAPI_VECTOR, 10, 2, 0 },

  { rtwCAPI_VECTOR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 12, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 14, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 16, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static const uint_T rtDimensionArray[] = {
  1,                                   /* 0 */
  1,                                   /* 1 */
  4,                                   /* 2 */
  1,                                   /* 3 */
  18,                                  /* 4 */
  1,                                   /* 5 */
  6,                                   /* 6 */
  1,                                   /* 7 */
  27,                                  /* 8 */
  1,                                   /* 9 */
  2,                                   /* 10 */
  1,                                   /* 11 */
  3,                                   /* 12 */
  1,                                   /* 13 */
  1,                                   /* 14 */
  3,                                   /* 15 */
  14,                                  /* 16 */
  1                                    /* 17 */
};

/* C-API stores floating point values in an array. The elements of this  *
 * are unique. This ensures that values which are shared across the model*
 * are stored in the most efficient way. These values are referenced by  *
 *           - rtwCAPI_FixPtMap.fracSlopePtr,                            *
 *           - rtwCAPI_FixPtMap.biasPtr,                                 *
 *           - rtwCAPI_SampleTimeMap.samplePeriodPtr,                    *
 *           - rtwCAPI_SampleTimeMap.sampleOffsetPtr                     */
static const real_T rtcapiStoredFloats[] = {
  0.001, 0.0, 0.05
};

/* Fixed Point Map */
static const rtwCAPI_FixPtMap rtFixPtMap[] = {
  /* fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned */
  { (NULL), (NULL), rtwCAPI_FIX_RESERVED, 0, 0, (boolean_T)0 },
};

/* Sample Time Map - use sTimeIndex to access elements of ths structure */
static const rtwCAPI_SampleTimeMap rtSampleTimeMap[] = {
  /* samplePeriodPtr, sampleOffsetPtr, tid, samplingMode */
  { (const void *) &rtcapiStoredFloats[0], (const void *) &rtcapiStoredFloats[1],
    (int8_T)1, (uint8_T)0 },

  { (const void *) &rtcapiStoredFloats[1], (const void *) &rtcapiStoredFloats[1],
    (int8_T)0, (uint8_T)0 },

  { (const void *) &rtcapiStoredFloats[2], (const void *) &rtcapiStoredFloats[1],
    (int8_T)2, (uint8_T)0 }
};

static rtwCAPI_ModelMappingStaticInfo mmiStatic = {
  /* Signals:{signals, numSignals,
   *           rootInputs, numRootInputs,
   *           rootOutputs, numRootOutputs},
   * Params: {blockParameters, numBlockParameters,
   *          modelParameters, numModelParameters},
   * States: {states, numStates},
   * Maps:   {dataTypeMap, dimensionMap, fixPtMap,
   *          elementMap, sampleTimeMap, dimensionArray},
   * TargetType: targetType
   */
  { rtBlockSignals, 25,
    rtRootInputs, 4,
    rtRootOutputs, 5 },

  { rtBlockParameters, 0,
    rtModelParameters, 0 },

  { rtBlockStates, 31 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 1048583616U,
    1982623195U,
    31718423U,
    3761967121U },
  (NULL), 0,
  (boolean_T)0
};

/* Function to get C API Model Mapping Static Info */
const rtwCAPI_ModelMappingStaticInfo*
  simulation_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

/* Cache pointers into DataMapInfo substructure of RTModel */
#ifndef HOST_CAPI_BUILD

void simulation_InitializeDataMapInfo(void)
{
  /* Set C-API version */
  rtwCAPI_SetVersion(simulation_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(simulation_M->DataMapInfo.mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(simulation_M->DataMapInfo.mmi, (NULL));

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetDataAddressMap(simulation_M->DataMapInfo.mmi, rtDataAddrMap);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetVarDimsAddressMap(simulation_M->DataMapInfo.mmi, rtVarDimsAddrMap);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(simulation_M->DataMapInfo.mmi, (NULL));

  /* Set reference to submodels */
  rtwCAPI_SetChildMMIArray(simulation_M->DataMapInfo.mmi, (NULL));
  rtwCAPI_SetChildMMIArrayLen(simulation_M->DataMapInfo.mmi, 0);
}

#else                                  /* HOST_CAPI_BUILD */
#ifdef __cplusplus

extern "C"
{

#endif

  void simulation_host_InitializeDataMapInfo(simulation_host_DataMapInfo_T
    *dataMap, const char *path)
  {
    /* Set C-API version */
    rtwCAPI_SetVersion(dataMap->mmi, 1);

    /* Cache static C-API data into the Real-time Model Data structure */
    rtwCAPI_SetStaticMap(dataMap->mmi, &mmiStatic);

    /* host data address map is NULL */
    rtwCAPI_SetDataAddressMap(dataMap->mmi, (NULL));

    /* host vardims address map is NULL */
    rtwCAPI_SetVarDimsAddressMap(dataMap->mmi, (NULL));

    /* Set Instance specific path */
    rtwCAPI_SetPath(dataMap->mmi, path);
    rtwCAPI_SetFullPath(dataMap->mmi, (NULL));

    /* Set reference to submodels */
    rtwCAPI_SetChildMMIArray(dataMap->mmi, (NULL));
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 0);
  }

#ifdef __cplusplus

}

#endif
#endif                                 /* HOST_CAPI_BUILD */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
