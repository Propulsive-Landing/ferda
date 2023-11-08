/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: simulation_types.h
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

#ifndef RTW_HEADER_simulation_types_h_
#define RTW_HEADER_simulation_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_sI9OZ8YWn5qr2iby6yfJzBB
#define struct_tag_sI9OZ8YWn5qr2iby6yfJzBB

struct tag_sI9OZ8YWn5qr2iby6yfJzBB
{
  real_T Numerator;
  real_T Denominator[2];
};

#endif                                 /* struct_tag_sI9OZ8YWn5qr2iby6yfJzBB */

#ifndef typedef_sI9OZ8YWn5qr2iby6yfJzBB_simul_T
#define typedef_sI9OZ8YWn5qr2iby6yfJzBB_simul_T

typedef struct tag_sI9OZ8YWn5qr2iby6yfJzBB sI9OZ8YWn5qr2iby6yfJzBB_simul_T;

#endif                             /* typedef_sI9OZ8YWn5qr2iby6yfJzBB_simul_T */

#ifndef struct_tag_EDDefAxlN8L7dR1ZfMmDTH
#define struct_tag_EDDefAxlN8L7dR1ZfMmDTH

struct tag_EDDefAxlN8L7dR1ZfMmDTH
{
  char_T Value[12];
};

#endif                                 /* struct_tag_EDDefAxlN8L7dR1ZfMmDTH */

#ifndef typedef_rtString_simulation_T
#define typedef_rtString_simulation_T

typedef struct tag_EDDefAxlN8L7dR1ZfMmDTH rtString_simulation_T;

#endif                                 /* typedef_rtString_simulation_T */

#ifndef struct_tag_i6CsTCVxB9hTYdRcCjoywF
#define struct_tag_i6CsTCVxB9hTYdRcCjoywF

struct tag_i6CsTCVxB9hTYdRcCjoywF
{
  boolean_T tunablePropertyChanged[12];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T MeasurementRange;
  real_T Resolution;
  real_T ConstantBias[3];
  real_T AxesMisalignment[9];
  real_T NoiseDensity[3];
  real_T BiasInstability[3];
  real_T RandomWalk[3];
  sI9OZ8YWn5qr2iby6yfJzBB_simul_T BiasInstabilityCoefficients;
  rtString_simulation_T NoiseType;
  real_T TemperatureBias[3];
  real_T TemperatureScaleFactor[3];
  real_T Temperature;
  real_T pBandwidth;
  real_T pCorrelationTime;
  real_T pBiasInstFilterNum;
  real_T pBiasInstFilterDen[2];
  real_T pBiasInstFilterStates[3];
  real_T pStdDevBiasInst[3];
  real_T pStdDevWhiteNoise[3];
  real_T pRandWalkFilterStates[3];
  real_T pStdDevRandWalk[3];
  real_T pGain[9];
};

#endif                                 /* struct_tag_i6CsTCVxB9hTYdRcCjoywF */

#ifndef typedef_g_fusion_internal_Acceleromet_T
#define typedef_g_fusion_internal_Acceleromet_T

typedef struct tag_i6CsTCVxB9hTYdRcCjoywF g_fusion_internal_Acceleromet_T;

#endif                             /* typedef_g_fusion_internal_Acceleromet_T */

#ifndef struct_tag_Mj47eX45769jebuqv2OytB
#define struct_tag_Mj47eX45769jebuqv2OytB

struct tag_Mj47eX45769jebuqv2OytB
{
  boolean_T tunablePropertyChanged[13];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T MeasurementRange;
  real_T Resolution;
  real_T ConstantBias[3];
  real_T AxesMisalignment[9];
  real_T NoiseDensity[3];
  real_T BiasInstability[3];
  real_T RandomWalk[3];
  sI9OZ8YWn5qr2iby6yfJzBB_simul_T BiasInstabilityCoefficients;
  rtString_simulation_T NoiseType;
  real_T TemperatureBias[3];
  real_T TemperatureScaleFactor[3];
  real_T Temperature;
  real_T pBandwidth;
  real_T pCorrelationTime;
  real_T pBiasInstFilterNum;
  real_T pBiasInstFilterDen[2];
  real_T pBiasInstFilterStates[3];
  real_T pStdDevBiasInst[3];
  real_T pStdDevWhiteNoise[3];
  real_T pRandWalkFilterStates[3];
  real_T pStdDevRandWalk[3];
  real_T pGain[9];
  real_T AccelerationBias[3];
  real_T pAcceleration[3];
};

#endif                                 /* struct_tag_Mj47eX45769jebuqv2OytB */

#ifndef typedef_h_fusion_internal_GyroscopeSi_T
#define typedef_h_fusion_internal_GyroscopeSi_T

typedef struct tag_Mj47eX45769jebuqv2OytB h_fusion_internal_GyroscopeSi_T;

#endif                             /* typedef_h_fusion_internal_GyroscopeSi_T */

#ifndef struct_tag_dGsU1cW6xvZMdrkopsDHLC
#define struct_tag_dGsU1cW6xvZMdrkopsDHLC

struct tag_dGsU1cW6xvZMdrkopsDHLC
{
  boolean_T tunablePropertyChanged[12];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T MeasurementRange;
  real_T Resolution;
  real_T ConstantBias[3];
  real_T AxesMisalignment[9];
  real_T NoiseDensity[3];
  real_T BiasInstability[3];
  real_T RandomWalk[3];
  sI9OZ8YWn5qr2iby6yfJzBB_simul_T BiasInstabilityCoefficients;
  rtString_simulation_T NoiseType;
  real_T TemperatureBias[3];
  real_T TemperatureScaleFactor[3];
  real_T Temperature;
  real_T pBandwidth;
  real_T pCorrelationTime;
  real_T pBiasInstFilterNum;
  real_T pBiasInstFilterDen[2];
  real_T pBiasInstFilterStates[3];
  real_T pStdDevBiasInst[3];
  real_T pStdDevWhiteNoise[3];
  real_T pRandWalkFilterStates[3];
  real_T pStdDevRandWalk[3];
  real_T pGain[9];
};

#endif                                 /* struct_tag_dGsU1cW6xvZMdrkopsDHLC */

#ifndef typedef_i_fusion_internal_Magnetomete_T
#define typedef_i_fusion_internal_Magnetomete_T

typedef struct tag_dGsU1cW6xvZMdrkopsDHLC i_fusion_internal_Magnetomete_T;

#endif                             /* typedef_i_fusion_internal_Magnetomete_T */

#ifndef struct_tag_LwEBllCB3waYkYGiImiIIE
#define struct_tag_LwEBllCB3waYkYGiImiIIE

struct tag_LwEBllCB3waYkYGiImiIIE
{
  boolean_T tunablePropertyChanged[38];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T Temperature;
  uint32_T pStreamState[625];
  g_fusion_internal_Acceleromet_T *pAccel;
  h_fusion_internal_GyroscopeSi_T *pGyro;
  i_fusion_internal_Magnetomete_T *pMag;
  real_T MagneticFieldNED[3];
  real_T MagneticField[3];
  real_T AccelParamsMeasurementRange;
  real_T AccelParamsResolution;
  real_T AccelParamsConstantBias[3];
  real_T AccelParamsAxesMisalignment[9];
  real_T AccelParamsNoiseDensity[3];
  real_T AccelParamsBiasInstability[3];
  real_T AccelParamsBiasInstabilityNumerator;
  real_T AccelParamsBiasInstabilityDenominator[2];
  real_T AccelParamsRandomWalk[3];
  real_T AccelParamsTemperatureBias[3];
  real_T AccelParamsTemperatureScaleFactor[3];
  real_T GyroParamsMeasurementRange;
  real_T GyroParamsResolution;
  real_T GyroParamsConstantBias[3];
  real_T GyroParamsAxesMisalignment[9];
  real_T GyroParamsNoiseDensity[3];
  real_T GyroParamsBiasInstability[3];
  real_T GyroParamsBiasInstabilityNumerator;
  real_T GyroParamsBiasInstabilityDenominator[2];
  real_T GyroParamsRandomWalk[3];
  real_T GyroParamsTemperatureBias[3];
  real_T GyroParamsTemperatureScaleFactor[3];
  real_T GyroParamsAccelerationBias[3];
  real_T MagParamsMeasurementRange;
  real_T MagParamsResolution;
  real_T MagParamsConstantBias[3];
  real_T MagParamsAxesMisalignment[9];
  real_T MagParamsNoiseDensity[3];
  real_T MagParamsBiasInstability[3];
  real_T MagParamsBiasInstabilityNumerator;
  real_T MagParamsBiasInstabilityDenominator[2];
  real_T MagParamsRandomWalk[3];
  real_T MagParamsTemperatureBias[3];
  real_T MagParamsTemperatureScaleFactor[3];
  i_fusion_internal_Magnetomete_T _pobj0;
  h_fusion_internal_GyroscopeSi_T _pobj1;
  g_fusion_internal_Acceleromet_T _pobj2;
};

#endif                                 /* struct_tag_LwEBllCB3waYkYGiImiIIE */

#ifndef typedef_fusion_simulink_imuSensor_sim_T
#define typedef_fusion_simulink_imuSensor_sim_T

typedef struct tag_LwEBllCB3waYkYGiImiIIE fusion_simulink_imuSensor_sim_T;

#endif                             /* typedef_fusion_simulink_imuSensor_sim_T */

#ifndef SS_UINT64
#define SS_UINT64                      18
#endif

#ifndef SS_INT64
#define SS_INT64                       19
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_simulation_T RT_MODEL_simulation_T;

#endif                                 /* RTW_HEADER_simulation_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
