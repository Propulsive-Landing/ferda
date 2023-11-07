//
// File: simulation_types.h
//
// Code generated for Simulink model 'simulation'.
//
// Model version                  : 4.1
// Simulink Coder version         : 23.2 (R2023b) 04-Oct-2023
// C/C++ source code generated on : Mon Nov  6 23:26:52 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_simulation_types_h_
#define RTW_HEADER_simulation_types_h_
#include "rtwtypes.h"
#ifndef struct_sI9OZ8YWn5qr2iby6yfJzBB_simul_T
#define struct_sI9OZ8YWn5qr2iby6yfJzBB_simul_T

struct sI9OZ8YWn5qr2iby6yfJzBB_simul_T
{
  real_T Numerator;
  real_T Denominator[2];
};

#endif                                // struct_sI9OZ8YWn5qr2iby6yfJzBB_simul_T

#ifndef struct_rtString_simulation_T
#define struct_rtString_simulation_T

struct rtString_simulation_T
{
  char_T Value[12];
};

#endif                                 // struct_rtString_simulation_T

#ifndef struct_g_fusion_internal_Acceleromet_T
#define struct_g_fusion_internal_Acceleromet_T

struct g_fusion_internal_Acceleromet_T
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

#endif                                // struct_g_fusion_internal_Acceleromet_T

#ifndef struct_h_fusion_internal_GyroscopeSi_T
#define struct_h_fusion_internal_GyroscopeSi_T

struct h_fusion_internal_GyroscopeSi_T
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

#endif                                // struct_h_fusion_internal_GyroscopeSi_T

#ifndef struct_i_fusion_internal_Magnetomete_T
#define struct_i_fusion_internal_Magnetomete_T

struct i_fusion_internal_Magnetomete_T
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

#endif                                // struct_i_fusion_internal_Magnetomete_T

#ifndef struct_fusion_simulink_imuSensor_sim_T
#define struct_fusion_simulink_imuSensor_sim_T

struct fusion_simulink_imuSensor_sim_T
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

#endif                                // struct_fusion_simulink_imuSensor_sim_T

// Forward declaration for rtModel
typedef struct tag_RTM_simulation_T RT_MODEL_simulation_T;

#endif                                 // RTW_HEADER_simulation_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
