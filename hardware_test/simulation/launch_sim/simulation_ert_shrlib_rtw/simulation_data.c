/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: simulation_data.c
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

/* Constant parameters (default storage) */
const ConstP_simulation_T simulation_ConstP = {
  /* Expression: [ 27.5550, -2.4169, -16.0849 ]
   * Referenced by: '<S17>/IMU'
   */
  { 27.555, -2.4169, -16.0849 },

  /* Pooled Parameter (Expression: [ 0, 0, 0 ])
   * Referenced by: '<S17>/IMU'
   */
  { 0.0, 0.0, 0.0 },

  /* Pooled Parameter (Expression: [ 100, 0, 0; 0, 100, 0; 0, 0, 100 ])
   * Referenced by: '<S17>/IMU'
   */
  { 100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0 },

  /* Pooled Parameter (Expression: fractalcoef().Denominator)
   * Referenced by: '<S17>/IMU'
   */
  { 1.0, -0.5 },

  /* Pooled Parameter (Expression: stage1_series)
   * Referenced by:
   *   '<S5>/series'
   *   '<S5>/series1'
   */
  { 0.0, 0.0, 0.275, 18.0, 0.35, 18.0, 1.5, 18.0, 4.6, 18.0, 4.65, 0.0 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
