/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'simulation/Plant/ World Setup/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"

void simulation_b048d748_1_setTargets(const RuntimeDerivedValuesBundle *rtdv,
  CTarget *targets)
{
  (void) rtdv;
  (void) targets;
}

void simulation_b048d748_1_resetAsmStateVector(const void *mech, double *state)
{
  double xx[2];
  (void) mech;
  xx[0] = 0.0;
  xx[1] = 1.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[1];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[0];
  state[19] = xx[0];
  state[20] = xx[1];
  state[21] = xx[0];
  state[22] = xx[0];
  state[23] = xx[0];
  state[24] = xx[0];
  state[25] = xx[0];
  state[26] = xx[0];
  state[27] = xx[0];
  state[28] = xx[0];
  state[29] = xx[0];
}

void simulation_b048d748_1_initializeTrackedAngleState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void simulation_b048d748_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void simulation_b048d748_1_adjustPosition(const void *mech, const double
  *dofDeltas, double *state)
{
  double xx[17];
  (void) mech;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = dofDeltas[3];
  xx[5] = dofDeltas[4];
  xx[6] = dofDeltas[5];
  pm_math_Quaternion_compDeriv_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = state[3] + xx[7];
  xx[1] = state[4] + xx[8];
  xx[2] = state[5] + xx[9];
  xx[3] = state[6] + xx[10];
  xx[4] = 1.0e-64;
  xx[5] = sqrt(xx[0] * xx[0] + xx[1] * xx[1] + xx[2] * xx[2] + xx[3] * xx[3]);
  if (xx[4] > xx[5])
    xx[5] = xx[4];
  xx[6] = state[20];
  xx[7] = state[21];
  xx[8] = state[22];
  xx[9] = state[23];
  xx[10] = dofDeltas[11];
  xx[11] = dofDeltas[12];
  xx[12] = dofDeltas[13];
  pm_math_Quaternion_compDeriv_ra(xx + 6, xx + 10, xx + 13);
  xx[6] = state[20] + xx[13];
  xx[7] = state[21] + xx[14];
  xx[8] = state[22] + xx[15];
  xx[9] = state[23] + xx[16];
  xx[10] = sqrt(xx[6] * xx[6] + xx[7] * xx[7] + xx[8] * xx[8] + xx[9] * xx[9]);
  if (xx[4] > xx[10])
    xx[10] = xx[4];
  state[0] = state[0] + dofDeltas[0];
  state[1] = state[1] + dofDeltas[1];
  state[2] = state[2] + dofDeltas[2];
  state[3] = xx[0] / xx[5];
  state[4] = xx[1] / xx[5];
  state[5] = xx[2] / xx[5];
  state[6] = xx[3] / xx[5];
  state[13] = state[13] + dofDeltas[6];
  state[14] = state[14] + dofDeltas[7];
  state[17] = state[17] + dofDeltas[8];
  state[18] = state[18] + dofDeltas[9];
  state[19] = state[19] + dofDeltas[10];
  state[20] = xx[6] / xx[10];
  state[21] = xx[7] / xx[10];
  state[22] = xx[8] / xx[10];
  state[23] = xx[9] / xx[10];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbAsmJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_3(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[3] = xx[4];
  state[4] = xx[5];
  state[5] = xx[6];
  state[6] = xx[7];
}

static void perturbAsmJointPrimitiveState_0_3v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[3];
  xx[4] = state[4];
  xx[5] = state[5];
  xx[6] = state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[3] = xx[7];
  state[4] = xx[8];
  state[5] = xx[9];
  state[6] = xx[10];
  state[10] = state[10] + 1.2 * mag;
  state[11] = state[11] - xx[2];
  state[12] = state[12] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[13] = state[13] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[13] = state[13] + mag;
  state[15] = state[15] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_1(double mag, double *state)
{
  state[14] = state[14] + mag;
}

static void perturbAsmJointPrimitiveState_1_1v(double mag, double *state)
{
  state[14] = state[14] + mag;
  state[16] = state[16] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[17] = state[17] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[17] = state[17] + mag;
  state[24] = state[24] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_1(double mag, double *state)
{
  state[18] = state[18] + mag;
}

static void perturbAsmJointPrimitiveState_2_1v(double mag, double *state)
{
  state[18] = state[18] + mag;
  state[25] = state[25] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_2(double mag, double *state)
{
  state[19] = state[19] + mag;
}

static void perturbAsmJointPrimitiveState_2_2v(double mag, double *state)
{
  state[19] = state[19] + mag;
  state[26] = state[26] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_3(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[20];
  xx[1] = state[21];
  xx[2] = state[22];
  xx[3] = state[23];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[20] = xx[4];
  state[21] = xx[5];
  state[22] = xx[6];
  state[23] = xx[7];
}

static void perturbAsmJointPrimitiveState_2_3v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[20];
  xx[4] = state[21];
  xx[5] = state[22];
  xx[6] = state[23];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[20] = xx[7];
  state[21] = xx[8];
  state[22] = xx[9];
  state[23] = xx[10];
  state[27] = state[27] + 1.2 * mag;
  state[28] = state[28] - xx[2];
  state[29] = state[29] + 0.9 * mag;
}

void simulation_b048d748_1_perturbAsmJointPrimitiveState(const void *mech,
  size_t stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbAsmJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbAsmJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbAsmJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbAsmJointPrimitiveState_0_2v(mag, state);
    break;

   case 6:
    perturbAsmJointPrimitiveState_0_3(mag, state);
    break;

   case 7:
    perturbAsmJointPrimitiveState_0_3v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 14:
    perturbAsmJointPrimitiveState_1_1(mag, state);
    break;

   case 15:
    perturbAsmJointPrimitiveState_1_1v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 26:
    perturbAsmJointPrimitiveState_2_1(mag, state);
    break;

   case 27:
    perturbAsmJointPrimitiveState_2_1v(mag, state);
    break;

   case 28:
    perturbAsmJointPrimitiveState_2_2(mag, state);
    break;

   case 29:
    perturbAsmJointPrimitiveState_2_2v(mag, state);
    break;

   case 30:
    perturbAsmJointPrimitiveState_2_3(mag, state);
    break;

   case 31:
    perturbAsmJointPrimitiveState_2_3v(mag, state);
    break;
  }
}

static void computePosDofBlendMatrix_0_3(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 9.87654321;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[4] * state[5] - state[3] * state[6]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[3] * state[3] + state[4] * state[4]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[4] * state[6] + state[3] * state[5]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_2_3(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 9.87654321;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[21] * state[22] - state[20] * state[23]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[20] * state[20] + state[21] * state[21]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[21] * state[23] + state[20] * state[22]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

void simulation_b048d748_1_computePosDofBlendMatrix(const void *mech, size_t
  stageIdx, size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    computePosDofBlendMatrix_0_3(state, partialType, matrix);
    break;

   case 15:
    computePosDofBlendMatrix_2_3(state, partialType, matrix);
    break;
  }
}

static void computeVelDofBlendMatrix_0_3(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 9.87654321;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_2_3(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 9.87654321;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

void simulation_b048d748_1_computeVelDofBlendMatrix(const void *mech, size_t
  stageIdx, size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    computeVelDofBlendMatrix_0_3(state, partialType, matrix);
    break;

   case 15:
    computeVelDofBlendMatrix_2_3(state, partialType, matrix);
    break;
  }
}

static void projectPartiallyTargetedPos_0_3(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[2];
  double xx[17];
  xx[0] = 2.0;
  xx[1] = (state[4] * state[6] + state[3] * state[5]) * xx[0];
  xx[2] = 0.99999999999999;
  bb[0] = fabs(xx[1]) > xx[2];
  xx[3] = 1.570796326794897;
  if (xx[1] < 0.0)
    xx[4] = -1.0;
  else if (xx[1] > 0.0)
    xx[4] = +1.0;
  else
    xx[4] = 0.0;
  xx[5] = fabs(xx[1]) > 1.0 ? atan2(xx[1], 0.0) : asin(xx[1]);
  xx[1] = bb[0] ? xx[3] * xx[4] : xx[5];
  xx[5] = (origState[4] * origState[6] + origState[3] * origState[5]) * xx[0];
  bb[1] = fabs(xx[5]) > xx[2];
  if (xx[5] < 0.0)
    xx[2] = -1.0;
  else if (xx[5] > 0.0)
    xx[2] = +1.0;
  else
    xx[2] = 0.0;
  xx[6] = fabs(xx[5]) > 1.0 ? atan2(xx[5], 0.0) : asin(xx[5]);
  xx[5] = bb[1] ? xx[3] * xx[2] : xx[6];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[5];
  xx[9] = xx[5];
  xx[10] = xx[1];
  xx[11] = xx[1];
  xx[12] = xx[5];
  xx[1] = xx[6 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[5] = 0.5;
  xx[6] = state[5] * state[6];
  xx[7] = state[3] * state[4];
  xx[8] = state[3] * state[3];
  xx[9] = 1.0;
  xx[10] = (xx[8] + state[5] * state[5]) * xx[0] - xx[9];
  xx[11] = (xx[6] + xx[7]) * xx[0];
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  xx[11] = (xx[8] + state[6] * state[6]) * xx[0] - xx[9];
  xx[12] = - (xx[0] * (xx[6] - xx[7]));
  xx[11] = (xx[12] == 0.0 && xx[11] == 0.0) ? 0.0 : atan2(xx[12], xx[11]);
  xx[6] = bb[0] ? xx[5] * xx[10] : xx[11];
  xx[7] = (xx[8] + state[4] * state[4]) * xx[0] - xx[9];
  xx[10] = - (xx[0] * (state[4] * state[5] - state[3] * state[6]));
  xx[7] = (xx[10] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[10], xx[7]);
  xx[8] = bb[0] ? xx[4] * xx[6] : xx[7];
  xx[4] = origState[5] * origState[6];
  xx[7] = origState[3] * origState[4];
  xx[10] = origState[3] * origState[3];
  xx[11] = (xx[10] + origState[5] * origState[5]) * xx[0] - xx[9];
  xx[12] = (xx[4] + xx[7]) * xx[0];
  xx[11] = (xx[12] == 0.0 && xx[11] == 0.0) ? 0.0 : atan2(xx[12], xx[11]);
  xx[12] = (xx[10] + origState[6] * origState[6]) * xx[0] - xx[9];
  xx[13] = - (xx[0] * (xx[4] - xx[7]));
  xx[12] = (xx[13] == 0.0 && xx[12] == 0.0) ? 0.0 : atan2(xx[13], xx[12]);
  xx[4] = bb[1] ? xx[5] * xx[11] : xx[12];
  xx[5] = (xx[10] + origState[4] * origState[4]) * xx[0] - xx[9];
  xx[7] = - (xx[0] * (origState[4] * origState[5] - origState[3] * origState[6]));
  xx[5] = (xx[7] == 0.0 && xx[5] == 0.0) ? 0.0 : atan2(xx[7], xx[5]);
  xx[0] = bb[1] ? xx[2] * xx[4] : xx[5];
  xx[9] = xx[8];
  xx[10] = xx[8];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[0];
  xx[14] = xx[0];
  xx[15] = xx[0];
  xx[0] = xx[9 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = sin(xx[1]);
  xx[7] = xx[6];
  xx[8] = xx[4];
  xx[9] = xx[6];
  xx[10] = xx[4];
  xx[11] = xx[6];
  xx[12] = xx[4];
  xx[13] = xx[6];
  xx[1] = xx[7 + (partialType)];
  xx[4] = cos(xx[1]);
  xx[6] = sin(xx[1]);
  xx[1] = xx[2] * xx[6];
  xx[7] = xx[4] * xx[2];
  xx[8] = xx[3] * xx[2];
  xx[9] = - (xx[3] * xx[5]);
  xx[10] = xx[0];
  xx[11] = xx[4] * xx[5] + xx[1] * xx[0];
  xx[12] = xx[7] - xx[6] * xx[0] * xx[5];
  xx[13] = - (xx[3] * xx[6]);
  xx[14] = xx[6] * xx[5] - xx[7] * xx[0];
  xx[15] = xx[1] + xx[4] * xx[0] * xx[5];
  xx[16] = xx[4] * xx[3];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 8, xx + 0);
  state[3] = xx[0];
  state[4] = xx[1];
  state[5] = xx[2];
  state[6] = xx[3];
}

static void projectPartiallyTargetedPos_2_3(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[2];
  double xx[17];
  xx[0] = 2.0;
  xx[1] = (state[21] * state[23] + state[20] * state[22]) * xx[0];
  xx[2] = 0.99999999999999;
  bb[0] = fabs(xx[1]) > xx[2];
  xx[3] = 1.570796326794897;
  if (xx[1] < 0.0)
    xx[4] = -1.0;
  else if (xx[1] > 0.0)
    xx[4] = +1.0;
  else
    xx[4] = 0.0;
  xx[5] = fabs(xx[1]) > 1.0 ? atan2(xx[1], 0.0) : asin(xx[1]);
  xx[1] = bb[0] ? xx[3] * xx[4] : xx[5];
  xx[5] = (origState[21] * origState[23] + origState[20] * origState[22]) * xx[0];
  bb[1] = fabs(xx[5]) > xx[2];
  if (xx[5] < 0.0)
    xx[2] = -1.0;
  else if (xx[5] > 0.0)
    xx[2] = +1.0;
  else
    xx[2] = 0.0;
  xx[6] = fabs(xx[5]) > 1.0 ? atan2(xx[5], 0.0) : asin(xx[5]);
  xx[5] = bb[1] ? xx[3] * xx[2] : xx[6];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[5];
  xx[9] = xx[5];
  xx[10] = xx[1];
  xx[11] = xx[1];
  xx[12] = xx[5];
  xx[1] = xx[6 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[5] = 0.5;
  xx[6] = state[22] * state[23];
  xx[7] = state[20] * state[21];
  xx[8] = state[20] * state[20];
  xx[9] = 1.0;
  xx[10] = (xx[8] + state[22] * state[22]) * xx[0] - xx[9];
  xx[11] = (xx[6] + xx[7]) * xx[0];
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  xx[11] = (xx[8] + state[23] * state[23]) * xx[0] - xx[9];
  xx[12] = - (xx[0] * (xx[6] - xx[7]));
  xx[11] = (xx[12] == 0.0 && xx[11] == 0.0) ? 0.0 : atan2(xx[12], xx[11]);
  xx[6] = bb[0] ? xx[5] * xx[10] : xx[11];
  xx[7] = (xx[8] + state[21] * state[21]) * xx[0] - xx[9];
  xx[10] = - (xx[0] * (state[21] * state[22] - state[20] * state[23]));
  xx[7] = (xx[10] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[10], xx[7]);
  xx[8] = bb[0] ? xx[4] * xx[6] : xx[7];
  xx[4] = origState[22] * origState[23];
  xx[7] = origState[20] * origState[21];
  xx[10] = origState[20] * origState[20];
  xx[11] = (xx[10] + origState[22] * origState[22]) * xx[0] - xx[9];
  xx[12] = (xx[4] + xx[7]) * xx[0];
  xx[11] = (xx[12] == 0.0 && xx[11] == 0.0) ? 0.0 : atan2(xx[12], xx[11]);
  xx[12] = (xx[10] + origState[23] * origState[23]) * xx[0] - xx[9];
  xx[13] = - (xx[0] * (xx[4] - xx[7]));
  xx[12] = (xx[13] == 0.0 && xx[12] == 0.0) ? 0.0 : atan2(xx[13], xx[12]);
  xx[4] = bb[1] ? xx[5] * xx[11] : xx[12];
  xx[5] = (xx[10] + origState[21] * origState[21]) * xx[0] - xx[9];
  xx[7] = - (xx[0] * (origState[21] * origState[22] - origState[20] * origState
                      [23]));
  xx[5] = (xx[7] == 0.0 && xx[5] == 0.0) ? 0.0 : atan2(xx[7], xx[5]);
  xx[0] = bb[1] ? xx[2] * xx[4] : xx[5];
  xx[9] = xx[8];
  xx[10] = xx[8];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[0];
  xx[14] = xx[0];
  xx[15] = xx[0];
  xx[0] = xx[9 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = sin(xx[1]);
  xx[7] = xx[6];
  xx[8] = xx[4];
  xx[9] = xx[6];
  xx[10] = xx[4];
  xx[11] = xx[6];
  xx[12] = xx[4];
  xx[13] = xx[6];
  xx[1] = xx[7 + (partialType)];
  xx[4] = cos(xx[1]);
  xx[6] = sin(xx[1]);
  xx[1] = xx[2] * xx[6];
  xx[7] = xx[4] * xx[2];
  xx[8] = xx[3] * xx[2];
  xx[9] = - (xx[3] * xx[5]);
  xx[10] = xx[0];
  xx[11] = xx[4] * xx[5] + xx[1] * xx[0];
  xx[12] = xx[7] - xx[6] * xx[0] * xx[5];
  xx[13] = - (xx[3] * xx[6]);
  xx[14] = xx[6] * xx[5] - xx[7] * xx[0];
  xx[15] = xx[1] + xx[4] * xx[0] * xx[5];
  xx[16] = xx[4] * xx[3];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 8, xx + 0);
  state[20] = xx[0];
  state[21] = xx[1];
  state[22] = xx[2];
  state[23] = xx[3];
}

void simulation_b048d748_1_projectPartiallyTargetedPos(const void *mech, size_t
  stageIdx, size_t primIdx, const double *origState, int partialType, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    projectPartiallyTargetedPos_0_3(origState, partialType, state);
    break;

   case 15:
    projectPartiallyTargetedPos_2_3(origState, partialType, state);
    break;
  }
}

void simulation_b048d748_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[50];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = - state[3];
  xx[1] = - state[4];
  xx[2] = - state[5];
  xx[3] = - state[6];
  xx[4] = 2.0;
  xx[5] = state[0] + (state[3] * state[5] + state[4] * state[6]) * xx[4];
  xx[6] = state[1] - xx[4] * (state[3] * state[4] - state[5] * state[6]);
  xx[7] = 1.0;
  xx[8] = state[2] - ((state[4] * state[4] + state[5] * state[5]) * xx[4] - xx[7]);
  xx[9] = 0.5;
  xx[10] = xx[9] * state[13];
  xx[11] = cos(xx[10]);
  xx[12] = xx[9] * state[14];
  xx[9] = cos(xx[12]);
  xx[13] = xx[11] * xx[9];
  xx[14] = - xx[13];
  xx[15] = sin(xx[10]);
  xx[10] = xx[9] * xx[15];
  xx[16] = - xx[10];
  xx[17] = sin(xx[12]);
  xx[12] = xx[11] * xx[17];
  xx[11] = - xx[12];
  xx[18] = xx[15] * xx[17];
  xx[15] = - xx[18];
  xx[19] = 0.04499999999999999;
  xx[20] = xx[19] * xx[12];
  xx[21] = xx[19] * xx[10];
  xx[22] = - ((xx[20] * xx[13] + xx[21] * xx[18]) * xx[4]);
  xx[23] = xx[4] * (xx[21] * xx[13] - xx[20] * xx[18]);
  xx[13] = (xx[21] * xx[10] + xx[20] * xx[12]) * xx[4] - xx[19] - xx[7];
  xx[10] = - state[20];
  xx[12] = - state[21];
  xx[18] = - state[22];
  xx[20] = - state[23];
  xx[24] = xx[14];
  xx[25] = xx[16];
  xx[26] = xx[11];
  xx[27] = xx[15];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 24, xx + 28);
  xx[32] = xx[22];
  xx[33] = xx[23];
  xx[34] = xx[13];
  pm_math_Quaternion_xform_ra(xx + 0, xx + 32, xx + 35);
  xx[38] = state[7];
  xx[39] = state[8];
  xx[40] = state[9];
  pm_math_Quaternion_inverseXform_ra(xx + 0, xx + 38, xx + 41);
  xx[21] = xx[41] + state[11];
  xx[38] = xx[42] - state[10];
  xx[39] = state[10];
  xx[40] = state[11];
  xx[41] = state[12];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 39, xx + 44);
  xx[42] = (xx[7] - xx[4] * xx[17] * xx[17]) * state[15];
  pm_math_Vector3_cross_ra(xx + 39, xx + 32, xx + 47);
  xx[32] = xx[47] + xx[21];
  xx[33] = xx[48] + xx[38];
  xx[34] = xx[49] + xx[43];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 32, xx + 39);
  xx[24] = xx[10];
  xx[25] = xx[12];
  xx[26] = xx[18];
  xx[27] = xx[20];
  xx[32] = state[24];
  xx[33] = state[25];
  xx[34] = state[26];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 32, xx + 47);
  motionData[0] = xx[0];
  motionData[1] = xx[1];
  motionData[2] = xx[2];
  motionData[3] = xx[3];
  motionData[4] = xx[5];
  motionData[5] = xx[6];
  motionData[6] = xx[8];
  motionData[7] = xx[14];
  motionData[8] = xx[16];
  motionData[9] = xx[11];
  motionData[10] = xx[15];
  motionData[11] = xx[22];
  motionData[12] = xx[23];
  motionData[13] = xx[13];
  motionData[14] = xx[10];
  motionData[15] = xx[12];
  motionData[16] = xx[18];
  motionData[17] = xx[20];
  motionData[18] = state[17];
  motionData[19] = state[18];
  motionData[20] = state[19];
  motionData[21] = xx[28];
  motionData[22] = xx[29];
  motionData[23] = xx[30];
  motionData[24] = xx[31];
  motionData[25] = xx[35] + xx[5];
  motionData[26] = xx[36] + xx[6];
  motionData[27] = xx[37] + xx[8];
  motionData[28] = state[10];
  motionData[29] = state[11];
  motionData[30] = state[12];
  motionData[31] = xx[21];
  motionData[32] = xx[38];
  motionData[33] = xx[43];
  motionData[34] = xx[44] + xx[42];
  motionData[35] = xx[45] + state[16];
  motionData[36] = xx[46] + xx[4] * xx[9] * xx[17] * state[15];
  motionData[37] = xx[39] - xx[19] * state[16];
  motionData[38] = xx[40] + xx[19] * xx[42];
  motionData[39] = xx[41];
  motionData[40] = state[27];
  motionData[41] = state[28];
  motionData[42] = state[29];
  motionData[43] = xx[47];
  motionData[44] = xx[48];
  motionData[45] = xx[49];
}

static size_t computeAssemblyError_0(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[18];
  (void) rtdvd;
  (void) rtdvi;
  ii[0] = modeVector[0] == -1 ? 0 : 1;
  xx[0] = motionData[21];
  xx[1] = motionData[22];
  xx[2] = motionData[23];
  xx[3] = motionData[24];
  xx[4] = motionData[14];
  xx[5] = motionData[15];
  xx[6] = motionData[16];
  xx[7] = motionData[17];
  pm_math_Quaternion_inverseCompose_ra(xx + 0, xx + 4, xx + 8);
  xx[0] = 0.0225;
  xx[1] = xx[0] * motionData[16];
  xx[2] = xx[0] * motionData[15];
  xx[0] = 2.0;
  xx[3] = 0.04500000000000001;
  xx[4] = xx[3] * motionData[23];
  xx[5] = xx[3] * motionData[22];
  xx[12] = xx[9];
  xx[13] = xx[10];
  xx[14] = xx[11];
  xx[15] = (xx[1] * motionData[14] + xx[2] * motionData[17]) * xx[0] +
    motionData[18] - (motionData[25] - (xx[4] * motionData[21] + xx[5] *
    motionData[24]) * xx[0]);
  xx[16] = xx[0] * (xx[1] * motionData[17] - xx[2] * motionData[14]) +
    motionData[19] - (xx[0] * (xx[5] * motionData[21] - xx[4] * motionData[24])
                      + motionData[26]);
  xx[17] = motionData[20] - (xx[2] * motionData[15] + xx[1] * motionData[16]) *
    xx[0] - (motionData[27] + (xx[5] * motionData[22] + xx[4] * motionData[23]) *
             xx[0]) + 0.0675;
  zeroMajor(1, 6, ii + 0, xx + 12);
  error[0] = xx[12];
  error[1] = xx[13];
  error[2] = xx[14];
  error[3] = xx[15];
  error[4] = xx[16];
  error[5] = xx[17];
  return 6;
}

size_t simulation_b048d748_1_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyError_0(rtdv, modeVector, motionData, error);
  }

  return 0;
}

static size_t computeAssemblyJacobian_0(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  boolean_T bb[1];
  double xx[114];
  (void) rtdvd;
  (void) rtdvi;
  bb[0] = modeVector[0] == -1;
  xx[0] = 0.0;
  xx[1] = motionData[21];
  xx[2] = motionData[22];
  xx[3] = motionData[23];
  xx[4] = motionData[24];
  xx[5] = motionData[14];
  xx[6] = motionData[15];
  xx[7] = motionData[16];
  xx[8] = motionData[17];
  pm_math_Quaternion_inverseCompose_ra(xx + 1, xx + 5, xx + 9);
  xx[13] = xx[0];
  xx[14] = xx[0];
  xx[15] = xx[0];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 13, xx + 16);
  xx[13] = bb[0] ? xx[0] : xx[17];
  xx[14] = 1.0;
  xx[15] = motionData[9] * motionData[9];
  xx[20] = motionData[10] * motionData[10];
  xx[21] = 2.0;
  xx[22] = xx[14] - (xx[15] + xx[20]) * xx[21];
  xx[23] = motionData[8] * motionData[9];
  xx[24] = motionData[7] * motionData[10];
  xx[25] = xx[23] - xx[24];
  xx[26] = motionData[7] * motionData[9];
  xx[27] = motionData[8] * motionData[10];
  xx[28] = xx[22];
  xx[29] = xx[21] * xx[25];
  xx[30] = (xx[26] + xx[27]) * xx[21];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 28, xx + 31);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 31, xx + 28);
  xx[31] = - xx[28];
  xx[32] = - xx[29];
  xx[33] = - xx[30];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 31, xx + 34);
  xx[28] = xx[24] + xx[23];
  xx[23] = motionData[8] * motionData[8];
  xx[24] = xx[14] - (xx[20] + xx[23]) * xx[21];
  xx[20] = motionData[9] * motionData[10];
  xx[29] = motionData[7] * motionData[8];
  xx[30] = xx[28] * xx[21];
  xx[31] = xx[24];
  xx[32] = xx[21] * (xx[20] - xx[29]);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 30, xx + 38);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 38, xx + 30);
  xx[38] = - xx[30];
  xx[39] = - xx[31];
  xx[40] = - xx[32];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 38, xx + 30);
  xx[38] = xx[27] - xx[26];
  xx[26] = xx[29] + xx[20];
  xx[39] = xx[21] * xx[38];
  xx[40] = xx[26] * xx[21];
  xx[41] = xx[14] - (xx[23] + xx[15]) * xx[21];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 39, xx + 42);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 42, xx + 39);
  xx[42] = - xx[39];
  xx[43] = - xx[40];
  xx[44] = - xx[41];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 42, xx + 45);
  xx[15] = 0.5;
  xx[20] = xx[15] * state[13];
  xx[23] = cos(xx[20]);
  xx[27] = xx[15] * state[14];
  xx[15] = sin(xx[27]);
  xx[29] = xx[23] * xx[15];
  xx[39] = sin(xx[20]);
  xx[20] = xx[39] * xx[15];
  xx[15] = xx[14] - (xx[29] * xx[29] + xx[20] * xx[20]) * xx[21];
  xx[40] = cos(xx[27]);
  xx[27] = xx[40] * xx[39];
  xx[39] = xx[23] * xx[40];
  xx[23] = xx[21] * (xx[29] * xx[27] - xx[39] * xx[20]);
  xx[40] = xx[15];
  xx[41] = xx[23];
  xx[42] = (xx[39] * xx[29] + xx[27] * xx[20]) * xx[21];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 40, xx + 49);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 49, xx + 1);
  xx[40] = - xx[1];
  xx[41] = - xx[2];
  xx[42] = - xx[3];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 40, xx + 1);
  xx[40] = xx[21] * (motionData[22] * motionData[23] - motionData[21] *
                     motionData[24]);
  xx[41] = xx[14] - (motionData[24] * motionData[24] + motionData[22] *
                     motionData[22]) * xx[21];
  xx[42] = (motionData[21] * motionData[22] + motionData[23] * motionData[24]) *
    xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 40, xx + 49);
  xx[5] = - xx[49];
  xx[6] = - xx[50];
  xx[7] = - xx[51];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 40);
  xx[5] = xx[14];
  xx[6] = xx[0];
  xx[7] = xx[0];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 49);
  xx[5] = xx[0];
  xx[6] = xx[14];
  xx[7] = xx[0];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 53);
  xx[5] = xx[0];
  xx[6] = xx[0];
  xx[7] = xx[14];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 57);
  xx[5] = bb[0] ? xx[0] : xx[18];
  xx[6] = bb[0] ? xx[0] : xx[19];
  xx[1] = bb[0] ? xx[0] : - xx[14];
  xx[7] = - state[4];
  xx[8] = - state[5];
  xx[9] = - state[6];
  xx[16] = - state[3];
  xx[17] = xx[7];
  xx[18] = xx[8];
  xx[19] = xx[9];
  xx[61] = motionData[7];
  xx[62] = motionData[8];
  xx[63] = motionData[9];
  xx[64] = motionData[10];
  pm_math_Quaternion_compose_ra(xx + 16, xx + 61, xx + 65);
  xx[10] = 0.04500000000000001;
  xx[11] = xx[10] * xx[22];
  xx[12] = xx[68] * xx[11];
  xx[16] = 0.09000000000000001;
  xx[17] = xx[16] * xx[25];
  xx[18] = xx[68] * xx[17];
  xx[19] = xx[66] * xx[11] + xx[67] * xx[17];
  xx[61] = - xx[12];
  xx[62] = - xx[18];
  xx[63] = xx[19];
  pm_math_Vector3_cross_ra(xx + 66, xx + 61, xx + 69);
  xx[22] = motionData[12] * state[5];
  xx[25] = motionData[13] * state[6];
  xx[30] = xx[22] + xx[25];
  xx[34] = motionData[12] * state[4];
  xx[40] = motionData[13] * state[4];
  xx[61] = - xx[30];
  xx[62] = xx[34];
  xx[63] = xx[40];
  pm_math_Vector3_cross_ra(xx + 7, xx + 61, xx + 72);
  xx[44] = state[3] * state[6];
  xx[45] = state[4] * state[5];
  xx[49] = xx[28] * xx[16];
  xx[28] = xx[68] * xx[49];
  xx[53] = xx[10] * xx[24];
  xx[24] = xx[68] * xx[53];
  xx[57] = xx[66] * xx[49] + xx[67] * xx[53];
  xx[61] = - xx[28];
  xx[62] = - xx[24];
  xx[63] = xx[57];
  pm_math_Vector3_cross_ra(xx + 66, xx + 61, xx + 75);
  xx[61] = motionData[11] * state[5];
  xx[62] = motionData[11] * state[4];
  xx[63] = xx[25] + xx[62];
  xx[25] = motionData[13] * state[5];
  xx[78] = xx[61];
  xx[79] = - xx[63];
  xx[80] = xx[25];
  pm_math_Vector3_cross_ra(xx + 7, xx + 78, xx + 81);
  xx[64] = state[6] * state[6];
  xx[78] = xx[16] * xx[38];
  xx[38] = xx[68] * xx[78];
  xx[79] = xx[26] * xx[16];
  xx[16] = xx[68] * xx[79];
  xx[26] = xx[66] * xx[78] + xx[67] * xx[79];
  xx[84] = - xx[38];
  xx[85] = - xx[16];
  xx[86] = xx[26];
  pm_math_Vector3_cross_ra(xx + 66, xx + 84, xx + 87);
  xx[66] = motionData[11] * state[6];
  xx[67] = motionData[12] * state[6];
  xx[68] = xx[62] + xx[22];
  xx[84] = xx[66];
  xx[85] = xx[67];
  xx[86] = - xx[68];
  pm_math_Vector3_cross_ra(xx + 7, xx + 84, xx + 90);
  xx[93] = motionData[0];
  xx[94] = motionData[1];
  xx[95] = motionData[2];
  xx[96] = motionData[3];
  xx[7] = - xx[27];
  xx[8] = - xx[29];
  xx[9] = - xx[20];
  xx[97] = - xx[39];
  xx[98] = xx[7];
  xx[99] = xx[8];
  xx[100] = xx[9];
  pm_math_Quaternion_compose_ra(xx + 93, xx + 97, xx + 101);
  xx[22] = xx[10] * xx[15];
  xx[62] = xx[104] * xx[22];
  xx[80] = xx[10] * xx[23];
  xx[84] = xx[104] * xx[80];
  xx[85] = xx[102] * xx[22] + xx[103] * xx[80];
  xx[97] = - xx[62];
  xx[98] = - xx[84];
  xx[99] = xx[85];
  pm_math_Vector3_cross_ra(xx + 102, xx + 97, xx + 105);
  xx[86] = 0.04499999999999999;
  xx[97] = xx[86] * xx[15];
  xx[15] = xx[97] * xx[20];
  xx[98] = xx[86] * xx[23];
  xx[23] = xx[98] * xx[20];
  xx[99] = xx[97] * xx[27] + xx[98] * xx[29];
  xx[108] = xx[15];
  xx[109] = xx[23];
  xx[110] = - xx[99];
  pm_math_Vector3_cross_ra(xx + 7, xx + 108, xx + 111);
  xx[7] = (xx[111] - xx[39] * xx[15]) * xx[21] - xx[98];
  xx[8] = (xx[112] - xx[39] * xx[23]) * xx[21] + xx[97];
  xx[9] = xx[21] * (xx[113] + xx[99] * xx[39]);
  pm_math_Quaternion_xform_ra(xx + 93, xx + 7, xx + 97);
  xx[7] = xx[86] * xx[29];
  xx[8] = xx[86] * xx[20];
  xx[108] = (xx[7] * xx[29] + xx[8] * xx[20]) * xx[21] - xx[86];
  xx[109] = - ((xx[8] * xx[39] + xx[7] * xx[27]) * xx[21]);
  xx[110] = xx[21] * (xx[7] * xx[39] - xx[8] * xx[27]);
  pm_math_Quaternion_xform_ra(xx + 93, xx + 108, xx + 7);
  xx[15] = xx[10] * xx[103];
  xx[20] = xx[10] * xx[104];
  xx[23] = bb[0] ? xx[0] : xx[14];
  xx[27] = 0.0225;
  xx[29] = xx[27] * state[23];
  xx[39] = xx[29] * state[20];
  xx[86] = xx[27] * state[21];
  xx[93] = xx[27] * state[22];
  xx[94] = xx[29] * state[23];
  J[0] = xx[13];
  J[1] = xx[13];
  J[2] = xx[13];
  J[3] = bb[0] ? xx[0] : xx[35];
  J[4] = bb[0] ? xx[0] : xx[31];
  J[5] = bb[0] ? xx[0] : xx[46];
  J[6] = bb[0] ? xx[0] : xx[2];
  J[7] = bb[0] ? xx[0] : xx[41];
  J[8] = xx[13];
  J[9] = xx[13];
  J[10] = xx[13];
  J[11] = bb[0] ? xx[0] : xx[50];
  J[12] = bb[0] ? xx[0] : xx[54];
  J[13] = bb[0] ? xx[0] : xx[58];
  J[14] = xx[5];
  J[15] = xx[5];
  J[16] = xx[5];
  J[17] = bb[0] ? xx[0] : xx[36];
  J[18] = bb[0] ? xx[0] : xx[32];
  J[19] = bb[0] ? xx[0] : xx[47];
  J[20] = bb[0] ? xx[0] : xx[3];
  J[21] = bb[0] ? xx[0] : xx[42];
  J[22] = xx[5];
  J[23] = xx[5];
  J[24] = xx[5];
  J[25] = bb[0] ? xx[0] : xx[51];
  J[26] = bb[0] ? xx[0] : xx[55];
  J[27] = bb[0] ? xx[0] : xx[59];
  J[28] = xx[6];
  J[29] = xx[6];
  J[30] = xx[6];
  J[31] = bb[0] ? xx[0] : xx[37];
  J[32] = bb[0] ? xx[0] : xx[33];
  J[33] = bb[0] ? xx[0] : xx[48];
  J[34] = bb[0] ? xx[0] : xx[4];
  J[35] = bb[0] ? xx[0] : xx[43];
  J[36] = xx[6];
  J[37] = xx[6];
  J[38] = xx[6];
  J[39] = bb[0] ? xx[0] : xx[52];
  J[40] = bb[0] ? xx[0] : xx[56];
  J[41] = bb[0] ? xx[0] : xx[60];
  J[42] = xx[1];
  J[43] = xx[0];
  J[44] = xx[0];
  J[45] = bb[0] ? xx[0] : - ((xx[69] - xx[65] * xx[12]) * xx[21] - xx[17] + (xx
    [30] * state[3] + xx[72]) * xx[21] + xx[21] * (xx[44] - xx[45]));
  J[46] = bb[0] ? xx[0] : - ((xx[75] - xx[65] * xx[28]) * xx[21] - xx[53] +
    motionData[13] + xx[21] * (xx[81] - xx[61] * state[3]) - (state[5] * state[5]
    + xx[64]) * xx[21] + xx[14]);
  J[47] = bb[0] ? xx[0] : - ((xx[87] - xx[65] * xx[38]) * xx[21] - xx[79] + xx
    [21] * (xx[90] - xx[66] * state[3]) - motionData[12]);
  J[48] = bb[0] ? xx[0] : - ((xx[105] - xx[101] * xx[62]) * xx[21] - xx[80] +
    xx[97]);
  J[49] = bb[0] ? xx[0] : - (xx[7] + (xx[103] * xx[15] + xx[104] * xx[20]) * xx
    [21] - xx[10]);
  J[50] = xx[23];
  J[51] = xx[0];
  J[52] = xx[0];
  J[53] = bb[0] ? xx[0] : xx[21] * (xx[39] - xx[86] * state[22]);
  J[54] = bb[0] ? xx[0] : xx[27] - (xx[93] * state[22] + xx[94]) * xx[21];
  J[55] = xx[0];
  J[56] = xx[0];
  J[57] = xx[1];
  J[58] = xx[0];
  J[59] = bb[0] ? xx[0] : - ((xx[70] - xx[65] * xx[18]) * xx[21] + xx[21] * (xx
    [73] - xx[34] * state[3]) + (xx[64] + state[4] * state[4]) * xx[21] -
    motionData[13] + xx[11] - xx[14]);
  J[60] = bb[0] ? xx[0] : - ((xx[76] - xx[65] * xx[24]) * xx[21] + xx[49] + (xx
    [63] * state[3] + xx[82]) * xx[21] + (xx[44] + xx[45]) * xx[21]);
  J[61] = bb[0] ? xx[0] : - ((xx[88] - xx[65] * xx[16]) * xx[21] + xx[78] +
    motionData[11] + xx[21] * (xx[91] - xx[67] * state[3]));
  J[62] = bb[0] ? xx[0] : - ((xx[106] - xx[101] * xx[84]) * xx[21] + xx[22] +
    xx[98]);
  J[63] = bb[0] ? xx[0] : - (xx[8] - (xx[101] * xx[20] + xx[102] * xx[15]) * xx
    [21]);
  J[64] = xx[0];
  J[65] = xx[23];
  J[66] = xx[0];
  J[67] = bb[0] ? xx[0] : (xx[94] + xx[86] * state[21]) * xx[21] - xx[27];
  J[68] = bb[0] ? xx[0] : (xx[39] + xx[93] * state[21]) * xx[21];
  J[69] = xx[0];
  J[70] = xx[0];
  J[71] = xx[0];
  J[72] = xx[1];
  J[73] = bb[0] ? xx[0] : - (xx[21] * (xx[71] + xx[19] * xx[65]) + motionData[12]
    + xx[21] * (xx[74] - xx[40] * state[3]) - (state[3] * state[4] + state[5] *
    state[6]) * xx[21]);
  J[74] = bb[0] ? xx[0] : - (xx[21] * (xx[77] + xx[57] * xx[65]) + xx[21] * (xx
    [83] - xx[25] * state[3]) - motionData[11] + xx[21] * (state[4] * state[6] -
    state[3] * state[5]));
  J[75] = bb[0] ? xx[0] : - (xx[21] * (xx[89] + xx[26] * xx[65]) + (xx[68] *
    state[3] + xx[92]) * xx[21]);
  J[76] = bb[0] ? xx[0] : - (xx[21] * (xx[107] + xx[85] * xx[101]) + xx[99]);
  J[77] = bb[0] ? xx[0] : - (xx[21] * (xx[101] * xx[15] - xx[102] * xx[20]) +
    xx[9]);
  J[78] = xx[0];
  J[79] = xx[0];
  J[80] = xx[23];
  J[81] = bb[0] ? xx[0] : - ((xx[86] * state[20] + xx[29] * state[22]) * xx[21]);
  J[82] = bb[0] ? xx[0] : xx[21] * (xx[29] * state[21] - xx[93] * state[20]);
  J[83] = xx[0];
  return 6;
}

size_t simulation_b048d748_1_computeAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyJacobian_0(rtdv, state, modeVector, motionData, J);
  }

  return 0;
}

size_t simulation_b048d748_1_computeFullAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  boolean_T bb[1];
  double xx[114];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  bb[0] = modeVector[0] == -1;
  xx[0] = 0.0;
  xx[1] = motionData[21];
  xx[2] = motionData[22];
  xx[3] = motionData[23];
  xx[4] = motionData[24];
  xx[5] = motionData[14];
  xx[6] = motionData[15];
  xx[7] = motionData[16];
  xx[8] = motionData[17];
  pm_math_Quaternion_inverseCompose_ra(xx + 1, xx + 5, xx + 9);
  xx[13] = xx[0];
  xx[14] = xx[0];
  xx[15] = xx[0];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 13, xx + 16);
  xx[13] = bb[0] ? xx[0] : xx[17];
  xx[14] = 1.0;
  xx[15] = motionData[9] * motionData[9];
  xx[20] = motionData[10] * motionData[10];
  xx[21] = 2.0;
  xx[22] = xx[14] - (xx[15] + xx[20]) * xx[21];
  xx[23] = motionData[8] * motionData[9];
  xx[24] = motionData[7] * motionData[10];
  xx[25] = xx[23] - xx[24];
  xx[26] = motionData[7] * motionData[9];
  xx[27] = motionData[8] * motionData[10];
  xx[28] = xx[22];
  xx[29] = xx[21] * xx[25];
  xx[30] = (xx[26] + xx[27]) * xx[21];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 28, xx + 31);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 31, xx + 28);
  xx[31] = - xx[28];
  xx[32] = - xx[29];
  xx[33] = - xx[30];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 31, xx + 34);
  xx[28] = xx[24] + xx[23];
  xx[23] = motionData[8] * motionData[8];
  xx[24] = xx[14] - (xx[20] + xx[23]) * xx[21];
  xx[20] = motionData[9] * motionData[10];
  xx[29] = motionData[7] * motionData[8];
  xx[30] = xx[28] * xx[21];
  xx[31] = xx[24];
  xx[32] = xx[21] * (xx[20] - xx[29]);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 30, xx + 38);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 38, xx + 30);
  xx[38] = - xx[30];
  xx[39] = - xx[31];
  xx[40] = - xx[32];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 38, xx + 30);
  xx[38] = xx[27] - xx[26];
  xx[26] = xx[29] + xx[20];
  xx[39] = xx[21] * xx[38];
  xx[40] = xx[26] * xx[21];
  xx[41] = xx[14] - (xx[23] + xx[15]) * xx[21];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 39, xx + 42);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 42, xx + 39);
  xx[42] = - xx[39];
  xx[43] = - xx[40];
  xx[44] = - xx[41];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 42, xx + 45);
  xx[15] = 0.5;
  xx[20] = xx[15] * state[13];
  xx[23] = cos(xx[20]);
  xx[27] = xx[15] * state[14];
  xx[15] = sin(xx[27]);
  xx[29] = xx[23] * xx[15];
  xx[39] = sin(xx[20]);
  xx[20] = xx[39] * xx[15];
  xx[15] = xx[14] - (xx[29] * xx[29] + xx[20] * xx[20]) * xx[21];
  xx[40] = cos(xx[27]);
  xx[27] = xx[40] * xx[39];
  xx[39] = xx[23] * xx[40];
  xx[23] = xx[21] * (xx[29] * xx[27] - xx[39] * xx[20]);
  xx[40] = xx[15];
  xx[41] = xx[23];
  xx[42] = (xx[39] * xx[29] + xx[27] * xx[20]) * xx[21];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 40, xx + 49);
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 49, xx + 1);
  xx[40] = - xx[1];
  xx[41] = - xx[2];
  xx[42] = - xx[3];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 40, xx + 1);
  xx[40] = xx[21] * (motionData[22] * motionData[23] - motionData[21] *
                     motionData[24]);
  xx[41] = xx[14] - (motionData[24] * motionData[24] + motionData[22] *
                     motionData[22]) * xx[21];
  xx[42] = (motionData[21] * motionData[22] + motionData[23] * motionData[24]) *
    xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 5, xx + 40, xx + 49);
  xx[5] = - xx[49];
  xx[6] = - xx[50];
  xx[7] = - xx[51];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 40);
  xx[5] = xx[14];
  xx[6] = xx[0];
  xx[7] = xx[0];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 49);
  xx[5] = xx[0];
  xx[6] = xx[14];
  xx[7] = xx[0];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 53);
  xx[5] = xx[0];
  xx[6] = xx[0];
  xx[7] = xx[14];
  pm_math_Quaternion_compDeriv_ra(xx + 9, xx + 5, xx + 57);
  xx[5] = bb[0] ? xx[0] : xx[18];
  xx[6] = bb[0] ? xx[0] : xx[19];
  xx[1] = bb[0] ? xx[0] : - xx[14];
  xx[7] = - state[4];
  xx[8] = - state[5];
  xx[9] = - state[6];
  xx[16] = - state[3];
  xx[17] = xx[7];
  xx[18] = xx[8];
  xx[19] = xx[9];
  xx[61] = motionData[7];
  xx[62] = motionData[8];
  xx[63] = motionData[9];
  xx[64] = motionData[10];
  pm_math_Quaternion_compose_ra(xx + 16, xx + 61, xx + 65);
  xx[10] = 0.04500000000000001;
  xx[11] = xx[10] * xx[22];
  xx[12] = xx[68] * xx[11];
  xx[16] = 0.09000000000000001;
  xx[17] = xx[16] * xx[25];
  xx[18] = xx[68] * xx[17];
  xx[19] = xx[66] * xx[11] + xx[67] * xx[17];
  xx[61] = - xx[12];
  xx[62] = - xx[18];
  xx[63] = xx[19];
  pm_math_Vector3_cross_ra(xx + 66, xx + 61, xx + 69);
  xx[22] = motionData[12] * state[5];
  xx[25] = motionData[13] * state[6];
  xx[30] = xx[22] + xx[25];
  xx[34] = motionData[12] * state[4];
  xx[40] = motionData[13] * state[4];
  xx[61] = - xx[30];
  xx[62] = xx[34];
  xx[63] = xx[40];
  pm_math_Vector3_cross_ra(xx + 7, xx + 61, xx + 72);
  xx[44] = state[3] * state[6];
  xx[45] = state[4] * state[5];
  xx[49] = xx[28] * xx[16];
  xx[28] = xx[68] * xx[49];
  xx[53] = xx[10] * xx[24];
  xx[24] = xx[68] * xx[53];
  xx[57] = xx[66] * xx[49] + xx[67] * xx[53];
  xx[61] = - xx[28];
  xx[62] = - xx[24];
  xx[63] = xx[57];
  pm_math_Vector3_cross_ra(xx + 66, xx + 61, xx + 75);
  xx[61] = motionData[11] * state[5];
  xx[62] = motionData[11] * state[4];
  xx[63] = xx[25] + xx[62];
  xx[25] = motionData[13] * state[5];
  xx[78] = xx[61];
  xx[79] = - xx[63];
  xx[80] = xx[25];
  pm_math_Vector3_cross_ra(xx + 7, xx + 78, xx + 81);
  xx[64] = state[6] * state[6];
  xx[78] = xx[16] * xx[38];
  xx[38] = xx[68] * xx[78];
  xx[79] = xx[26] * xx[16];
  xx[16] = xx[68] * xx[79];
  xx[26] = xx[66] * xx[78] + xx[67] * xx[79];
  xx[84] = - xx[38];
  xx[85] = - xx[16];
  xx[86] = xx[26];
  pm_math_Vector3_cross_ra(xx + 66, xx + 84, xx + 87);
  xx[66] = motionData[11] * state[6];
  xx[67] = motionData[12] * state[6];
  xx[68] = xx[62] + xx[22];
  xx[84] = xx[66];
  xx[85] = xx[67];
  xx[86] = - xx[68];
  pm_math_Vector3_cross_ra(xx + 7, xx + 84, xx + 90);
  xx[93] = motionData[0];
  xx[94] = motionData[1];
  xx[95] = motionData[2];
  xx[96] = motionData[3];
  xx[7] = - xx[27];
  xx[8] = - xx[29];
  xx[9] = - xx[20];
  xx[97] = - xx[39];
  xx[98] = xx[7];
  xx[99] = xx[8];
  xx[100] = xx[9];
  pm_math_Quaternion_compose_ra(xx + 93, xx + 97, xx + 101);
  xx[22] = xx[10] * xx[15];
  xx[62] = xx[104] * xx[22];
  xx[80] = xx[10] * xx[23];
  xx[84] = xx[104] * xx[80];
  xx[85] = xx[102] * xx[22] + xx[103] * xx[80];
  xx[97] = - xx[62];
  xx[98] = - xx[84];
  xx[99] = xx[85];
  pm_math_Vector3_cross_ra(xx + 102, xx + 97, xx + 105);
  xx[86] = 0.04499999999999999;
  xx[97] = xx[86] * xx[15];
  xx[15] = xx[97] * xx[20];
  xx[98] = xx[86] * xx[23];
  xx[23] = xx[98] * xx[20];
  xx[99] = xx[97] * xx[27] + xx[98] * xx[29];
  xx[108] = xx[15];
  xx[109] = xx[23];
  xx[110] = - xx[99];
  pm_math_Vector3_cross_ra(xx + 7, xx + 108, xx + 111);
  xx[7] = (xx[111] - xx[39] * xx[15]) * xx[21] - xx[98];
  xx[8] = (xx[112] - xx[39] * xx[23]) * xx[21] + xx[97];
  xx[9] = xx[21] * (xx[113] + xx[99] * xx[39]);
  pm_math_Quaternion_xform_ra(xx + 93, xx + 7, xx + 97);
  xx[7] = xx[86] * xx[29];
  xx[8] = xx[86] * xx[20];
  xx[108] = (xx[7] * xx[29] + xx[8] * xx[20]) * xx[21] - xx[86];
  xx[109] = - ((xx[8] * xx[39] + xx[7] * xx[27]) * xx[21]);
  xx[110] = xx[21] * (xx[7] * xx[39] - xx[8] * xx[27]);
  pm_math_Quaternion_xform_ra(xx + 93, xx + 108, xx + 7);
  xx[15] = xx[10] * xx[103];
  xx[20] = xx[10] * xx[104];
  xx[23] = bb[0] ? xx[0] : xx[14];
  xx[27] = 0.0225;
  xx[29] = xx[27] * state[23];
  xx[39] = xx[29] * state[20];
  xx[86] = xx[27] * state[21];
  xx[93] = xx[27] * state[22];
  xx[94] = xx[29] * state[23];
  J[0] = xx[13];
  J[1] = xx[13];
  J[2] = xx[13];
  J[3] = bb[0] ? xx[0] : xx[35];
  J[4] = bb[0] ? xx[0] : xx[31];
  J[5] = bb[0] ? xx[0] : xx[46];
  J[6] = bb[0] ? xx[0] : xx[2];
  J[7] = bb[0] ? xx[0] : xx[41];
  J[8] = xx[13];
  J[9] = xx[13];
  J[10] = xx[13];
  J[11] = bb[0] ? xx[0] : xx[50];
  J[12] = bb[0] ? xx[0] : xx[54];
  J[13] = bb[0] ? xx[0] : xx[58];
  J[14] = xx[5];
  J[15] = xx[5];
  J[16] = xx[5];
  J[17] = bb[0] ? xx[0] : xx[36];
  J[18] = bb[0] ? xx[0] : xx[32];
  J[19] = bb[0] ? xx[0] : xx[47];
  J[20] = bb[0] ? xx[0] : xx[3];
  J[21] = bb[0] ? xx[0] : xx[42];
  J[22] = xx[5];
  J[23] = xx[5];
  J[24] = xx[5];
  J[25] = bb[0] ? xx[0] : xx[51];
  J[26] = bb[0] ? xx[0] : xx[55];
  J[27] = bb[0] ? xx[0] : xx[59];
  J[28] = xx[6];
  J[29] = xx[6];
  J[30] = xx[6];
  J[31] = bb[0] ? xx[0] : xx[37];
  J[32] = bb[0] ? xx[0] : xx[33];
  J[33] = bb[0] ? xx[0] : xx[48];
  J[34] = bb[0] ? xx[0] : xx[4];
  J[35] = bb[0] ? xx[0] : xx[43];
  J[36] = xx[6];
  J[37] = xx[6];
  J[38] = xx[6];
  J[39] = bb[0] ? xx[0] : xx[52];
  J[40] = bb[0] ? xx[0] : xx[56];
  J[41] = bb[0] ? xx[0] : xx[60];
  J[42] = xx[1];
  J[43] = xx[0];
  J[44] = xx[0];
  J[45] = bb[0] ? xx[0] : - ((xx[69] - xx[65] * xx[12]) * xx[21] - xx[17] + (xx
    [30] * state[3] + xx[72]) * xx[21] + xx[21] * (xx[44] - xx[45]));
  J[46] = bb[0] ? xx[0] : - ((xx[75] - xx[65] * xx[28]) * xx[21] - xx[53] +
    motionData[13] + xx[21] * (xx[81] - xx[61] * state[3]) - (state[5] * state[5]
    + xx[64]) * xx[21] + xx[14]);
  J[47] = bb[0] ? xx[0] : - ((xx[87] - xx[65] * xx[38]) * xx[21] - xx[79] + xx
    [21] * (xx[90] - xx[66] * state[3]) - motionData[12]);
  J[48] = bb[0] ? xx[0] : - ((xx[105] - xx[101] * xx[62]) * xx[21] - xx[80] +
    xx[97]);
  J[49] = bb[0] ? xx[0] : - (xx[7] + (xx[103] * xx[15] + xx[104] * xx[20]) * xx
    [21] - xx[10]);
  J[50] = xx[23];
  J[51] = xx[0];
  J[52] = xx[0];
  J[53] = bb[0] ? xx[0] : xx[21] * (xx[39] - xx[86] * state[22]);
  J[54] = bb[0] ? xx[0] : xx[27] - (xx[93] * state[22] + xx[94]) * xx[21];
  J[55] = xx[0];
  J[56] = xx[0];
  J[57] = xx[1];
  J[58] = xx[0];
  J[59] = bb[0] ? xx[0] : - ((xx[70] - xx[65] * xx[18]) * xx[21] + xx[21] * (xx
    [73] - xx[34] * state[3]) + (xx[64] + state[4] * state[4]) * xx[21] -
    motionData[13] + xx[11] - xx[14]);
  J[60] = bb[0] ? xx[0] : - ((xx[76] - xx[65] * xx[24]) * xx[21] + xx[49] + (xx
    [63] * state[3] + xx[82]) * xx[21] + (xx[44] + xx[45]) * xx[21]);
  J[61] = bb[0] ? xx[0] : - ((xx[88] - xx[65] * xx[16]) * xx[21] + xx[78] +
    motionData[11] + xx[21] * (xx[91] - xx[67] * state[3]));
  J[62] = bb[0] ? xx[0] : - ((xx[106] - xx[101] * xx[84]) * xx[21] + xx[22] +
    xx[98]);
  J[63] = bb[0] ? xx[0] : - (xx[8] - (xx[101] * xx[20] + xx[102] * xx[15]) * xx
    [21]);
  J[64] = xx[0];
  J[65] = xx[23];
  J[66] = xx[0];
  J[67] = bb[0] ? xx[0] : (xx[94] + xx[86] * state[21]) * xx[21] - xx[27];
  J[68] = bb[0] ? xx[0] : (xx[39] + xx[93] * state[21]) * xx[21];
  J[69] = xx[0];
  J[70] = xx[0];
  J[71] = xx[0];
  J[72] = xx[1];
  J[73] = bb[0] ? xx[0] : - (xx[21] * (xx[71] + xx[19] * xx[65]) + motionData[12]
    + xx[21] * (xx[74] - xx[40] * state[3]) - (state[3] * state[4] + state[5] *
    state[6]) * xx[21]);
  J[74] = bb[0] ? xx[0] : - (xx[21] * (xx[77] + xx[57] * xx[65]) + xx[21] * (xx
    [83] - xx[25] * state[3]) - motionData[11] + xx[21] * (state[4] * state[6] -
    state[3] * state[5]));
  J[75] = bb[0] ? xx[0] : - (xx[21] * (xx[89] + xx[26] * xx[65]) + (xx[68] *
    state[3] + xx[92]) * xx[21]);
  J[76] = bb[0] ? xx[0] : - (xx[21] * (xx[107] + xx[85] * xx[101]) + xx[99]);
  J[77] = bb[0] ? xx[0] : - (xx[21] * (xx[101] * xx[15] - xx[102] * xx[20]) +
    xx[9]);
  J[78] = xx[0];
  J[79] = xx[0];
  J[80] = xx[23];
  J[81] = bb[0] ? xx[0] : - ((xx[86] * state[20] + xx[29] * state[22]) * xx[21]);
  J[82] = bb[0] ? xx[0] : xx[21] * (xx[29] * state[21] - xx[93] * state[20]);
  J[83] = xx[0];
  return 6;
}

static boolean_T isInKinematicSingularity_0(const RuntimeDerivedValuesBundle
  *rtdv, const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

boolean_T simulation_b048d748_1_isInKinematicSingularity(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
   case 0:
    return isInKinematicSingularity_0(rtdv, modeVector, motionData);
  }

  return 0;
}

void simulation_b048d748_1_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  xx[0] = - asmState[20];
  xx[1] = - asmState[21];
  xx[2] = - asmState[22];
  xx[3] = - asmState[23];
  xx[4] = asmState[24];
  xx[5] = asmState[25];
  xx[6] = asmState[26];
  pm_math_Quaternion_inverseXform_ra(xx + 0, xx + 4, xx + 7);
  pm_math_Quaternion_xform_ra(xx + 0, xx + 7, xx + 4);
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
  simState[20] = xx[0];
  simState[21] = xx[1];
  simState[22] = xx[2];
  simState[23] = xx[3];
  simState[24] = xx[4];
  simState[25] = xx[5];
  simState[26] = xx[6];
  simState[27] = asmState[27];
  simState[28] = asmState[28];
  simState[29] = asmState[29];
}
