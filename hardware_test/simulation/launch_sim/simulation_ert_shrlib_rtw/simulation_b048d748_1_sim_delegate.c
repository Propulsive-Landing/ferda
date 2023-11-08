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

void simulation_b048d748_1_resetSimStateVector(const void *mech, double *state)
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

static void perturbSimJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbSimJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbSimJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbSimJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_3(double mag, double *state)
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

static void perturbSimJointPrimitiveState_0_3v(double mag, double *state)
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

static void perturbSimJointPrimitiveState_1_0(double mag, double *state)
{
  state[13] = state[13] + mag;
}

static void perturbSimJointPrimitiveState_1_0v(double mag, double *state)
{
  state[13] = state[13] + mag;
  state[15] = state[15] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_1_1(double mag, double *state)
{
  state[14] = state[14] + mag;
}

static void perturbSimJointPrimitiveState_1_1v(double mag, double *state)
{
  state[14] = state[14] + mag;
  state[16] = state[16] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_2_0(double mag, double *state)
{
  state[17] = state[17] + mag;
}

static void perturbSimJointPrimitiveState_2_0v(double mag, double *state)
{
  state[17] = state[17] + mag;
  state[24] = state[24] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_2_1(double mag, double *state)
{
  state[18] = state[18] + mag;
}

static void perturbSimJointPrimitiveState_2_1v(double mag, double *state)
{
  state[18] = state[18] + mag;
  state[25] = state[25] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_2_2(double mag, double *state)
{
  state[19] = state[19] + mag;
}

static void perturbSimJointPrimitiveState_2_2v(double mag, double *state)
{
  state[19] = state[19] + mag;
  state[26] = state[26] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_2_3(double mag, double *state)
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

static void perturbSimJointPrimitiveState_2_3v(double mag, double *state)
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

void simulation_b048d748_1_perturbSimJointPrimitiveState(const void *mech,
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
    perturbSimJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbSimJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbSimJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbSimJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbSimJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbSimJointPrimitiveState_0_2v(mag, state);
    break;

   case 6:
    perturbSimJointPrimitiveState_0_3(mag, state);
    break;

   case 7:
    perturbSimJointPrimitiveState_0_3v(mag, state);
    break;

   case 12:
    perturbSimJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbSimJointPrimitiveState_1_0v(mag, state);
    break;

   case 14:
    perturbSimJointPrimitiveState_1_1(mag, state);
    break;

   case 15:
    perturbSimJointPrimitiveState_1_1v(mag, state);
    break;

   case 24:
    perturbSimJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbSimJointPrimitiveState_2_0v(mag, state);
    break;

   case 26:
    perturbSimJointPrimitiveState_2_1(mag, state);
    break;

   case 27:
    perturbSimJointPrimitiveState_2_1v(mag, state);
    break;

   case 28:
    perturbSimJointPrimitiveState_2_2(mag, state);
    break;

   case 29:
    perturbSimJointPrimitiveState_2_2v(mag, state);
    break;

   case 30:
    perturbSimJointPrimitiveState_2_3(mag, state);
    break;

   case 31:
    perturbSimJointPrimitiveState_2_3v(mag, state);
    break;
  }
}

void simulation_b048d748_1_perturbFlexibleBodyState(const void *mech, size_t
  stageIdx, double mag, boolean_T doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch (stageIdx * 2 + (doPerturbVelocity ? 1 : 0))
  {
  }
}

void simulation_b048d748_1_constructStateVector(const void *mech, const double
  *solverState, const double *u, const double *uDot, double *discreteState,
  double *fullState)
{
  (void) mech;
  (void) discreteState;
  fullState[0] = solverState[0];
  fullState[1] = solverState[1];
  fullState[2] = solverState[2];
  fullState[3] = solverState[3];
  fullState[4] = solverState[4];
  fullState[5] = solverState[5];
  fullState[6] = solverState[6];
  fullState[7] = solverState[7];
  fullState[8] = solverState[8];
  fullState[9] = solverState[9];
  fullState[10] = solverState[10];
  fullState[11] = solverState[11];
  fullState[12] = solverState[12];
  fullState[13] = u[0];
  fullState[14] = u[1];
  fullState[15] = uDot[0];
  fullState[16] = uDot[1];
  fullState[17] = solverState[13];
  fullState[18] = solverState[14];
  fullState[19] = solverState[15];
  fullState[20] = solverState[16];
  fullState[21] = solverState[17];
  fullState[22] = solverState[18];
  fullState[23] = solverState[19];
  fullState[24] = solverState[20];
  fullState[25] = solverState[21];
  fullState[26] = solverState[22];
  fullState[27] = solverState[23];
  fullState[28] = solverState[24];
  fullState[29] = solverState[25];
}

void simulation_b048d748_1_extractSolverStateVector(const void *mech, const
  double *fullState, double *solverState)
{
  (void) mech;
  solverState[0] = fullState[0];
  solverState[1] = fullState[1];
  solverState[2] = fullState[2];
  solverState[3] = fullState[3];
  solverState[4] = fullState[4];
  solverState[5] = fullState[5];
  solverState[6] = fullState[6];
  solverState[7] = fullState[7];
  solverState[8] = fullState[8];
  solverState[9] = fullState[9];
  solverState[10] = fullState[10];
  solverState[11] = fullState[11];
  solverState[12] = fullState[12];
  solverState[13] = fullState[17];
  solverState[14] = fullState[18];
  solverState[15] = fullState[19];
  solverState[16] = fullState[20];
  solverState[17] = fullState[21];
  solverState[18] = fullState[22];
  solverState[19] = fullState[23];
  solverState[20] = fullState[24];
  solverState[21] = fullState[25];
  solverState[22] = fullState[26];
  solverState[23] = fullState[27];
  solverState[24] = fullState[28];
  solverState[25] = fullState[29];
}

boolean_T simulation_b048d748_1_isPositionViolation(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[44];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  ii[0] = modeVector[0] == -1 ? 0 : 1;
  xx[0] = state[20] * state[20];
  xx[1] = 2.0;
  xx[2] = 1.0;
  xx[3] = state[21] * state[22];
  xx[4] = state[20] * state[23];
  xx[5] = state[21] * state[23];
  xx[6] = state[20] * state[22];
  xx[7] = (xx[0] + state[21] * state[21]) * xx[1] - xx[2];
  xx[8] = (xx[3] + xx[4]) * xx[1];
  xx[9] = xx[1] * (xx[5] - xx[6]);
  xx[10] = - state[3];
  xx[11] = - state[4];
  xx[12] = - state[5];
  xx[13] = - state[6];
  xx[14] = 0.5;
  xx[15] = xx[14] * state[13];
  xx[16] = cos(xx[15]);
  xx[17] = xx[14] * state[14];
  xx[14] = cos(xx[17]);
  xx[18] = xx[16] * xx[14];
  xx[19] = sin(xx[15]);
  xx[15] = xx[14] * xx[19];
  xx[14] = sin(xx[17]);
  xx[17] = xx[16] * xx[14];
  xx[16] = xx[19] * xx[14];
  xx[19] = - xx[18];
  xx[20] = - xx[15];
  xx[21] = - xx[17];
  xx[22] = - xx[16];
  pm_math_Quaternion_compose_ra(xx + 10, xx + 19, xx + 23);
  xx[14] = xx[24] * xx[25];
  xx[19] = xx[23] * xx[26];
  xx[20] = xx[23] * xx[23];
  xx[21] = xx[25] * xx[26];
  xx[22] = xx[23] * xx[24];
  xx[27] = xx[1] * (xx[14] - xx[19]);
  xx[28] = (xx[20] + xx[25] * xx[25]) * xx[1] - xx[2];
  xx[29] = (xx[21] + xx[22]) * xx[1];
  xx[30] = state[22] * state[23];
  xx[31] = state[20] * state[21];
  xx[32] = xx[1] * (xx[3] - xx[4]);
  xx[33] = (xx[0] + state[22] * state[22]) * xx[1] - xx[2];
  xx[34] = (xx[30] + xx[31]) * xx[1];
  xx[3] = xx[24] * xx[26];
  xx[4] = xx[23] * xx[25];
  xx[35] = (xx[3] + xx[4]) * xx[1];
  xx[36] = xx[1] * (xx[21] - xx[22]);
  xx[37] = (xx[20] + xx[26] * xx[26]) * xx[1] - xx[2];
  xx[38] = (xx[5] + xx[6]) * xx[1];
  xx[39] = xx[1] * (xx[30] - xx[31]);
  xx[40] = (xx[0] + state[23] * state[23]) * xx[1] - xx[2];
  xx[41] = (xx[20] + xx[24] * xx[24]) * xx[1] - xx[2];
  xx[42] = (xx[14] + xx[19]) * xx[1];
  xx[43] = xx[1] * (xx[3] - xx[4]);
  xx[0] = 0.0225;
  xx[2] = xx[0] * state[22];
  xx[3] = xx[0] * state[21];
  xx[0] = 0.04499999999999999;
  xx[4] = xx[0] * xx[17];
  xx[5] = xx[0] * xx[15];
  xx[19] = - ((xx[4] * xx[18] + xx[5] * xx[16]) * xx[1]);
  xx[20] = xx[1] * (xx[5] * xx[18] - xx[4] * xx[16]);
  xx[21] = (xx[5] * xx[15] + xx[4] * xx[17]) * xx[1] - 1.045;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 19, xx + 4);
  xx[0] = 0.04500000000000001;
  xx[10] = xx[0] * xx[25];
  xx[11] = xx[0] * xx[24];
  xx[12] = pm_math_Vector3_dot_ra(xx + 7, xx + 27);
  xx[13] = pm_math_Vector3_dot_ra(xx + 32, xx + 35);
  xx[14] = pm_math_Vector3_dot_ra(xx + 38, xx + 41);
  xx[15] = (xx[2] * state[20] + xx[3] * state[23]) * xx[1] + state[17] - (xx[4]
    + state[0] + (state[3] * state[5] + state[4] * state[6]) * xx[1] - (xx[23] *
    xx[10] + xx[26] * xx[11]) * xx[1]);
  xx[16] = xx[1] * (xx[2] * state[23] - xx[3] * state[20]) + state[18] - (xx[1] *
    (xx[23] * xx[11] - xx[26] * xx[10]) + xx[5] + state[1] - xx[1] * (state[3] *
    state[4] - state[5] * state[6]));
  xx[17] = state[19] - (xx[3] * state[21] + xx[2] * state[22]) * xx[1] - (xx[6]
    + state[2] - (state[4] * state[4] + state[5] * state[5]) * xx[1] + (xx[24] *
    xx[11] + xx[25] * xx[10]) * xx[1]) - 0.9325;
  zeroMajor(1, 6, ii + 0, xx + 12);
  xx[0] = fabs(xx[12]);
  xx[1] = fabs(xx[13]);
  xx[2] = fabs(xx[14]);
  xx[3] = fabs(xx[15]);
  xx[4] = fabs(xx[16]);
  xx[5] = fabs(xx[17]);
  ii[0] = 0;

  {
    int ll;
    for (ll = 1; ll < 6; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 0;
  xx[6] = xx[0 + (ii[0])];
  return xx[6] > 1.0e-9;
}

boolean_T simulation_b048d748_1_isVelocityViolation(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[91];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  ii[0] = modeVector[0] == -1 ? 0 : 1;
  xx[0] = 2.0;
  xx[1] = - state[21];
  xx[2] = - state[22];
  xx[3] = - state[23];
  xx[4] = state[22] * state[28];
  xx[5] = state[23] * state[29];
  xx[6] = xx[4] + xx[5];
  xx[7] = state[21] * state[28];
  xx[8] = state[21] * state[29];
  xx[9] = xx[6];
  xx[10] = - xx[7];
  xx[11] = - xx[8];
  pm_math_Vector3_cross_ra(xx + 1, xx + 9, xx + 12);
  xx[9] = xx[0] * (xx[12] - xx[6] * state[20]);
  xx[10] = state[29] + (xx[7] * state[20] + xx[13]) * xx[0];
  xx[11] = (xx[8] * state[20] + xx[14]) * xx[0] - state[28];
  xx[6] = - state[4];
  xx[7] = - state[5];
  xx[8] = - state[6];
  xx[12] = - state[3];
  xx[13] = xx[6];
  xx[14] = xx[7];
  xx[15] = xx[8];
  xx[16] = 0.5;
  xx[17] = xx[16] * state[13];
  xx[18] = cos(xx[17]);
  xx[19] = xx[16] * state[14];
  xx[16] = cos(xx[19]);
  xx[20] = xx[18] * xx[16];
  xx[21] = sin(xx[17]);
  xx[17] = xx[16] * xx[21];
  xx[16] = - xx[17];
  xx[22] = sin(xx[19]);
  xx[19] = xx[18] * xx[22];
  xx[18] = - xx[19];
  xx[23] = xx[21] * xx[22];
  xx[21] = - xx[23];
  xx[24] = - xx[20];
  xx[25] = xx[16];
  xx[26] = xx[18];
  xx[27] = xx[21];
  pm_math_Quaternion_compose_ra(xx + 12, xx + 24, xx + 28);
  xx[22] = xx[29] * xx[30];
  xx[32] = xx[28] * xx[31];
  xx[33] = xx[28] * xx[28];
  xx[34] = 1.0;
  xx[35] = xx[30] * xx[31];
  xx[36] = xx[28] * xx[29];
  xx[37] = xx[0] * (xx[22] - xx[32]);
  xx[38] = (xx[33] + xx[30] * xx[30]) * xx[0] - xx[34];
  xx[39] = (xx[35] + xx[36]) * xx[0];
  xx[40] = state[10];
  xx[41] = state[11];
  xx[42] = state[12];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 40, xx + 43);
  xx[24] = (xx[34] - (xx[19] * xx[19] + xx[23] * xx[23]) * xx[0]) * state[15];
  xx[25] = xx[43] + xx[24];
  xx[26] = xx[25] * xx[30];
  xx[27] = xx[45] + xx[0] * (xx[20] * xx[19] + xx[17] * xx[23]) * state[15];
  xx[46] = xx[27] * xx[31];
  xx[47] = xx[25] * xx[29];
  xx[48] = xx[46] + xx[47];
  xx[49] = xx[27] * xx[30];
  xx[50] = xx[26];
  xx[51] = - xx[48];
  xx[52] = xx[49];
  pm_math_Vector3_cross_ra(xx + 29, xx + 50, xx + 53);
  xx[50] = (xx[28] * xx[26] + xx[53]) * xx[0] - xx[27];
  xx[51] = xx[0] * (xx[54] - xx[48] * xx[28]);
  xx[52] = xx[25] + (xx[28] * xx[49] + xx[55]) * xx[0];
  xx[26] = state[20] * state[20];
  xx[48] = state[21] * state[22];
  xx[49] = state[20] * state[23];
  xx[53] = state[21] * state[23];
  xx[54] = state[20] * state[22];
  xx[55] = (xx[26] + state[21] * state[21]) * xx[0] - xx[34];
  xx[56] = (xx[48] + xx[49]) * xx[0];
  xx[57] = xx[0] * (xx[53] - xx[54]);
  xx[58] = state[22] * state[27];
  xx[59] = state[21] * state[27];
  xx[60] = xx[5] + xx[59];
  xx[5] = state[22] * state[29];
  xx[61] = - xx[58];
  xx[62] = xx[60];
  xx[63] = - xx[5];
  pm_math_Vector3_cross_ra(xx + 1, xx + 61, xx + 64);
  xx[61] = (xx[58] * state[20] + xx[64]) * xx[0] - state[29];
  xx[62] = xx[0] * (xx[65] - xx[60] * state[20]);
  xx[63] = state[27] + (xx[5] * state[20] + xx[66]) * xx[0];
  xx[5] = xx[29] * xx[31];
  xx[58] = xx[28] * xx[30];
  xx[64] = (xx[5] + xx[58]) * xx[0];
  xx[65] = xx[0] * (xx[35] - xx[36]);
  xx[66] = (xx[33] + xx[31] * xx[31]) * xx[0] - xx[34];
  xx[35] = xx[0] * (xx[19] * xx[17] - xx[20] * xx[23]) * state[15] + state[16];
  xx[36] = xx[44] + xx[35];
  xx[43] = xx[25] * xx[31];
  xx[44] = xx[36] * xx[31];
  xx[45] = xx[36] * xx[30];
  xx[60] = xx[47] + xx[45];
  xx[67] = xx[43];
  xx[68] = xx[44];
  xx[69] = - xx[60];
  pm_math_Vector3_cross_ra(xx + 29, xx + 67, xx + 70);
  xx[67] = xx[36] + (xx[28] * xx[43] + xx[70]) * xx[0];
  xx[68] = (xx[28] * xx[44] + xx[71]) * xx[0] - xx[25];
  xx[69] = xx[0] * (xx[72] - xx[60] * xx[28]);
  xx[43] = state[22] * state[23];
  xx[44] = state[20] * state[21];
  xx[70] = xx[0] * (xx[48] - xx[49]);
  xx[71] = (xx[26] + state[22] * state[22]) * xx[0] - xx[34];
  xx[72] = (xx[43] + xx[44]) * xx[0];
  xx[47] = state[23] * state[27];
  xx[48] = state[23] * state[28];
  xx[49] = xx[59] + xx[4];
  xx[73] = - xx[47];
  xx[74] = - xx[48];
  xx[75] = xx[49];
  pm_math_Vector3_cross_ra(xx + 1, xx + 73, xx + 76);
  xx[73] = state[28] + (xx[47] * state[20] + xx[76]) * xx[0];
  xx[74] = (xx[48] * state[20] + xx[77]) * xx[0] - state[27];
  xx[75] = xx[0] * (xx[78] - xx[49] * state[20]);
  xx[47] = (xx[33] + xx[29] * xx[29]) * xx[0] - xx[34];
  xx[48] = (xx[22] + xx[32]) * xx[0];
  xx[49] = xx[0] * (xx[5] - xx[58]);
  xx[4] = xx[45] + xx[46];
  xx[5] = xx[36] * xx[29];
  xx[22] = xx[27] * xx[29];
  xx[58] = - xx[4];
  xx[59] = xx[5];
  xx[60] = xx[22];
  pm_math_Vector3_cross_ra(xx + 29, xx + 58, xx + 76);
  xx[58] = xx[0] * (xx[76] - xx[4] * xx[28]);
  xx[59] = xx[27] + (xx[28] * xx[5] + xx[77]) * xx[0];
  xx[60] = (xx[28] * xx[22] + xx[78]) * xx[0] - xx[36];
  xx[76] = (xx[53] + xx[54]) * xx[0];
  xx[77] = xx[0] * (xx[43] - xx[44]);
  xx[78] = (xx[26] + state[23] * state[23]) * xx[0] - xx[34];
  xx[4] = 0.0225;
  xx[5] = xx[4] * state[28];
  xx[22] = xx[4] * state[27];
  xx[4] = xx[22] * state[23];
  xx[26] = xx[5] * state[23];
  xx[27] = xx[22] * state[21] + xx[5] * state[22];
  xx[32] = - xx[4];
  xx[33] = - xx[26];
  xx[34] = xx[27];
  pm_math_Vector3_cross_ra(xx + 1, xx + 32, xx + 43);
  xx[1] = 0.04500000000000001;
  xx[2] = xx[25] * xx[1];
  xx[3] = xx[31] * xx[2];
  xx[25] = xx[36] * xx[1];
  xx[1] = xx[31] * xx[25];
  xx[32] = xx[29] * xx[2] + xx[30] * xx[25];
  xx[79] = - xx[3];
  xx[80] = - xx[1];
  xx[81] = xx[32];
  pm_math_Vector3_cross_ra(xx + 29, xx + 79, xx + 82);
  xx[29] = state[6] * state[10];
  xx[30] = state[6] * state[11];
  xx[31] = state[4] * state[10] + state[5] * state[11];
  xx[79] = - xx[29];
  xx[80] = - xx[30];
  xx[81] = xx[31];
  pm_math_Vector3_cross_ra(xx + 6, xx + 79, xx + 85);
  xx[6] = xx[16];
  xx[7] = xx[18];
  xx[8] = xx[21];
  xx[16] = 0.04499999999999999;
  xx[18] = xx[16] * xx[24];
  xx[21] = xx[18] * xx[23];
  xx[24] = xx[35] * xx[16];
  xx[33] = xx[24] * xx[23];
  xx[34] = xx[18] * xx[17] + xx[24] * xx[19];
  xx[79] = xx[21];
  xx[80] = xx[33];
  xx[81] = - xx[34];
  pm_math_Vector3_cross_ra(xx + 6, xx + 79, xx + 88);
  xx[6] = xx[16] * xx[19];
  xx[7] = xx[16] * xx[17];
  xx[79] = - ((xx[6] * xx[20] + xx[7] * xx[23]) * xx[0]);
  xx[80] = xx[0] * (xx[7] * xx[20] - xx[6] * xx[23]);
  xx[81] = (xx[7] * xx[17] + xx[6] * xx[19]) * xx[0] - 1.045;
  pm_math_Vector3_cross_ra(xx + 40, xx + 79, xx + 6);
  xx[40] = (xx[88] - xx[20] * xx[21]) * xx[0] - xx[24] + xx[6];
  xx[41] = (xx[89] - xx[20] * xx[33]) * xx[0] + xx[18] + xx[7];
  xx[42] = xx[0] * (xx[90] + xx[34] * xx[20]) + xx[8];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 40, xx + 6);
  xx[12] = pm_math_Vector3_dot_ra(xx + 9, xx + 37) + pm_math_Vector3_dot_ra(xx +
    50, xx + 55);
  xx[13] = pm_math_Vector3_dot_ra(xx + 61, xx + 64) + pm_math_Vector3_dot_ra(xx
    + 67, xx + 70);
  xx[14] = pm_math_Vector3_dot_ra(xx + 73, xx + 47) + pm_math_Vector3_dot_ra(xx
    + 58, xx + 76);
  xx[15] = xx[5] + (xx[4] * state[20] + xx[43]) * xx[0] + state[24] - ((xx[82] -
    xx[28] * xx[3]) * xx[0] - xx[25] + state[11] + (xx[29] * state[3] + xx[85]) *
    xx[0] + state[7] + xx[6]);
  xx[16] = (xx[26] * state[20] + xx[44]) * xx[0] - xx[22] + state[25] - ((xx[83]
    - xx[28] * xx[1]) * xx[0] + xx[2] + (xx[30] * state[3] + xx[86]) * xx[0] -
    state[10] + state[8] + xx[7]);
  xx[17] = xx[0] * (xx[45] - xx[27] * state[20]) + state[26] - (xx[0] * (xx[84]
    + xx[32] * xx[28]) + xx[0] * (xx[87] - xx[31] * state[3]) + state[9] + xx[8]);
  zeroMajor(1, 6, ii + 0, xx + 12);
  xx[0] = fabs(xx[12]);
  xx[1] = fabs(xx[13]);
  xx[2] = fabs(xx[14]);
  xx[3] = fabs(xx[15]);
  xx[4] = fabs(xx[16]);
  xx[5] = fabs(xx[17]);
  ii[0] = 0;

  {
    int ll;
    for (ll = 1; ll < 6; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 0;
  xx[6] = xx[0 + (ii[0])];
  return xx[6] > 1.0e-9;
}

PmfMessageId simulation_b048d748_1_projectStateSim(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const int
  *modeVector, double *state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  boolean_T bb[1];
  int ii[7];
  double xx[328];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) neDiagMgr;
  xx[0] = 0.0;
  bb[0] = modeVector[0] == -1;
  xx[1] = sqrt(state[3] * state[3] + state[4] * state[4] + state[5] * state[5] +
               state[6] * state[6]);
  xx[2] = state[3] / xx[1];
  xx[3] = state[4] / xx[1];
  xx[4] = - xx[3];
  xx[5] = state[5] / xx[1];
  xx[6] = - xx[5];
  xx[7] = state[6] / xx[1];
  xx[1] = - xx[7];
  xx[8] = - xx[2];
  xx[9] = xx[4];
  xx[10] = xx[6];
  xx[11] = xx[1];
  xx[12] = 0.5;
  xx[13] = xx[12] * state[13];
  xx[14] = cos(xx[13]);
  xx[15] = xx[12] * state[14];
  xx[16] = cos(xx[15]);
  xx[17] = xx[14] * xx[16];
  xx[18] = sin(xx[13]);
  xx[13] = xx[16] * xx[18];
  xx[16] = sin(xx[15]);
  xx[15] = xx[14] * xx[16];
  xx[14] = xx[18] * xx[16];
  xx[18] = - xx[17];
  xx[19] = - xx[13];
  xx[20] = - xx[15];
  xx[21] = - xx[14];
  pm_math_Quaternion_compose_ra(xx + 8, xx + 18, xx + 22);
  xx[16] = 1.0;
  xx[26] = xx[15] * xx[15];
  xx[27] = xx[14] * xx[14];
  xx[28] = 2.0;
  xx[29] = xx[16] - (xx[26] + xx[27]) * xx[28];
  xx[30] = xx[24] * xx[29];
  xx[31] = xx[17] * xx[15];
  xx[32] = xx[13] * xx[14];
  xx[33] = (xx[31] + xx[32]) * xx[28];
  xx[34] = xx[25] * xx[33];
  xx[35] = xx[23] * xx[29];
  xx[36] = xx[34] + xx[35];
  xx[37] = xx[24] * xx[33];
  xx[38] = xx[30];
  xx[39] = - xx[36];
  xx[40] = xx[37];
  pm_math_Vector3_cross_ra(xx + 23, xx + 38, xx + 41);
  xx[38] = (xx[22] * xx[30] + xx[41]) * xx[28] - xx[33];
  xx[39] = xx[28] * (xx[42] - xx[36] * xx[22]);
  xx[40] = xx[29] + (xx[22] * xx[37] + xx[43]) * xx[28];
  xx[30] = sqrt(state[20] * state[20] + state[21] * state[21] + state[22] *
                state[22] + state[23] * state[23]);
  xx[36] = state[20] / xx[30];
  xx[37] = xx[36] * xx[36];
  xx[41] = state[21] / xx[30];
  xx[42] = xx[41] * xx[41];
  xx[43] = state[22] / xx[30];
  xx[44] = xx[41] * xx[43];
  xx[45] = state[23] / xx[30];
  xx[30] = xx[36] * xx[45];
  xx[46] = xx[41] * xx[45];
  xx[47] = xx[36] * xx[43];
  xx[48] = xx[28] * (xx[46] - xx[47]);
  xx[49] = (xx[37] + xx[42]) * xx[28] - xx[16];
  xx[50] = (xx[44] + xx[30]) * xx[28];
  xx[51] = xx[48];
  xx[52] = xx[17] * xx[14];
  xx[53] = xx[15] * xx[13];
  xx[54] = (xx[52] + xx[53]) * xx[28];
  xx[55] = xx[24] * xx[54];
  xx[56] = xx[15] * xx[14];
  xx[57] = xx[17] * xx[13];
  xx[58] = xx[28] * (xx[56] - xx[57]);
  xx[59] = xx[25] * xx[58];
  xx[60] = xx[23] * xx[54];
  xx[61] = xx[59] + xx[60];
  xx[62] = xx[24] * xx[58];
  xx[63] = xx[55];
  xx[64] = - xx[61];
  xx[65] = xx[62];
  pm_math_Vector3_cross_ra(xx + 23, xx + 63, xx + 66);
  xx[63] = (xx[22] * xx[55] + xx[66]) * xx[28] - xx[58];
  xx[64] = xx[28] * (xx[67] - xx[61] * xx[22]);
  xx[65] = xx[54] + (xx[22] * xx[62] + xx[68]) * xx[28];
  xx[55] = xx[28] * (xx[32] - xx[31]);
  xx[31] = xx[24] * xx[55];
  xx[32] = xx[13] * xx[13];
  xx[61] = xx[16] - (xx[32] + xx[26]) * xx[28];
  xx[26] = xx[25] * xx[61];
  xx[62] = xx[23] * xx[55];
  xx[66] = xx[26] + xx[62];
  xx[67] = xx[24] * xx[61];
  xx[68] = xx[31];
  xx[69] = - xx[66];
  xx[70] = xx[67];
  pm_math_Vector3_cross_ra(xx + 23, xx + 68, xx + 71);
  xx[68] = (xx[22] * xx[31] + xx[71]) * xx[28] - xx[61];
  xx[69] = xx[28] * (xx[72] - xx[66] * xx[22]);
  xx[70] = xx[55] + (xx[22] * xx[67] + xx[73]) * xx[28];
  xx[31] = (xx[47] + xx[46]) * xx[28];
  xx[66] = xx[36] * xx[41];
  xx[67] = xx[43] * xx[45];
  xx[71] = xx[43] * xx[43];
  xx[72] = (xx[42] + xx[71]) * xx[28];
  xx[73] = - xx[31];
  xx[74] = xx[28] * (xx[66] - xx[67]);
  xx[75] = xx[72] - xx[16];
  xx[76] = xx[23] * xx[24];
  xx[77] = xx[22] * xx[25];
  xx[78] = xx[22] * xx[22];
  xx[79] = xx[24] * xx[25];
  xx[80] = xx[22] * xx[23];
  xx[81] = xx[28] * (xx[76] - xx[77]);
  xx[82] = (xx[78] + xx[24] * xx[24]) * xx[28] - xx[16];
  xx[83] = (xx[79] + xx[80]) * xx[28];
  xx[84] = xx[28] * (xx[44] - xx[30]);
  xx[85] = xx[45] * xx[45];
  xx[86] = (xx[85] + xx[42]) * xx[28];
  xx[42] = (xx[66] + xx[67]) * xx[28];
  xx[87] = xx[84];
  xx[88] = xx[16] - xx[86];
  xx[89] = xx[42];
  xx[90] = xx[28] * (xx[53] - xx[52]);
  xx[52] = xx[25] * xx[29];
  xx[53] = xx[25] * xx[90];
  xx[91] = xx[24] * xx[90];
  xx[92] = xx[35] + xx[91];
  xx[93] = xx[52];
  xx[94] = xx[53];
  xx[95] = - xx[92];
  pm_math_Vector3_cross_ra(xx + 23, xx + 93, xx + 96);
  xx[93] = xx[90] + (xx[22] * xx[52] + xx[96]) * xx[28];
  xx[94] = (xx[22] * xx[53] + xx[97]) * xx[28] - xx[29];
  xx[95] = xx[28] * (xx[98] - xx[92] * xx[22]);
  xx[96] = xx[84];
  xx[97] = (xx[37] + xx[71]) * xx[28] - xx[16];
  xx[98] = (xx[67] + xx[66]) * xx[28];
  xx[35] = xx[16] - (xx[27] + xx[32]) * xx[28];
  xx[27] = xx[25] * xx[54];
  xx[32] = xx[25] * xx[35];
  xx[52] = xx[24] * xx[35];
  xx[53] = xx[60] + xx[52];
  xx[99] = xx[27];
  xx[100] = xx[32];
  xx[101] = - xx[53];
  pm_math_Vector3_cross_ra(xx + 23, xx + 99, xx + 102);
  xx[99] = xx[35] + (xx[22] * xx[27] + xx[102]) * xx[28];
  xx[100] = (xx[22] * xx[32] + xx[103]) * xx[28] - xx[54];
  xx[101] = xx[28] * (xx[104] - xx[53] * xx[22]);
  xx[27] = (xx[57] + xx[56]) * xx[28];
  xx[32] = xx[25] * xx[55];
  xx[53] = xx[25] * xx[27];
  xx[56] = xx[24] * xx[27];
  xx[57] = xx[62] + xx[56];
  xx[102] = xx[32];
  xx[103] = xx[53];
  xx[104] = - xx[57];
  pm_math_Vector3_cross_ra(xx + 23, xx + 102, xx + 105);
  xx[102] = xx[27] + (xx[22] * xx[32] + xx[105]) * xx[28];
  xx[103] = (xx[22] * xx[53] + xx[106]) * xx[28] - xx[55];
  xx[104] = xx[28] * (xx[107] - xx[57] * xx[22]);
  xx[32] = xx[28] * (xx[67] - xx[66]);
  xx[105] = xx[31];
  xx[106] = xx[32];
  xx[107] = xx[16] - xx[72];
  xx[31] = xx[23] * xx[25];
  xx[53] = xx[22] * xx[24];
  xx[108] = (xx[31] + xx[53]) * xx[28];
  xx[109] = xx[28] * (xx[79] - xx[80]);
  xx[110] = (xx[78] + xx[25] * xx[25]) * xx[28] - xx[16];
  xx[57] = (xx[71] + xx[85]) * xx[28];
  xx[60] = (xx[30] + xx[44]) * xx[28];
  xx[111] = xx[57] - xx[16];
  xx[112] = - xx[60];
  xx[113] = xx[28] * (xx[47] - xx[46]);
  xx[62] = xx[91] + xx[34];
  xx[34] = xx[23] * xx[90];
  xx[66] = xx[23] * xx[33];
  xx[114] = - xx[62];
  xx[115] = xx[34];
  xx[116] = xx[66];
  pm_math_Vector3_cross_ra(xx + 23, xx + 114, xx + 117);
  xx[114] = xx[28] * (xx[117] - xx[62] * xx[22]);
  xx[115] = xx[33] + (xx[22] * xx[34] + xx[118]) * xx[28];
  xx[116] = (xx[22] * xx[66] + xx[119]) * xx[28] - xx[90];
  xx[117] = (xx[46] + xx[47]) * xx[28];
  xx[118] = xx[32];
  xx[119] = (xx[37] + xx[85]) * xx[28] - xx[16];
  xx[32] = xx[52] + xx[59];
  xx[34] = xx[23] * xx[35];
  xx[37] = xx[23] * xx[58];
  xx[120] = - xx[32];
  xx[121] = xx[34];
  xx[122] = xx[37];
  pm_math_Vector3_cross_ra(xx + 23, xx + 120, xx + 123);
  xx[120] = xx[28] * (xx[123] - xx[32] * xx[22]);
  xx[121] = xx[58] + (xx[22] * xx[34] + xx[124]) * xx[28];
  xx[122] = (xx[22] * xx[37] + xx[125]) * xx[28] - xx[35];
  xx[32] = xx[56] + xx[26];
  xx[26] = xx[23] * xx[27];
  xx[34] = xx[23] * xx[61];
  xx[123] = - xx[32];
  xx[124] = xx[26];
  xx[125] = xx[34];
  pm_math_Vector3_cross_ra(xx + 23, xx + 123, xx + 126);
  xx[123] = xx[28] * (xx[126] - xx[32] * xx[22]);
  xx[124] = xx[61] + (xx[22] * xx[26] + xx[127]) * xx[28];
  xx[125] = (xx[22] * xx[34] + xx[128]) * xx[28] - xx[27];
  xx[126] = xx[28] * (xx[30] - xx[44]);
  xx[127] = xx[86] - xx[16];
  xx[128] = - xx[42];
  xx[84] = (xx[78] + xx[23] * xx[23]) * xx[28] - xx[16];
  xx[85] = (xx[76] + xx[77]) * xx[28];
  xx[86] = xx[28] * (xx[31] - xx[53]);
  xx[30] = xx[16] - xx[57];
  xx[31] = xx[60];
  xx[32] = xx[48];
  xx[26] = bb[0] ? xx[0] : - xx[16];
  xx[34] = 0.04500000000000001;
  xx[37] = xx[34] * xx[29];
  xx[42] = xx[25] * xx[37];
  xx[44] = xx[34] * xx[90];
  xx[46] = xx[25] * xx[44];
  xx[47] = xx[23] * xx[37] + xx[24] * xx[44];
  xx[76] = - xx[42];
  xx[77] = - xx[46];
  xx[78] = xx[47];
  pm_math_Vector3_cross_ra(xx + 23, xx + 76, xx + 129);
  xx[48] = 0.04499999999999999;
  xx[52] = xx[48] * xx[13];
  xx[53] = xx[48] * xx[15];
  xx[56] = xx[28] * (xx[52] * xx[17] - xx[53] * xx[14]);
  xx[57] = xx[5] * xx[56];
  xx[59] = (xx[52] * xx[13] + xx[53] * xx[15]) * xx[28];
  xx[13] = xx[59] - xx[48] - xx[16];
  xx[15] = xx[13] * xx[7];
  xx[60] = xx[57] + xx[15];
  xx[76] = xx[4];
  xx[77] = xx[6];
  xx[78] = xx[1];
  xx[1] = xx[3] * xx[56];
  xx[4] = xx[13] * xx[3];
  xx[132] = - xx[60];
  xx[133] = xx[1];
  xx[134] = xx[4];
  pm_math_Vector3_cross_ra(xx + 76, xx + 132, xx + 135);
  xx[6] = xx[2] * xx[7];
  xx[62] = xx[3] * xx[5];
  xx[66] = xx[34] * xx[54];
  xx[67] = xx[25] * xx[66];
  xx[71] = xx[34] * xx[35];
  xx[72] = xx[25] * xx[71];
  xx[79] = xx[23] * xx[66] + xx[24] * xx[71];
  xx[132] = - xx[67];
  xx[133] = - xx[72];
  xx[134] = xx[79];
  pm_math_Vector3_cross_ra(xx + 23, xx + 132, xx + 138);
  xx[80] = (xx[53] * xx[17] + xx[52] * xx[14]) * xx[28];
  xx[14] = xx[5] * xx[80];
  xx[17] = xx[3] * xx[80];
  xx[52] = xx[15] - xx[17];
  xx[15] = xx[13] * xx[5];
  xx[132] = - xx[14];
  xx[133] = - xx[52];
  xx[134] = xx[15];
  pm_math_Vector3_cross_ra(xx + 76, xx + 132, xx + 141);
  xx[53] = xx[5] * xx[5];
  xx[91] = xx[7] * xx[7];
  xx[92] = xx[34] * xx[55];
  xx[132] = xx[25] * xx[92];
  xx[133] = xx[34] * xx[27];
  xx[134] = xx[25] * xx[133];
  xx[144] = xx[23] * xx[92] + xx[24] * xx[133];
  xx[145] = - xx[132];
  xx[146] = - xx[134];
  xx[147] = xx[144];
  pm_math_Vector3_cross_ra(xx + 23, xx + 145, xx + 148);
  xx[145] = xx[7] * xx[80];
  xx[146] = xx[7] * xx[56];
  xx[147] = xx[57] - xx[17];
  xx[151] = - xx[145];
  xx[152] = xx[146];
  xx[153] = - xx[147];
  pm_math_Vector3_cross_ra(xx + 76, xx + 151, xx + 154);
  xx[17] = bb[0] ? xx[0] : xx[16];
  xx[57] = 0.0225;
  xx[76] = xx[57] * xx[45];
  xx[77] = xx[36] * xx[76];
  xx[78] = xx[57] * xx[41];
  xx[151] = xx[57] * xx[43];
  xx[152] = xx[43] * xx[151];
  xx[153] = xx[45] * xx[76];
  xx[157] = xx[3] * xx[3];
  xx[158] = xx[41] * xx[78];
  xx[159] = xx[2] * xx[3];
  xx[160] = xx[5] * xx[7];
  xx[161] = xx[3] * xx[7];
  xx[162] = xx[2] * xx[5];
  xx[163] = xx[36] * xx[78];
  xx[164] = xx[36] * xx[151];
  xx[165] = xx[0];
  xx[166] = xx[0];
  xx[167] = xx[0];
  xx[168] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 38, xx + 49);
  xx[169] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 63, xx + 49);
  xx[170] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 68, xx + 49);
  xx[171] = xx[0];
  xx[172] = xx[0];
  xx[173] = xx[0];
  xx[174] = xx[0];
  xx[175] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 73, xx + 81);
  xx[176] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 87, xx + 81);
  xx[177] = xx[0];
  xx[178] = xx[0];
  xx[179] = xx[0];
  xx[180] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 93, xx + 96);
  xx[181] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 99, xx + 96);
  xx[182] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 102, xx + 96);
  xx[183] = xx[0];
  xx[184] = xx[0];
  xx[185] = xx[0];
  xx[186] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 105, xx + 108);
  xx[187] = xx[0];
  xx[188] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 111, xx + 108);
  xx[189] = xx[0];
  xx[190] = xx[0];
  xx[191] = xx[0];
  xx[192] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 114, xx + 117);
  xx[193] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 120, xx + 117);
  xx[194] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 123, xx + 117);
  xx[195] = xx[0];
  xx[196] = xx[0];
  xx[197] = xx[0];
  xx[198] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 126, xx + 84);
  xx[199] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 30, xx + 84);
  xx[200] = xx[0];
  xx[201] = xx[26];
  xx[202] = xx[0];
  xx[203] = xx[0];
  xx[204] = bb[0] ? xx[0] : - ((xx[129] - xx[22] * xx[42]) * xx[28] - xx[44] +
    (xx[60] * xx[2] + xx[135]) * xx[28] + xx[28] * (xx[6] - xx[62]));
  xx[205] = bb[0] ? xx[0] : - ((xx[138] - xx[22] * xx[67]) * xx[28] - xx[71] +
    xx[13] + xx[28] * (xx[141] + xx[2] * xx[14]) - (xx[53] + xx[91]) * xx[28] +
    xx[16]);
  xx[206] = bb[0] ? xx[0] : - ((xx[148] - xx[22] * xx[132]) * xx[28] - xx[133] +
    xx[28] * (xx[154] + xx[2] * xx[145]) - xx[56]);
  xx[207] = xx[17];
  xx[208] = xx[0];
  xx[209] = xx[0];
  xx[210] = bb[0] ? xx[0] : xx[28] * (xx[77] - xx[43] * xx[78]);
  xx[211] = bb[0] ? xx[0] : xx[57] - (xx[152] + xx[153]) * xx[28];
  xx[212] = xx[0];
  xx[213] = xx[0];
  xx[214] = xx[26];
  xx[215] = xx[0];
  xx[216] = bb[0] ? xx[0] : - ((xx[130] - xx[22] * xx[46]) * xx[28] + xx[28] *
    (xx[136] - xx[2] * xx[1]) + (xx[91] + xx[157]) * xx[28] - xx[13] + xx[37] -
    xx[16]);
  xx[217] = bb[0] ? xx[0] : - ((xx[139] - xx[22] * xx[72]) * xx[28] + xx[66] +
    (xx[52] * xx[2] + xx[142]) * xx[28] + (xx[6] + xx[62]) * xx[28]);
  xx[218] = bb[0] ? xx[0] : - ((xx[149] - xx[22] * xx[134]) * xx[28] + xx[92] +
    xx[28] * (xx[155] - xx[2] * xx[146]) - xx[80]);
  xx[219] = xx[0];
  xx[220] = xx[17];
  xx[221] = xx[0];
  xx[222] = bb[0] ? xx[0] : (xx[153] + xx[158]) * xx[28] - xx[57];
  xx[223] = bb[0] ? xx[0] : (xx[77] + xx[41] * xx[151]) * xx[28];
  xx[224] = xx[0];
  xx[225] = xx[0];
  xx[226] = xx[0];
  xx[227] = xx[26];
  xx[228] = bb[0] ? xx[0] : - (xx[28] * (xx[131] + xx[47] * xx[22]) + xx[56] +
    xx[28] * (xx[137] - xx[2] * xx[4]) - (xx[159] + xx[160]) * xx[28]);
  xx[229] = bb[0] ? xx[0] : - (xx[28] * (xx[140] + xx[79] * xx[22]) + xx[28] *
    (xx[143] - xx[2] * xx[15]) + xx[80] + xx[28] * (xx[161] - xx[162]));
  xx[230] = bb[0] ? xx[0] : - (xx[28] * (xx[150] + xx[144] * xx[22]) + (xx[147] *
    xx[2] + xx[156]) * xx[28]);
  xx[231] = xx[0];
  xx[232] = xx[0];
  xx[233] = xx[17];
  xx[234] = bb[0] ? xx[0] : - ((xx[163] + xx[43] * xx[76]) * xx[28]);
  xx[235] = bb[0] ? xx[0] : xx[28] * (xx[41] * xx[76] - xx[164]);
  xx[236] = xx[0];
  ii[0] = bb[0] ? 0 : 1;
  xx[1] = - xx[80];
  xx[30] = xx[1];
  xx[31] = xx[56];
  xx[32] = xx[13];
  pm_math_Quaternion_xform_ra(xx + 8, xx + 30, xx + 38);
  xx[4] = xx[34] * xx[24];
  xx[6] = xx[34] * xx[23];
  xx[8] = 0.9325;
  xx[72] = pm_math_Vector3_dot_ra(xx + 49, xx + 81);
  xx[73] = pm_math_Vector3_dot_ra(xx + 96, xx + 108);
  xx[74] = pm_math_Vector3_dot_ra(xx + 117, xx + 84);
  xx[75] = (xx[164] + xx[45] * xx[78]) * xx[28] + state[17] - (xx[38] + state[0]
    + (xx[162] + xx[161]) * xx[28] - (xx[22] * xx[4] + xx[25] * xx[6]) * xx[28]);
  xx[76] = xx[28] * (xx[45] * xx[151] - xx[163]) + state[18] - (xx[28] * (xx[22]
    * xx[6] - xx[25] * xx[4]) + xx[39] + state[1] - xx[28] * (xx[159] - xx[160]));
  xx[77] = state[19] - (xx[158] + xx[152]) * xx[28] - (xx[40] + state[2] - (xx
    [157] + xx[53]) * xx[28] + (xx[23] * xx[6] + xx[24] * xx[4]) * xx[28]) - xx
    [8];
  zeroMajor(1, 6, ii + 0, xx + 72);
  xx[81] = - xx[72];
  xx[82] = - xx[73];
  xx[83] = - xx[74];
  xx[84] = - xx[75];
  xx[85] = - xx[76];
  xx[86] = - xx[77];
  xx[4] = 1.0e-8;
  memcpy(xx + 237, xx + 165, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 237, xx + 72, xx + 105, ii + 1, xx + 81, xx[4],
                     xx + 93);
  xx[6] = state[0] + xx[93];
  xx[22] = xx[2];
  xx[23] = xx[3];
  xx[24] = xx[5];
  xx[25] = xx[7];
  pm_math_Quaternion_compDeriv_ra(xx + 22, xx + 96, xx + 49);
  xx[9] = xx[2] + xx[49];
  xx[2] = xx[3] + xx[50];
  xx[3] = xx[5] + xx[51];
  xx[5] = xx[7] + xx[52];
  xx[7] = 1.0e-64;
  xx[10] = sqrt(xx[9] * xx[9] + xx[2] * xx[2] + xx[3] * xx[3] + xx[5] * xx[5]);
  if (xx[7] > xx[10])
    xx[10] = xx[7];
  xx[11] = xx[9] / xx[10];
  xx[9] = xx[2] / xx[10];
  xx[2] = - xx[9];
  xx[14] = xx[3] / xx[10];
  xx[3] = - xx[14];
  xx[15] = xx[5] / xx[10];
  xx[5] = - xx[15];
  xx[22] = - xx[11];
  xx[23] = xx[2];
  xx[24] = xx[3];
  xx[25] = xx[5];
  pm_math_Quaternion_compose_ra(xx + 22, xx + 18, xx + 49);
  xx[10] = xx[51] * xx[29];
  xx[38] = xx[52] * xx[33];
  xx[39] = xx[50] * xx[29];
  xx[40] = xx[38] + xx[39];
  xx[42] = xx[51] * xx[33];
  xx[62] = xx[10];
  xx[63] = - xx[40];
  xx[64] = xx[42];
  pm_math_Vector3_cross_ra(xx + 50, xx + 62, xx + 67);
  xx[62] = (xx[49] * xx[10] + xx[67]) * xx[28] - xx[33];
  xx[63] = xx[28] * (xx[68] - xx[40] * xx[49]);
  xx[64] = xx[29] + (xx[49] * xx[42] + xx[69]) * xx[28];
  xx[67] = xx[36];
  xx[68] = xx[41];
  xx[69] = xx[43];
  xx[70] = xx[45];
  pm_math_Quaternion_compDeriv_ra(xx + 67, xx + 102, xx + 72);
  xx[10] = xx[36] + xx[72];
  xx[36] = xx[41] + xx[73];
  xx[40] = xx[43] + xx[74];
  xx[41] = xx[45] + xx[75];
  xx[42] = sqrt(xx[10] * xx[10] + xx[36] * xx[36] + xx[40] * xx[40] + xx[41] *
                xx[41]);
  if (xx[7] > xx[42])
    xx[42] = xx[7];
  xx[43] = xx[10] / xx[42];
  xx[10] = xx[43] * xx[43];
  xx[45] = xx[36] / xx[42];
  xx[36] = xx[45] * xx[45];
  xx[46] = xx[40] / xx[42];
  xx[40] = xx[45] * xx[46];
  xx[47] = xx[41] / xx[42];
  xx[41] = xx[43] * xx[47];
  xx[42] = xx[45] * xx[47];
  xx[53] = xx[43] * xx[46];
  xx[60] = xx[28] * (xx[42] - xx[53]);
  xx[67] = (xx[10] + xx[36]) * xx[28] - xx[16];
  xx[68] = (xx[40] + xx[41]) * xx[28];
  xx[69] = xx[60];
  xx[65] = xx[51] * xx[54];
  xx[70] = xx[52] * xx[58];
  xx[72] = xx[50] * xx[54];
  xx[73] = xx[70] + xx[72];
  xx[74] = xx[51] * xx[58];
  xx[75] = xx[65];
  xx[76] = - xx[73];
  xx[77] = xx[74];
  pm_math_Vector3_cross_ra(xx + 50, xx + 75, xx + 81);
  xx[75] = (xx[49] * xx[65] + xx[81]) * xx[28] - xx[58];
  xx[76] = xx[28] * (xx[82] - xx[73] * xx[49]);
  xx[77] = xx[54] + (xx[49] * xx[74] + xx[83]) * xx[28];
  xx[65] = xx[51] * xx[55];
  xx[73] = xx[52] * xx[61];
  xx[74] = xx[50] * xx[55];
  xx[78] = xx[73] + xx[74];
  xx[79] = xx[51] * xx[61];
  xx[81] = xx[65];
  xx[82] = - xx[78];
  xx[83] = xx[79];
  pm_math_Vector3_cross_ra(xx + 50, xx + 81, xx + 84);
  xx[81] = (xx[49] * xx[65] + xx[84]) * xx[28] - xx[61];
  xx[82] = xx[28] * (xx[85] - xx[78] * xx[49]);
  xx[83] = xx[55] + (xx[49] * xx[79] + xx[86]) * xx[28];
  xx[65] = (xx[53] + xx[42]) * xx[28];
  xx[78] = xx[43] * xx[45];
  xx[79] = xx[46] * xx[47];
  xx[84] = xx[46] * xx[46];
  xx[85] = (xx[36] + xx[84]) * xx[28];
  xx[86] = - xx[65];
  xx[87] = xx[28] * (xx[78] - xx[79]);
  xx[88] = xx[85] - xx[16];
  xx[89] = xx[50] * xx[51];
  xx[91] = xx[49] * xx[52];
  xx[105] = xx[49] * xx[49];
  xx[106] = xx[51] * xx[52];
  xx[107] = xx[49] * xx[50];
  xx[108] = xx[28] * (xx[89] - xx[91]);
  xx[109] = (xx[105] + xx[51] * xx[51]) * xx[28] - xx[16];
  xx[110] = (xx[106] + xx[107]) * xx[28];
  xx[111] = xx[28] * (xx[40] - xx[41]);
  xx[112] = xx[47] * xx[47];
  xx[113] = (xx[112] + xx[36]) * xx[28];
  xx[36] = (xx[78] + xx[79]) * xx[28];
  xx[114] = xx[111];
  xx[115] = xx[16] - xx[113];
  xx[116] = xx[36];
  xx[117] = xx[52] * xx[29];
  xx[118] = xx[52] * xx[90];
  xx[119] = xx[51] * xx[90];
  xx[120] = xx[39] + xx[119];
  xx[121] = xx[117];
  xx[122] = xx[118];
  xx[123] = - xx[120];
  pm_math_Vector3_cross_ra(xx + 50, xx + 121, xx + 124);
  xx[121] = xx[90] + (xx[49] * xx[117] + xx[124]) * xx[28];
  xx[122] = (xx[49] * xx[118] + xx[125]) * xx[28] - xx[29];
  xx[123] = xx[28] * (xx[126] - xx[120] * xx[49]);
  xx[124] = xx[111];
  xx[125] = (xx[10] + xx[84]) * xx[28] - xx[16];
  xx[126] = (xx[79] + xx[78]) * xx[28];
  xx[29] = xx[52] * xx[54];
  xx[39] = xx[52] * xx[35];
  xx[111] = xx[51] * xx[35];
  xx[117] = xx[72] + xx[111];
  xx[127] = xx[29];
  xx[128] = xx[39];
  xx[129] = - xx[117];
  pm_math_Vector3_cross_ra(xx + 50, xx + 127, xx + 130);
  xx[127] = xx[35] + (xx[49] * xx[29] + xx[130]) * xx[28];
  xx[128] = (xx[49] * xx[39] + xx[131]) * xx[28] - xx[54];
  xx[129] = xx[28] * (xx[132] - xx[117] * xx[49]);
  xx[29] = xx[52] * xx[55];
  xx[39] = xx[52] * xx[27];
  xx[54] = xx[51] * xx[27];
  xx[72] = xx[74] + xx[54];
  xx[130] = xx[29];
  xx[131] = xx[39];
  xx[132] = - xx[72];
  pm_math_Vector3_cross_ra(xx + 50, xx + 130, xx + 134);
  xx[130] = xx[27] + (xx[49] * xx[29] + xx[134]) * xx[28];
  xx[131] = (xx[49] * xx[39] + xx[135]) * xx[28] - xx[55];
  xx[132] = xx[28] * (xx[136] - xx[72] * xx[49]);
  xx[29] = xx[28] * (xx[79] - xx[78]);
  xx[134] = xx[65];
  xx[135] = xx[29];
  xx[136] = xx[16] - xx[85];
  xx[39] = xx[50] * xx[52];
  xx[55] = xx[49] * xx[51];
  xx[137] = (xx[39] + xx[55]) * xx[28];
  xx[138] = xx[28] * (xx[106] - xx[107]);
  xx[139] = (xx[105] + xx[52] * xx[52]) * xx[28] - xx[16];
  xx[65] = (xx[84] + xx[112]) * xx[28];
  xx[72] = (xx[41] + xx[40]) * xx[28];
  xx[140] = xx[65] - xx[16];
  xx[141] = - xx[72];
  xx[142] = xx[28] * (xx[53] - xx[42]);
  xx[74] = xx[119] + xx[38];
  xx[38] = xx[50] * xx[90];
  xx[78] = xx[50] * xx[33];
  xx[117] = - xx[74];
  xx[118] = xx[38];
  xx[119] = xx[78];
  pm_math_Vector3_cross_ra(xx + 50, xx + 117, xx + 143);
  xx[117] = xx[28] * (xx[143] - xx[74] * xx[49]);
  xx[118] = xx[33] + (xx[49] * xx[38] + xx[144]) * xx[28];
  xx[119] = (xx[49] * xx[78] + xx[145]) * xx[28] - xx[90];
  xx[143] = (xx[42] + xx[53]) * xx[28];
  xx[144] = xx[29];
  xx[145] = (xx[10] + xx[112]) * xx[28] - xx[16];
  xx[10] = xx[111] + xx[70];
  xx[29] = xx[50] * xx[35];
  xx[33] = xx[50] * xx[58];
  xx[146] = - xx[10];
  xx[147] = xx[29];
  xx[148] = xx[33];
  pm_math_Vector3_cross_ra(xx + 50, xx + 146, xx + 149);
  xx[146] = xx[28] * (xx[149] - xx[10] * xx[49]);
  xx[147] = xx[58] + (xx[49] * xx[29] + xx[150]) * xx[28];
  xx[148] = (xx[49] * xx[33] + xx[151]) * xx[28] - xx[35];
  xx[10] = xx[54] + xx[73];
  xx[29] = xx[50] * xx[27];
  xx[33] = xx[50] * xx[61];
  xx[149] = - xx[10];
  xx[150] = xx[29];
  xx[151] = xx[33];
  pm_math_Vector3_cross_ra(xx + 50, xx + 149, xx + 152);
  xx[149] = xx[28] * (xx[152] - xx[10] * xx[49]);
  xx[150] = xx[61] + (xx[49] * xx[29] + xx[153]) * xx[28];
  xx[151] = (xx[49] * xx[33] + xx[154]) * xx[28] - xx[27];
  xx[152] = xx[28] * (xx[41] - xx[40]);
  xx[153] = xx[113] - xx[16];
  xx[154] = - xx[36];
  xx[40] = (xx[105] + xx[50] * xx[50]) * xx[28] - xx[16];
  xx[41] = (xx[89] + xx[91]) * xx[28];
  xx[42] = xx[28] * (xx[39] - xx[55]);
  xx[53] = xx[16] - xx[65];
  xx[54] = xx[72];
  xx[55] = xx[60];
  xx[10] = xx[52] * xx[37];
  xx[27] = xx[52] * xx[44];
  xx[29] = xx[50] * xx[37] + xx[51] * xx[44];
  xx[72] = - xx[10];
  xx[73] = - xx[27];
  xx[74] = xx[29];
  pm_math_Vector3_cross_ra(xx + 50, xx + 72, xx + 89);
  xx[33] = xx[14] * xx[56];
  xx[35] = xx[13] * xx[15];
  xx[36] = xx[33] + xx[35];
  xx[72] = xx[2];
  xx[73] = xx[3];
  xx[74] = xx[5];
  xx[2] = xx[9] * xx[56];
  xx[3] = xx[13] * xx[9];
  xx[105] = - xx[36];
  xx[106] = xx[2];
  xx[107] = xx[3];
  pm_math_Vector3_cross_ra(xx + 72, xx + 105, xx + 111);
  xx[5] = xx[11] * xx[15];
  xx[38] = xx[9] * xx[14];
  xx[39] = xx[52] * xx[66];
  xx[58] = xx[52] * xx[71];
  xx[60] = xx[50] * xx[66] + xx[51] * xx[71];
  xx[105] = - xx[39];
  xx[106] = - xx[58];
  xx[107] = xx[60];
  pm_math_Vector3_cross_ra(xx + 50, xx + 105, xx + 155);
  xx[61] = xx[14] * xx[80];
  xx[65] = xx[9] * xx[80];
  xx[70] = xx[35] - xx[65];
  xx[35] = xx[13] * xx[14];
  xx[105] = - xx[61];
  xx[106] = - xx[70];
  xx[107] = xx[35];
  pm_math_Vector3_cross_ra(xx + 72, xx + 105, xx + 158);
  xx[78] = xx[14] * xx[14];
  xx[79] = xx[15] * xx[15];
  xx[84] = xx[52] * xx[92];
  xx[85] = xx[52] * xx[133];
  xx[105] = xx[50] * xx[92] + xx[51] * xx[133];
  xx[161] = - xx[84];
  xx[162] = - xx[85];
  xx[163] = xx[105];
  pm_math_Vector3_cross_ra(xx + 50, xx + 161, xx + 164);
  xx[106] = xx[15] * xx[80];
  xx[107] = xx[15] * xx[56];
  xx[120] = xx[33] - xx[65];
  xx[161] = - xx[106];
  xx[162] = xx[107];
  xx[163] = - xx[120];
  pm_math_Vector3_cross_ra(xx + 72, xx + 161, xx + 167);
  xx[33] = xx[57] * xx[47];
  xx[65] = xx[43] * xx[33];
  xx[72] = xx[57] * xx[45];
  xx[73] = xx[57] * xx[46];
  xx[74] = xx[46] * xx[73];
  xx[161] = xx[47] * xx[33];
  xx[162] = xx[9] * xx[9];
  xx[163] = xx[45] * xx[72];
  xx[170] = xx[11] * xx[9];
  xx[171] = xx[14] * xx[15];
  xx[172] = xx[9] * xx[15];
  xx[173] = xx[11] * xx[14];
  xx[174] = xx[43] * xx[72];
  xx[175] = xx[43] * xx[73];
  xx[176] = xx[0];
  xx[177] = xx[0];
  xx[178] = xx[0];
  xx[179] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 62, xx + 67);
  xx[180] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 75, xx + 67);
  xx[181] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 81, xx + 67);
  xx[182] = xx[0];
  xx[183] = xx[0];
  xx[184] = xx[0];
  xx[185] = xx[0];
  xx[186] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 86, xx + 108);
  xx[187] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 114, xx + 108);
  xx[188] = xx[0];
  xx[189] = xx[0];
  xx[190] = xx[0];
  xx[191] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 121, xx + 124);
  xx[192] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 127, xx + 124);
  xx[193] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 130, xx + 124);
  xx[194] = xx[0];
  xx[195] = xx[0];
  xx[196] = xx[0];
  xx[197] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 134, xx + 137);
  xx[198] = xx[0];
  xx[199] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 140, xx + 137);
  xx[200] = xx[0];
  xx[201] = xx[0];
  xx[202] = xx[0];
  xx[203] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 117, xx + 143);
  xx[204] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 146, xx + 143);
  xx[205] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 149, xx + 143);
  xx[206] = xx[0];
  xx[207] = xx[0];
  xx[208] = xx[0];
  xx[209] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 152, xx + 40);
  xx[210] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 53, xx + 40);
  xx[211] = xx[0];
  xx[212] = xx[26];
  xx[213] = xx[0];
  xx[214] = xx[0];
  xx[215] = bb[0] ? xx[0] : - ((xx[89] - xx[49] * xx[10]) * xx[28] - xx[44] +
    (xx[36] * xx[11] + xx[111]) * xx[28] + xx[28] * (xx[5] - xx[38]));
  xx[216] = bb[0] ? xx[0] : - ((xx[155] - xx[49] * xx[39]) * xx[28] - xx[71] +
    xx[13] + xx[28] * (xx[158] + xx[11] * xx[61]) - (xx[78] + xx[79]) * xx[28] +
    xx[16]);
  xx[217] = bb[0] ? xx[0] : - ((xx[164] - xx[49] * xx[84]) * xx[28] - xx[133] +
    xx[28] * (xx[167] + xx[11] * xx[106]) - xx[56]);
  xx[218] = xx[17];
  xx[219] = xx[0];
  xx[220] = xx[0];
  xx[221] = bb[0] ? xx[0] : xx[28] * (xx[65] - xx[46] * xx[72]);
  xx[222] = bb[0] ? xx[0] : xx[57] - (xx[74] + xx[161]) * xx[28];
  xx[223] = xx[0];
  xx[224] = xx[0];
  xx[225] = xx[26];
  xx[226] = xx[0];
  xx[227] = bb[0] ? xx[0] : - ((xx[90] - xx[49] * xx[27]) * xx[28] + xx[28] *
    (xx[112] - xx[11] * xx[2]) + (xx[79] + xx[162]) * xx[28] - xx[13] + xx[37] -
    xx[16]);
  xx[228] = bb[0] ? xx[0] : - ((xx[156] - xx[49] * xx[58]) * xx[28] + xx[66] +
    (xx[70] * xx[11] + xx[159]) * xx[28] + (xx[5] + xx[38]) * xx[28]);
  xx[229] = bb[0] ? xx[0] : - ((xx[165] - xx[49] * xx[85]) * xx[28] + xx[92] +
    xx[28] * (xx[168] - xx[11] * xx[107]) - xx[80]);
  xx[230] = xx[0];
  xx[231] = xx[17];
  xx[232] = xx[0];
  xx[233] = bb[0] ? xx[0] : (xx[161] + xx[163]) * xx[28] - xx[57];
  xx[234] = bb[0] ? xx[0] : (xx[65] + xx[45] * xx[73]) * xx[28];
  xx[235] = xx[0];
  xx[236] = xx[0];
  xx[237] = xx[0];
  xx[238] = xx[26];
  xx[239] = bb[0] ? xx[0] : - (xx[28] * (xx[91] + xx[29] * xx[49]) + xx[56] +
    xx[28] * (xx[113] - xx[11] * xx[3]) - (xx[170] + xx[171]) * xx[28]);
  xx[240] = bb[0] ? xx[0] : - (xx[28] * (xx[157] + xx[60] * xx[49]) + xx[28] *
    (xx[160] - xx[11] * xx[35]) + xx[80] + xx[28] * (xx[172] - xx[173]));
  xx[241] = bb[0] ? xx[0] : - (xx[28] * (xx[166] + xx[105] * xx[49]) + (xx[120] *
    xx[11] + xx[169]) * xx[28]);
  xx[242] = xx[0];
  xx[243] = xx[0];
  xx[244] = xx[17];
  xx[245] = bb[0] ? xx[0] : - ((xx[174] + xx[46] * xx[33]) * xx[28]);
  xx[246] = bb[0] ? xx[0] : xx[28] * (xx[45] * xx[33] - xx[175]);
  xx[247] = xx[0];
  xx[2] = state[17] + xx[99];
  pm_math_Quaternion_xform_ra(xx + 22, xx + 30, xx + 35);
  xx[3] = xx[34] * xx[51];
  xx[5] = xx[34] * xx[50];
  xx[10] = state[18] + xx[100];
  xx[13] = state[1] + xx[94];
  xx[22] = state[19] + xx[101];
  xx[23] = state[2] + xx[95];
  xx[60] = pm_math_Vector3_dot_ra(xx + 67, xx + 108);
  xx[61] = pm_math_Vector3_dot_ra(xx + 124, xx + 137);
  xx[62] = pm_math_Vector3_dot_ra(xx + 143, xx + 40);
  xx[63] = (xx[175] + xx[47] * xx[72]) * xx[28] + xx[2] - (xx[35] + xx[6] + (xx
    [173] + xx[172]) * xx[28] - (xx[49] * xx[3] + xx[52] * xx[5]) * xx[28]);
  xx[64] = xx[28] * (xx[47] * xx[73] - xx[174]) + xx[10] - (xx[28] * (xx[49] *
    xx[5] - xx[52] * xx[3]) + xx[36] + xx[13] - xx[28] * (xx[170] - xx[171]));
  xx[65] = xx[57] - (xx[163] + xx[74]) * xx[28] + xx[22] - (xx[37] + xx[23] -
    ((xx[162] + xx[78]) * xx[28] - xx[16]) + (xx[50] * xx[5] + xx[51] * xx[3]) *
    xx[28]) + xx[34];
  zeroMajor(1, 6, ii + 0, xx + 60);
  xx[35] = - xx[60];
  xx[36] = - xx[61];
  xx[37] = - xx[62];
  xx[38] = - xx[63];
  xx[39] = - xx[64];
  xx[40] = - xx[65];
  memcpy(xx + 72, xx + 176, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 72, xx + 49, xx + 144, ii + 1, xx + 35, xx[4],
                     xx + 60);
  xx[3] = xx[6] + xx[60];
  xx[5] = xx[13] + xx[61];
  xx[6] = xx[23] + xx[62];
  xx[29] = xx[11];
  xx[30] = xx[9];
  xx[31] = xx[14];
  xx[32] = xx[15];
  pm_math_Quaternion_compDeriv_ra(xx + 29, xx + 63, xx + 35);
  xx[13] = xx[11] + xx[35];
  xx[11] = xx[9] + xx[36];
  xx[9] = xx[14] + xx[37];
  xx[14] = xx[15] + xx[38];
  xx[15] = sqrt(xx[13] * xx[13] + xx[11] * xx[11] + xx[9] * xx[9] + xx[14] * xx
                [14]);
  if (xx[7] > xx[15])
    xx[15] = xx[7];
  xx[23] = xx[13] / xx[15];
  xx[13] = xx[11] / xx[15];
  xx[11] = xx[9] / xx[15];
  xx[9] = xx[14] / xx[15];
  xx[14] = xx[2] + xx[66];
  xx[2] = xx[10] + xx[67];
  xx[10] = xx[22] + xx[68];
  xx[29] = xx[43];
  xx[30] = xx[45];
  xx[31] = xx[46];
  xx[32] = xx[47];
  pm_math_Quaternion_compDeriv_ra(xx + 29, xx + 69, xx + 35);
  xx[15] = xx[43] + xx[35];
  xx[22] = xx[45] + xx[36];
  xx[24] = xx[46] + xx[37];
  xx[25] = xx[47] + xx[38];
  xx[27] = sqrt(xx[15] * xx[15] + xx[22] * xx[22] + xx[24] * xx[24] + xx[25] *
                xx[25]);
  if (xx[7] > xx[27])
    xx[27] = xx[7];
  xx[7] = xx[15] / xx[27];
  xx[15] = xx[22] / xx[27];
  xx[22] = xx[24] / xx[27];
  xx[24] = xx[25] / xx[27];
  xx[60] = xx[3];
  xx[61] = xx[5];
  xx[62] = xx[6];
  xx[63] = xx[23];
  xx[64] = xx[13];
  xx[65] = xx[11];
  xx[66] = xx[9];
  xx[67] = state[7];
  xx[68] = state[8];
  xx[69] = state[9];
  xx[70] = state[10];
  xx[71] = state[11];
  xx[72] = state[12];
  xx[73] = state[13];
  xx[74] = state[14];
  xx[75] = state[15];
  xx[76] = state[16];
  xx[77] = xx[14];
  xx[78] = xx[2];
  xx[79] = xx[10];
  xx[80] = xx[7];
  xx[81] = xx[15];
  xx[82] = xx[22];
  xx[83] = xx[24];
  xx[84] = state[24];
  xx[85] = state[25];
  xx[86] = state[26];
  xx[87] = state[27];
  xx[88] = state[28];
  xx[89] = state[29];
  xx[25] = xx[7] * xx[7];
  xx[27] = xx[15] * xx[22];
  xx[29] = xx[7] * xx[24];
  xx[30] = xx[15] * xx[24];
  xx[31] = xx[7] * xx[22];
  xx[35] = (xx[25] + xx[15] * xx[15]) * xx[28] - xx[16];
  xx[36] = (xx[27] + xx[29]) * xx[28];
  xx[37] = xx[28] * (xx[30] - xx[31]);
  xx[38] = - xx[23];
  xx[39] = - xx[13];
  xx[40] = - xx[11];
  xx[41] = - xx[9];
  pm_math_Quaternion_compose_ra(xx + 38, xx + 18, xx + 42);
  xx[18] = xx[43] * xx[44];
  xx[19] = xx[42] * xx[45];
  xx[20] = xx[42] * xx[42];
  xx[21] = xx[44] * xx[45];
  xx[32] = xx[42] * xx[43];
  xx[49] = xx[28] * (xx[18] - xx[19]);
  xx[50] = (xx[20] + xx[44] * xx[44]) * xx[28] - xx[16];
  xx[51] = (xx[21] + xx[32]) * xx[28];
  xx[33] = xx[22] * xx[24];
  xx[46] = xx[7] * xx[15];
  xx[52] = xx[28] * (xx[27] - xx[29]);
  xx[53] = (xx[25] + xx[22] * xx[22]) * xx[28] - xx[16];
  xx[54] = (xx[33] + xx[46]) * xx[28];
  xx[27] = xx[43] * xx[45];
  xx[29] = xx[42] * xx[44];
  xx[90] = (xx[27] + xx[29]) * xx[28];
  xx[91] = xx[28] * (xx[21] - xx[32]);
  xx[92] = (xx[20] + xx[45] * xx[45]) * xx[28] - xx[16];
  xx[93] = (xx[30] + xx[31]) * xx[28];
  xx[94] = xx[28] * (xx[33] - xx[46]);
  xx[95] = (xx[25] + xx[24] * xx[24]) * xx[28] - xx[16];
  xx[30] = (xx[20] + xx[43] * xx[43]) * xx[28] - xx[16];
  xx[31] = (xx[18] + xx[19]) * xx[28];
  xx[32] = xx[28] * (xx[27] - xx[29]);
  xx[18] = xx[57] * xx[22];
  xx[19] = xx[57] * xx[15];
  xx[20] = 1.045;
  xx[96] = xx[1];
  xx[97] = xx[56];
  xx[98] = xx[59] - xx[20];
  pm_math_Quaternion_xform_ra(xx + 38, xx + 96, xx + 99);
  xx[1] = xx[34] * xx[44];
  xx[21] = xx[34] * xx[43];
  xx[102] = pm_math_Vector3_dot_ra(xx + 35, xx + 49);
  xx[103] = pm_math_Vector3_dot_ra(xx + 52, xx + 90);
  xx[104] = pm_math_Vector3_dot_ra(xx + 93, xx + 30);
  xx[105] = (xx[7] * xx[18] + xx[24] * xx[19]) * xx[28] + xx[14] - (xx[99] + xx
    [3] + (xx[23] * xx[11] + xx[13] * xx[9]) * xx[28] - (xx[42] * xx[1] + xx[45]
    * xx[21]) * xx[28]);
  xx[106] = xx[28] * (xx[24] * xx[18] - xx[7] * xx[19]) + xx[2] - (xx[28] * (xx
    [42] * xx[21] - xx[45] * xx[1]) + xx[100] + xx[5] - xx[28] * (xx[23] * xx[13]
    - xx[11] * xx[9]));
  xx[107] = xx[10] - (xx[15] * xx[19] + xx[22] * xx[18]) * xx[28] - (xx[101] +
    xx[6] - (xx[13] * xx[13] + xx[11] * xx[11]) * xx[28] + (xx[43] * xx[21] +
    xx[44] * xx[1]) * xx[28]) - xx[8];
  zeroMajor(1, 6, ii + 0, xx + 102);
  xx[5] = fabs(xx[102]);
  xx[6] = fabs(xx[103]);
  xx[7] = fabs(xx[104]);
  xx[8] = fabs(xx[105]);
  xx[9] = fabs(xx[106]);
  xx[10] = fabs(xx[107]);
  ii[1] = 5;

  {
    int ll;
    for (ll = 6; ll < 11; ++ll)
      if (xx[ll] > xx[ii[1]])
        ii[1] = ll;
  }

  ii[1] -= 5;
  xx[1] = xx[5 + (ii[1])];
  xx[2] = 1.0e-9;
  if (xx[1] > xx[2]) {
    switch (ii[1])
    {
     case 0:
     case 1:
     case 2:
     case 3:
     case 4:
     case 5:
      {
        return sm_ssci_recordRunTimeError(
          "physmod:sm:core:compiler:mechanism:mechanism:constraintViolation",
          "'simulation/Plant/ TVC Physics/Staging Joint' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem.",
          neDiagMgr);
      }
    }
  }

  xx[1] = - xx[64];
  xx[3] = - xx[65];
  xx[5] = - xx[66];
  xx[6] = - xx[63];
  xx[7] = xx[1];
  xx[8] = xx[3];
  xx[9] = xx[5];
  xx[10] = xx[12] * xx[73];
  xx[11] = cos(xx[10]);
  xx[13] = xx[12] * xx[74];
  xx[12] = cos(xx[13]);
  xx[14] = xx[11] * xx[12];
  xx[15] = sin(xx[10]);
  xx[10] = xx[12] * xx[15];
  xx[12] = - xx[10];
  xx[18] = sin(xx[13]);
  xx[13] = xx[11] * xx[18];
  xx[11] = - xx[13];
  xx[19] = xx[15] * xx[18];
  xx[15] = - xx[19];
  xx[21] = - xx[14];
  xx[22] = xx[12];
  xx[23] = xx[11];
  xx[24] = xx[15];
  pm_math_Quaternion_compose_ra(xx + 6, xx + 21, xx + 29);
  xx[18] = xx[13] * xx[13];
  xx[25] = xx[19] * xx[19];
  xx[27] = xx[16] - (xx[18] + xx[25]) * xx[28];
  xx[33] = xx[31] * xx[27];
  xx[35] = xx[14] * xx[13];
  xx[36] = xx[10] * xx[19];
  xx[37] = xx[35] + xx[36];
  xx[38] = xx[37] * xx[28];
  xx[39] = xx[32] * xx[38];
  xx[40] = xx[30] * xx[27];
  xx[41] = xx[39] + xx[40];
  xx[42] = xx[31] * xx[38];
  xx[43] = xx[33];
  xx[44] = - xx[41];
  xx[45] = xx[42];
  pm_math_Vector3_cross_ra(xx + 30, xx + 43, xx + 49);
  xx[43] = (xx[29] * xx[33] + xx[49]) * xx[28] - xx[38];
  xx[44] = xx[28] * (xx[50] - xx[41] * xx[29]);
  xx[45] = xx[27] + (xx[29] * xx[42] + xx[51]) * xx[28];
  xx[33] = xx[80] * xx[80];
  xx[41] = xx[81] * xx[81];
  xx[42] = xx[81] * xx[82];
  xx[46] = xx[80] * xx[83];
  xx[47] = xx[81] * xx[83];
  xx[49] = xx[80] * xx[82];
  xx[50] = xx[28] * (xx[47] - xx[49]);
  xx[51] = (xx[33] + xx[41]) * xx[28] - xx[16];
  xx[52] = (xx[42] + xx[46]) * xx[28];
  xx[53] = xx[50];
  xx[54] = xx[14] * xx[19];
  xx[55] = xx[13] * xx[10];
  xx[56] = (xx[54] + xx[55]) * xx[28];
  xx[58] = xx[31] * xx[56];
  xx[59] = xx[13] * xx[19];
  xx[90] = xx[14] * xx[10];
  xx[91] = xx[28] * (xx[59] - xx[90]);
  xx[92] = xx[32] * xx[91];
  xx[93] = xx[30] * xx[56];
  xx[94] = xx[92] + xx[93];
  xx[95] = xx[31] * xx[91];
  xx[96] = xx[58];
  xx[97] = - xx[94];
  xx[98] = xx[95];
  pm_math_Vector3_cross_ra(xx + 30, xx + 96, xx + 99);
  xx[96] = (xx[29] * xx[58] + xx[99]) * xx[28] - xx[91];
  xx[97] = xx[28] * (xx[100] - xx[94] * xx[29]);
  xx[98] = xx[56] + (xx[29] * xx[95] + xx[101]) * xx[28];
  xx[58] = xx[28] * (xx[36] - xx[35]);
  xx[35] = xx[31] * xx[58];
  xx[36] = xx[10] * xx[10];
  xx[94] = xx[16] - (xx[36] + xx[18]) * xx[28];
  xx[18] = xx[32] * xx[94];
  xx[95] = xx[30] * xx[58];
  xx[99] = xx[18] + xx[95];
  xx[100] = xx[31] * xx[94];
  xx[101] = xx[35];
  xx[102] = - xx[99];
  xx[103] = xx[100];
  pm_math_Vector3_cross_ra(xx + 30, xx + 101, xx + 104);
  xx[101] = (xx[29] * xx[35] + xx[104]) * xx[28] - xx[94];
  xx[102] = xx[28] * (xx[105] - xx[99] * xx[29]);
  xx[103] = xx[58] + (xx[29] * xx[100] + xx[106]) * xx[28];
  xx[35] = (xx[49] + xx[47]) * xx[28];
  xx[99] = xx[80] * xx[81];
  xx[100] = xx[82] * xx[83];
  xx[104] = xx[82] * xx[82];
  xx[105] = (xx[41] + xx[104]) * xx[28];
  xx[106] = - xx[35];
  xx[107] = xx[28] * (xx[99] - xx[100]);
  xx[108] = xx[105] - xx[16];
  xx[109] = xx[30] * xx[31];
  xx[110] = xx[29] * xx[32];
  xx[111] = xx[29] * xx[29];
  xx[112] = xx[31] * xx[32];
  xx[113] = xx[29] * xx[30];
  xx[114] = xx[28] * (xx[109] - xx[110]);
  xx[115] = (xx[111] + xx[31] * xx[31]) * xx[28] - xx[16];
  xx[116] = (xx[112] + xx[113]) * xx[28];
  xx[117] = xx[28] * (xx[42] - xx[46]);
  xx[118] = xx[83] * xx[83];
  xx[119] = (xx[118] + xx[41]) * xx[28];
  xx[41] = (xx[99] + xx[100]) * xx[28];
  xx[120] = xx[117];
  xx[121] = xx[16] - xx[119];
  xx[122] = xx[41];
  xx[123] = xx[55] - xx[54];
  xx[54] = xx[28] * xx[123];
  xx[55] = xx[32] * xx[27];
  xx[124] = xx[32] * xx[54];
  xx[125] = xx[31] * xx[54];
  xx[126] = xx[40] + xx[125];
  xx[127] = xx[55];
  xx[128] = xx[124];
  xx[129] = - xx[126];
  pm_math_Vector3_cross_ra(xx + 30, xx + 127, xx + 130);
  xx[127] = xx[54] + (xx[29] * xx[55] + xx[130]) * xx[28];
  xx[128] = (xx[29] * xx[124] + xx[131]) * xx[28] - xx[27];
  xx[129] = xx[28] * (xx[132] - xx[126] * xx[29]);
  xx[130] = xx[117];
  xx[131] = (xx[33] + xx[104]) * xx[28] - xx[16];
  xx[132] = (xx[100] + xx[99]) * xx[28];
  xx[40] = xx[16] - (xx[25] + xx[36]) * xx[28];
  xx[25] = xx[32] * xx[56];
  xx[36] = xx[32] * xx[40];
  xx[55] = xx[31] * xx[40];
  xx[117] = xx[93] + xx[55];
  xx[133] = xx[25];
  xx[134] = xx[36];
  xx[135] = - xx[117];
  pm_math_Vector3_cross_ra(xx + 30, xx + 133, xx + 136);
  xx[133] = xx[40] + (xx[29] * xx[25] + xx[136]) * xx[28];
  xx[134] = (xx[29] * xx[36] + xx[137]) * xx[28] - xx[56];
  xx[135] = xx[28] * (xx[138] - xx[117] * xx[29]);
  xx[25] = (xx[90] + xx[59]) * xx[28];
  xx[36] = xx[32] * xx[58];
  xx[59] = xx[32] * xx[25];
  xx[90] = xx[31] * xx[25];
  xx[93] = xx[95] + xx[90];
  xx[136] = xx[36];
  xx[137] = xx[59];
  xx[138] = - xx[93];
  pm_math_Vector3_cross_ra(xx + 30, xx + 136, xx + 139);
  xx[136] = xx[25] + (xx[29] * xx[36] + xx[139]) * xx[28];
  xx[137] = (xx[29] * xx[59] + xx[140]) * xx[28] - xx[58];
  xx[138] = xx[28] * (xx[141] - xx[93] * xx[29]);
  xx[36] = xx[28] * (xx[100] - xx[99]);
  xx[139] = xx[35];
  xx[140] = xx[36];
  xx[141] = xx[16] - xx[105];
  xx[35] = xx[30] * xx[32];
  xx[59] = xx[29] * xx[31];
  xx[142] = (xx[35] + xx[59]) * xx[28];
  xx[143] = xx[28] * (xx[112] - xx[113]);
  xx[144] = (xx[111] + xx[32] * xx[32]) * xx[28] - xx[16];
  xx[93] = (xx[104] + xx[118]) * xx[28];
  xx[95] = (xx[46] + xx[42]) * xx[28];
  xx[145] = xx[93] - xx[16];
  xx[146] = - xx[95];
  xx[147] = xx[28] * (xx[49] - xx[47]);
  xx[99] = xx[125] + xx[39];
  xx[39] = xx[30] * xx[54];
  xx[100] = xx[30] * xx[38];
  xx[124] = - xx[99];
  xx[125] = xx[39];
  xx[126] = xx[100];
  pm_math_Vector3_cross_ra(xx + 30, xx + 124, xx + 148);
  xx[124] = xx[28] * (xx[148] - xx[99] * xx[29]);
  xx[125] = xx[38] + (xx[29] * xx[39] + xx[149]) * xx[28];
  xx[126] = (xx[29] * xx[100] + xx[150]) * xx[28] - xx[54];
  xx[148] = (xx[47] + xx[49]) * xx[28];
  xx[149] = xx[36];
  xx[150] = (xx[33] + xx[118]) * xx[28] - xx[16];
  xx[33] = xx[55] + xx[92];
  xx[36] = xx[30] * xx[40];
  xx[38] = xx[30] * xx[91];
  xx[151] = - xx[33];
  xx[152] = xx[36];
  xx[153] = xx[38];
  pm_math_Vector3_cross_ra(xx + 30, xx + 151, xx + 154);
  xx[151] = xx[28] * (xx[154] - xx[33] * xx[29]);
  xx[152] = xx[91] + (xx[29] * xx[36] + xx[155]) * xx[28];
  xx[153] = (xx[29] * xx[38] + xx[156]) * xx[28] - xx[40];
  xx[33] = xx[90] + xx[18];
  xx[18] = xx[30] * xx[25];
  xx[36] = xx[30] * xx[94];
  xx[90] = - xx[33];
  xx[91] = xx[18];
  xx[92] = xx[36];
  pm_math_Vector3_cross_ra(xx + 30, xx + 90, xx + 154);
  xx[90] = xx[28] * (xx[154] - xx[33] * xx[29]);
  xx[91] = xx[94] + (xx[29] * xx[18] + xx[155]) * xx[28];
  xx[92] = (xx[29] * xx[36] + xx[156]) * xx[28] - xx[25];
  xx[154] = xx[28] * (xx[46] - xx[42]);
  xx[155] = xx[119] - xx[16];
  xx[156] = - xx[41];
  xx[117] = (xx[111] + xx[30] * xx[30]) * xx[28] - xx[16];
  xx[118] = (xx[109] + xx[110]) * xx[28];
  xx[119] = xx[28] * (xx[35] - xx[59]);
  xx[109] = xx[16] - xx[93];
  xx[110] = xx[95];
  xx[111] = xx[50];
  xx[18] = xx[34] * xx[27];
  xx[33] = xx[32] * xx[18];
  xx[35] = xx[34] * xx[54];
  xx[36] = xx[32] * xx[35];
  xx[38] = xx[30] * xx[18] + xx[31] * xx[35];
  xx[93] = - xx[33];
  xx[94] = - xx[36];
  xx[95] = xx[38];
  pm_math_Vector3_cross_ra(xx + 30, xx + 93, xx + 157);
  xx[39] = xx[48] * xx[10];
  xx[41] = xx[48] * xx[13];
  xx[42] = xx[28] * (xx[39] * xx[14] - xx[41] * xx[19]);
  xx[46] = xx[65] * xx[42];
  xx[47] = (xx[39] * xx[10] + xx[41] * xx[13]) * xx[28];
  xx[49] = xx[47] - xx[48] - xx[16];
  xx[50] = xx[49] * xx[66];
  xx[54] = xx[46] + xx[50];
  xx[93] = xx[1];
  xx[94] = xx[3];
  xx[95] = xx[5];
  xx[1] = xx[64] * xx[42];
  xx[3] = xx[49] * xx[64];
  xx[160] = - xx[54];
  xx[161] = xx[1];
  xx[162] = xx[3];
  pm_math_Vector3_cross_ra(xx + 93, xx + 160, xx + 163);
  xx[5] = xx[63] * xx[66];
  xx[55] = xx[64] * xx[65];
  xx[59] = xx[34] * xx[56];
  xx[56] = xx[32] * xx[59];
  xx[99] = xx[34] * xx[40];
  xx[40] = xx[32] * xx[99];
  xx[100] = xx[30] * xx[59] + xx[31] * xx[99];
  xx[160] = - xx[56];
  xx[161] = - xx[40];
  xx[162] = xx[100];
  pm_math_Vector3_cross_ra(xx + 30, xx + 160, xx + 166);
  xx[104] = (xx[41] * xx[14] + xx[39] * xx[19]) * xx[28];
  xx[39] = xx[65] * xx[104];
  xx[41] = xx[64] * xx[104];
  xx[105] = xx[50] - xx[41];
  xx[50] = xx[49] * xx[65];
  xx[160] = - xx[39];
  xx[161] = - xx[105];
  xx[162] = xx[50];
  pm_math_Vector3_cross_ra(xx + 93, xx + 160, xx + 169);
  xx[112] = xx[66] * xx[66];
  xx[113] = xx[34] * xx[58];
  xx[58] = xx[32] * xx[113];
  xx[160] = xx[34] * xx[25];
  xx[25] = xx[32] * xx[160];
  xx[161] = xx[30] * xx[113] + xx[31] * xx[160];
  xx[172] = - xx[58];
  xx[173] = - xx[25];
  xx[174] = xx[161];
  pm_math_Vector3_cross_ra(xx + 30, xx + 172, xx + 175);
  xx[162] = xx[66] * xx[104];
  xx[172] = xx[66] * xx[42];
  xx[173] = xx[46] - xx[41];
  xx[178] = - xx[162];
  xx[179] = xx[172];
  xx[180] = - xx[173];
  pm_math_Vector3_cross_ra(xx + 93, xx + 178, xx + 181);
  xx[41] = xx[57] * xx[83];
  xx[46] = xx[80] * xx[41];
  xx[174] = xx[57] * xx[81];
  xx[178] = xx[57] * xx[82];
  xx[179] = xx[83] * xx[41];
  xx[184] = xx[0];
  xx[185] = xx[0];
  xx[186] = xx[0];
  xx[187] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 43, xx + 51);
  xx[188] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 96, xx + 51);
  xx[189] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 101, xx + 51);
  xx[190] = xx[0];
  xx[191] = xx[0];
  xx[192] = xx[0];
  xx[193] = xx[0];
  xx[194] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 106, xx + 114);
  xx[195] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 120, xx + 114);
  xx[196] = xx[0];
  xx[197] = xx[0];
  xx[198] = xx[0];
  xx[199] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 127, xx + 130);
  xx[200] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 133, xx + 130);
  xx[201] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 136, xx + 130);
  xx[202] = xx[0];
  xx[203] = xx[0];
  xx[204] = xx[0];
  xx[205] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 139, xx + 142);
  xx[206] = xx[0];
  xx[207] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 145, xx + 142);
  xx[208] = xx[0];
  xx[209] = xx[0];
  xx[210] = xx[0];
  xx[211] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 124, xx + 148);
  xx[212] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 151, xx + 148);
  xx[213] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 90, xx + 148);
  xx[214] = xx[0];
  xx[215] = xx[0];
  xx[216] = xx[0];
  xx[217] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 154, xx + 117);
  xx[218] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 109, xx + 117);
  xx[219] = xx[0];
  xx[220] = xx[26];
  xx[221] = xx[0];
  xx[222] = xx[0];
  xx[223] = bb[0] ? xx[0] : - ((xx[157] - xx[29] * xx[33]) * xx[28] - xx[35] +
    (xx[54] * xx[63] + xx[163]) * xx[28] + xx[28] * (xx[5] - xx[55]));
  xx[224] = bb[0] ? xx[0] : - ((xx[166] - xx[29] * xx[56]) * xx[28] - xx[99] +
    xx[49] + xx[28] * (xx[169] + xx[63] * xx[39]) - (xx[65] * xx[65] + xx[112]) *
    xx[28] + xx[16]);
  xx[225] = bb[0] ? xx[0] : - ((xx[175] - xx[29] * xx[58]) * xx[28] - xx[160] +
    xx[28] * (xx[181] + xx[63] * xx[162]) - xx[42]);
  xx[226] = xx[17];
  xx[227] = xx[0];
  xx[228] = xx[0];
  xx[229] = bb[0] ? xx[0] : xx[28] * (xx[46] - xx[82] * xx[174]);
  xx[230] = bb[0] ? xx[0] : xx[57] - (xx[82] * xx[178] + xx[179]) * xx[28];
  xx[231] = xx[0];
  xx[232] = xx[0];
  xx[233] = xx[26];
  xx[234] = xx[0];
  xx[235] = bb[0] ? xx[0] : - ((xx[158] - xx[29] * xx[36]) * xx[28] + xx[28] *
    (xx[164] - xx[63] * xx[1]) + (xx[112] + xx[64] * xx[64]) * xx[28] - xx[49] +
    xx[18] - xx[16]);
  xx[236] = bb[0] ? xx[0] : - ((xx[167] - xx[29] * xx[40]) * xx[28] + xx[59] +
    (xx[105] * xx[63] + xx[170]) * xx[28] + (xx[5] + xx[55]) * xx[28]);
  xx[237] = bb[0] ? xx[0] : - ((xx[176] - xx[29] * xx[25]) * xx[28] + xx[113] +
    xx[28] * (xx[182] - xx[63] * xx[172]) - xx[104]);
  xx[238] = xx[0];
  xx[239] = xx[17];
  xx[240] = xx[0];
  xx[241] = bb[0] ? xx[0] : (xx[179] + xx[81] * xx[174]) * xx[28] - xx[57];
  xx[242] = bb[0] ? xx[0] : (xx[46] + xx[81] * xx[178]) * xx[28];
  xx[243] = xx[0];
  xx[244] = xx[0];
  xx[245] = xx[0];
  xx[246] = xx[26];
  xx[247] = bb[0] ? xx[0] : - (xx[28] * (xx[159] + xx[38] * xx[29]) + xx[42] +
    xx[28] * (xx[165] - xx[63] * xx[3]) - (xx[63] * xx[64] + xx[65] * xx[66]) *
    xx[28]);
  xx[248] = bb[0] ? xx[0] : - (xx[28] * (xx[168] + xx[100] * xx[29]) + xx[28] *
    (xx[171] - xx[63] * xx[50]) + xx[104] + xx[28] * (xx[64] * xx[66] - xx[63] *
    xx[65]));
  xx[249] = bb[0] ? xx[0] : - (xx[28] * (xx[177] + xx[161] * xx[29]) + (xx[173] *
    xx[63] + xx[183]) * xx[28]);
  xx[250] = xx[0];
  xx[251] = xx[0];
  xx[252] = xx[17];
  xx[253] = bb[0] ? xx[0] : - ((xx[80] * xx[174] + xx[82] * xx[41]) * xx[28]);
  xx[254] = bb[0] ? xx[0] : xx[28] * (xx[81] * xx[41] - xx[80] * xx[178]);
  xx[255] = xx[0];
  xx[16] = - xx[81];
  xx[17] = - xx[82];
  xx[18] = - xx[83];
  xx[0] = xx[82] * xx[88];
  xx[1] = xx[83] * xx[89];
  xx[3] = xx[0] + xx[1];
  xx[5] = xx[81] * xx[88];
  xx[25] = xx[81] * xx[89];
  xx[38] = xx[3];
  xx[39] = - xx[5];
  xx[40] = - xx[25];
  pm_math_Vector3_cross_ra(xx + 16, xx + 38, xx + 43);
  xx[38] = xx[28] * (xx[43] - xx[3] * xx[80]);
  xx[39] = xx[89] + (xx[80] * xx[5] + xx[44]) * xx[28];
  xx[40] = (xx[80] * xx[25] + xx[45]) * xx[28] - xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 21, xx + 70, xx + 43);
  xx[3] = xx[75] * xx[27];
  xx[5] = xx[43] + xx[3];
  xx[25] = xx[5] * xx[31];
  xx[26] = xx[28] * xx[37] * xx[75];
  xx[27] = xx[45] + xx[26];
  xx[33] = xx[27] * xx[32];
  xx[35] = xx[5] * xx[30];
  xx[36] = xx[33] + xx[35];
  xx[37] = xx[27] * xx[31];
  xx[54] = xx[25];
  xx[55] = - xx[36];
  xx[56] = xx[37];
  pm_math_Vector3_cross_ra(xx + 30, xx + 54, xx + 90);
  xx[54] = (xx[29] * xx[25] + xx[90]) * xx[28] - xx[27];
  xx[55] = xx[28] * (xx[91] - xx[36] * xx[29]);
  xx[56] = xx[5] + (xx[29] * xx[37] + xx[92]) * xx[28];
  xx[25] = xx[82] * xx[87];
  xx[36] = xx[81] * xx[87];
  xx[37] = xx[1] + xx[36];
  xx[1] = xx[82] * xx[89];
  xx[90] = - xx[25];
  xx[91] = xx[37];
  xx[92] = - xx[1];
  pm_math_Vector3_cross_ra(xx + 16, xx + 90, xx + 96);
  xx[90] = (xx[80] * xx[25] + xx[96]) * xx[28] - xx[89];
  xx[91] = xx[28] * (xx[97] - xx[37] * xx[80]);
  xx[92] = xx[87] + (xx[80] * xx[1] + xx[98]) * xx[28];
  xx[1] = xx[28] * xx[75] * xx[123] + xx[76];
  xx[25] = xx[44] + xx[1];
  xx[37] = xx[5] * xx[32];
  xx[41] = xx[25] * xx[32];
  xx[43] = xx[25] * xx[31];
  xx[44] = xx[35] + xx[43];
  xx[96] = xx[37];
  xx[97] = xx[41];
  xx[98] = - xx[44];
  pm_math_Vector3_cross_ra(xx + 30, xx + 96, xx + 99);
  xx[96] = xx[25] + (xx[29] * xx[37] + xx[99]) * xx[28];
  xx[97] = (xx[29] * xx[41] + xx[100]) * xx[28] - xx[5];
  xx[98] = xx[28] * (xx[101] - xx[44] * xx[29]);
  xx[35] = xx[83] * xx[87];
  xx[37] = xx[83] * xx[88];
  xx[41] = xx[36] + xx[0];
  xx[44] = - xx[35];
  xx[45] = - xx[37];
  xx[46] = xx[41];
  pm_math_Vector3_cross_ra(xx + 16, xx + 44, xx + 99);
  xx[44] = xx[88] + (xx[80] * xx[35] + xx[99]) * xx[28];
  xx[45] = (xx[80] * xx[37] + xx[100]) * xx[28] - xx[87];
  xx[46] = xx[28] * (xx[101] - xx[41] * xx[80]);
  xx[0] = xx[43] + xx[33];
  xx[33] = xx[25] * xx[30];
  xx[35] = xx[27] * xx[30];
  xx[99] = - xx[0];
  xx[100] = xx[33];
  xx[101] = xx[35];
  pm_math_Vector3_cross_ra(xx + 30, xx + 99, xx + 105);
  xx[99] = xx[28] * (xx[105] - xx[0] * xx[29]);
  xx[100] = xx[27] + (xx[29] * xx[33] + xx[106]) * xx[28];
  xx[101] = (xx[29] * xx[35] + xx[107]) * xx[28] - xx[25];
  xx[0] = xx[57] * xx[88];
  xx[27] = xx[57] * xx[87];
  xx[33] = xx[83] * xx[27];
  xx[35] = xx[83] * xx[0];
  xx[36] = xx[81] * xx[27] + xx[82] * xx[0];
  xx[105] = - xx[33];
  xx[106] = - xx[35];
  xx[107] = xx[36];
  pm_math_Vector3_cross_ra(xx + 16, xx + 105, xx + 108);
  xx[37] = xx[5] * xx[34];
  xx[5] = xx[32] * xx[37];
  xx[41] = xx[25] * xx[34];
  xx[25] = xx[32] * xx[41];
  xx[43] = xx[30] * xx[37] + xx[31] * xx[41];
  xx[105] = - xx[5];
  xx[106] = - xx[25];
  xx[107] = xx[43];
  pm_math_Vector3_cross_ra(xx + 30, xx + 105, xx + 111);
  xx[50] = xx[66] * xx[70];
  xx[58] = xx[66] * xx[71];
  xx[59] = xx[64] * xx[70] + xx[65] * xx[71];
  xx[105] = - xx[50];
  xx[106] = - xx[58];
  xx[107] = xx[59];
  pm_math_Vector3_cross_ra(xx + 93, xx + 105, xx + 120);
  xx[105] = xx[12];
  xx[106] = xx[11];
  xx[107] = xx[15];
  xx[11] = xx[48] * xx[3];
  xx[12] = xx[11] * xx[19];
  xx[15] = xx[1] * xx[48];
  xx[48] = xx[15] * xx[19];
  xx[19] = xx[11] * xx[10] + xx[15] * xx[13];
  xx[123] = xx[12];
  xx[124] = xx[48];
  xx[125] = - xx[19];
  pm_math_Vector3_cross_ra(xx + 105, xx + 123, xx + 126);
  xx[10] = (xx[126] - xx[14] * xx[12]) * xx[28] - xx[15];
  xx[12] = - xx[104];
  xx[102] = xx[12];
  xx[103] = xx[42];
  xx[104] = xx[49];
  pm_math_Vector3_cross_ra(xx + 70, xx + 102, xx + 105);
  xx[13] = (xx[127] - xx[14] * xx[48]) * xx[28] + xx[11];
  xx[11] = xx[28] * (xx[128] + xx[19] * xx[14]);
  xx[102] = xx[10] + xx[105];
  xx[103] = xx[13] + xx[106];
  xx[104] = xx[11] + xx[107];
  pm_math_Quaternion_xform_ra(xx + 6, xx + 102, xx + 105);
  xx[123] = pm_math_Vector3_dot_ra(xx + 38, xx + 114) + pm_math_Vector3_dot_ra
    (xx + 54, xx + 51);
  xx[124] = pm_math_Vector3_dot_ra(xx + 90, xx + 142) + pm_math_Vector3_dot_ra
    (xx + 96, xx + 130);
  xx[125] = pm_math_Vector3_dot_ra(xx + 44, xx + 117) + pm_math_Vector3_dot_ra
    (xx + 99, xx + 148);
  xx[126] = xx[0] + (xx[80] * xx[33] + xx[108]) * xx[28] + xx[84] - ((xx[111] -
    xx[29] * xx[5]) * xx[28] - xx[41] + xx[71] + (xx[63] * xx[50] + xx[120]) *
    xx[28] + xx[67] + xx[105]);
  xx[127] = (xx[80] * xx[35] + xx[109]) * xx[28] - xx[27] + xx[85] - ((xx[112] -
    xx[29] * xx[25]) * xx[28] + xx[37] + (xx[63] * xx[58] + xx[121]) * xx[28] -
    xx[70] + xx[68] + xx[106]);
  xx[128] = xx[28] * (xx[110] - xx[36] * xx[80]) + xx[86] - (xx[28] * (xx[113] +
    xx[43] * xx[29]) + xx[28] * (xx[122] - xx[59] * xx[63]) + xx[69] + xx[107]);
  zeroMajor(1, 6, ii + 0, xx + 123);
  xx[35] = - xx[123];
  xx[36] = - xx[124];
  xx[37] = - xx[125];
  xx[38] = - xx[126];
  xx[39] = - xx[127];
  xx[40] = - xx[128];
  memcpy(xx + 256, xx + 184, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 256, xx + 108, xx + 120, ii + 1, xx + 35, xx[4],
                     xx + 96);
  xx[0] = xx[67] + xx[96];
  xx[4] = xx[68] + xx[97];
  xx[5] = xx[69] + xx[98];
  xx[14] = xx[70] + xx[99];
  xx[15] = xx[71] + xx[100];
  xx[19] = xx[72] + xx[101];
  xx[25] = xx[84] + xx[102];
  xx[27] = xx[85] + xx[103];
  xx[33] = xx[86] + xx[104];
  xx[35] = xx[87] + xx[105];
  xx[36] = xx[88] + xx[106];
  xx[37] = xx[89] + xx[107];
  xx[151] = xx[60];
  xx[152] = xx[61];
  xx[153] = xx[62];
  xx[154] = xx[63];
  xx[155] = xx[64];
  xx[156] = xx[65];
  xx[157] = xx[66];
  xx[158] = xx[0];
  xx[159] = xx[4];
  xx[160] = xx[5];
  xx[161] = xx[14];
  xx[162] = xx[15];
  xx[163] = xx[19];
  xx[164] = xx[73];
  xx[165] = xx[74];
  xx[166] = xx[75];
  xx[167] = xx[76];
  xx[168] = xx[77];
  xx[169] = xx[78];
  xx[170] = xx[79];
  xx[171] = xx[80];
  xx[172] = xx[81];
  xx[173] = xx[82];
  xx[174] = xx[83];
  xx[175] = xx[25];
  xx[176] = xx[27];
  xx[177] = xx[33];
  xx[178] = xx[35];
  xx[179] = xx[36];
  xx[180] = xx[37];
  xx[38] = xx[36] * xx[82];
  xx[39] = xx[37] * xx[83];
  xx[40] = xx[38] + xx[39];
  xx[41] = xx[36] * xx[81];
  xx[43] = xx[37] * xx[81];
  xx[44] = xx[40];
  xx[45] = - xx[41];
  xx[46] = - xx[43];
  pm_math_Vector3_cross_ra(xx + 16, xx + 44, xx + 48);
  xx[44] = xx[28] * (xx[48] - xx[40] * xx[80]);
  xx[45] = xx[37] + (xx[80] * xx[41] + xx[49]) * xx[28];
  xx[46] = (xx[80] * xx[43] + xx[50]) * xx[28] - xx[36];
  xx[48] = xx[14];
  xx[49] = xx[15];
  xx[50] = xx[19];
  pm_math_Quaternion_inverseXform_ra(xx + 21, xx + 48, xx + 54);
  xx[19] = xx[54] + xx[3];
  xx[3] = xx[19] * xx[31];
  xx[21] = xx[56] + xx[26];
  xx[22] = xx[21] * xx[32];
  xx[23] = xx[19] * xx[30];
  xx[24] = xx[22] + xx[23];
  xx[26] = xx[21] * xx[31];
  xx[58] = xx[3];
  xx[59] = - xx[24];
  xx[60] = xx[26];
  pm_math_Vector3_cross_ra(xx + 30, xx + 58, xx + 67);
  xx[58] = (xx[29] * xx[3] + xx[67]) * xx[28] - xx[21];
  xx[59] = xx[28] * (xx[68] - xx[24] * xx[29]);
  xx[60] = xx[19] + (xx[29] * xx[26] + xx[69]) * xx[28];
  xx[3] = xx[35] * xx[82];
  xx[24] = xx[35] * xx[81];
  xx[26] = xx[39] + xx[24];
  xx[39] = xx[37] * xx[82];
  xx[67] = - xx[3];
  xx[68] = xx[26];
  xx[69] = - xx[39];
  pm_math_Vector3_cross_ra(xx + 16, xx + 67, xx + 70);
  xx[67] = (xx[80] * xx[3] + xx[70]) * xx[28] - xx[37];
  xx[68] = xx[28] * (xx[71] - xx[26] * xx[80]);
  xx[69] = xx[35] + (xx[80] * xx[39] + xx[72]) * xx[28];
  xx[3] = xx[55] + xx[1];
  xx[1] = xx[19] * xx[32];
  xx[26] = xx[3] * xx[32];
  xx[37] = xx[3] * xx[31];
  xx[39] = xx[23] + xx[37];
  xx[54] = xx[1];
  xx[55] = xx[26];
  xx[56] = - xx[39];
  pm_math_Vector3_cross_ra(xx + 30, xx + 54, xx + 70);
  xx[54] = xx[3] + (xx[29] * xx[1] + xx[70]) * xx[28];
  xx[55] = (xx[29] * xx[26] + xx[71]) * xx[28] - xx[19];
  xx[56] = xx[28] * (xx[72] - xx[39] * xx[29]);
  xx[1] = xx[35] * xx[83];
  xx[23] = xx[36] * xx[83];
  xx[26] = xx[24] + xx[38];
  xx[38] = - xx[1];
  xx[39] = - xx[23];
  xx[40] = xx[26];
  pm_math_Vector3_cross_ra(xx + 16, xx + 38, xx + 70);
  xx[38] = xx[36] + (xx[80] * xx[1] + xx[70]) * xx[28];
  xx[39] = (xx[80] * xx[23] + xx[71]) * xx[28] - xx[35];
  xx[40] = xx[28] * (xx[72] - xx[26] * xx[80]);
  xx[1] = xx[37] + xx[22];
  xx[22] = xx[3] * xx[30];
  xx[23] = xx[21] * xx[30];
  xx[70] = - xx[1];
  xx[71] = xx[22];
  xx[72] = xx[23];
  pm_math_Vector3_cross_ra(xx + 30, xx + 70, xx + 73);
  xx[70] = xx[28] * (xx[73] - xx[1] * xx[29]);
  xx[71] = xx[21] + (xx[29] * xx[22] + xx[74]) * xx[28];
  xx[72] = (xx[29] * xx[23] + xx[75]) * xx[28] - xx[3];
  xx[1] = xx[36] * xx[57];
  xx[21] = xx[35] * xx[57];
  xx[22] = xx[83] * xx[21];
  xx[23] = xx[83] * xx[1];
  xx[24] = xx[81] * xx[21] + xx[82] * xx[1];
  xx[35] = - xx[22];
  xx[36] = - xx[23];
  xx[37] = xx[24];
  pm_math_Vector3_cross_ra(xx + 16, xx + 35, xx + 73);
  xx[16] = xx[19] * xx[34];
  xx[17] = xx[32] * xx[16];
  xx[18] = xx[3] * xx[34];
  xx[3] = xx[32] * xx[18];
  xx[19] = xx[30] * xx[16] + xx[31] * xx[18];
  xx[34] = - xx[17];
  xx[35] = - xx[3];
  xx[36] = xx[19];
  pm_math_Vector3_cross_ra(xx + 30, xx + 34, xx + 76);
  xx[26] = xx[14] * xx[66];
  xx[30] = xx[15] * xx[66];
  xx[31] = xx[14] * xx[64] + xx[15] * xx[65];
  xx[34] = - xx[26];
  xx[35] = - xx[30];
  xx[36] = xx[31];
  pm_math_Vector3_cross_ra(xx + 93, xx + 34, xx + 64);
  xx[34] = xx[12];
  xx[35] = xx[42];
  xx[36] = xx[47] - xx[20];
  pm_math_Vector3_cross_ra(xx + 48, xx + 34, xx + 41);
  xx[34] = xx[10] + xx[41];
  xx[35] = xx[13] + xx[42];
  xx[36] = xx[11] + xx[43];
  pm_math_Quaternion_xform_ra(xx + 6, xx + 34, xx + 10);
  xx[81] = pm_math_Vector3_dot_ra(xx + 44, xx + 114) + pm_math_Vector3_dot_ra(xx
    + 58, xx + 51);
  xx[82] = pm_math_Vector3_dot_ra(xx + 67, xx + 142) + pm_math_Vector3_dot_ra(xx
    + 54, xx + 130);
  xx[83] = pm_math_Vector3_dot_ra(xx + 38, xx + 117) + pm_math_Vector3_dot_ra(xx
    + 70, xx + 148);
  xx[84] = xx[1] + (xx[80] * xx[22] + xx[73]) * xx[28] + xx[25] - ((xx[76] - xx
    [29] * xx[17]) * xx[28] - xx[18] + xx[15] + (xx[63] * xx[26] + xx[64]) * xx
    [28] + xx[0] + xx[10]);
  xx[85] = (xx[80] * xx[23] + xx[74]) * xx[28] - xx[21] + xx[27] - ((xx[77] -
    xx[29] * xx[3]) * xx[28] + xx[16] + (xx[63] * xx[30] + xx[65]) * xx[28] -
    xx[14] + xx[4] + xx[11]);
  xx[86] = xx[28] * (xx[75] - xx[24] * xx[80]) + xx[33] - (xx[28] * (xx[78] +
    xx[19] * xx[29]) + xx[28] * (xx[66] - xx[31] * xx[63]) + xx[5] + xx[12]);
  zeroMajor(1, 6, ii + 0, xx + 81);
  xx[3] = fabs(xx[81]);
  xx[4] = fabs(xx[82]);
  xx[5] = fabs(xx[83]);
  xx[6] = fabs(xx[84]);
  xx[7] = fabs(xx[85]);
  xx[8] = fabs(xx[86]);
  ii[0] = 3;

  {
    int ll;
    for (ll = 4; ll < 9; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 3;
  xx[0] = xx[3 + (ii[0])];
  if (xx[0] > xx[2]) {
    switch (ii[0])
    {
     case 0:
     case 1:
     case 2:
     case 3:
     case 4:
     case 5:
      {
        return sm_ssci_recordRunTimeError(
          "physmod:sm:core:compiler:mechanism:mechanism:constraintViolation",
          "'simulation/Plant/ TVC Physics/Staging Joint' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem.",
          neDiagMgr);
      }
    }
  }

  state[0] = xx[151];
  state[1] = xx[152];
  state[2] = xx[153];
  state[3] = xx[154];
  state[4] = xx[155];
  state[5] = xx[156];
  state[6] = xx[157];
  state[7] = xx[158];
  state[8] = xx[159];
  state[9] = xx[160];
  state[10] = xx[161];
  state[11] = xx[162];
  state[12] = xx[163];
  state[13] = xx[164];
  state[14] = xx[165];
  state[15] = xx[166];
  state[16] = xx[167];
  state[17] = xx[168];
  state[18] = xx[169];
  state[19] = xx[170];
  state[20] = xx[171];
  state[21] = xx[172];
  state[22] = xx[173];
  state[23] = xx[174];
  state[24] = xx[175];
  state[25] = xx[176];
  state[26] = xx[177];
  state[27] = xx[178];
  state[28] = xx[179];
  state[29] = xx[180];
  return NULL;
}

void simulation_b048d748_1_computeConstraintError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[44];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  ii[0] = modeVector[0] == -1 ? 0 : 1;
  xx[0] = state[20] * state[20];
  xx[1] = 2.0;
  xx[2] = 1.0;
  xx[3] = state[21] * state[22];
  xx[4] = state[20] * state[23];
  xx[5] = state[21] * state[23];
  xx[6] = state[20] * state[22];
  xx[7] = (xx[0] + state[21] * state[21]) * xx[1] - xx[2];
  xx[8] = (xx[3] + xx[4]) * xx[1];
  xx[9] = xx[1] * (xx[5] - xx[6]);
  xx[10] = - state[3];
  xx[11] = - state[4];
  xx[12] = - state[5];
  xx[13] = - state[6];
  xx[14] = 0.5;
  xx[15] = xx[14] * state[13];
  xx[16] = cos(xx[15]);
  xx[17] = xx[14] * state[14];
  xx[14] = cos(xx[17]);
  xx[18] = xx[16] * xx[14];
  xx[19] = sin(xx[15]);
  xx[15] = xx[14] * xx[19];
  xx[14] = sin(xx[17]);
  xx[17] = xx[16] * xx[14];
  xx[16] = xx[19] * xx[14];
  xx[19] = - xx[18];
  xx[20] = - xx[15];
  xx[21] = - xx[17];
  xx[22] = - xx[16];
  pm_math_Quaternion_compose_ra(xx + 10, xx + 19, xx + 23);
  xx[14] = xx[24] * xx[25];
  xx[19] = xx[23] * xx[26];
  xx[20] = xx[23] * xx[23];
  xx[21] = xx[25] * xx[26];
  xx[22] = xx[23] * xx[24];
  xx[27] = xx[1] * (xx[14] - xx[19]);
  xx[28] = (xx[20] + xx[25] * xx[25]) * xx[1] - xx[2];
  xx[29] = (xx[21] + xx[22]) * xx[1];
  xx[30] = state[22] * state[23];
  xx[31] = state[20] * state[21];
  xx[32] = xx[1] * (xx[3] - xx[4]);
  xx[33] = (xx[0] + state[22] * state[22]) * xx[1] - xx[2];
  xx[34] = (xx[30] + xx[31]) * xx[1];
  xx[3] = xx[24] * xx[26];
  xx[4] = xx[23] * xx[25];
  xx[35] = (xx[3] + xx[4]) * xx[1];
  xx[36] = xx[1] * (xx[21] - xx[22]);
  xx[37] = (xx[20] + xx[26] * xx[26]) * xx[1] - xx[2];
  xx[38] = (xx[5] + xx[6]) * xx[1];
  xx[39] = xx[1] * (xx[30] - xx[31]);
  xx[40] = (xx[0] + state[23] * state[23]) * xx[1] - xx[2];
  xx[41] = (xx[20] + xx[24] * xx[24]) * xx[1] - xx[2];
  xx[42] = (xx[14] + xx[19]) * xx[1];
  xx[43] = xx[1] * (xx[3] - xx[4]);
  xx[0] = 0.0225;
  xx[2] = xx[0] * state[22];
  xx[3] = xx[0] * state[21];
  xx[0] = 0.04499999999999999;
  xx[4] = xx[0] * xx[17];
  xx[5] = xx[0] * xx[15];
  xx[19] = - ((xx[4] * xx[18] + xx[5] * xx[16]) * xx[1]);
  xx[20] = xx[1] * (xx[5] * xx[18] - xx[4] * xx[16]);
  xx[21] = (xx[5] * xx[15] + xx[4] * xx[17]) * xx[1] - 1.045;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 19, xx + 4);
  xx[0] = 0.04500000000000001;
  xx[10] = xx[0] * xx[25];
  xx[11] = xx[0] * xx[24];
  xx[12] = pm_math_Vector3_dot_ra(xx + 7, xx + 27);
  xx[13] = pm_math_Vector3_dot_ra(xx + 32, xx + 35);
  xx[14] = pm_math_Vector3_dot_ra(xx + 38, xx + 41);
  xx[15] = (xx[2] * state[20] + xx[3] * state[23]) * xx[1] + state[17] - (xx[4]
    + state[0] + (state[3] * state[5] + state[4] * state[6]) * xx[1] - (xx[23] *
    xx[10] + xx[26] * xx[11]) * xx[1]);
  xx[16] = xx[1] * (xx[2] * state[23] - xx[3] * state[20]) + state[18] - (xx[1] *
    (xx[23] * xx[11] - xx[26] * xx[10]) + xx[5] + state[1] - xx[1] * (state[3] *
    state[4] - state[5] * state[6]));
  xx[17] = state[19] - (xx[3] * state[21] + xx[2] * state[22]) * xx[1] - (xx[6]
    + state[2] - (state[4] * state[4] + state[5] * state[5]) * xx[1] + (xx[24] *
    xx[11] + xx[25] * xx[10]) * xx[1]) - 0.9325;
  zeroMajor(1, 6, ii + 0, xx + 12);
  error[0] = xx[12];
  error[1] = xx[13];
  error[2] = xx[14];
  error[3] = xx[15];
  error[4] = xx[16];
  error[5] = xx[17];
}

void simulation_b048d748_1_resetModeVector(const void *mech, int *modeVector)
{
  (void) mech;
  modeVector[0] = 0;
}

boolean_T simulation_b048d748_1_hasJointDisToNormModeChange(const void *mech,
  const int *prevModeVector, const int *modeVector)
{
  (void) mech;
  return prevModeVector[0] == -1 && modeVector[0] == 0;
}

PmfMessageId simulation_b048d748_1_performJointDisToNormModeChange(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags,
  const int *prevModeVector, const int *modeVector, const double *input, double *
  state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  boolean_T bb[2];
  int ii[15];
  double xx[401];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) input;
  (void) neDiagMgr;
  xx[0] = 0.0;
  bb[0] = modeVector[0] == -1;
  xx[1] = sqrt(state[3] * state[3] + state[4] * state[4] + state[5] * state[5] +
               state[6] * state[6]);
  xx[2] = state[3] / xx[1];
  xx[3] = state[4] / xx[1];
  xx[4] = - xx[3];
  xx[5] = state[5] / xx[1];
  xx[6] = - xx[5];
  xx[7] = state[6] / xx[1];
  xx[1] = - xx[7];
  xx[8] = - xx[2];
  xx[9] = xx[4];
  xx[10] = xx[6];
  xx[11] = xx[1];
  xx[12] = 0.5;
  xx[13] = xx[12] * state[13];
  xx[14] = cos(xx[13]);
  xx[15] = xx[12] * state[14];
  xx[16] = cos(xx[15]);
  xx[17] = xx[14] * xx[16];
  xx[18] = sin(xx[13]);
  xx[13] = xx[16] * xx[18];
  xx[19] = sin(xx[15]);
  xx[15] = xx[14] * xx[19];
  xx[14] = xx[18] * xx[19];
  xx[20] = - xx[17];
  xx[21] = - xx[13];
  xx[22] = - xx[15];
  xx[23] = - xx[14];
  pm_math_Quaternion_compose_ra(xx + 8, xx + 20, xx + 24);
  xx[18] = 1.0;
  xx[28] = xx[15] * xx[15];
  xx[29] = xx[14] * xx[14];
  xx[30] = 2.0;
  xx[31] = xx[18] - (xx[28] + xx[29]) * xx[30];
  xx[32] = xx[26] * xx[31];
  xx[33] = xx[17] * xx[15];
  xx[34] = xx[13] * xx[14];
  xx[35] = (xx[33] + xx[34]) * xx[30];
  xx[36] = xx[27] * xx[35];
  xx[37] = xx[25] * xx[31];
  xx[38] = xx[36] + xx[37];
  xx[39] = xx[26] * xx[35];
  xx[40] = xx[32];
  xx[41] = - xx[38];
  xx[42] = xx[39];
  pm_math_Vector3_cross_ra(xx + 25, xx + 40, xx + 43);
  xx[40] = (xx[24] * xx[32] + xx[43]) * xx[30] - xx[35];
  xx[41] = xx[30] * (xx[44] - xx[38] * xx[24]);
  xx[42] = xx[31] + (xx[24] * xx[39] + xx[45]) * xx[30];
  xx[32] = sqrt(state[20] * state[20] + state[21] * state[21] + state[22] *
                state[22] + state[23] * state[23]);
  xx[38] = state[20] / xx[32];
  xx[39] = xx[38] * xx[38];
  xx[43] = state[21] / xx[32];
  xx[44] = xx[43] * xx[43];
  xx[45] = state[22] / xx[32];
  xx[46] = xx[43] * xx[45];
  xx[47] = state[23] / xx[32];
  xx[32] = xx[38] * xx[47];
  xx[48] = xx[43] * xx[47];
  xx[49] = xx[38] * xx[45];
  xx[50] = xx[30] * (xx[48] - xx[49]);
  xx[51] = (xx[39] + xx[44]) * xx[30] - xx[18];
  xx[52] = (xx[46] + xx[32]) * xx[30];
  xx[53] = xx[50];
  xx[54] = xx[17] * xx[14];
  xx[55] = xx[15] * xx[13];
  xx[56] = (xx[54] + xx[55]) * xx[30];
  xx[57] = xx[26] * xx[56];
  xx[58] = xx[15] * xx[14];
  xx[59] = xx[17] * xx[13];
  xx[60] = xx[30] * (xx[58] - xx[59]);
  xx[61] = xx[27] * xx[60];
  xx[62] = xx[25] * xx[56];
  xx[63] = xx[61] + xx[62];
  xx[64] = xx[26] * xx[60];
  xx[65] = xx[57];
  xx[66] = - xx[63];
  xx[67] = xx[64];
  pm_math_Vector3_cross_ra(xx + 25, xx + 65, xx + 68);
  xx[65] = (xx[24] * xx[57] + xx[68]) * xx[30] - xx[60];
  xx[66] = xx[30] * (xx[69] - xx[63] * xx[24]);
  xx[67] = xx[56] + (xx[24] * xx[64] + xx[70]) * xx[30];
  xx[57] = xx[30] * (xx[34] - xx[33]);
  xx[63] = xx[26] * xx[57];
  xx[64] = xx[13] * xx[13];
  xx[68] = xx[18] - (xx[64] + xx[28]) * xx[30];
  xx[69] = xx[27] * xx[68];
  xx[70] = xx[25] * xx[57];
  xx[71] = xx[69] + xx[70];
  xx[72] = xx[26] * xx[68];
  xx[73] = xx[63];
  xx[74] = - xx[71];
  xx[75] = xx[72];
  pm_math_Vector3_cross_ra(xx + 25, xx + 73, xx + 76);
  xx[73] = (xx[24] * xx[63] + xx[76]) * xx[30] - xx[68];
  xx[74] = xx[30] * (xx[77] - xx[71] * xx[24]);
  xx[75] = xx[57] + (xx[24] * xx[72] + xx[78]) * xx[30];
  xx[63] = (xx[49] + xx[48]) * xx[30];
  xx[71] = xx[38] * xx[43];
  xx[72] = xx[45] * xx[47];
  xx[76] = xx[45] * xx[45];
  xx[77] = (xx[44] + xx[76]) * xx[30];
  xx[78] = - xx[63];
  xx[79] = xx[30] * (xx[71] - xx[72]);
  xx[80] = xx[77] - xx[18];
  xx[81] = xx[25] * xx[26];
  xx[82] = xx[24] * xx[27];
  xx[83] = xx[24] * xx[24];
  xx[84] = xx[26] * xx[27];
  xx[85] = xx[24] * xx[25];
  xx[86] = xx[30] * (xx[81] - xx[82]);
  xx[87] = (xx[83] + xx[26] * xx[26]) * xx[30] - xx[18];
  xx[88] = (xx[84] + xx[85]) * xx[30];
  xx[89] = xx[30] * (xx[46] - xx[32]);
  xx[90] = xx[47] * xx[47];
  xx[91] = (xx[90] + xx[44]) * xx[30];
  xx[44] = (xx[71] + xx[72]) * xx[30];
  xx[92] = xx[89];
  xx[93] = xx[18] - xx[91];
  xx[94] = xx[44];
  xx[95] = xx[30] * (xx[55] - xx[54]);
  xx[96] = xx[27] * xx[31];
  xx[97] = xx[27] * xx[95];
  xx[98] = xx[26] * xx[95];
  xx[99] = xx[37] + xx[98];
  xx[100] = xx[96];
  xx[101] = xx[97];
  xx[102] = - xx[99];
  pm_math_Vector3_cross_ra(xx + 25, xx + 100, xx + 103);
  xx[100] = xx[95] + (xx[24] * xx[96] + xx[103]) * xx[30];
  xx[101] = (xx[24] * xx[97] + xx[104]) * xx[30] - xx[31];
  xx[102] = xx[30] * (xx[105] - xx[99] * xx[24]);
  xx[103] = xx[89];
  xx[104] = (xx[39] + xx[76]) * xx[30] - xx[18];
  xx[105] = (xx[72] + xx[71]) * xx[30];
  xx[37] = xx[18] - (xx[29] + xx[64]) * xx[30];
  xx[89] = xx[27] * xx[56];
  xx[96] = xx[27] * xx[37];
  xx[97] = xx[26] * xx[37];
  xx[99] = xx[62] + xx[97];
  xx[106] = xx[89];
  xx[107] = xx[96];
  xx[108] = - xx[99];
  pm_math_Vector3_cross_ra(xx + 25, xx + 106, xx + 109);
  xx[106] = xx[37] + (xx[24] * xx[89] + xx[109]) * xx[30];
  xx[107] = (xx[24] * xx[96] + xx[110]) * xx[30] - xx[56];
  xx[108] = xx[30] * (xx[111] - xx[99] * xx[24]);
  xx[62] = (xx[59] + xx[58]) * xx[30];
  xx[89] = xx[27] * xx[57];
  xx[96] = xx[27] * xx[62];
  xx[99] = xx[26] * xx[62];
  xx[109] = xx[70] + xx[99];
  xx[110] = xx[89];
  xx[111] = xx[96];
  xx[112] = - xx[109];
  pm_math_Vector3_cross_ra(xx + 25, xx + 110, xx + 113);
  xx[110] = xx[62] + (xx[24] * xx[89] + xx[113]) * xx[30];
  xx[111] = (xx[24] * xx[96] + xx[114]) * xx[30] - xx[57];
  xx[112] = xx[30] * (xx[115] - xx[109] * xx[24]);
  xx[70] = xx[30] * (xx[72] - xx[71]);
  xx[113] = xx[63];
  xx[114] = xx[70];
  xx[115] = xx[18] - xx[77];
  xx[63] = xx[25] * xx[27];
  xx[71] = xx[24] * xx[26];
  xx[116] = (xx[63] + xx[71]) * xx[30];
  xx[117] = xx[30] * (xx[84] - xx[85]);
  xx[118] = (xx[83] + xx[27] * xx[27]) * xx[30] - xx[18];
  xx[72] = (xx[76] + xx[90]) * xx[30];
  xx[76] = (xx[32] + xx[46]) * xx[30];
  xx[119] = xx[72] - xx[18];
  xx[120] = - xx[76];
  xx[121] = xx[30] * (xx[49] - xx[48]);
  xx[77] = xx[98] + xx[36];
  xx[36] = xx[25] * xx[95];
  xx[84] = xx[25] * xx[35];
  xx[122] = - xx[77];
  xx[123] = xx[36];
  xx[124] = xx[84];
  pm_math_Vector3_cross_ra(xx + 25, xx + 122, xx + 125);
  xx[122] = xx[30] * (xx[125] - xx[77] * xx[24]);
  xx[123] = xx[35] + (xx[24] * xx[36] + xx[126]) * xx[30];
  xx[124] = (xx[24] * xx[84] + xx[127]) * xx[30] - xx[95];
  xx[125] = (xx[48] + xx[49]) * xx[30];
  xx[126] = xx[70];
  xx[127] = (xx[39] + xx[90]) * xx[30] - xx[18];
  xx[36] = xx[97] + xx[61];
  xx[39] = xx[25] * xx[37];
  xx[48] = xx[25] * xx[60];
  xx[96] = - xx[36];
  xx[97] = xx[39];
  xx[98] = xx[48];
  pm_math_Vector3_cross_ra(xx + 25, xx + 96, xx + 128);
  xx[96] = xx[30] * (xx[128] - xx[36] * xx[24]);
  xx[97] = xx[60] + (xx[24] * xx[39] + xx[129]) * xx[30];
  xx[98] = (xx[24] * xx[48] + xx[130]) * xx[30] - xx[37];
  xx[36] = xx[99] + xx[69];
  xx[39] = xx[25] * xx[62];
  xx[48] = xx[25] * xx[68];
  xx[128] = - xx[36];
  xx[129] = xx[39];
  xx[130] = xx[48];
  pm_math_Vector3_cross_ra(xx + 25, xx + 128, xx + 131);
  xx[128] = xx[30] * (xx[131] - xx[36] * xx[24]);
  xx[129] = xx[68] + (xx[24] * xx[39] + xx[132]) * xx[30];
  xx[130] = (xx[24] * xx[48] + xx[133]) * xx[30] - xx[62];
  xx[131] = xx[30] * (xx[32] - xx[46]);
  xx[132] = xx[91] - xx[18];
  xx[133] = - xx[44];
  xx[89] = (xx[83] + xx[25] * xx[25]) * xx[30] - xx[18];
  xx[90] = (xx[81] + xx[82]) * xx[30];
  xx[91] = xx[30] * (xx[63] - xx[71]);
  xx[69] = xx[18] - xx[72];
  xx[70] = xx[76];
  xx[71] = xx[50];
  xx[32] = bb[0] ? xx[0] : - xx[18];
  xx[36] = 0.04500000000000001;
  xx[39] = xx[36] * xx[31];
  xx[44] = xx[27] * xx[39];
  xx[46] = xx[36] * xx[95];
  xx[48] = xx[27] * xx[46];
  xx[49] = xx[25] * xx[39] + xx[26] * xx[46];
  xx[81] = - xx[44];
  xx[82] = - xx[48];
  xx[83] = xx[49];
  pm_math_Vector3_cross_ra(xx + 25, xx + 81, xx + 134);
  xx[50] = 0.04499999999999999;
  xx[61] = xx[50] * xx[13];
  xx[63] = xx[50] * xx[15];
  xx[72] = xx[30] * (xx[61] * xx[17] - xx[63] * xx[14]);
  xx[76] = xx[5] * xx[72];
  xx[77] = (xx[61] * xx[13] + xx[63] * xx[15]) * xx[30] - xx[50] - xx[18];
  xx[81] = xx[77] * xx[7];
  xx[82] = xx[76] + xx[81];
  xx[83] = xx[4];
  xx[84] = xx[6];
  xx[85] = xx[1];
  xx[1] = xx[3] * xx[72];
  xx[4] = xx[77] * xx[3];
  xx[137] = - xx[82];
  xx[138] = xx[1];
  xx[139] = xx[4];
  pm_math_Vector3_cross_ra(xx + 83, xx + 137, xx + 140);
  xx[6] = xx[2] * xx[7];
  xx[99] = xx[3] * xx[5];
  xx[109] = xx[36] * xx[56];
  xx[137] = xx[27] * xx[109];
  xx[138] = xx[36] * xx[37];
  xx[139] = xx[27] * xx[138];
  xx[143] = xx[25] * xx[109] + xx[26] * xx[138];
  xx[144] = - xx[137];
  xx[145] = - xx[139];
  xx[146] = xx[143];
  pm_math_Vector3_cross_ra(xx + 25, xx + 144, xx + 147);
  xx[144] = (xx[63] * xx[17] + xx[61] * xx[14]) * xx[30];
  xx[61] = xx[5] * xx[144];
  xx[63] = xx[3] * xx[144];
  xx[145] = xx[81] - xx[63];
  xx[81] = xx[77] * xx[5];
  xx[150] = - xx[61];
  xx[151] = - xx[145];
  xx[152] = xx[81];
  pm_math_Vector3_cross_ra(xx + 83, xx + 150, xx + 153);
  xx[146] = xx[5] * xx[5];
  xx[150] = xx[7] * xx[7];
  xx[151] = xx[36] * xx[57];
  xx[152] = xx[27] * xx[151];
  xx[156] = xx[36] * xx[62];
  xx[157] = xx[27] * xx[156];
  xx[158] = xx[25] * xx[151] + xx[26] * xx[156];
  xx[159] = - xx[152];
  xx[160] = - xx[157];
  xx[161] = xx[158];
  pm_math_Vector3_cross_ra(xx + 25, xx + 159, xx + 162);
  xx[159] = xx[7] * xx[144];
  xx[160] = xx[7] * xx[72];
  xx[161] = xx[76] - xx[63];
  xx[165] = - xx[159];
  xx[166] = xx[160];
  xx[167] = - xx[161];
  pm_math_Vector3_cross_ra(xx + 83, xx + 165, xx + 168);
  xx[63] = bb[0] ? xx[0] : xx[18];
  xx[76] = 0.0225;
  xx[83] = xx[76] * xx[47];
  xx[84] = xx[38] * xx[83];
  xx[85] = xx[76] * xx[43];
  xx[165] = xx[76] * xx[45];
  xx[166] = xx[45] * xx[165];
  xx[167] = xx[47] * xx[83];
  xx[171] = xx[3] * xx[3];
  xx[172] = xx[43] * xx[85];
  xx[173] = xx[2] * xx[3];
  xx[174] = xx[5] * xx[7];
  xx[175] = xx[3] * xx[7];
  xx[176] = xx[2] * xx[5];
  xx[177] = xx[38] * xx[85];
  xx[178] = xx[38] * xx[165];
  xx[179] = xx[0];
  xx[180] = xx[0];
  xx[181] = xx[0];
  xx[182] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 40, xx + 51);
  xx[183] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 65, xx + 51);
  xx[184] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 73, xx + 51);
  xx[185] = xx[0];
  xx[186] = xx[0];
  xx[187] = xx[0];
  xx[188] = xx[0];
  xx[189] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 78, xx + 86);
  xx[190] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 92, xx + 86);
  xx[191] = xx[0];
  xx[192] = xx[0];
  xx[193] = xx[0];
  xx[194] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 100, xx + 103);
  xx[195] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 106, xx + 103);
  xx[196] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 110, xx + 103);
  xx[197] = xx[0];
  xx[198] = xx[0];
  xx[199] = xx[0];
  xx[200] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 113, xx + 116);
  xx[201] = xx[0];
  xx[202] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 119, xx + 116);
  xx[203] = xx[0];
  xx[204] = xx[0];
  xx[205] = xx[0];
  xx[206] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 122, xx + 125);
  xx[207] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 96, xx + 125);
  xx[208] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 128, xx + 125);
  xx[209] = xx[0];
  xx[210] = xx[0];
  xx[211] = xx[0];
  xx[212] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 131, xx + 89);
  xx[213] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 69, xx + 89);
  xx[214] = xx[0];
  xx[215] = xx[32];
  xx[216] = xx[0];
  xx[217] = xx[0];
  xx[218] = bb[0] ? xx[0] : - ((xx[134] - xx[24] * xx[44]) * xx[30] - xx[46] +
    (xx[82] * xx[2] + xx[140]) * xx[30] + xx[30] * (xx[6] - xx[99]));
  xx[219] = bb[0] ? xx[0] : - ((xx[147] - xx[24] * xx[137]) * xx[30] - xx[138] +
    xx[77] + xx[30] * (xx[153] + xx[2] * xx[61]) - (xx[146] + xx[150]) * xx[30]
    + xx[18]);
  xx[220] = bb[0] ? xx[0] : - ((xx[162] - xx[24] * xx[152]) * xx[30] - xx[156] +
    xx[30] * (xx[168] + xx[2] * xx[159]) - xx[72]);
  xx[221] = xx[63];
  xx[222] = xx[0];
  xx[223] = xx[0];
  xx[224] = bb[0] ? xx[0] : xx[30] * (xx[84] - xx[45] * xx[85]);
  xx[225] = bb[0] ? xx[0] : xx[76] - (xx[166] + xx[167]) * xx[30];
  xx[226] = xx[0];
  xx[227] = xx[0];
  xx[228] = xx[32];
  xx[229] = xx[0];
  xx[230] = bb[0] ? xx[0] : - ((xx[135] - xx[24] * xx[48]) * xx[30] + xx[30] *
    (xx[141] - xx[2] * xx[1]) + (xx[150] + xx[171]) * xx[30] - xx[77] + xx[39] -
    xx[18]);
  xx[231] = bb[0] ? xx[0] : - ((xx[148] - xx[24] * xx[139]) * xx[30] + xx[109] +
    (xx[145] * xx[2] + xx[154]) * xx[30] + (xx[6] + xx[99]) * xx[30]);
  xx[232] = bb[0] ? xx[0] : - ((xx[163] - xx[24] * xx[157]) * xx[30] + xx[151] +
    xx[30] * (xx[169] - xx[2] * xx[160]) - xx[144]);
  xx[233] = xx[0];
  xx[234] = xx[63];
  xx[235] = xx[0];
  xx[236] = bb[0] ? xx[0] : (xx[167] + xx[172]) * xx[30] - xx[76];
  xx[237] = bb[0] ? xx[0] : (xx[84] + xx[43] * xx[165]) * xx[30];
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[0];
  xx[241] = xx[32];
  xx[242] = bb[0] ? xx[0] : - (xx[30] * (xx[136] + xx[49] * xx[24]) + xx[72] +
    xx[30] * (xx[142] - xx[2] * xx[4]) - (xx[173] + xx[174]) * xx[30]);
  xx[243] = bb[0] ? xx[0] : - (xx[30] * (xx[149] + xx[143] * xx[24]) + xx[30] *
    (xx[155] - xx[2] * xx[81]) + xx[144] + xx[30] * (xx[175] - xx[176]));
  xx[244] = bb[0] ? xx[0] : - (xx[30] * (xx[164] + xx[158] * xx[24]) + (xx[161] *
    xx[2] + xx[170]) * xx[30]);
  xx[245] = xx[0];
  xx[246] = xx[0];
  xx[247] = xx[63];
  xx[248] = bb[0] ? xx[0] : - ((xx[177] + xx[45] * xx[83]) * xx[30]);
  xx[249] = bb[0] ? xx[0] : xx[30] * (xx[43] * xx[83] - xx[178]);
  xx[250] = xx[0];
  ii[0] = bb[0] ? 0 : 1;
  xx[40] = - xx[144];
  xx[41] = xx[72];
  xx[42] = xx[77];
  pm_math_Quaternion_xform_ra(xx + 8, xx + 40, xx + 65);
  xx[1] = xx[36] * xx[26];
  xx[4] = xx[36] * xx[25];
  xx[78] = pm_math_Vector3_dot_ra(xx + 51, xx + 86);
  xx[79] = pm_math_Vector3_dot_ra(xx + 103, xx + 116);
  xx[80] = pm_math_Vector3_dot_ra(xx + 125, xx + 89);
  xx[81] = (xx[178] + xx[47] * xx[85]) * xx[30] + state[17] - (xx[65] + state[0]
    + (xx[176] + xx[175]) * xx[30] - (xx[24] * xx[1] + xx[27] * xx[4]) * xx[30]);
  xx[82] = xx[30] * (xx[47] * xx[165] - xx[177]) + state[18] - (xx[30] * (xx[24]
    * xx[4] - xx[27] * xx[1]) + xx[66] + state[1] - xx[30] * (xx[173] - xx[174]));
  xx[83] = state[19] - (xx[172] + xx[166]) * xx[30] - (xx[67] + state[2] - (xx
    [171] + xx[146]) * xx[30] + (xx[25] * xx[4] + xx[26] * xx[1]) * xx[30]) -
    0.9325;
  zeroMajor(1, 6, ii + 0, xx + 78);
  xx[84] = - xx[78];
  xx[85] = - xx[79];
  xx[86] = - xx[80];
  xx[87] = - xx[81];
  xx[88] = - xx[82];
  xx[89] = - xx[83];
  xx[1] = 1.0e-8;
  memcpy(xx + 251, xx + 179, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 251, xx + 78, xx + 110, ii + 1, xx + 84, xx[1],
                     xx + 96);
  xx[4] = state[0] + xx[96];
  xx[8] = xx[2];
  xx[9] = xx[3];
  xx[10] = xx[5];
  xx[11] = xx[7];
  pm_math_Quaternion_compDeriv_ra(xx + 8, xx + 99, xx + 24);
  xx[6] = xx[2] + xx[24];
  xx[2] = xx[3] + xx[25];
  xx[3] = xx[5] + xx[26];
  xx[5] = xx[7] + xx[27];
  xx[7] = 1.0e-64;
  xx[8] = sqrt(xx[6] * xx[6] + xx[2] * xx[2] + xx[3] * xx[3] + xx[5] * xx[5]);
  if (xx[7] > xx[8])
    xx[8] = xx[7];
  xx[9] = xx[6] / xx[8];
  xx[6] = xx[2] / xx[8];
  xx[2] = - xx[6];
  xx[10] = xx[3] / xx[8];
  xx[3] = - xx[10];
  xx[11] = xx[5] / xx[8];
  xx[5] = - xx[11];
  xx[24] = - xx[9];
  xx[25] = xx[2];
  xx[26] = xx[3];
  xx[27] = xx[5];
  pm_math_Quaternion_compose_ra(xx + 24, xx + 20, xx + 78);
  xx[8] = xx[80] * xx[31];
  xx[44] = xx[81] * xx[35];
  xx[48] = xx[79] * xx[31];
  xx[49] = xx[44] + xx[48];
  xx[51] = xx[80] * xx[35];
  xx[65] = xx[8];
  xx[66] = - xx[49];
  xx[67] = xx[51];
  pm_math_Vector3_cross_ra(xx + 79, xx + 65, xx + 69);
  xx[65] = (xx[78] * xx[8] + xx[69]) * xx[30] - xx[35];
  xx[66] = xx[30] * (xx[70] - xx[49] * xx[78]);
  xx[67] = xx[31] + (xx[78] * xx[51] + xx[71]) * xx[30];
  xx[82] = xx[38];
  xx[83] = xx[43];
  xx[84] = xx[45];
  xx[85] = xx[47];
  pm_math_Quaternion_compDeriv_ra(xx + 82, xx + 105, xx + 86);
  xx[8] = xx[38] + xx[86];
  xx[38] = xx[43] + xx[87];
  xx[43] = xx[45] + xx[88];
  xx[45] = xx[47] + xx[89];
  xx[47] = sqrt(xx[8] * xx[8] + xx[38] * xx[38] + xx[43] * xx[43] + xx[45] * xx
                [45]);
  if (xx[7] > xx[47])
    xx[47] = xx[7];
  xx[49] = xx[8] / xx[47];
  xx[8] = xx[49] * xx[49];
  xx[51] = xx[38] / xx[47];
  xx[38] = xx[51] * xx[51];
  xx[52] = xx[43] / xx[47];
  xx[43] = xx[51] * xx[52];
  xx[53] = xx[45] / xx[47];
  xx[45] = xx[49] * xx[53];
  xx[47] = xx[51] * xx[53];
  xx[61] = xx[49] * xx[52];
  xx[69] = xx[30] * (xx[47] - xx[61]);
  xx[73] = (xx[8] + xx[38]) * xx[30] - xx[18];
  xx[74] = (xx[43] + xx[45]) * xx[30];
  xx[75] = xx[69];
  xx[70] = xx[80] * xx[56];
  xx[71] = xx[81] * xx[60];
  xx[82] = xx[79] * xx[56];
  xx[83] = xx[71] + xx[82];
  xx[84] = xx[80] * xx[60];
  xx[85] = xx[70];
  xx[86] = - xx[83];
  xx[87] = xx[84];
  pm_math_Vector3_cross_ra(xx + 79, xx + 85, xx + 88);
  xx[85] = (xx[78] * xx[70] + xx[88]) * xx[30] - xx[60];
  xx[86] = xx[30] * (xx[89] - xx[83] * xx[78]);
  xx[87] = xx[56] + (xx[78] * xx[84] + xx[90]) * xx[30];
  xx[70] = xx[80] * xx[57];
  xx[83] = xx[81] * xx[68];
  xx[84] = xx[79] * xx[57];
  xx[88] = xx[83] + xx[84];
  xx[89] = xx[80] * xx[68];
  xx[90] = xx[70];
  xx[91] = - xx[88];
  xx[92] = xx[89];
  pm_math_Vector3_cross_ra(xx + 79, xx + 90, xx + 110);
  xx[90] = (xx[78] * xx[70] + xx[110]) * xx[30] - xx[68];
  xx[91] = xx[30] * (xx[111] - xx[88] * xx[78]);
  xx[92] = xx[57] + (xx[78] * xx[89] + xx[112]) * xx[30];
  xx[70] = (xx[61] + xx[47]) * xx[30];
  xx[88] = xx[49] * xx[51];
  xx[89] = xx[52] * xx[53];
  xx[93] = xx[52] * xx[52];
  xx[94] = (xx[38] + xx[93]) * xx[30];
  xx[110] = - xx[70];
  xx[111] = xx[30] * (xx[88] - xx[89]);
  xx[112] = xx[94] - xx[18];
  xx[108] = xx[79] * xx[80];
  xx[113] = xx[78] * xx[81];
  xx[114] = xx[78] * xx[78];
  xx[115] = xx[80] * xx[81];
  xx[116] = xx[78] * xx[79];
  xx[117] = xx[30] * (xx[108] - xx[113]);
  xx[118] = (xx[114] + xx[80] * xx[80]) * xx[30] - xx[18];
  xx[119] = (xx[115] + xx[116]) * xx[30];
  xx[120] = xx[30] * (xx[43] - xx[45]);
  xx[121] = xx[53] * xx[53];
  xx[122] = (xx[121] + xx[38]) * xx[30];
  xx[38] = (xx[88] + xx[89]) * xx[30];
  xx[123] = xx[120];
  xx[124] = xx[18] - xx[122];
  xx[125] = xx[38];
  xx[126] = xx[81] * xx[31];
  xx[127] = xx[81] * xx[95];
  xx[128] = xx[80] * xx[95];
  xx[129] = xx[48] + xx[128];
  xx[130] = xx[126];
  xx[131] = xx[127];
  xx[132] = - xx[129];
  pm_math_Vector3_cross_ra(xx + 79, xx + 130, xx + 133);
  xx[130] = xx[95] + (xx[78] * xx[126] + xx[133]) * xx[30];
  xx[131] = (xx[78] * xx[127] + xx[134]) * xx[30] - xx[31];
  xx[132] = xx[30] * (xx[135] - xx[129] * xx[78]);
  xx[133] = xx[120];
  xx[134] = (xx[8] + xx[93]) * xx[30] - xx[18];
  xx[135] = (xx[89] + xx[88]) * xx[30];
  xx[31] = xx[81] * xx[56];
  xx[48] = xx[81] * xx[37];
  xx[120] = xx[80] * xx[37];
  xx[126] = xx[82] + xx[120];
  xx[139] = xx[31];
  xx[140] = xx[48];
  xx[141] = - xx[126];
  pm_math_Vector3_cross_ra(xx + 79, xx + 139, xx + 145);
  xx[139] = xx[37] + (xx[78] * xx[31] + xx[145]) * xx[30];
  xx[140] = (xx[78] * xx[48] + xx[146]) * xx[30] - xx[56];
  xx[141] = xx[30] * (xx[147] - xx[126] * xx[78]);
  xx[31] = xx[81] * xx[57];
  xx[48] = xx[81] * xx[62];
  xx[56] = xx[80] * xx[62];
  xx[82] = xx[84] + xx[56];
  xx[145] = xx[31];
  xx[146] = xx[48];
  xx[147] = - xx[82];
  pm_math_Vector3_cross_ra(xx + 79, xx + 145, xx + 148);
  xx[145] = xx[62] + (xx[78] * xx[31] + xx[148]) * xx[30];
  xx[146] = (xx[78] * xx[48] + xx[149]) * xx[30] - xx[57];
  xx[147] = xx[30] * (xx[150] - xx[82] * xx[78]);
  xx[31] = xx[30] * (xx[89] - xx[88]);
  xx[148] = xx[70];
  xx[149] = xx[31];
  xx[150] = xx[18] - xx[94];
  xx[48] = xx[79] * xx[81];
  xx[70] = xx[78] * xx[80];
  xx[152] = (xx[48] + xx[70]) * xx[30];
  xx[153] = xx[30] * (xx[115] - xx[116]);
  xx[154] = (xx[114] + xx[81] * xx[81]) * xx[30] - xx[18];
  xx[82] = (xx[93] + xx[121]) * xx[30];
  xx[84] = (xx[45] + xx[43]) * xx[30];
  xx[157] = xx[82] - xx[18];
  xx[158] = - xx[84];
  xx[159] = xx[30] * (xx[61] - xx[47]);
  xx[88] = xx[128] + xx[44];
  xx[44] = xx[79] * xx[95];
  xx[89] = xx[79] * xx[35];
  xx[126] = - xx[88];
  xx[127] = xx[44];
  xx[128] = xx[89];
  pm_math_Vector3_cross_ra(xx + 79, xx + 126, xx + 160);
  xx[126] = xx[30] * (xx[160] - xx[88] * xx[78]);
  xx[127] = xx[35] + (xx[78] * xx[44] + xx[161]) * xx[30];
  xx[128] = (xx[78] * xx[89] + xx[162]) * xx[30] - xx[95];
  xx[160] = (xx[47] + xx[61]) * xx[30];
  xx[161] = xx[31];
  xx[162] = (xx[8] + xx[121]) * xx[30] - xx[18];
  xx[8] = xx[120] + xx[71];
  xx[31] = xx[79] * xx[37];
  xx[35] = xx[79] * xx[60];
  xx[163] = - xx[8];
  xx[164] = xx[31];
  xx[165] = xx[35];
  pm_math_Vector3_cross_ra(xx + 79, xx + 163, xx + 166);
  xx[163] = xx[30] * (xx[166] - xx[8] * xx[78]);
  xx[164] = xx[60] + (xx[78] * xx[31] + xx[167]) * xx[30];
  xx[165] = (xx[78] * xx[35] + xx[168]) * xx[30] - xx[37];
  xx[8] = xx[56] + xx[83];
  xx[31] = xx[79] * xx[62];
  xx[35] = xx[79] * xx[68];
  xx[166] = - xx[8];
  xx[167] = xx[31];
  xx[168] = xx[35];
  pm_math_Vector3_cross_ra(xx + 79, xx + 166, xx + 169);
  xx[166] = xx[30] * (xx[169] - xx[8] * xx[78]);
  xx[167] = xx[68] + (xx[78] * xx[31] + xx[170]) * xx[30];
  xx[168] = (xx[78] * xx[35] + xx[171]) * xx[30] - xx[62];
  xx[169] = xx[30] * (xx[45] - xx[43]);
  xx[170] = xx[122] - xx[18];
  xx[171] = - xx[38];
  xx[43] = (xx[114] + xx[79] * xx[79]) * xx[30] - xx[18];
  xx[44] = (xx[108] + xx[113]) * xx[30];
  xx[45] = xx[30] * (xx[48] - xx[70]);
  xx[113] = xx[18] - xx[82];
  xx[114] = xx[84];
  xx[115] = xx[69];
  xx[8] = xx[81] * xx[39];
  xx[31] = xx[81] * xx[46];
  xx[35] = xx[79] * xx[39] + xx[80] * xx[46];
  xx[68] = - xx[8];
  xx[69] = - xx[31];
  xx[70] = xx[35];
  pm_math_Vector3_cross_ra(xx + 79, xx + 68, xx + 82);
  xx[37] = xx[10] * xx[72];
  xx[38] = xx[77] * xx[11];
  xx[47] = xx[37] + xx[38];
  xx[68] = xx[2];
  xx[69] = xx[3];
  xx[70] = xx[5];
  xx[2] = xx[6] * xx[72];
  xx[3] = xx[77] * xx[6];
  xx[120] = - xx[47];
  xx[121] = xx[2];
  xx[122] = xx[3];
  pm_math_Vector3_cross_ra(xx + 68, xx + 120, xx + 172);
  xx[5] = xx[9] * xx[11];
  xx[48] = xx[6] * xx[10];
  xx[56] = xx[81] * xx[109];
  xx[61] = xx[81] * xx[138];
  xx[62] = xx[79] * xx[109] + xx[80] * xx[138];
  xx[120] = - xx[56];
  xx[121] = - xx[61];
  xx[122] = xx[62];
  pm_math_Vector3_cross_ra(xx + 79, xx + 120, xx + 175);
  xx[71] = xx[10] * xx[144];
  xx[88] = xx[6] * xx[144];
  xx[89] = xx[38] - xx[88];
  xx[38] = xx[77] * xx[10];
  xx[120] = - xx[71];
  xx[121] = - xx[89];
  xx[122] = xx[38];
  pm_math_Vector3_cross_ra(xx + 68, xx + 120, xx + 178);
  xx[93] = xx[10] * xx[10];
  xx[94] = xx[11] * xx[11];
  xx[108] = xx[81] * xx[151];
  xx[116] = xx[81] * xx[156];
  xx[120] = xx[79] * xx[151] + xx[80] * xx[156];
  xx[181] = - xx[108];
  xx[182] = - xx[116];
  xx[183] = xx[120];
  pm_math_Vector3_cross_ra(xx + 79, xx + 181, xx + 184);
  xx[121] = xx[11] * xx[144];
  xx[122] = xx[11] * xx[72];
  xx[129] = xx[37] - xx[88];
  xx[181] = - xx[121];
  xx[182] = xx[122];
  xx[183] = - xx[129];
  pm_math_Vector3_cross_ra(xx + 68, xx + 181, xx + 187);
  xx[37] = xx[76] * xx[53];
  xx[68] = xx[49] * xx[37];
  xx[69] = xx[76] * xx[51];
  xx[70] = xx[76] * xx[52];
  xx[88] = xx[52] * xx[70];
  xx[136] = xx[53] * xx[37];
  xx[137] = xx[6] * xx[6];
  xx[142] = xx[51] * xx[69];
  xx[143] = xx[9] * xx[6];
  xx[155] = xx[10] * xx[11];
  xx[181] = xx[6] * xx[11];
  xx[182] = xx[9] * xx[10];
  xx[183] = xx[49] * xx[69];
  xx[190] = xx[49] * xx[70];
  xx[191] = xx[0];
  xx[192] = xx[0];
  xx[193] = xx[0];
  xx[194] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 65, xx + 73);
  xx[195] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 85, xx + 73);
  xx[196] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 90, xx + 73);
  xx[197] = xx[0];
  xx[198] = xx[0];
  xx[199] = xx[0];
  xx[200] = xx[0];
  xx[201] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 110, xx + 117);
  xx[202] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 123, xx + 117);
  xx[203] = xx[0];
  xx[204] = xx[0];
  xx[205] = xx[0];
  xx[206] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 130, xx + 133);
  xx[207] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 139, xx + 133);
  xx[208] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 145, xx + 133);
  xx[209] = xx[0];
  xx[210] = xx[0];
  xx[211] = xx[0];
  xx[212] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 148, xx + 152);
  xx[213] = xx[0];
  xx[214] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 157, xx + 152);
  xx[215] = xx[0];
  xx[216] = xx[0];
  xx[217] = xx[0];
  xx[218] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 126, xx + 160);
  xx[219] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 163, xx + 160);
  xx[220] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 166, xx + 160);
  xx[221] = xx[0];
  xx[222] = xx[0];
  xx[223] = xx[0];
  xx[224] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 169, xx + 43);
  xx[225] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 113, xx + 43);
  xx[226] = xx[0];
  xx[227] = xx[32];
  xx[228] = xx[0];
  xx[229] = xx[0];
  xx[230] = bb[0] ? xx[0] : - ((xx[82] - xx[78] * xx[8]) * xx[30] - xx[46] +
    (xx[47] * xx[9] + xx[172]) * xx[30] + xx[30] * (xx[5] - xx[48]));
  xx[231] = bb[0] ? xx[0] : - ((xx[175] - xx[78] * xx[56]) * xx[30] - xx[138] +
    xx[77] + xx[30] * (xx[178] + xx[9] * xx[71]) - (xx[93] + xx[94]) * xx[30] +
    xx[18]);
  xx[232] = bb[0] ? xx[0] : - ((xx[184] - xx[78] * xx[108]) * xx[30] - xx[156] +
    xx[30] * (xx[187] + xx[9] * xx[121]) - xx[72]);
  xx[233] = xx[63];
  xx[234] = xx[0];
  xx[235] = xx[0];
  xx[236] = bb[0] ? xx[0] : xx[30] * (xx[68] - xx[52] * xx[69]);
  xx[237] = bb[0] ? xx[0] : xx[76] - (xx[88] + xx[136]) * xx[30];
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[32];
  xx[241] = xx[0];
  xx[242] = bb[0] ? xx[0] : - ((xx[83] - xx[78] * xx[31]) * xx[30] + xx[30] *
    (xx[173] - xx[9] * xx[2]) + (xx[94] + xx[137]) * xx[30] - xx[77] + xx[39] -
    xx[18]);
  xx[243] = bb[0] ? xx[0] : - ((xx[176] - xx[78] * xx[61]) * xx[30] + xx[109] +
    (xx[89] * xx[9] + xx[179]) * xx[30] + (xx[5] + xx[48]) * xx[30]);
  xx[244] = bb[0] ? xx[0] : - ((xx[185] - xx[78] * xx[116]) * xx[30] + xx[151] +
    xx[30] * (xx[188] - xx[9] * xx[122]) - xx[144]);
  xx[245] = xx[0];
  xx[246] = xx[63];
  xx[247] = xx[0];
  xx[248] = bb[0] ? xx[0] : (xx[136] + xx[142]) * xx[30] - xx[76];
  xx[249] = bb[0] ? xx[0] : (xx[68] + xx[51] * xx[70]) * xx[30];
  xx[250] = xx[0];
  xx[251] = xx[0];
  xx[252] = xx[0];
  xx[253] = xx[32];
  xx[254] = bb[0] ? xx[0] : - (xx[30] * (xx[84] + xx[35] * xx[78]) + xx[72] +
    xx[30] * (xx[174] - xx[9] * xx[3]) - (xx[143] + xx[155]) * xx[30]);
  xx[255] = bb[0] ? xx[0] : - (xx[30] * (xx[177] + xx[62] * xx[78]) + xx[30] *
    (xx[180] - xx[9] * xx[38]) + xx[144] + xx[30] * (xx[181] - xx[182]));
  xx[256] = bb[0] ? xx[0] : - (xx[30] * (xx[186] + xx[120] * xx[78]) + (xx[129] *
    xx[9] + xx[189]) * xx[30]);
  xx[257] = xx[0];
  xx[258] = xx[0];
  xx[259] = xx[63];
  xx[260] = bb[0] ? xx[0] : - ((xx[183] + xx[52] * xx[37]) * xx[30]);
  xx[261] = bb[0] ? xx[0] : xx[30] * (xx[51] * xx[37] - xx[190]);
  xx[262] = xx[0];
  xx[2] = state[17] + xx[102];
  pm_math_Quaternion_xform_ra(xx + 24, xx + 40, xx + 37);
  xx[3] = xx[36] * xx[80];
  xx[5] = xx[36] * xx[79];
  xx[8] = state[18] + xx[103];
  xx[24] = state[1] + xx[97];
  xx[25] = state[19] + xx[104];
  xx[26] = state[2] + xx[98];
  xx[82] = pm_math_Vector3_dot_ra(xx + 73, xx + 117);
  xx[83] = pm_math_Vector3_dot_ra(xx + 133, xx + 152);
  xx[84] = pm_math_Vector3_dot_ra(xx + 160, xx + 43);
  xx[85] = (xx[190] + xx[53] * xx[69]) * xx[30] + xx[2] - (xx[37] + xx[4] + (xx
    [182] + xx[181]) * xx[30] - (xx[78] * xx[3] + xx[81] * xx[5]) * xx[30]);
  xx[86] = xx[30] * (xx[53] * xx[70] - xx[183]) + xx[8] - (xx[30] * (xx[78] *
    xx[5] - xx[81] * xx[3]) + xx[38] + xx[24] - xx[30] * (xx[143] - xx[155]));
  xx[87] = xx[76] - (xx[142] + xx[88]) * xx[30] + xx[25] - (xx[39] + xx[26] -
    ((xx[137] + xx[93]) * xx[30] - xx[18]) + (xx[79] * xx[5] + xx[80] * xx[3]) *
    xx[30]) + xx[36];
  zeroMajor(1, 6, ii + 0, xx + 82);
  xx[43] = - xx[82];
  xx[44] = - xx[83];
  xx[45] = - xx[84];
  xx[46] = - xx[85];
  xx[47] = - xx[86];
  xx[48] = - xx[87];
  memcpy(xx + 96, xx + 191, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 96, xx + 65, xx + 89, ii + 1, xx + 43, xx[1],
                     xx + 77);
  xx[1] = xx[4] + xx[77];
  xx[3] = xx[24] + xx[78];
  xx[4] = xx[26] + xx[79];
  xx[43] = xx[9];
  xx[44] = xx[6];
  xx[45] = xx[10];
  xx[46] = xx[11];
  pm_math_Quaternion_compDeriv_ra(xx + 43, xx + 80, xx + 65);
  xx[5] = xx[9] + xx[65];
  xx[9] = xx[6] + xx[66];
  xx[6] = xx[10] + xx[67];
  xx[10] = xx[11] + xx[68];
  xx[11] = sqrt(xx[5] * xx[5] + xx[9] * xx[9] + xx[6] * xx[6] + xx[10] * xx[10]);
  if (xx[7] > xx[11])
    xx[11] = xx[7];
  xx[24] = xx[5] / xx[11];
  xx[5] = xx[9] / xx[11];
  xx[9] = xx[6] / xx[11];
  xx[6] = xx[10] / xx[11];
  xx[10] = xx[2] + xx[83];
  xx[2] = xx[8] + xx[84];
  xx[8] = xx[25] + xx[85];
  xx[43] = xx[49];
  xx[44] = xx[51];
  xx[45] = xx[52];
  xx[46] = xx[53];
  pm_math_Quaternion_compDeriv_ra(xx + 43, xx + 86, xx + 65);
  xx[11] = xx[49] + xx[65];
  xx[25] = xx[51] + xx[66];
  xx[26] = xx[52] + xx[67];
  xx[27] = xx[53] + xx[68];
  xx[31] = sqrt(xx[11] * xx[11] + xx[25] * xx[25] + xx[26] * xx[26] + xx[27] *
                xx[27]);
  if (xx[7] > xx[31])
    xx[31] = xx[7];
  xx[7] = xx[11] / xx[31];
  xx[11] = xx[25] / xx[31];
  xx[25] = xx[26] / xx[31];
  xx[26] = xx[27] / xx[31];
  xx[96] = xx[1];
  xx[97] = xx[3];
  xx[98] = xx[4];
  xx[99] = xx[24];
  xx[100] = xx[5];
  xx[101] = xx[9];
  xx[102] = xx[6];
  xx[103] = state[7];
  xx[104] = state[8];
  xx[105] = state[9];
  xx[106] = state[10];
  xx[107] = state[11];
  xx[108] = state[12];
  xx[109] = state[13];
  xx[110] = state[14];
  xx[111] = state[15];
  xx[112] = state[16];
  xx[113] = xx[10];
  xx[114] = xx[2];
  xx[115] = xx[8];
  xx[116] = xx[7];
  xx[117] = xx[11];
  xx[118] = xx[25];
  xx[119] = xx[26];
  xx[120] = state[24];
  xx[121] = state[25];
  xx[122] = state[26];
  xx[123] = state[27];
  xx[124] = state[28];
  xx[125] = state[29];
  xx[27] = xx[7] * xx[7];
  xx[31] = xx[11] * xx[25];
  xx[35] = xx[7] * xx[26];
  xx[37] = xx[11] * xx[26];
  xx[38] = xx[7] * xx[25];
  xx[43] = (xx[27] + xx[11] * xx[11]) * xx[30] - xx[18];
  xx[44] = (xx[31] + xx[35]) * xx[30];
  xx[45] = xx[30] * (xx[37] - xx[38]);
  xx[46] = - xx[24];
  xx[47] = - xx[5];
  xx[48] = - xx[9];
  xx[49] = - xx[6];
  pm_math_Quaternion_compose_ra(xx + 46, xx + 20, xx + 65);
  xx[39] = xx[66] * xx[67];
  xx[51] = xx[65] * xx[68];
  xx[52] = xx[65] * xx[65];
  xx[53] = xx[67] * xx[68];
  xx[56] = xx[65] * xx[66];
  xx[69] = xx[30] * (xx[39] - xx[51]);
  xx[70] = (xx[52] + xx[67] * xx[67]) * xx[30] - xx[18];
  xx[71] = (xx[53] + xx[56]) * xx[30];
  xx[61] = xx[25] * xx[26];
  xx[62] = xx[7] * xx[11];
  xx[72] = xx[30] * (xx[31] - xx[35]);
  xx[73] = (xx[27] + xx[25] * xx[25]) * xx[30] - xx[18];
  xx[74] = (xx[61] + xx[62]) * xx[30];
  xx[31] = xx[66] * xx[68];
  xx[35] = xx[65] * xx[67];
  xx[77] = (xx[31] + xx[35]) * xx[30];
  xx[78] = xx[30] * (xx[53] - xx[56]);
  xx[79] = (xx[52] + xx[68] * xx[68]) * xx[30] - xx[18];
  xx[80] = (xx[37] + xx[38]) * xx[30];
  xx[81] = xx[30] * (xx[61] - xx[62]);
  xx[82] = (xx[27] + xx[26] * xx[26]) * xx[30] - xx[18];
  xx[83] = (xx[52] + xx[66] * xx[66]) * xx[30] - xx[18];
  xx[84] = (xx[39] + xx[51]) * xx[30];
  xx[85] = xx[30] * (xx[31] - xx[35]);
  xx[27] = xx[76] * xx[25];
  xx[31] = xx[76] * xx[11];
  pm_math_Quaternion_xform_ra(xx + 46, xx + 40, xx + 37);
  xx[35] = xx[37] + xx[1] + (xx[24] * xx[9] + xx[5] * xx[6]) * xx[30];
  xx[1] = xx[36] * xx[67];
  xx[51] = xx[36] * xx[66];
  xx[52] = xx[38] + xx[3] - xx[30] * (xx[24] * xx[5] - xx[9] * xx[6]);
  xx[3] = xx[39] + xx[4] - (xx[5] * xx[5] + xx[9] * xx[9]) * xx[30] + xx[18];
  xx[86] = pm_math_Vector3_dot_ra(xx + 43, xx + 69);
  xx[87] = pm_math_Vector3_dot_ra(xx + 72, xx + 77);
  xx[88] = pm_math_Vector3_dot_ra(xx + 80, xx + 83);
  xx[89] = (xx[7] * xx[27] + xx[26] * xx[31]) * xx[30] + xx[10] - (xx[35] - (xx
    [65] * xx[1] + xx[68] * xx[51]) * xx[30]);
  xx[90] = xx[30] * (xx[26] * xx[27] - xx[7] * xx[31]) + xx[2] - (xx[30] * (xx
    [65] * xx[51] - xx[68] * xx[1]) + xx[52]);
  xx[91] = xx[76] - (xx[11] * xx[31] + xx[25] * xx[27]) * xx[30] + xx[8] - ((xx
    [66] * xx[51] + xx[67] * xx[1]) * xx[30] - xx[36] + xx[3]);
  zeroMajor(1, 6, ii + 0, xx + 86);
  xx[77] = fabs(xx[86]);
  xx[78] = fabs(xx[87]);
  xx[79] = fabs(xx[88]);
  xx[80] = fabs(xx[89]);
  xx[81] = fabs(xx[90]);
  xx[82] = fabs(xx[91]);
  ii[1] = 77;

  {
    int ll;
    for (ll = 78; ll < 83; ++ll)
      if (xx[ll] > xx[ii[1]])
        ii[1] = ll;
  }

  ii[1] -= 77;
  xx[1] = xx[77 + (ii[1])];
  xx[4] = 1.0e-9;
  if (!(!(xx[1] > xx[4]) && (bb[0] ? 1 : pm_math_Vector3_dot_ra(xx + 43, xx + 83)
        > xx[12] && pm_math_Vector3_dot_ra(xx + 72, xx + 69) > xx[12]))) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:mechanism:mechanism:jointDisToNormPositionSatisfactionFailure",
      "The position components of the kinematic constraints of the mechanism cannot be satisfied when attempting to transition joints from disengaged to normal mode. This is more likely to occur for large rotation errors compared to large translation errors. Consider reducing the error in those joints before attempting the transition or consider increasing the joint mode transition nonlinear iterations parameter in the associated Mechanism Configuration block.",
      neDiagMgr);
  }

  xx[1] = xx[101] * xx[101];
  xx[5] = xx[102] * xx[102];
  xx[6] = xx[18] - (xx[1] + xx[5]) * xx[30];
  xx[9] = xx[100] * xx[101];
  xx[24] = xx[99] * xx[102];
  xx[27] = xx[30] * (xx[9] - xx[24]);
  xx[31] = xx[99] * xx[101];
  xx[37] = xx[100] * xx[102];
  xx[38] = (xx[31] + xx[37]) * xx[30];
  xx[43] = xx[6];
  xx[44] = xx[27];
  xx[45] = xx[38];
  xx[39] = 1.1;
  xx[51] = xx[17] * xx[17];
  xx[53] = (xx[51] + xx[64]) * xx[30] - xx[18];
  xx[56] = (xx[34] + xx[33]) * xx[30];
  xx[33] = (xx[55] + xx[54]) * xx[30];
  xx[34] = (xx[51] + xx[28]) * xx[30] - xx[18];
  xx[28] = (xx[58] + xx[59]) * xx[30];
  xx[54] = (xx[51] + xx[29]) * xx[30] - xx[18];
  xx[77] = xx[53];
  xx[78] = xx[95];
  xx[79] = xx[56];
  xx[80] = xx[33];
  xx[81] = xx[34];
  xx[82] = xx[60];
  xx[83] = xx[57];
  xx[84] = xx[28];
  xx[85] = xx[54];
  xx[29] = 0.1;
  xx[86] = xx[29] * xx[53];
  xx[87] = xx[29] * xx[33];
  xx[88] = xx[29] * xx[57];
  xx[89] = xx[29] * xx[95];
  xx[90] = xx[29] * xx[34];
  xx[91] = xx[29] * xx[28];
  xx[92] = xx[29] * xx[56];
  xx[93] = xx[29] * xx[60];
  xx[94] = xx[29] * xx[54];
  pm_math_Matrix3x3_compose_ra(xx + 77, xx + 86, xx + 126);
  xx[51] = xx[39] + xx[126];
  xx[55] = xx[39] + xx[130];
  xx[86] = xx[51];
  xx[87] = xx[127];
  xx[88] = xx[128];
  xx[89] = xx[129];
  xx[90] = xx[55];
  xx[91] = xx[131];
  xx[92] = xx[132];
  xx[93] = xx[133];
  xx[94] = xx[39] + xx[134];
  pm_math_Matrix3x3_xform_ra(xx + 86, xx + 43, xx + 69);
  xx[39] = (xx[24] + xx[9]) * xx[30];
  xx[9] = xx[100] * xx[100];
  xx[24] = xx[18] - (xx[5] + xx[9]) * xx[30];
  xx[5] = xx[101] * xx[102];
  xx[58] = xx[99] * xx[100];
  xx[59] = xx[30] * (xx[5] - xx[58]);
  xx[72] = xx[39];
  xx[73] = xx[24];
  xx[74] = xx[59];
  pm_math_Matrix3x3_xform_ra(xx + 86, xx + 72, xx + 135);
  xx[61] = pm_math_Vector3_dot_ra(xx + 43, xx + 135);
  xx[62] = xx[30] * (xx[37] - xx[31]);
  xx[31] = (xx[58] + xx[5]) * xx[30];
  xx[5] = xx[18] - (xx[9] + xx[1]) * xx[30];
  xx[138] = xx[62];
  xx[139] = xx[31];
  xx[140] = xx[5];
  pm_math_Matrix3x3_xform_ra(xx + 86, xx + 138, xx + 141);
  xx[1] = pm_math_Vector3_dot_ra(xx + 43, xx + 141);
  pm_math_Matrix3x3_postCross_ra(xx + 126, xx + 40, xx + 86);
  xx[144] = - (xx[86] + xx[127]);
  xx[145] = - (xx[89] + xx[55]);
  xx[146] = - (xx[92] + xx[133]);
  xx[9] = pm_math_Vector3_dot_ra(xx + 43, xx + 144);
  xx[37] = xx[129] - xx[90];
  xx[126] = xx[51] - xx[87];
  xx[127] = xx[37];
  xx[128] = xx[132] - xx[93];
  xx[58] = pm_math_Vector3_dot_ra(xx + 43, xx + 126);
  xx[129] = - xx[88];
  xx[130] = - xx[91];
  xx[131] = - xx[94];
  xx[64] = pm_math_Vector3_dot_ra(xx + 43, xx + 129);
  xx[75] = pm_math_Vector3_dot_ra(xx + 72, xx + 141);
  xx[132] = pm_math_Vector3_dot_ra(xx + 72, xx + 144);
  xx[133] = pm_math_Vector3_dot_ra(xx + 72, xx + 126);
  xx[134] = pm_math_Vector3_dot_ra(xx + 72, xx + 129);
  xx[147] = pm_math_Vector3_dot_ra(xx + 138, xx + 144);
  xx[144] = pm_math_Vector3_dot_ra(xx + 138, xx + 126);
  xx[126] = pm_math_Vector3_dot_ra(xx + 138, xx + 129);
  xx[127] = 1.181250000000001e-4;
  xx[128] = 1.0125e-4;
  xx[148] = xx[127] * xx[53];
  xx[149] = xx[127] * xx[33];
  xx[150] = xx[127] * xx[57];
  xx[151] = xx[127] * xx[95];
  xx[152] = xx[127] * xx[34];
  xx[153] = xx[127] * xx[28];
  xx[154] = xx[128] * xx[56];
  xx[155] = xx[128] * xx[60];
  xx[156] = xx[128] * xx[54];
  pm_math_Matrix3x3_compose_ra(xx + 77, xx + 148, xx + 157);
  pm_math_Matrix3x3_preCross_ra(xx + 86, xx + 40, xx + 77);
  xx[28] = xx[158] - xx[78] - xx[86] - xx[37];
  xx[33] = xx[159] - xx[79] + xx[91];
  xx[34] = xx[162] - xx[82] - xx[88];
  xx[166] = pm_math_Vector3_dot_ra(xx + 43, xx + 69);
  xx[167] = xx[61];
  xx[168] = xx[1];
  xx[169] = xx[9];
  xx[170] = xx[58];
  xx[171] = xx[64];
  xx[172] = xx[61];
  xx[173] = pm_math_Vector3_dot_ra(xx + 72, xx + 135);
  xx[174] = xx[75];
  xx[175] = xx[132];
  xx[176] = xx[133];
  xx[177] = xx[134];
  xx[178] = xx[1];
  xx[179] = xx[75];
  xx[180] = pm_math_Vector3_dot_ra(xx + 138, xx + 141);
  xx[181] = xx[147];
  xx[182] = xx[144];
  xx[183] = xx[126];
  xx[184] = xx[9];
  xx[185] = xx[132];
  xx[186] = xx[147];
  xx[187] = xx[157] - xx[77] + xx[89] + xx[89] + xx[55] + xx[18];
  xx[188] = xx[28];
  xx[189] = xx[33];
  xx[190] = xx[58];
  xx[191] = xx[133];
  xx[192] = xx[144];
  xx[193] = xx[28];
  xx[194] = xx[161] - xx[81] - xx[87] + xx[51] - xx[87] + xx[18];
  xx[195] = xx[34];
  xx[196] = xx[64];
  xx[197] = xx[134];
  xx[198] = xx[126];
  xx[199] = xx[33];
  xx[200] = xx[34];
  xx[201] = xx[18] + xx[165] - xx[85];
  ii[1] = factorSymmetricPosDef(xx + 166, 6, xx + 53);
  if (ii[1] != 0) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:mechanism:mechanism:degenerateMassFoll",
      "'simulation/Plant/ Rocket Body/Free Movement' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  bb[1] = prevModeVector[0] == -1 && modeVector[0] == 0;
  ii[1] = bb[1] ? 1 : 0;
  ii[2] = ii[1] + ii[1] + ii[1] + ii[1] + ii[1] + ii[1];
  ii[3] = ii[1];
  ii[4] = ii[1];
  ii[5] = ii[1];
  ii[6] = ii[1];
  ii[7] = ii[1];
  ii[8] = ii[1];
  sm_core_math_computeIndicesNonzeroElements(6, ii + 3, ii + 9);
  xx[1] = xx[118] * xx[118];
  xx[9] = xx[119] * xx[119];
  xx[28] = xx[18] - (xx[1] + xx[9]) * xx[30];
  xx[33] = xx[117] * xx[118];
  xx[34] = xx[116] * xx[119];
  xx[37] = xx[30] * (xx[33] - xx[34]);
  xx[51] = xx[116] * xx[118];
  xx[53] = xx[117] * xx[119];
  xx[54] = (xx[51] + xx[53]) * xx[30];
  xx[55] = xx[28];
  xx[56] = xx[37];
  xx[57] = xx[54];
  xx[69] = xx[29] * xx[28];
  xx[70] = xx[29] * xx[37];
  xx[71] = xx[29] * xx[54];
  xx[58] = (xx[34] + xx[33]) * xx[30];
  xx[60] = xx[117] * xx[117];
  xx[61] = xx[18] - (xx[9] + xx[60]) * xx[30];
  xx[64] = xx[118] * xx[119];
  xx[75] = xx[116] * xx[117];
  xx[77] = xx[30] * (xx[64] - xx[75]);
  xx[78] = xx[29] * xx[58];
  xx[79] = xx[29] * xx[61];
  xx[80] = xx[29] * xx[77];
  xx[81] = pm_math_Vector3_dot_ra(xx + 55, xx + 78);
  xx[82] = xx[30] * (xx[53] - xx[51]);
  xx[83] = (xx[75] + xx[64]) * xx[30];
  xx[84] = xx[18] - (xx[60] + xx[1]) * xx[30];
  xx[85] = xx[29] * xx[82];
  xx[86] = xx[29] * xx[83];
  xx[87] = xx[29] * xx[84];
  xx[29] = pm_math_Vector3_dot_ra(xx + 55, xx + 85);
  xx[88] = xx[58];
  xx[89] = xx[61];
  xx[90] = xx[77];
  xx[91] = pm_math_Vector3_dot_ra(xx + 88, xx + 85);
  xx[92] = 6.750000000000001e-5;
  xx[202] = pm_math_Vector3_dot_ra(xx + 55, xx + 69);
  xx[203] = xx[81];
  xx[204] = xx[29];
  xx[205] = xx[0];
  xx[206] = xx[0];
  xx[207] = xx[0];
  xx[208] = xx[81];
  xx[209] = pm_math_Vector3_dot_ra(xx + 88, xx + 78);
  xx[210] = xx[91];
  xx[211] = xx[0];
  xx[212] = xx[0];
  xx[213] = xx[0];
  xx[214] = xx[29];
  xx[215] = xx[91];
  xx[216] = pm_math_Vector3_dot_ra(xx + 82, xx + 85);
  xx[217] = xx[0];
  xx[218] = xx[0];
  xx[219] = xx[0];
  xx[220] = xx[0];
  xx[221] = xx[0];
  xx[222] = xx[0];
  xx[223] = xx[92];
  xx[224] = xx[0];
  xx[225] = xx[0];
  xx[226] = xx[0];
  xx[227] = xx[0];
  xx[228] = xx[0];
  xx[229] = xx[0];
  xx[230] = xx[92];
  xx[231] = xx[0];
  xx[232] = xx[0];
  xx[233] = xx[0];
  xx[234] = xx[0];
  xx[235] = xx[0];
  xx[236] = xx[0];
  xx[237] = xx[128];
  ii[1] = factorSymmetricPosDef(xx + 202, 6, xx + 126);
  if (ii[1] != 0) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:mechanism:mechanism:degenerateMassImplicit6Dof",
      "An implicit 6-DOF joint is attached to a degenerate mass distribution.",
      neDiagMgr);
  }

  xx[29] = bb[1] ? xx[63] : xx[0];
  xx[126] = xx[0];
  xx[127] = xx[0];
  xx[128] = xx[0];
  xx[129] = xx[29];
  xx[130] = xx[0];
  xx[131] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 126, 6, 1, xx + 132, xx + 141);
  xx[78] = - xx[7];
  xx[79] = - xx[11];
  xx[80] = - xx[25];
  xx[81] = - xx[26];
  pm_math_Quaternion_inverseCompose_ra(xx + 78, xx + 65, xx + 91);
  xx[65] = - xx[91];
  xx[66] = - xx[92];
  xx[67] = - xx[93];
  xx[68] = - xx[94];
  xx[7] = bb[1] ? xx[32] : xx[0];
  xx[11] = xx[15] * xx[7];
  xx[25] = xx[15] * xx[11];
  xx[26] = xx[14] * xx[7];
  xx[69] = xx[26] * xx[14];
  xx[70] = (xx[25] + xx[69]) * xx[30];
  xx[71] = xx[17] * xx[26];
  xx[85] = (xx[71] + xx[13] * xx[11]) * xx[30];
  xx[86] = xx[17] * xx[11];
  xx[87] = xx[30] * (xx[86] - xx[13] * xx[26]);
  xx[126] = xx[0];
  xx[127] = xx[0];
  xx[128] = xx[0];
  xx[129] = xx[7] - xx[70];
  xx[130] = xx[85];
  xx[131] = - xx[87];
  solveSymmetricPosDef(xx + 166, xx + 126, 6, 1, xx + 141, xx + 147);
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 144, xx + 91);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 91, xx + 126);
  xx[91] = xx[135] - xx[126];
  xx[92] = xx[136] - xx[127];
  xx[93] = bb[0] ? xx[0] : xx[76];
  xx[94] = xx[76] * xx[63];
  xx[95] = xx[93] - xx[94];
  xx[129] = xx[35] - xx[10];
  xx[130] = xx[52] - xx[2];
  xx[131] = xx[3] - xx[8];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 129, xx + 147);
  xx[129] = xx[147];
  xx[130] = xx[148];
  xx[131] = xx[149] - xx[76];
  pm_math_Vector3_cross_ra(xx + 129, xx + 126, xx + 147);
  pm_math_Vector3_cross_ra(xx + 144, xx + 40, xx + 150);
  xx[153] = xx[141] * xx[6] + xx[142] * xx[39] + xx[143] * xx[62] + xx[145] +
    xx[150];
  xx[154] = xx[141] * xx[27] + xx[142] * xx[24] + xx[143] * xx[31] - xx[144] +
    xx[151];
  xx[155] = xx[141] * xx[38] + xx[142] * xx[59] + xx[143] * xx[5] + xx[152];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 153, xx + 141);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 141, xx + 144);
  xx[2] = bb[0] ? xx[0] : - xx[76];
  xx[3] = xx[2] + xx[94];
  xx[150] = xx[0];
  xx[151] = xx[0];
  xx[152] = xx[0];
  xx[153] = xx[0];
  xx[154] = xx[29];
  xx[155] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 150, 6, 1, xx + 156, xx + 238);
  xx[8] = xx[13] * xx[7];
  xx[10] = xx[30] * (xx[71] - xx[15] * xx[8]);
  xx[35] = xx[13] * xx[8];
  xx[52] = (xx[69] + xx[35]) * xx[30];
  xx[69] = xx[17] * xx[8];
  xx[71] = (xx[69] + xx[15] * xx[26]) * xx[30];
  xx[150] = xx[0];
  xx[151] = xx[0];
  xx[152] = xx[0];
  xx[153] = - xx[10];
  xx[154] = xx[7] - xx[52];
  xx[155] = xx[71];
  solveSymmetricPosDef(xx + 166, xx + 150, 6, 1, xx + 238, xx + 244);
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 241, xx + 141);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 141, xx + 150);
  xx[26] = xx[159] - xx[150];
  xx[94] = xx[160] - xx[151];
  pm_math_Vector3_cross_ra(xx + 129, xx + 150, xx + 141);
  pm_math_Vector3_cross_ra(xx + 241, xx + 40, xx + 153);
  xx[162] = xx[238] * xx[6] + xx[239] * xx[39] + xx[240] * xx[62] + xx[242] +
    xx[153];
  xx[163] = xx[238] * xx[27] + xx[239] * xx[24] + xx[240] * xx[31] - xx[241] +
    xx[154];
  xx[164] = xx[238] * xx[38] + xx[239] * xx[59] + xx[240] * xx[5] + xx[155];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 162, xx + 153);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 153, xx + 162);
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[0];
  xx[241] = xx[0];
  xx[242] = xx[0];
  xx[243] = xx[29];
  solveSymmetricPosDef(xx + 202, xx + 238, 6, 1, xx + 244, xx + 250);
  xx[126] = (xx[86] + xx[8] * xx[14]) * xx[30];
  xx[8] = xx[30] * (xx[69] - xx[11] * xx[14]);
  xx[11] = (xx[35] + xx[25]) * xx[30];
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[0];
  xx[241] = xx[126];
  xx[242] = - xx[8];
  xx[243] = xx[7] - xx[11];
  solveSymmetricPosDef(xx + 166, xx + 238, 6, 1, xx + 250, xx + 256);
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 253, xx + 153);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 153, xx + 238);
  xx[25] = xx[247] - xx[238];
  xx[35] = xx[248] - xx[239];
  pm_math_Vector3_cross_ra(xx + 129, xx + 238, xx + 153);
  pm_math_Vector3_cross_ra(xx + 253, xx + 40, xx + 241);
  xx[255] = xx[250] * xx[6] + xx[251] * xx[39] + xx[252] * xx[62] + xx[254] +
    xx[241];
  xx[256] = xx[250] * xx[27] + xx[251] * xx[24] + xx[252] * xx[31] - xx[253] +
    xx[242];
  xx[257] = xx[250] * xx[38] + xx[251] * xx[59] + xx[252] * xx[5] + xx[243];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 255, xx + 241);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 241, xx + 250);
  xx[253] = xx[28] * xx[29];
  xx[254] = xx[58] * xx[29];
  xx[255] = xx[82] * xx[29];
  xx[256] = xx[0];
  xx[257] = bb[1] ? xx[93] : xx[0];
  xx[258] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 253, 6, 1, xx + 259, xx + 265);
  xx[69] = xx[70] - xx[7];
  xx[241] = xx[69];
  xx[242] = - xx[85];
  xx[243] = xx[87];
  xx[70] = bb[0] ? xx[0] : xx[36];
  xx[86] = bb[1] ? xx[70] : xx[0];
  xx[87] = xx[14] * xx[86];
  xx[127] = xx[13] * xx[86];
  pm_math_Vector3_cross_ra(xx + 40, xx + 241, xx + 253);
  xx[265] = - pm_math_Vector3_dot_ra(xx + 43, xx + 241);
  xx[266] = - pm_math_Vector3_dot_ra(xx + 72, xx + 241);
  xx[267] = - pm_math_Vector3_dot_ra(xx + 138, xx + 241);
  xx[268] = - (xx[30] * (xx[17] * xx[87] - xx[15] * xx[127]) + xx[253] + xx[85]);
  xx[269] = - ((xx[87] * xx[14] + xx[13] * xx[127]) * xx[30] - xx[86] + xx[254]
               + xx[69]);
  xx[270] = (xx[17] * xx[127] + xx[15] * xx[87]) * xx[30] - xx[255];
  solveSymmetricPosDef(xx + 166, xx + 265, 6, 1, xx + 253, xx + 271);
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 256, xx + 85);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 85, xx + 241);
  xx[69] = xx[262] - xx[241];
  xx[85] = xx[263] - xx[242];
  pm_math_Vector3_cross_ra(xx + 129, xx + 241, xx + 265);
  pm_math_Vector3_cross_ra(xx + 256, xx + 40, xx + 268);
  xx[271] = xx[253] * xx[6] + xx[254] * xx[39] + xx[255] * xx[62] + xx[257] +
    xx[268];
  xx[272] = xx[253] * xx[27] + xx[254] * xx[24] + xx[255] * xx[31] - xx[256] +
    xx[269];
  xx[273] = xx[253] * xx[38] + xx[254] * xx[59] + xx[255] * xx[5] + xx[270];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 271, xx + 253);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 253, xx + 256);
  xx[268] = xx[37] * xx[29];
  xx[269] = xx[61] * xx[29];
  xx[270] = xx[83] * xx[29];
  xx[271] = bb[1] ? xx[2] : xx[0];
  xx[272] = xx[0];
  xx[273] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 268, 6, 1, xx + 274, xx + 280);
  xx[86] = xx[52] - xx[7];
  xx[253] = xx[10];
  xx[254] = xx[86];
  xx[255] = - xx[71];
  xx[52] = bb[0] ? xx[0] : - xx[36];
  xx[71] = bb[1] ? xx[52] : xx[0];
  xx[87] = xx[15] * xx[71];
  xx[127] = xx[14] * xx[71];
  pm_math_Vector3_cross_ra(xx + 40, xx + 253, xx + 268);
  xx[280] = - pm_math_Vector3_dot_ra(xx + 43, xx + 253);
  xx[281] = - pm_math_Vector3_dot_ra(xx + 72, xx + 253);
  xx[282] = - pm_math_Vector3_dot_ra(xx + 138, xx + 253);
  xx[283] = xx[86] - ((xx[15] * xx[87] + xx[127] * xx[14]) * xx[30] - xx[71] +
                      xx[268]);
  xx[284] = - (xx[269] - (xx[17] * xx[127] + xx[13] * xx[87]) * xx[30] + xx[10]);
  xx[285] = - (xx[30] * (xx[17] * xx[87] - xx[13] * xx[127]) + xx[270]);
  solveSymmetricPosDef(xx + 166, xx + 280, 6, 1, xx + 268, xx + 286);
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 271, xx + 13);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 13, xx + 253);
  xx[10] = xx[277] - xx[253];
  xx[13] = xx[278] - xx[254];
  pm_math_Vector3_cross_ra(xx + 129, xx + 253, xx + 280);
  pm_math_Vector3_cross_ra(xx + 271, xx + 40, xx + 283);
  xx[286] = xx[268] * xx[6] + xx[269] * xx[39] + xx[270] * xx[62] + xx[272] +
    xx[283];
  xx[287] = xx[268] * xx[27] + xx[269] * xx[24] + xx[270] * xx[31] - xx[271] +
    xx[284];
  xx[288] = xx[268] * xx[38] + xx[269] * xx[59] + xx[270] * xx[5] + xx[285];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 286, xx + 268);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 268, xx + 271);
  xx[283] = xx[54] * xx[29];
  xx[284] = xx[77] * xx[29];
  xx[285] = xx[84] * xx[29];
  xx[286] = xx[0];
  xx[287] = xx[0];
  xx[288] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 283, 6, 1, xx + 289, xx + 295);
  xx[268] = - xx[126];
  xx[269] = xx[8];
  xx[270] = xx[11] - xx[7];
  pm_math_Vector3_cross_ra(xx + 40, xx + 268, xx + 283);
  xx[295] = - pm_math_Vector3_dot_ra(xx + 43, xx + 268);
  xx[296] = - pm_math_Vector3_dot_ra(xx + 72, xx + 268);
  xx[297] = - pm_math_Vector3_dot_ra(xx + 138, xx + 268);
  xx[298] = xx[8] - xx[283];
  xx[299] = xx[126] - xx[284];
  xx[300] = - xx[285];
  solveSymmetricPosDef(xx + 166, xx + 295, 6, 1, xx + 283, xx + 301);
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 286, xx + 268);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 268, xx + 295);
  xx[7] = xx[292] - xx[295];
  xx[8] = xx[293] - xx[296];
  pm_math_Vector3_cross_ra(xx + 129, xx + 295, xx + 268);
  pm_math_Vector3_cross_ra(xx + 286, xx + 40, xx + 298);
  xx[301] = xx[283] * xx[6] + xx[284] * xx[39] + xx[285] * xx[62] + xx[287] +
    xx[298];
  xx[302] = xx[283] * xx[27] + xx[284] * xx[24] + xx[285] * xx[31] - xx[286] +
    xx[299];
  xx[303] = xx[283] * xx[38] + xx[284] * xx[59] + xx[285] * xx[5] + xx[300];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 301, xx + 283);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 283, xx + 286);
  xx[298] = xx[91] * xx[63];
  xx[299] = xx[92] * xx[63];
  xx[300] = (xx[137] - xx[128]) * xx[63];
  xx[301] = xx[95] * xx[92] + (xx[132] * xx[28] + xx[133] * xx[58] + xx[134] *
    xx[82] + xx[76] * xx[136] - (xx[147] + xx[144])) * xx[63];
  xx[302] = xx[91] * xx[3] + (xx[132] * xx[37] + xx[133] * xx[61] + xx[134] *
    xx[83] - xx[76] * xx[135] - (xx[148] + xx[145])) * xx[63];
  xx[303] = (xx[132] * xx[54] + xx[133] * xx[77] + xx[134] * xx[84] - (xx[149] +
              xx[146])) * xx[63];
  xx[304] = xx[26] * xx[63];
  xx[305] = xx[94] * xx[63];
  xx[306] = (xx[161] - xx[152]) * xx[63];
  xx[307] = xx[95] * xx[94] + (xx[156] * xx[28] + xx[157] * xx[58] + xx[158] *
    xx[82] + xx[76] * xx[160] - (xx[141] + xx[162])) * xx[63];
  xx[308] = xx[26] * xx[3] + (xx[156] * xx[37] + xx[157] * xx[61] + xx[158] *
    xx[83] - xx[76] * xx[159] - (xx[142] + xx[163])) * xx[63];
  xx[309] = (xx[156] * xx[54] + xx[157] * xx[77] + xx[158] * xx[84] - (xx[143] +
              xx[164])) * xx[63];
  xx[310] = xx[25] * xx[63];
  xx[311] = xx[35] * xx[63];
  xx[312] = (xx[249] - xx[240]) * xx[63];
  xx[313] = xx[95] * xx[35] + (xx[244] * xx[28] + xx[245] * xx[58] + xx[246] *
    xx[82] + xx[76] * xx[248] - (xx[153] + xx[250])) * xx[63];
  xx[314] = xx[25] * xx[3] + (xx[244] * xx[37] + xx[245] * xx[61] + xx[246] *
    xx[83] - xx[76] * xx[247] - (xx[154] + xx[251])) * xx[63];
  xx[315] = (xx[244] * xx[54] + xx[245] * xx[77] + xx[246] * xx[84] - (xx[155] +
              xx[252])) * xx[63];
  xx[316] = xx[69] * xx[63];
  xx[317] = xx[85] * xx[63];
  xx[318] = (xx[264] - xx[243]) * xx[63];
  xx[319] = xx[95] * xx[85] + (xx[259] * xx[28] + xx[260] * xx[58] + xx[261] *
    xx[82] + xx[76] * xx[263] - (xx[265] + xx[256])) * xx[63];
  xx[320] = xx[69] * xx[3] + (xx[259] * xx[37] + xx[260] * xx[61] + xx[261] *
    xx[83] - xx[76] * xx[262] - (xx[266] + xx[257])) * xx[63];
  xx[321] = (xx[259] * xx[54] + xx[260] * xx[77] + xx[261] * xx[84] - (xx[267] +
              xx[258])) * xx[63];
  xx[322] = xx[10] * xx[63];
  xx[323] = xx[13] * xx[63];
  xx[324] = (xx[279] - xx[255]) * xx[63];
  xx[325] = xx[95] * xx[13] + (xx[274] * xx[28] + xx[275] * xx[58] + xx[276] *
    xx[82] + xx[76] * xx[278] - (xx[280] + xx[271])) * xx[63];
  xx[326] = xx[10] * xx[3] + (xx[274] * xx[37] + xx[275] * xx[61] + xx[276] *
    xx[83] - xx[76] * xx[277] - (xx[281] + xx[272])) * xx[63];
  xx[327] = (xx[274] * xx[54] + xx[275] * xx[77] + xx[276] * xx[84] - (xx[282] +
              xx[273])) * xx[63];
  xx[328] = xx[7] * xx[63];
  xx[329] = xx[8] * xx[63];
  xx[330] = (xx[294] - xx[297]) * xx[63];
  xx[331] = xx[95] * xx[8] + (xx[289] * xx[28] + xx[290] * xx[58] + xx[291] *
    xx[82] + xx[76] * xx[293] - (xx[268] + xx[286])) * xx[63];
  xx[332] = xx[7] * xx[3] + (xx[289] * xx[37] + xx[290] * xx[61] + xx[291] * xx
    [83] - xx[76] * xx[292] - (xx[269] + xx[287])) * xx[63];
  xx[333] = (xx[289] * xx[54] + xx[290] * xx[77] + xx[291] * xx[84] - (xx[270] +
              xx[288])) * xx[63];
  reduceMatrix(6, 6, ii[2], ii + 9, xx + 298);
  transposeColMajor(6, ii[2], xx + 298, xx + 238);
  reduceMatrix(ii[2], 6, ii[2], ii + 9, xx + 238);
  transposeColMajor(ii[2], ii[2], xx + 238, xx + 274);
  svd(ii[2], ii[2], xx + 274, TRUE, xx + 24, xx + 238, xx + 310, xx + 346);
  bb[0] = ii[2] > 0 ? xx[24] > 9.999999999999999e-21 : 1;
  xx[5] = state[10];
  xx[6] = state[11];
  xx[7] = state[12];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 5, xx + 13);
  xx[8] = (xx[18] - xx[30] * xx[19] * xx[19]) * state[15];
  xx[85] = xx[13] + xx[8];
  xx[86] = xx[14] + state[16];
  xx[87] = xx[15] + xx[30] * xx[16] * xx[19] * state[15];
  pm_math_Quaternion_xform_ra(xx + 65, xx + 85, xx + 13);
  xx[10] = state[27] - xx[13];
  xx[11] = state[28] - xx[14];
  xx[85] = state[24];
  xx[86] = state[25];
  xx[87] = state[26];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 85, xx + 126);
  pm_math_Vector3_cross_ra(xx + 129, xx + 13, xx + 78);
  pm_math_Vector3_cross_ra(xx + 5, xx + 40, xx + 85);
  xx[5] = state[7];
  xx[6] = state[8];
  xx[7] = state[9];
  pm_math_Quaternion_inverseXform_ra(xx + 46, xx + 5, xx + 129);
  xx[5] = xx[85] + xx[129] + state[11];
  xx[6] = xx[86] + xx[130] - state[10];
  xx[7] = xx[87] + xx[131];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 5, xx + 46);
  xx[5] = xx[46] - xx[50] * state[16];
  xx[6] = xx[47] + xx[50] * xx[8];
  xx[7] = xx[48];
  pm_math_Quaternion_xform_ra(xx + 65, xx + 5, xx + 46);
  xx[129] = - (xx[10] * xx[63]);
  xx[130] = - (xx[11] * xx[63]);
  xx[131] = - ((state[29] - xx[15]) * xx[63]);
  xx[132] = - (xx[95] * xx[11] + (xx[126] + xx[76] * state[28] - (xx[78] + xx[46]))
               * xx[63]);
  xx[133] = - (xx[3] * xx[10] + (xx[127] - xx[76] * state[27] - (xx[79] + xx[47]))
               * xx[63]);
  xx[134] = - ((xx[128] - (xx[80] + xx[48])) * xx[63]);
  reduceMatrix(1, 6, ii[2], ii + 9, xx + 129);
  ii[1] = svdSolveFromFactorization(ii[2], ii[2], xx + 24, xx + 238, xx + 310,
    xx + 129, 1.0e-12, xx + 141, xx + 147);
  xx[24] = bb[0] ? xx[141] : xx[0];
  xx[25] = bb[0] ? xx[142] : xx[0];
  xx[26] = bb[0] ? xx[143] : xx[0];
  xx[27] = bb[0] ? xx[144] : xx[0];
  xx[28] = bb[0] ? xx[145] : xx[0];
  xx[29] = bb[0] ? xx[146] : xx[0];
  expandVector(6, ii[2], ii + 9, xx + 24);
  xx[5] = - (xx[27] * xx[32]);
  xx[6] = - (xx[28] * xx[32]);
  xx[7] = - (xx[29] * xx[32]);
  pm_math_Quaternion_xform_ra(xx + 20, xx + 5, xx + 13);
  xx[5] = - (xx[24] * xx[32] + xx[28] * xx[52]);
  xx[6] = - (xx[25] * xx[32] + xx[27] * xx[70]);
  xx[7] = - (xx[26] * xx[32]);
  pm_math_Quaternion_xform_ra(xx + 20, xx + 5, xx + 46);
  pm_math_Vector3_cross_ra(xx + 40, xx + 13, xx + 5);
  xx[65] = - pm_math_Vector3_dot_ra(xx + 43, xx + 13);
  xx[66] = - pm_math_Vector3_dot_ra(xx + 72, xx + 13);
  xx[67] = - pm_math_Vector3_dot_ra(xx + 138, xx + 13);
  xx[68] = xx[14] - (xx[46] + xx[5]);
  xx[69] = - (xx[47] + xx[6] + xx[13]);
  xx[70] = - (xx[48] + xx[7]);
  solveSymmetricPosDef(xx + 166, xx + 65, 6, 1, xx + 38, xx + 44);
  xx[0] = xx[103] + xx[38];
  xx[3] = xx[104] + xx[39];
  xx[5] = xx[105] + xx[40];
  xx[6] = xx[106] + xx[41];
  xx[7] = xx[107] + xx[42];
  xx[8] = xx[108] + xx[43];
  xx[13] = - (xx[27] * xx[63]);
  xx[14] = - (xx[28] * xx[63]);
  xx[15] = - (xx[29] * xx[63]);
  xx[38] = - pm_math_Vector3_dot_ra(xx + 55, xx + 13);
  xx[39] = - pm_math_Vector3_dot_ra(xx + 88, xx + 13);
  xx[40] = - pm_math_Vector3_dot_ra(xx + 82, xx + 13);
  xx[41] = xx[24] * xx[63] + xx[28] * xx[2];
  xx[42] = xx[25] * xx[63] + xx[27] * xx[93];
  xx[43] = xx[26] * xx[63];
  solveSymmetricPosDef(xx + 202, xx + 38, 6, 1, xx + 19, xx + 44);
  xx[2] = xx[120] + xx[19];
  xx[10] = xx[121] + xx[20];
  xx[11] = xx[122] + xx[21];
  xx[13] = xx[123] + xx[22];
  xx[14] = xx[124] + xx[23];
  xx[15] = xx[125] + xx[24];
  xx[120] = xx[96];
  xx[121] = xx[97];
  xx[122] = xx[98];
  xx[123] = xx[99];
  xx[124] = xx[100];
  xx[125] = xx[101];
  xx[126] = xx[102];
  xx[127] = xx[0];
  xx[128] = xx[3];
  xx[129] = xx[5];
  xx[130] = xx[6];
  xx[131] = xx[7];
  xx[132] = xx[8];
  xx[133] = xx[109];
  xx[134] = xx[110];
  xx[135] = xx[111];
  xx[136] = xx[112];
  xx[137] = xx[113];
  xx[138] = xx[114];
  xx[139] = xx[115];
  xx[140] = xx[116];
  xx[141] = xx[117];
  xx[142] = xx[118];
  xx[143] = xx[119];
  xx[144] = xx[2];
  xx[145] = xx[10];
  xx[146] = xx[11];
  xx[147] = xx[13];
  xx[148] = xx[14];
  xx[149] = xx[15];
  xx[19] = - xx[117];
  xx[20] = - xx[118];
  xx[21] = - xx[119];
  xx[16] = xx[14] * xx[118];
  xx[17] = xx[15] * xx[119];
  xx[22] = xx[16] + xx[17];
  xx[23] = xx[14] * xx[117];
  xx[24] = xx[15] * xx[117];
  xx[25] = xx[22];
  xx[26] = - xx[23];
  xx[27] = - xx[24];
  pm_math_Vector3_cross_ra(xx + 19, xx + 25, xx + 38);
  xx[25] = xx[30] * (xx[38] - xx[22] * xx[116]);
  xx[26] = xx[15] + (xx[116] * xx[23] + xx[39]) * xx[30];
  xx[27] = (xx[116] * xx[24] + xx[40]) * xx[30] - xx[14];
  xx[22] = - xx[100];
  xx[23] = - xx[101];
  xx[24] = - xx[102];
  xx[38] = - xx[99];
  xx[39] = xx[22];
  xx[40] = xx[23];
  xx[41] = xx[24];
  xx[28] = xx[12] * xx[109];
  xx[29] = cos(xx[28]);
  xx[31] = xx[12] * xx[110];
  xx[12] = cos(xx[31]);
  xx[32] = xx[29] * xx[12];
  xx[35] = sin(xx[28]);
  xx[28] = xx[12] * xx[35];
  xx[12] = - xx[28];
  xx[42] = sin(xx[31]);
  xx[31] = xx[29] * xx[42];
  xx[29] = - xx[31];
  xx[43] = xx[35] * xx[42];
  xx[35] = - xx[43];
  xx[44] = - xx[32];
  xx[45] = xx[12];
  xx[46] = xx[29];
  xx[47] = xx[35];
  pm_math_Quaternion_compose_ra(xx + 38, xx + 44, xx + 54);
  xx[42] = xx[55] * xx[56];
  xx[48] = xx[54] * xx[57];
  xx[49] = xx[54] * xx[54];
  xx[52] = xx[56] * xx[57];
  xx[58] = xx[54] * xx[55];
  xx[61] = xx[30] * (xx[42] - xx[48]);
  xx[62] = (xx[49] + xx[56] * xx[56]) * xx[30] - xx[18];
  xx[63] = (xx[52] + xx[58]) * xx[30];
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 6, xx + 65);
  xx[44] = xx[111] * (xx[18] - (xx[31] * xx[31] + xx[43] * xx[43]) * xx[30]);
  xx[45] = xx[65] + xx[44];
  xx[46] = xx[45] * xx[56];
  xx[47] = xx[67] + xx[30] * (xx[32] * xx[31] + xx[28] * xx[43]) * xx[111];
  xx[59] = xx[47] * xx[57];
  xx[68] = xx[45] * xx[55];
  xx[69] = xx[59] + xx[68];
  xx[70] = xx[47] * xx[56];
  xx[71] = xx[46];
  xx[72] = - xx[69];
  xx[73] = xx[70];
  pm_math_Vector3_cross_ra(xx + 55, xx + 71, xx + 78);
  xx[71] = (xx[54] * xx[46] + xx[78]) * xx[30] - xx[47];
  xx[72] = xx[30] * (xx[79] - xx[69] * xx[54]);
  xx[73] = xx[45] + (xx[54] * xx[70] + xx[80]) * xx[30];
  xx[46] = xx[116] * xx[116];
  xx[78] = (xx[46] + xx[60]) * xx[30] - xx[18];
  xx[79] = (xx[33] + xx[34]) * xx[30];
  xx[80] = xx[82];
  xx[33] = xx[13] * xx[118];
  xx[34] = xx[13] * xx[117];
  xx[60] = xx[17] + xx[34];
  xx[17] = xx[15] * xx[118];
  xx[81] = - xx[33];
  xx[82] = xx[60];
  xx[83] = - xx[17];
  pm_math_Vector3_cross_ra(xx + 19, xx + 81, xx + 84);
  xx[81] = (xx[116] * xx[33] + xx[84]) * xx[30] - xx[15];
  xx[82] = xx[30] * (xx[85] - xx[60] * xx[116]);
  xx[83] = xx[13] + (xx[116] * xx[17] + xx[86]) * xx[30];
  xx[15] = xx[55] * xx[57];
  xx[17] = xx[54] * xx[56];
  xx[84] = (xx[15] + xx[17]) * xx[30];
  xx[85] = xx[30] * (xx[52] - xx[58]);
  xx[86] = (xx[49] + xx[57] * xx[57]) * xx[30] - xx[18];
  xx[33] = xx[30] * xx[111] * (xx[31] * xx[28] - xx[32] * xx[43]) + xx[112];
  xx[52] = xx[66] + xx[33];
  xx[58] = xx[45] * xx[57];
  xx[60] = xx[52] * xx[57];
  xx[65] = xx[52] * xx[56];
  xx[66] = xx[68] + xx[65];
  xx[67] = xx[58];
  xx[68] = xx[60];
  xx[69] = - xx[66];
  pm_math_Vector3_cross_ra(xx + 55, xx + 67, xx + 87);
  xx[67] = xx[52] + (xx[54] * xx[58] + xx[87]) * xx[30];
  xx[68] = (xx[54] * xx[60] + xx[88]) * xx[30] - xx[45];
  xx[69] = xx[30] * (xx[89] - xx[66] * xx[54]);
  xx[87] = xx[37];
  xx[88] = (xx[46] + xx[1]) * xx[30] - xx[18];
  xx[89] = (xx[64] + xx[75]) * xx[30];
  xx[1] = xx[13] * xx[119];
  xx[37] = xx[14] * xx[119];
  xx[58] = xx[34] + xx[16];
  xx[90] = - xx[1];
  xx[91] = - xx[37];
  xx[92] = xx[58];
  pm_math_Vector3_cross_ra(xx + 19, xx + 90, xx + 93);
  xx[90] = xx[14] + (xx[116] * xx[1] + xx[93]) * xx[30];
  xx[91] = (xx[116] * xx[37] + xx[94]) * xx[30] - xx[13];
  xx[92] = xx[30] * (xx[95] - xx[58] * xx[116]);
  xx[93] = (xx[49] + xx[55] * xx[55]) * xx[30] - xx[18];
  xx[94] = (xx[42] + xx[48]) * xx[30];
  xx[95] = xx[30] * (xx[15] - xx[17]);
  xx[1] = xx[65] + xx[59];
  xx[15] = xx[52] * xx[55];
  xx[16] = xx[47] * xx[55];
  xx[58] = - xx[1];
  xx[59] = xx[15];
  xx[60] = xx[16];
  pm_math_Vector3_cross_ra(xx + 55, xx + 58, xx + 64);
  xx[58] = xx[30] * (xx[64] - xx[1] * xx[54]);
  xx[59] = xx[47] + (xx[54] * xx[15] + xx[65]) * xx[30];
  xx[60] = (xx[54] * xx[16] + xx[66]) * xx[30] - xx[52];
  xx[15] = (xx[53] + xx[51]) * xx[30];
  xx[16] = xx[77];
  xx[17] = (xx[46] + xx[9]) * xx[30] - xx[18];
  xx[1] = xx[14] * xx[76];
  xx[9] = xx[13] * xx[76];
  xx[13] = xx[119] * xx[9];
  xx[14] = xx[119] * xx[1];
  xx[18] = xx[117] * xx[9] + xx[118] * xx[1];
  xx[46] = - xx[13];
  xx[47] = - xx[14];
  xx[48] = xx[18];
  pm_math_Vector3_cross_ra(xx + 19, xx + 46, xx + 64);
  xx[19] = xx[45] * xx[36];
  xx[20] = xx[57] * xx[19];
  xx[21] = xx[52] * xx[36];
  xx[34] = xx[57] * xx[21];
  xx[36] = xx[55] * xx[19] + xx[56] * xx[21];
  xx[45] = - xx[20];
  xx[46] = - xx[34];
  xx[47] = xx[36];
  pm_math_Vector3_cross_ra(xx + 55, xx + 45, xx + 51);
  xx[37] = xx[6] * xx[102];
  xx[42] = xx[7] * xx[102];
  xx[45] = xx[6] * xx[100] + xx[7] * xx[101];
  xx[46] = - xx[37];
  xx[47] = - xx[42];
  xx[48] = xx[45];
  pm_math_Vector3_cross_ra(xx + 22, xx + 46, xx + 55);
  xx[22] = xx[12];
  xx[23] = xx[29];
  xx[24] = xx[35];
  xx[12] = xx[50] * xx[44];
  xx[29] = xx[12] * xx[43];
  xx[35] = xx[33] * xx[50];
  xx[33] = xx[35] * xx[43];
  xx[44] = xx[12] * xx[28] + xx[35] * xx[31];
  xx[46] = xx[29];
  xx[47] = xx[33];
  xx[48] = - xx[44];
  pm_math_Vector3_cross_ra(xx + 22, xx + 46, xx + 74);
  xx[22] = xx[50] * xx[31];
  xx[23] = xx[50] * xx[28];
  xx[46] = - ((xx[22] * xx[32] + xx[23] * xx[43]) * xx[30]);
  xx[47] = xx[30] * (xx[23] * xx[32] - xx[22] * xx[43]);
  xx[48] = (xx[23] * xx[28] + xx[22] * xx[31]) * xx[30] - 1.045;
  pm_math_Vector3_cross_ra(xx + 6, xx + 46, xx + 22);
  xx[46] = (xx[74] - xx[32] * xx[29]) * xx[30] - xx[35] + xx[22];
  xx[47] = (xx[75] - xx[32] * xx[33]) * xx[30] + xx[12] + xx[23];
  xx[48] = xx[30] * (xx[76] + xx[44] * xx[32]) + xx[24];
  pm_math_Quaternion_xform_ra(xx + 38, xx + 46, xx + 22);
  xx[100] = pm_math_Vector3_dot_ra(xx + 25, xx + 61) + pm_math_Vector3_dot_ra(xx
    + 71, xx + 78);
  xx[101] = pm_math_Vector3_dot_ra(xx + 81, xx + 84) + pm_math_Vector3_dot_ra(xx
    + 67, xx + 87);
  xx[102] = pm_math_Vector3_dot_ra(xx + 90, xx + 93) + pm_math_Vector3_dot_ra(xx
    + 58, xx + 15);
  xx[103] = xx[1] + (xx[116] * xx[13] + xx[64]) * xx[30] + xx[2] - ((xx[51] -
    xx[54] * xx[20]) * xx[30] - xx[21] + xx[7] + (xx[99] * xx[37] + xx[55]) *
    xx[30] + xx[0] + xx[22]);
  xx[104] = (xx[116] * xx[14] + xx[65]) * xx[30] - xx[9] + xx[10] - ((xx[52] -
    xx[54] * xx[34]) * xx[30] + xx[19] + (xx[99] * xx[42] + xx[56]) * xx[30] -
    xx[6] + xx[3] + xx[23]);
  xx[105] = xx[30] * (xx[66] - xx[18] * xx[116]) + xx[11] - (xx[30] * (xx[53] +
    xx[36] * xx[54]) + xx[30] * (xx[57] - xx[45] * xx[99]) + xx[5] + xx[24]);
  zeroMajor(1, 6, ii + 0, xx + 100);
  xx[5] = fabs(xx[100]);
  xx[6] = fabs(xx[101]);
  xx[7] = fabs(xx[102]);
  xx[8] = fabs(xx[103]);
  xx[9] = fabs(xx[104]);
  xx[10] = fabs(xx[105]);
  ii[0] = 5;

  {
    int ll;
    for (ll = 6; ll < 11; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 5;
  xx[0] = xx[5 + (ii[0])];
  if (xx[0] > xx[4]) {
    const char *msgs[] = {
      "Impulses to satisfy the velocity components of the kinematic constraint equations of the joints transitioning from disengaged to normal mode cannot be computed. This is either because the mechanism has insufficient non-motion input degrees-of-freedom available for the required instantaneous velocity change, or, after the position components of the kinematic constraint equations of the mechanism were satisfied, the velocity of the mechanism is incompatible with the new positio",
      "n of the mechanism. Consider increasing the number of non-motion actuated degrees-of-freedom, or reducing the position errors in the transitioning joints before attempting the transition."
    };

    char msg[668];
    return sm_ssci_recordRunTimeErrorSegmented(
      "physmod:sm:core:compiler:mechanism:mechanism:jointDisToNormVelocitySatisfactionFailure",
      2, msgs, msg, neDiagMgr);
  }

  state[0] = xx[120];
  state[1] = xx[121];
  state[2] = xx[122];
  state[3] = xx[123];
  state[4] = xx[124];
  state[5] = xx[125];
  state[6] = xx[126];
  state[7] = xx[127];
  state[8] = xx[128];
  state[9] = xx[129];
  state[10] = xx[130];
  state[11] = xx[131];
  state[12] = xx[132];
  state[13] = xx[133];
  state[14] = xx[134];
  state[15] = xx[135];
  state[16] = xx[136];
  state[17] = xx[137];
  state[18] = xx[138];
  state[19] = xx[139];
  state[20] = xx[140];
  state[21] = xx[141];
  state[22] = xx[142];
  state[23] = xx[143];
  state[24] = xx[144];
  state[25] = xx[145];
  state[26] = xx[146];
  state[27] = xx[147];
  state[28] = xx[148];
  state[29] = xx[149];
  return NULL;
}

void simulation_b048d748_1_onModeChangedCutJoints(const void *mech, const int
  *prevModeVector, const int *modeVector, double *state)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  (void) state;
}
