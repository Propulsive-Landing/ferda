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
  xx[0] = 2.0;
  xx[1] = state[20] * state[20];
  xx[2] = 1.0;
  xx[3] = state[21] * state[22];
  xx[4] = state[20] * state[23];
  xx[5] = state[21] * state[23];
  xx[6] = state[20] * state[22];
  xx[7] = xx[0] * (xx[1] + state[21] * state[21]) - xx[2];
  xx[8] = xx[0] * (xx[3] + xx[4]);
  xx[9] = xx[0] * (xx[5] - xx[6]);
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
  xx[27] = xx[0] * (xx[14] - xx[19]);
  xx[28] = xx[0] * (xx[20] + xx[25] * xx[25]) - xx[2];
  xx[29] = xx[0] * (xx[21] + xx[22]);
  xx[30] = state[22] * state[23];
  xx[31] = state[20] * state[21];
  xx[32] = xx[0] * (xx[3] - xx[4]);
  xx[33] = xx[0] * (xx[1] + state[22] * state[22]) - xx[2];
  xx[34] = xx[0] * (xx[30] + xx[31]);
  xx[3] = xx[24] * xx[26];
  xx[4] = xx[23] * xx[25];
  xx[35] = xx[0] * (xx[3] + xx[4]);
  xx[36] = xx[0] * (xx[21] - xx[22]);
  xx[37] = xx[0] * (xx[20] + xx[26] * xx[26]) - xx[2];
  xx[38] = xx[0] * (xx[5] + xx[6]);
  xx[39] = xx[0] * (xx[30] - xx[31]);
  xx[40] = xx[0] * (xx[1] + state[23] * state[23]) - xx[2];
  xx[41] = xx[0] * (xx[20] + xx[24] * xx[24]) - xx[2];
  xx[42] = xx[0] * (xx[14] + xx[19]);
  xx[43] = xx[0] * (xx[3] - xx[4]);
  xx[1] = 0.0225;
  xx[2] = xx[1] * state[22];
  xx[3] = xx[1] * state[21];
  xx[1] = 0.04499999999999999;
  xx[4] = xx[1] * xx[17];
  xx[5] = xx[1] * xx[15];
  xx[19] = - (xx[0] * (xx[4] * xx[18] + xx[5] * xx[16]));
  xx[20] = xx[0] * (xx[5] * xx[18] - xx[4] * xx[16]);
  xx[21] = xx[0] * (xx[5] * xx[15] + xx[4] * xx[17]) - 1.045;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 19, xx + 4);
  xx[1] = 0.04500000000000001;
  xx[10] = xx[1] * xx[25];
  xx[11] = xx[1] * xx[24];
  xx[12] = pm_math_Vector3_dot_ra(xx + 7, xx + 27);
  xx[13] = pm_math_Vector3_dot_ra(xx + 32, xx + 35);
  xx[14] = pm_math_Vector3_dot_ra(xx + 38, xx + 41);
  xx[15] = xx[0] * (xx[2] * state[20] + xx[3] * state[23]) + state[17] - (xx[4]
    + state[0] + xx[0] * (state[3] * state[5] + state[4] * state[6]) - xx[0] *
    (xx[10] * xx[23] + xx[11] * xx[26]));
  xx[16] = xx[0] * (xx[2] * state[23] - xx[3] * state[20]) + state[18] - (xx[0] *
    (xx[11] * xx[23] - xx[10] * xx[26]) + xx[5] + state[1] - xx[0] * (state[3] *
    state[4] - state[5] * state[6]));
  xx[17] = state[19] - xx[0] * (xx[3] * state[21] + xx[2] * state[22]) - (xx[6]
    + state[2] - xx[0] * (state[4] * state[4] + state[5] * state[5]) + xx[0] *
    (xx[11] * xx[24] + xx[10] * xx[25])) - 0.9325;
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
  xx[9] = xx[0] * (xx[12] - state[20] * xx[6]);
  xx[10] = state[29] + xx[0] * (xx[7] * state[20] + xx[13]);
  xx[11] = xx[0] * (xx[8] * state[20] + xx[14]) - state[28];
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
  xx[38] = xx[0] * (xx[33] + xx[30] * xx[30]) - xx[34];
  xx[39] = xx[0] * (xx[35] + xx[36]);
  xx[40] = state[10];
  xx[41] = state[11];
  xx[42] = state[12];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 40, xx + 43);
  xx[24] = (xx[34] - xx[0] * (xx[19] * xx[19] + xx[23] * xx[23])) * state[15];
  xx[25] = xx[43] + xx[24];
  xx[26] = xx[25] * xx[30];
  xx[27] = xx[45] + xx[0] * state[15] * (xx[20] * xx[19] + xx[17] * xx[23]);
  xx[46] = xx[27] * xx[31];
  xx[47] = xx[25] * xx[29];
  xx[48] = xx[46] + xx[47];
  xx[49] = xx[27] * xx[30];
  xx[50] = xx[26];
  xx[51] = - xx[48];
  xx[52] = xx[49];
  pm_math_Vector3_cross_ra(xx + 29, xx + 50, xx + 53);
  xx[50] = xx[0] * (xx[26] * xx[28] + xx[53]) - xx[27];
  xx[51] = xx[0] * (xx[54] - xx[48] * xx[28]);
  xx[52] = xx[25] + xx[0] * (xx[49] * xx[28] + xx[55]);
  xx[26] = state[20] * state[20];
  xx[48] = state[21] * state[22];
  xx[49] = state[20] * state[23];
  xx[53] = state[21] * state[23];
  xx[54] = state[20] * state[22];
  xx[55] = xx[0] * (xx[26] + state[21] * state[21]) - xx[34];
  xx[56] = xx[0] * (xx[48] + xx[49]);
  xx[57] = xx[0] * (xx[53] - xx[54]);
  xx[58] = state[22] * state[27];
  xx[59] = state[21] * state[27];
  xx[60] = xx[5] + xx[59];
  xx[5] = state[22] * state[29];
  xx[61] = - xx[58];
  xx[62] = xx[60];
  xx[63] = - xx[5];
  pm_math_Vector3_cross_ra(xx + 1, xx + 61, xx + 64);
  xx[61] = xx[0] * (xx[58] * state[20] + xx[64]) - state[29];
  xx[62] = xx[0] * (xx[65] - state[20] * xx[60]);
  xx[63] = state[27] + xx[0] * (xx[5] * state[20] + xx[66]);
  xx[5] = xx[29] * xx[31];
  xx[58] = xx[28] * xx[30];
  xx[64] = xx[0] * (xx[5] + xx[58]);
  xx[65] = xx[0] * (xx[35] - xx[36]);
  xx[66] = xx[0] * (xx[33] + xx[31] * xx[31]) - xx[34];
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
  xx[67] = xx[36] + xx[0] * (xx[43] * xx[28] + xx[70]);
  xx[68] = xx[0] * (xx[44] * xx[28] + xx[71]) - xx[25];
  xx[69] = xx[0] * (xx[72] - xx[60] * xx[28]);
  xx[43] = state[22] * state[23];
  xx[44] = state[20] * state[21];
  xx[70] = xx[0] * (xx[48] - xx[49]);
  xx[71] = xx[0] * (xx[26] + state[22] * state[22]) - xx[34];
  xx[72] = xx[0] * (xx[43] + xx[44]);
  xx[47] = state[23] * state[27];
  xx[48] = state[23] * state[28];
  xx[49] = xx[59] + xx[4];
  xx[73] = - xx[47];
  xx[74] = - xx[48];
  xx[75] = xx[49];
  pm_math_Vector3_cross_ra(xx + 1, xx + 73, xx + 76);
  xx[73] = state[28] + xx[0] * (xx[47] * state[20] + xx[76]);
  xx[74] = xx[0] * (xx[48] * state[20] + xx[77]) - state[27];
  xx[75] = xx[0] * (xx[78] - state[20] * xx[49]);
  xx[47] = xx[0] * (xx[33] + xx[29] * xx[29]) - xx[34];
  xx[48] = xx[0] * (xx[22] + xx[32]);
  xx[49] = xx[0] * (xx[5] - xx[58]);
  xx[4] = xx[45] + xx[46];
  xx[5] = xx[36] * xx[29];
  xx[22] = xx[27] * xx[29];
  xx[58] = - xx[4];
  xx[59] = xx[5];
  xx[60] = xx[22];
  pm_math_Vector3_cross_ra(xx + 29, xx + 58, xx + 76);
  xx[58] = xx[0] * (xx[76] - xx[4] * xx[28]);
  xx[59] = xx[27] + xx[0] * (xx[5] * xx[28] + xx[77]);
  xx[60] = xx[0] * (xx[22] * xx[28] + xx[78]) - xx[36];
  xx[76] = xx[0] * (xx[53] + xx[54]);
  xx[77] = xx[0] * (xx[43] - xx[44]);
  xx[78] = xx[0] * (xx[26] + state[23] * state[23]) - xx[34];
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
  xx[2] = xx[1] * xx[25];
  xx[3] = xx[2] * xx[31];
  xx[25] = xx[1] * xx[36];
  xx[1] = xx[25] * xx[31];
  xx[32] = xx[2] * xx[29] + xx[25] * xx[30];
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
  xx[24] = xx[16] * xx[35];
  xx[33] = xx[24] * xx[23];
  xx[34] = xx[18] * xx[17] + xx[24] * xx[19];
  xx[79] = xx[21];
  xx[80] = xx[33];
  xx[81] = - xx[34];
  pm_math_Vector3_cross_ra(xx + 6, xx + 79, xx + 88);
  xx[6] = xx[16] * xx[19];
  xx[7] = xx[16] * xx[17];
  xx[79] = - (xx[0] * (xx[6] * xx[20] + xx[7] * xx[23]));
  xx[80] = xx[0] * (xx[7] * xx[20] - xx[6] * xx[23]);
  xx[81] = xx[0] * (xx[7] * xx[17] + xx[6] * xx[19]) - 1.045;
  pm_math_Vector3_cross_ra(xx + 40, xx + 79, xx + 6);
  xx[40] = xx[0] * (xx[88] - xx[21] * xx[20]) - xx[24] + xx[6];
  xx[41] = xx[0] * (xx[89] - xx[33] * xx[20]) + xx[18] + xx[7];
  xx[42] = xx[0] * (xx[90] + xx[20] * xx[34]) + xx[8];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 40, xx + 6);
  xx[12] = pm_math_Vector3_dot_ra(xx + 9, xx + 37) + pm_math_Vector3_dot_ra(xx +
    50, xx + 55);
  xx[13] = pm_math_Vector3_dot_ra(xx + 61, xx + 64) + pm_math_Vector3_dot_ra(xx
    + 67, xx + 70);
  xx[14] = pm_math_Vector3_dot_ra(xx + 73, xx + 47) + pm_math_Vector3_dot_ra(xx
    + 58, xx + 76);
  xx[15] = xx[5] + xx[0] * (xx[4] * state[20] + xx[43]) + state[24] - (xx[0] *
    (xx[82] - xx[3] * xx[28]) - xx[25] + state[11] + xx[0] * (xx[29] * state[3]
    + xx[85]) + state[7] + xx[6]);
  xx[16] = xx[0] * (xx[26] * state[20] + xx[44]) - xx[22] + state[25] - (xx[0] *
    (xx[83] - xx[1] * xx[28]) + xx[2] + xx[0] * (xx[30] * state[3] + xx[86]) -
    state[10] + state[8] + xx[7]);
  xx[17] = xx[0] * (xx[45] - state[20] * xx[27]) + state[26] - (xx[0] * (xx[84]
    + xx[32] * xx[28]) + xx[0] * (xx[87] - state[3] * xx[31]) + state[9] + xx[8]);
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
  xx[1] = 2.0;
  xx[2] = 1.0;
  xx[3] = 0.5;
  xx[4] = xx[3] * state[13];
  xx[5] = cos(xx[4]);
  xx[6] = xx[3] * state[14];
  xx[7] = sin(xx[6]);
  xx[8] = xx[5] * xx[7];
  xx[9] = xx[8] * xx[8];
  xx[10] = sin(xx[4]);
  xx[4] = xx[10] * xx[7];
  xx[7] = xx[4] * xx[4];
  xx[11] = xx[2] - xx[1] * (xx[9] + xx[7]);
  xx[12] = sqrt(state[3] * state[3] + state[4] * state[4] + state[5] * state[5]
                + state[6] * state[6]);
  xx[13] = state[3] / xx[12];
  xx[14] = state[4] / xx[12];
  xx[15] = - xx[14];
  xx[16] = state[5] / xx[12];
  xx[17] = - xx[16];
  xx[18] = state[6] / xx[12];
  xx[12] = - xx[18];
  xx[19] = - xx[13];
  xx[20] = xx[15];
  xx[21] = xx[17];
  xx[22] = xx[12];
  xx[23] = cos(xx[6]);
  xx[6] = xx[5] * xx[23];
  xx[5] = xx[23] * xx[10];
  xx[23] = - xx[6];
  xx[24] = - xx[5];
  xx[25] = - xx[8];
  xx[26] = - xx[4];
  pm_math_Quaternion_compose_ra(xx + 19, xx + 23, xx + 27);
  xx[10] = xx[11] * xx[29];
  xx[31] = xx[6] * xx[8];
  xx[32] = xx[5] * xx[4];
  xx[33] = xx[1] * (xx[31] + xx[32]);
  xx[34] = xx[33] * xx[30];
  xx[35] = xx[11] * xx[28];
  xx[36] = xx[34] + xx[35];
  xx[37] = xx[33] * xx[29];
  xx[38] = xx[10];
  xx[39] = - xx[36];
  xx[40] = xx[37];
  pm_math_Vector3_cross_ra(xx + 28, xx + 38, xx + 41);
  xx[38] = xx[1] * (xx[10] * xx[27] + xx[41]) - xx[33];
  xx[39] = xx[1] * (xx[42] - xx[36] * xx[27]);
  xx[40] = xx[11] + xx[1] * (xx[37] * xx[27] + xx[43]);
  xx[10] = sqrt(state[20] * state[20] + state[21] * state[21] + state[22] *
                state[22] + state[23] * state[23]);
  xx[36] = state[20] / xx[10];
  xx[37] = xx[36] * xx[36];
  xx[41] = state[21] / xx[10];
  xx[42] = xx[41] * xx[41];
  xx[43] = state[22] / xx[10];
  xx[44] = xx[41] * xx[43];
  xx[45] = state[23] / xx[10];
  xx[10] = xx[36] * xx[45];
  xx[46] = xx[41] * xx[45];
  xx[47] = xx[36] * xx[43];
  xx[48] = xx[1] * (xx[46] - xx[47]);
  xx[49] = xx[1] * (xx[37] + xx[42]) - xx[2];
  xx[50] = xx[1] * (xx[44] + xx[10]);
  xx[51] = xx[48];
  xx[52] = xx[6] * xx[4];
  xx[53] = xx[8] * xx[5];
  xx[54] = xx[1] * (xx[52] + xx[53]);
  xx[55] = xx[54] * xx[29];
  xx[56] = xx[8] * xx[4];
  xx[57] = xx[6] * xx[5];
  xx[58] = xx[1] * (xx[56] - xx[57]);
  xx[59] = xx[58] * xx[30];
  xx[60] = xx[54] * xx[28];
  xx[61] = xx[59] + xx[60];
  xx[62] = xx[58] * xx[29];
  xx[63] = xx[55];
  xx[64] = - xx[61];
  xx[65] = xx[62];
  pm_math_Vector3_cross_ra(xx + 28, xx + 63, xx + 66);
  xx[63] = xx[1] * (xx[55] * xx[27] + xx[66]) - xx[58];
  xx[64] = xx[1] * (xx[67] - xx[61] * xx[27]);
  xx[65] = xx[54] + xx[1] * (xx[62] * xx[27] + xx[68]);
  xx[55] = xx[1] * (xx[32] - xx[31]);
  xx[31] = xx[55] * xx[29];
  xx[32] = xx[5] * xx[5];
  xx[61] = xx[2] - xx[1] * (xx[32] + xx[9]);
  xx[9] = xx[61] * xx[30];
  xx[62] = xx[55] * xx[28];
  xx[66] = xx[9] + xx[62];
  xx[67] = xx[61] * xx[29];
  xx[68] = xx[31];
  xx[69] = - xx[66];
  xx[70] = xx[67];
  pm_math_Vector3_cross_ra(xx + 28, xx + 68, xx + 71);
  xx[68] = xx[1] * (xx[31] * xx[27] + xx[71]) - xx[61];
  xx[69] = xx[1] * (xx[72] - xx[66] * xx[27]);
  xx[70] = xx[55] + xx[1] * (xx[67] * xx[27] + xx[73]);
  xx[31] = xx[1] * (xx[47] + xx[46]);
  xx[66] = xx[36] * xx[41];
  xx[67] = xx[43] * xx[45];
  xx[71] = xx[43] * xx[43];
  xx[72] = xx[1] * (xx[42] + xx[71]);
  xx[73] = - xx[31];
  xx[74] = xx[1] * (xx[66] - xx[67]);
  xx[75] = xx[72] - xx[2];
  xx[76] = xx[28] * xx[29];
  xx[77] = xx[27] * xx[30];
  xx[78] = xx[27] * xx[27];
  xx[79] = xx[29] * xx[30];
  xx[80] = xx[27] * xx[28];
  xx[81] = xx[1] * (xx[76] - xx[77]);
  xx[82] = xx[1] * (xx[78] + xx[29] * xx[29]) - xx[2];
  xx[83] = xx[1] * (xx[79] + xx[80]);
  xx[84] = xx[1] * (xx[44] - xx[10]);
  xx[85] = xx[45] * xx[45];
  xx[86] = xx[1] * (xx[85] + xx[42]);
  xx[42] = xx[1] * (xx[66] + xx[67]);
  xx[87] = xx[84];
  xx[88] = xx[2] - xx[86];
  xx[89] = xx[42];
  xx[90] = xx[1] * (xx[53] - xx[52]);
  xx[52] = xx[11] * xx[30];
  xx[53] = xx[90] * xx[30];
  xx[91] = xx[90] * xx[29];
  xx[92] = xx[35] + xx[91];
  xx[93] = xx[52];
  xx[94] = xx[53];
  xx[95] = - xx[92];
  pm_math_Vector3_cross_ra(xx + 28, xx + 93, xx + 96);
  xx[93] = xx[90] + xx[1] * (xx[52] * xx[27] + xx[96]);
  xx[94] = xx[1] * (xx[53] * xx[27] + xx[97]) - xx[11];
  xx[95] = xx[1] * (xx[98] - xx[92] * xx[27]);
  xx[96] = xx[84];
  xx[97] = xx[1] * (xx[37] + xx[71]) - xx[2];
  xx[98] = xx[1] * (xx[67] + xx[66]);
  xx[35] = xx[2] - xx[1] * (xx[7] + xx[32]);
  xx[7] = xx[54] * xx[30];
  xx[32] = xx[35] * xx[30];
  xx[52] = xx[35] * xx[29];
  xx[53] = xx[60] + xx[52];
  xx[99] = xx[7];
  xx[100] = xx[32];
  xx[101] = - xx[53];
  pm_math_Vector3_cross_ra(xx + 28, xx + 99, xx + 102);
  xx[99] = xx[35] + xx[1] * (xx[7] * xx[27] + xx[102]);
  xx[100] = xx[1] * (xx[32] * xx[27] + xx[103]) - xx[54];
  xx[101] = xx[1] * (xx[104] - xx[53] * xx[27]);
  xx[7] = xx[1] * (xx[57] + xx[56]);
  xx[32] = xx[55] * xx[30];
  xx[53] = xx[7] * xx[30];
  xx[56] = xx[7] * xx[29];
  xx[57] = xx[62] + xx[56];
  xx[102] = xx[32];
  xx[103] = xx[53];
  xx[104] = - xx[57];
  pm_math_Vector3_cross_ra(xx + 28, xx + 102, xx + 105);
  xx[102] = xx[7] + xx[1] * (xx[32] * xx[27] + xx[105]);
  xx[103] = xx[1] * (xx[53] * xx[27] + xx[106]) - xx[55];
  xx[104] = xx[1] * (xx[107] - xx[57] * xx[27]);
  xx[32] = xx[1] * (xx[67] - xx[66]);
  xx[105] = xx[31];
  xx[106] = xx[32];
  xx[107] = xx[2] - xx[72];
  xx[31] = xx[28] * xx[30];
  xx[53] = xx[27] * xx[29];
  xx[108] = xx[1] * (xx[31] + xx[53]);
  xx[109] = xx[1] * (xx[79] - xx[80]);
  xx[110] = xx[1] * (xx[78] + xx[30] * xx[30]) - xx[2];
  xx[57] = xx[1] * (xx[71] + xx[85]);
  xx[60] = xx[1] * (xx[10] + xx[44]);
  xx[111] = xx[57] - xx[2];
  xx[112] = - xx[60];
  xx[113] = xx[1] * (xx[47] - xx[46]);
  xx[62] = xx[91] + xx[34];
  xx[34] = xx[90] * xx[28];
  xx[66] = xx[33] * xx[28];
  xx[114] = - xx[62];
  xx[115] = xx[34];
  xx[116] = xx[66];
  pm_math_Vector3_cross_ra(xx + 28, xx + 114, xx + 117);
  xx[114] = xx[1] * (xx[117] - xx[62] * xx[27]);
  xx[115] = xx[33] + xx[1] * (xx[34] * xx[27] + xx[118]);
  xx[116] = xx[1] * (xx[66] * xx[27] + xx[119]) - xx[90];
  xx[117] = xx[1] * (xx[46] + xx[47]);
  xx[118] = xx[32];
  xx[119] = xx[1] * (xx[37] + xx[85]) - xx[2];
  xx[32] = xx[52] + xx[59];
  xx[34] = xx[35] * xx[28];
  xx[37] = xx[58] * xx[28];
  xx[120] = - xx[32];
  xx[121] = xx[34];
  xx[122] = xx[37];
  pm_math_Vector3_cross_ra(xx + 28, xx + 120, xx + 123);
  xx[120] = xx[1] * (xx[123] - xx[32] * xx[27]);
  xx[121] = xx[58] + xx[1] * (xx[34] * xx[27] + xx[124]);
  xx[122] = xx[1] * (xx[37] * xx[27] + xx[125]) - xx[35];
  xx[32] = xx[56] + xx[9];
  xx[9] = xx[7] * xx[28];
  xx[34] = xx[61] * xx[28];
  xx[123] = - xx[32];
  xx[124] = xx[9];
  xx[125] = xx[34];
  pm_math_Vector3_cross_ra(xx + 28, xx + 123, xx + 126);
  xx[123] = xx[1] * (xx[126] - xx[32] * xx[27]);
  xx[124] = xx[61] + xx[1] * (xx[9] * xx[27] + xx[127]);
  xx[125] = xx[1] * (xx[34] * xx[27] + xx[128]) - xx[7];
  xx[126] = xx[1] * (xx[10] - xx[44]);
  xx[127] = xx[86] - xx[2];
  xx[128] = - xx[42];
  xx[84] = xx[1] * (xx[78] + xx[28] * xx[28]) - xx[2];
  xx[85] = xx[1] * (xx[76] + xx[77]);
  xx[86] = xx[1] * (xx[31] - xx[53]);
  xx[76] = xx[2] - xx[57];
  xx[77] = xx[60];
  xx[78] = xx[48];
  xx[9] = bb[0] ? xx[0] : - xx[2];
  xx[10] = 0.04500000000000001;
  xx[31] = xx[10] * xx[11];
  xx[32] = xx[31] * xx[30];
  xx[34] = xx[10] * xx[90];
  xx[37] = xx[34] * xx[30];
  xx[42] = xx[31] * xx[28] + xx[34] * xx[29];
  xx[46] = - xx[32];
  xx[47] = - xx[37];
  xx[48] = xx[42];
  pm_math_Vector3_cross_ra(xx + 28, xx + 46, xx + 129);
  xx[44] = 0.04499999999999999;
  xx[46] = xx[44] * xx[5];
  xx[47] = xx[44] * xx[8];
  xx[48] = xx[1] * (xx[46] * xx[6] - xx[47] * xx[4]);
  xx[52] = xx[48] * xx[16];
  xx[53] = xx[1] * (xx[46] * xx[5] + xx[47] * xx[8]);
  xx[5] = xx[53] - xx[44] - xx[2];
  xx[8] = xx[5] * xx[18];
  xx[56] = xx[52] + xx[8];
  xx[132] = xx[15];
  xx[133] = xx[17];
  xx[134] = xx[12];
  xx[12] = xx[48] * xx[14];
  xx[15] = xx[5] * xx[14];
  xx[135] = - xx[56];
  xx[136] = xx[12];
  xx[137] = xx[15];
  pm_math_Vector3_cross_ra(xx + 132, xx + 135, xx + 138);
  xx[17] = xx[13] * xx[18];
  xx[57] = xx[14] * xx[16];
  xx[59] = xx[10] * xx[54];
  xx[60] = xx[59] * xx[30];
  xx[62] = xx[10] * xx[35];
  xx[66] = xx[62] * xx[30];
  xx[67] = xx[59] * xx[28] + xx[62] * xx[29];
  xx[135] = - xx[60];
  xx[136] = - xx[66];
  xx[137] = xx[67];
  pm_math_Vector3_cross_ra(xx + 28, xx + 135, xx + 141);
  xx[71] = xx[1] * (xx[47] * xx[6] + xx[46] * xx[4]);
  xx[4] = xx[71] * xx[16];
  xx[6] = xx[71] * xx[14];
  xx[46] = xx[8] - xx[6];
  xx[8] = xx[5] * xx[16];
  xx[135] = - xx[4];
  xx[136] = - xx[46];
  xx[137] = xx[8];
  pm_math_Vector3_cross_ra(xx + 132, xx + 135, xx + 144);
  xx[47] = xx[16] * xx[16];
  xx[72] = xx[18] * xx[18];
  xx[79] = xx[10] * xx[55];
  xx[80] = xx[79] * xx[30];
  xx[91] = xx[10] * xx[7];
  xx[92] = xx[91] * xx[30];
  xx[135] = xx[79] * xx[28] + xx[91] * xx[29];
  xx[147] = - xx[80];
  xx[148] = - xx[92];
  xx[149] = xx[135];
  pm_math_Vector3_cross_ra(xx + 28, xx + 147, xx + 150);
  xx[136] = xx[71] * xx[18];
  xx[137] = xx[48] * xx[18];
  xx[147] = xx[52] - xx[6];
  xx[153] = - xx[136];
  xx[154] = xx[137];
  xx[155] = - xx[147];
  pm_math_Vector3_cross_ra(xx + 132, xx + 153, xx + 156);
  xx[6] = bb[0] ? xx[0] : xx[2];
  xx[52] = 0.0225;
  xx[132] = xx[52] * xx[45];
  xx[133] = xx[132] * xx[36];
  xx[134] = xx[52] * xx[41];
  xx[148] = xx[52] * xx[43];
  xx[149] = xx[148] * xx[43];
  xx[153] = xx[132] * xx[45];
  xx[154] = xx[14] * xx[14];
  xx[155] = xx[134] * xx[41];
  xx[159] = xx[13] * xx[14];
  xx[160] = xx[16] * xx[18];
  xx[161] = xx[14] * xx[18];
  xx[162] = xx[13] * xx[16];
  xx[163] = xx[134] * xx[36];
  xx[164] = xx[148] * xx[36];
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
  xx[199] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 76, xx + 84);
  xx[200] = xx[0];
  xx[201] = xx[9];
  xx[202] = xx[0];
  xx[203] = xx[0];
  xx[204] = bb[0] ? xx[0] : - (xx[1] * (xx[129] - xx[32] * xx[27]) - xx[34] +
    xx[1] * (xx[56] * xx[13] + xx[138]) + xx[1] * (xx[17] - xx[57]));
  xx[205] = bb[0] ? xx[0] : - (xx[1] * (xx[141] - xx[60] * xx[27]) - xx[62] +
    xx[5] + xx[1] * (xx[144] + xx[4] * xx[13]) - xx[1] * (xx[47] + xx[72]) + xx
    [2]);
  xx[206] = bb[0] ? xx[0] : - (xx[1] * (xx[150] - xx[80] * xx[27]) - xx[91] +
    xx[1] * (xx[156] + xx[136] * xx[13]) - xx[48]);
  xx[207] = xx[6];
  xx[208] = xx[0];
  xx[209] = xx[0];
  xx[210] = bb[0] ? xx[0] : xx[1] * (xx[133] - xx[134] * xx[43]);
  xx[211] = bb[0] ? xx[0] : xx[52] - xx[1] * (xx[149] + xx[153]);
  xx[212] = xx[0];
  xx[213] = xx[0];
  xx[214] = xx[9];
  xx[215] = xx[0];
  xx[216] = bb[0] ? xx[0] : - (xx[1] * (xx[130] - xx[37] * xx[27]) + xx[1] *
    (xx[139] - xx[12] * xx[13]) + xx[1] * (xx[72] + xx[154]) - xx[5] + xx[31] -
    xx[2]);
  xx[217] = bb[0] ? xx[0] : - (xx[1] * (xx[142] - xx[66] * xx[27]) + xx[59] +
    xx[1] * (xx[46] * xx[13] + xx[145]) + xx[1] * (xx[17] + xx[57]));
  xx[218] = bb[0] ? xx[0] : - (xx[1] * (xx[151] - xx[92] * xx[27]) + xx[79] +
    xx[1] * (xx[157] - xx[137] * xx[13]) - xx[71]);
  xx[219] = xx[0];
  xx[220] = xx[6];
  xx[221] = xx[0];
  xx[222] = bb[0] ? xx[0] : xx[1] * (xx[153] + xx[155]) - xx[52];
  xx[223] = bb[0] ? xx[0] : xx[1] * (xx[133] + xx[148] * xx[41]);
  xx[224] = xx[0];
  xx[225] = xx[0];
  xx[226] = xx[0];
  xx[227] = xx[9];
  xx[228] = bb[0] ? xx[0] : - (xx[1] * (xx[131] + xx[42] * xx[27]) + xx[48] +
    xx[1] * (xx[140] - xx[15] * xx[13]) - xx[1] * (xx[159] + xx[160]));
  xx[229] = bb[0] ? xx[0] : - (xx[1] * (xx[143] + xx[67] * xx[27]) + xx[1] *
    (xx[146] - xx[8] * xx[13]) + xx[71] + xx[1] * (xx[161] - xx[162]));
  xx[230] = bb[0] ? xx[0] : - (xx[1] * (xx[152] + xx[135] * xx[27]) + xx[1] *
    (xx[147] * xx[13] + xx[158]));
  xx[231] = xx[0];
  xx[232] = xx[0];
  xx[233] = xx[6];
  xx[234] = bb[0] ? xx[0] : - (xx[1] * (xx[163] + xx[132] * xx[43]));
  xx[235] = bb[0] ? xx[0] : xx[1] * (xx[132] * xx[41] - xx[164]);
  xx[236] = xx[0];
  ii[0] = bb[0] ? 0 : 1;
  xx[4] = - xx[71];
  xx[37] = xx[4];
  xx[38] = xx[48];
  xx[39] = xx[5];
  pm_math_Quaternion_xform_ra(xx + 19, xx + 37, xx + 63);
  xx[8] = xx[10] * xx[29];
  xx[12] = xx[10] * xx[28];
  xx[15] = 0.9325;
  xx[72] = pm_math_Vector3_dot_ra(xx + 49, xx + 81);
  xx[73] = pm_math_Vector3_dot_ra(xx + 96, xx + 108);
  xx[74] = pm_math_Vector3_dot_ra(xx + 117, xx + 84);
  xx[75] = xx[1] * (xx[164] + xx[134] * xx[45]) + state[17] - (xx[63] + state[0]
    + xx[1] * (xx[162] + xx[161]) - xx[1] * (xx[8] * xx[27] + xx[12] * xx[30]));
  xx[76] = xx[1] * (xx[148] * xx[45] - xx[163]) + state[18] - (xx[1] * (xx[12] *
    xx[27] - xx[8] * xx[30]) + xx[64] + state[1] - xx[1] * (xx[159] - xx[160]));
  xx[77] = state[19] - xx[1] * (xx[155] + xx[149]) - (xx[65] + state[2] - xx[1] *
    (xx[154] + xx[47]) + xx[1] * (xx[12] * xx[28] + xx[8] * xx[29])) - xx[15];
  zeroMajor(1, 6, ii + 0, xx + 72);
  xx[63] = - xx[72];
  xx[64] = - xx[73];
  xx[65] = - xx[74];
  xx[66] = - xx[75];
  xx[67] = - xx[76];
  xx[68] = - xx[77];
  xx[8] = 1.0e-8;
  memcpy(xx + 237, xx + 165, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 237, xx + 72, xx + 80, ii + 1, xx + 63, xx[8],
                     xx + 92);
  xx[12] = state[0] + xx[92];
  xx[19] = xx[13];
  xx[20] = xx[14];
  xx[21] = xx[16];
  xx[22] = xx[18];
  pm_math_Quaternion_compDeriv_ra(xx + 19, xx + 95, xx + 27);
  xx[17] = xx[13] + xx[27];
  xx[13] = xx[14] + xx[28];
  xx[14] = xx[16] + xx[29];
  xx[16] = xx[18] + xx[30];
  xx[18] = 1.0e-64;
  xx[19] = sqrt(xx[17] * xx[17] + xx[13] * xx[13] + xx[14] * xx[14] + xx[16] *
                xx[16]);
  if (xx[18] > xx[19])
    xx[19] = xx[18];
  xx[20] = xx[17] / xx[19];
  xx[17] = xx[13] / xx[19];
  xx[13] = - xx[17];
  xx[21] = xx[14] / xx[19];
  xx[14] = - xx[21];
  xx[22] = xx[16] / xx[19];
  xx[16] = - xx[22];
  xx[27] = - xx[20];
  xx[28] = xx[13];
  xx[29] = xx[14];
  xx[30] = xx[16];
  pm_math_Quaternion_compose_ra(xx + 27, xx + 23, xx + 63);
  xx[19] = xx[11] * xx[65];
  xx[32] = xx[33] * xx[66];
  xx[40] = xx[11] * xx[64];
  xx[42] = xx[32] + xx[40];
  xx[46] = xx[33] * xx[65];
  xx[49] = xx[19];
  xx[50] = - xx[42];
  xx[51] = xx[46];
  pm_math_Vector3_cross_ra(xx + 64, xx + 49, xx + 67);
  xx[49] = xx[1] * (xx[19] * xx[63] + xx[67]) - xx[33];
  xx[50] = xx[1] * (xx[68] - xx[42] * xx[63]);
  xx[51] = xx[11] + xx[1] * (xx[46] * xx[63] + xx[69]);
  xx[67] = xx[36];
  xx[68] = xx[41];
  xx[69] = xx[43];
  xx[70] = xx[45];
  pm_math_Quaternion_compDeriv_ra(xx + 67, xx + 101, xx + 72);
  xx[19] = xx[36] + xx[72];
  xx[36] = xx[41] + xx[73];
  xx[41] = xx[43] + xx[74];
  xx[42] = xx[45] + xx[75];
  xx[43] = sqrt(xx[19] * xx[19] + xx[36] * xx[36] + xx[41] * xx[41] + xx[42] *
                xx[42]);
  if (xx[18] > xx[43])
    xx[43] = xx[18];
  xx[45] = xx[19] / xx[43];
  xx[19] = xx[45] * xx[45];
  xx[46] = xx[36] / xx[43];
  xx[36] = xx[46] * xx[46];
  xx[47] = xx[41] / xx[43];
  xx[41] = xx[46] * xx[47];
  xx[56] = xx[42] / xx[43];
  xx[42] = xx[45] * xx[56];
  xx[43] = xx[46] * xx[56];
  xx[57] = xx[45] * xx[47];
  xx[60] = xx[1] * (xx[43] - xx[57]);
  xx[67] = xx[1] * (xx[19] + xx[36]) - xx[2];
  xx[68] = xx[1] * (xx[41] + xx[42]);
  xx[69] = xx[60];
  xx[70] = xx[54] * xx[65];
  xx[72] = xx[58] * xx[66];
  xx[73] = xx[54] * xx[64];
  xx[74] = xx[72] + xx[73];
  xx[75] = xx[58] * xx[65];
  xx[76] = xx[70];
  xx[77] = - xx[74];
  xx[78] = xx[75];
  pm_math_Vector3_cross_ra(xx + 64, xx + 76, xx + 80);
  xx[76] = xx[1] * (xx[70] * xx[63] + xx[80]) - xx[58];
  xx[77] = xx[1] * (xx[81] - xx[74] * xx[63]);
  xx[78] = xx[54] + xx[1] * (xx[75] * xx[63] + xx[82]);
  xx[70] = xx[55] * xx[65];
  xx[74] = xx[61] * xx[66];
  xx[75] = xx[55] * xx[64];
  xx[80] = xx[74] + xx[75];
  xx[81] = xx[61] * xx[65];
  xx[82] = xx[70];
  xx[83] = - xx[80];
  xx[84] = xx[81];
  pm_math_Vector3_cross_ra(xx + 64, xx + 82, xx + 85);
  xx[82] = xx[1] * (xx[70] * xx[63] + xx[85]) - xx[61];
  xx[83] = xx[1] * (xx[86] - xx[80] * xx[63]);
  xx[84] = xx[55] + xx[1] * (xx[81] * xx[63] + xx[87]);
  xx[70] = xx[1] * (xx[57] + xx[43]);
  xx[80] = xx[45] * xx[46];
  xx[81] = xx[47] * xx[56];
  xx[85] = xx[47] * xx[47];
  xx[86] = xx[1] * (xx[36] + xx[85]);
  xx[87] = - xx[70];
  xx[88] = xx[1] * (xx[80] - xx[81]);
  xx[89] = xx[86] - xx[2];
  xx[104] = xx[64] * xx[65];
  xx[105] = xx[63] * xx[66];
  xx[106] = xx[63] * xx[63];
  xx[107] = xx[65] * xx[66];
  xx[108] = xx[63] * xx[64];
  xx[109] = xx[1] * (xx[104] - xx[105]);
  xx[110] = xx[1] * (xx[106] + xx[65] * xx[65]) - xx[2];
  xx[111] = xx[1] * (xx[107] + xx[108]);
  xx[112] = xx[1] * (xx[41] - xx[42]);
  xx[113] = xx[56] * xx[56];
  xx[114] = xx[1] * (xx[113] + xx[36]);
  xx[36] = xx[1] * (xx[80] + xx[81]);
  xx[115] = xx[112];
  xx[116] = xx[2] - xx[114];
  xx[117] = xx[36];
  xx[118] = xx[11] * xx[66];
  xx[119] = xx[90] * xx[66];
  xx[120] = xx[90] * xx[65];
  xx[121] = xx[40] + xx[120];
  xx[122] = xx[118];
  xx[123] = xx[119];
  xx[124] = - xx[121];
  pm_math_Vector3_cross_ra(xx + 64, xx + 122, xx + 125);
  xx[122] = xx[90] + xx[1] * (xx[118] * xx[63] + xx[125]);
  xx[123] = xx[1] * (xx[119] * xx[63] + xx[126]) - xx[11];
  xx[124] = xx[1] * (xx[127] - xx[121] * xx[63]);
  xx[125] = xx[112];
  xx[126] = xx[1] * (xx[19] + xx[85]) - xx[2];
  xx[127] = xx[1] * (xx[81] + xx[80]);
  xx[11] = xx[54] * xx[66];
  xx[40] = xx[35] * xx[66];
  xx[112] = xx[35] * xx[65];
  xx[118] = xx[73] + xx[112];
  xx[128] = xx[11];
  xx[129] = xx[40];
  xx[130] = - xx[118];
  pm_math_Vector3_cross_ra(xx + 64, xx + 128, xx + 131);
  xx[128] = xx[35] + xx[1] * (xx[11] * xx[63] + xx[131]);
  xx[129] = xx[1] * (xx[40] * xx[63] + xx[132]) - xx[54];
  xx[130] = xx[1] * (xx[133] - xx[118] * xx[63]);
  xx[11] = xx[55] * xx[66];
  xx[40] = xx[7] * xx[66];
  xx[54] = xx[7] * xx[65];
  xx[73] = xx[75] + xx[54];
  xx[131] = xx[11];
  xx[132] = xx[40];
  xx[133] = - xx[73];
  pm_math_Vector3_cross_ra(xx + 64, xx + 131, xx + 134);
  xx[131] = xx[7] + xx[1] * (xx[11] * xx[63] + xx[134]);
  xx[132] = xx[1] * (xx[40] * xx[63] + xx[135]) - xx[55];
  xx[133] = xx[1] * (xx[136] - xx[73] * xx[63]);
  xx[11] = xx[1] * (xx[81] - xx[80]);
  xx[134] = xx[70];
  xx[135] = xx[11];
  xx[136] = xx[2] - xx[86];
  xx[40] = xx[64] * xx[66];
  xx[55] = xx[63] * xx[65];
  xx[137] = xx[1] * (xx[40] + xx[55]);
  xx[138] = xx[1] * (xx[107] - xx[108]);
  xx[139] = xx[1] * (xx[106] + xx[66] * xx[66]) - xx[2];
  xx[70] = xx[1] * (xx[85] + xx[113]);
  xx[73] = xx[1] * (xx[42] + xx[41]);
  xx[140] = xx[70] - xx[2];
  xx[141] = - xx[73];
  xx[142] = xx[1] * (xx[57] - xx[43]);
  xx[75] = xx[120] + xx[32];
  xx[32] = xx[90] * xx[64];
  xx[80] = xx[33] * xx[64];
  xx[118] = - xx[75];
  xx[119] = xx[32];
  xx[120] = xx[80];
  pm_math_Vector3_cross_ra(xx + 64, xx + 118, xx + 143);
  xx[118] = xx[1] * (xx[143] - xx[75] * xx[63]);
  xx[119] = xx[33] + xx[1] * (xx[32] * xx[63] + xx[144]);
  xx[120] = xx[1] * (xx[80] * xx[63] + xx[145]) - xx[90];
  xx[143] = xx[1] * (xx[43] + xx[57]);
  xx[144] = xx[11];
  xx[145] = xx[1] * (xx[19] + xx[113]) - xx[2];
  xx[11] = xx[112] + xx[72];
  xx[19] = xx[35] * xx[64];
  xx[32] = xx[58] * xx[64];
  xx[146] = - xx[11];
  xx[147] = xx[19];
  xx[148] = xx[32];
  pm_math_Vector3_cross_ra(xx + 64, xx + 146, xx + 149);
  xx[146] = xx[1] * (xx[149] - xx[11] * xx[63]);
  xx[147] = xx[58] + xx[1] * (xx[19] * xx[63] + xx[150]);
  xx[148] = xx[1] * (xx[32] * xx[63] + xx[151]) - xx[35];
  xx[11] = xx[54] + xx[74];
  xx[19] = xx[7] * xx[64];
  xx[32] = xx[61] * xx[64];
  xx[149] = - xx[11];
  xx[150] = xx[19];
  xx[151] = xx[32];
  pm_math_Vector3_cross_ra(xx + 64, xx + 149, xx + 152);
  xx[149] = xx[1] * (xx[152] - xx[11] * xx[63]);
  xx[150] = xx[61] + xx[1] * (xx[19] * xx[63] + xx[153]);
  xx[151] = xx[1] * (xx[32] * xx[63] + xx[154]) - xx[7];
  xx[152] = xx[1] * (xx[42] - xx[41]);
  xx[153] = xx[114] - xx[2];
  xx[154] = - xx[36];
  xx[41] = xx[1] * (xx[106] + xx[64] * xx[64]) - xx[2];
  xx[42] = xx[1] * (xx[104] + xx[105]);
  xx[43] = xx[1] * (xx[40] - xx[55]);
  xx[104] = xx[2] - xx[70];
  xx[105] = xx[73];
  xx[106] = xx[60];
  xx[7] = xx[31] * xx[66];
  xx[11] = xx[34] * xx[66];
  xx[19] = xx[31] * xx[64] + xx[34] * xx[65];
  xx[72] = - xx[7];
  xx[73] = - xx[11];
  xx[74] = xx[19];
  pm_math_Vector3_cross_ra(xx + 64, xx + 72, xx + 112);
  xx[32] = xx[48] * xx[21];
  xx[33] = xx[5] * xx[22];
  xx[35] = xx[32] + xx[33];
  xx[72] = xx[13];
  xx[73] = xx[14];
  xx[74] = xx[16];
  xx[13] = xx[48] * xx[17];
  xx[14] = xx[5] * xx[17];
  xx[155] = - xx[35];
  xx[156] = xx[13];
  xx[157] = xx[14];
  pm_math_Vector3_cross_ra(xx + 72, xx + 155, xx + 158);
  xx[16] = xx[20] * xx[22];
  xx[36] = xx[17] * xx[21];
  xx[40] = xx[59] * xx[66];
  xx[54] = xx[62] * xx[66];
  xx[55] = xx[59] * xx[64] + xx[62] * xx[65];
  xx[155] = - xx[40];
  xx[156] = - xx[54];
  xx[157] = xx[55];
  pm_math_Vector3_cross_ra(xx + 64, xx + 155, xx + 161);
  xx[57] = xx[71] * xx[21];
  xx[58] = xx[71] * xx[17];
  xx[60] = xx[33] - xx[58];
  xx[33] = xx[5] * xx[21];
  xx[155] = - xx[57];
  xx[156] = - xx[60];
  xx[157] = xx[33];
  pm_math_Vector3_cross_ra(xx + 72, xx + 155, xx + 164);
  xx[61] = xx[21] * xx[21];
  xx[70] = xx[22] * xx[22];
  xx[75] = xx[79] * xx[66];
  xx[80] = xx[91] * xx[66];
  xx[81] = xx[79] * xx[64] + xx[91] * xx[65];
  xx[155] = - xx[75];
  xx[156] = - xx[80];
  xx[157] = xx[81];
  pm_math_Vector3_cross_ra(xx + 64, xx + 155, xx + 167);
  xx[85] = xx[71] * xx[22];
  xx[86] = xx[48] * xx[22];
  xx[90] = xx[32] - xx[58];
  xx[155] = - xx[85];
  xx[156] = xx[86];
  xx[157] = - xx[90];
  pm_math_Vector3_cross_ra(xx + 72, xx + 155, xx + 170);
  xx[32] = xx[52] * xx[56];
  xx[58] = xx[32] * xx[45];
  xx[72] = xx[52] * xx[46];
  xx[73] = xx[52] * xx[47];
  xx[74] = xx[73] * xx[47];
  xx[107] = xx[32] * xx[56];
  xx[108] = xx[17] * xx[17];
  xx[121] = xx[72] * xx[46];
  xx[155] = xx[20] * xx[17];
  xx[156] = xx[21] * xx[22];
  xx[157] = xx[17] * xx[22];
  xx[173] = xx[20] * xx[21];
  xx[174] = xx[72] * xx[45];
  xx[175] = xx[73] * xx[45];
  xx[176] = xx[0];
  xx[177] = xx[0];
  xx[178] = xx[0];
  xx[179] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 49, xx + 67);
  xx[180] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 76, xx + 67);
  xx[181] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 82, xx + 67);
  xx[182] = xx[0];
  xx[183] = xx[0];
  xx[184] = xx[0];
  xx[185] = xx[0];
  xx[186] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 87, xx + 109);
  xx[187] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 115, xx + 109);
  xx[188] = xx[0];
  xx[189] = xx[0];
  xx[190] = xx[0];
  xx[191] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 122, xx + 125);
  xx[192] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 128, xx + 125);
  xx[193] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 131, xx + 125);
  xx[194] = xx[0];
  xx[195] = xx[0];
  xx[196] = xx[0];
  xx[197] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 134, xx + 137);
  xx[198] = xx[0];
  xx[199] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 140, xx + 137);
  xx[200] = xx[0];
  xx[201] = xx[0];
  xx[202] = xx[0];
  xx[203] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 118, xx + 143);
  xx[204] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 146, xx + 143);
  xx[205] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 149, xx + 143);
  xx[206] = xx[0];
  xx[207] = xx[0];
  xx[208] = xx[0];
  xx[209] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 152, xx + 41);
  xx[210] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 104, xx + 41);
  xx[211] = xx[0];
  xx[212] = xx[9];
  xx[213] = xx[0];
  xx[214] = xx[0];
  xx[215] = bb[0] ? xx[0] : - (xx[1] * (xx[112] - xx[7] * xx[63]) - xx[34] + xx
    [1] * (xx[35] * xx[20] + xx[158]) + xx[1] * (xx[16] - xx[36]));
  xx[216] = bb[0] ? xx[0] : - (xx[1] * (xx[161] - xx[40] * xx[63]) - xx[62] +
    xx[5] + xx[1] * (xx[164] + xx[57] * xx[20]) - xx[1] * (xx[61] + xx[70]) +
    xx[2]);
  xx[217] = bb[0] ? xx[0] : - (xx[1] * (xx[167] - xx[75] * xx[63]) - xx[91] +
    xx[1] * (xx[170] + xx[85] * xx[20]) - xx[48]);
  xx[218] = xx[6];
  xx[219] = xx[0];
  xx[220] = xx[0];
  xx[221] = bb[0] ? xx[0] : xx[1] * (xx[58] - xx[72] * xx[47]);
  xx[222] = bb[0] ? xx[0] : xx[52] - xx[1] * (xx[74] + xx[107]);
  xx[223] = xx[0];
  xx[224] = xx[0];
  xx[225] = xx[9];
  xx[226] = xx[0];
  xx[227] = bb[0] ? xx[0] : - (xx[1] * (xx[113] - xx[11] * xx[63]) + xx[1] *
    (xx[159] - xx[13] * xx[20]) + xx[1] * (xx[70] + xx[108]) - xx[5] + xx[31] -
    xx[2]);
  xx[228] = bb[0] ? xx[0] : - (xx[1] * (xx[162] - xx[54] * xx[63]) + xx[59] +
    xx[1] * (xx[60] * xx[20] + xx[165]) + xx[1] * (xx[16] + xx[36]));
  xx[229] = bb[0] ? xx[0] : - (xx[1] * (xx[168] - xx[80] * xx[63]) + xx[79] +
    xx[1] * (xx[171] - xx[86] * xx[20]) - xx[71]);
  xx[230] = xx[0];
  xx[231] = xx[6];
  xx[232] = xx[0];
  xx[233] = bb[0] ? xx[0] : xx[1] * (xx[107] + xx[121]) - xx[52];
  xx[234] = bb[0] ? xx[0] : xx[1] * (xx[58] + xx[73] * xx[46]);
  xx[235] = xx[0];
  xx[236] = xx[0];
  xx[237] = xx[0];
  xx[238] = xx[9];
  xx[239] = bb[0] ? xx[0] : - (xx[1] * (xx[114] + xx[19] * xx[63]) + xx[48] +
    xx[1] * (xx[160] - xx[14] * xx[20]) - xx[1] * (xx[155] + xx[156]));
  xx[240] = bb[0] ? xx[0] : - (xx[1] * (xx[163] + xx[55] * xx[63]) + xx[1] *
    (xx[166] - xx[33] * xx[20]) + xx[71] + xx[1] * (xx[157] - xx[173]));
  xx[241] = bb[0] ? xx[0] : - (xx[1] * (xx[169] + xx[81] * xx[63]) + xx[1] *
    (xx[90] * xx[20] + xx[172]));
  xx[242] = xx[0];
  xx[243] = xx[0];
  xx[244] = xx[6];
  xx[245] = bb[0] ? xx[0] : - (xx[1] * (xx[174] + xx[32] * xx[47]));
  xx[246] = bb[0] ? xx[0] : xx[1] * (xx[32] * xx[46] - xx[175]);
  xx[247] = xx[0];
  xx[5] = state[17] + xx[98];
  pm_math_Quaternion_xform_ra(xx + 27, xx + 37, xx + 31);
  xx[7] = xx[10] * xx[65];
  xx[11] = xx[10] * xx[64];
  xx[13] = state[18] + xx[99];
  xx[14] = state[1] + xx[93];
  xx[16] = state[19] + xx[100];
  xx[19] = state[2] + xx[94];
  xx[34] = pm_math_Vector3_dot_ra(xx + 67, xx + 109);
  xx[35] = pm_math_Vector3_dot_ra(xx + 125, xx + 137);
  xx[36] = pm_math_Vector3_dot_ra(xx + 143, xx + 41);
  xx[37] = xx[1] * (xx[175] + xx[72] * xx[56]) + xx[5] - (xx[31] + xx[12] + xx[1]
    * (xx[173] + xx[157]) - xx[1] * (xx[7] * xx[63] + xx[11] * xx[66]));
  xx[38] = xx[1] * (xx[73] * xx[56] - xx[174]) + xx[13] - (xx[1] * (xx[11] * xx
    [63] - xx[7] * xx[66]) + xx[32] + xx[14] - xx[1] * (xx[155] - xx[156]));
  xx[39] = xx[52] - xx[1] * (xx[121] + xx[74]) + xx[16] - (xx[33] + xx[19] -
    (xx[1] * (xx[108] + xx[61]) - xx[2]) + xx[1] * (xx[11] * xx[64] + xx[7] *
    xx[65])) + xx[10];
  zeroMajor(1, 6, ii + 0, xx + 34);
  xx[27] = - xx[34];
  xx[28] = - xx[35];
  xx[29] = - xx[36];
  xx[30] = - xx[37];
  xx[31] = - xx[38];
  xx[32] = - xx[39];
  memcpy(xx + 69, xx + 176, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 69, xx + 33, xx + 141, ii + 1, xx + 27, xx[8],
                     xx + 57);
  xx[7] = xx[12] + xx[57];
  xx[11] = xx[14] + xx[58];
  xx[12] = xx[19] + xx[59];
  xx[27] = xx[20];
  xx[28] = xx[17];
  xx[29] = xx[21];
  xx[30] = xx[22];
  pm_math_Quaternion_compDeriv_ra(xx + 27, xx + 60, xx + 31);
  xx[14] = xx[20] + xx[31];
  xx[19] = xx[17] + xx[32];
  xx[17] = xx[21] + xx[33];
  xx[20] = xx[22] + xx[34];
  xx[21] = sqrt(xx[14] * xx[14] + xx[19] * xx[19] + xx[17] * xx[17] + xx[20] *
                xx[20]);
  if (xx[18] > xx[21])
    xx[21] = xx[18];
  xx[22] = xx[14] / xx[21];
  xx[14] = xx[19] / xx[21];
  xx[19] = xx[17] / xx[21];
  xx[17] = xx[20] / xx[21];
  xx[20] = xx[5] + xx[63];
  xx[5] = xx[13] + xx[64];
  xx[13] = xx[16] + xx[65];
  xx[27] = xx[45];
  xx[28] = xx[46];
  xx[29] = xx[47];
  xx[30] = xx[56];
  pm_math_Quaternion_compDeriv_ra(xx + 27, xx + 66, xx + 31);
  xx[16] = xx[45] + xx[31];
  xx[21] = xx[46] + xx[32];
  xx[27] = xx[47] + xx[33];
  xx[28] = xx[56] + xx[34];
  xx[29] = sqrt(xx[16] * xx[16] + xx[21] * xx[21] + xx[27] * xx[27] + xx[28] *
                xx[28]);
  if (xx[18] > xx[29])
    xx[29] = xx[18];
  xx[18] = xx[16] / xx[29];
  xx[16] = xx[21] / xx[29];
  xx[21] = xx[27] / xx[29];
  xx[27] = xx[28] / xx[29];
  xx[54] = xx[7];
  xx[55] = xx[11];
  xx[56] = xx[12];
  xx[57] = xx[22];
  xx[58] = xx[14];
  xx[59] = xx[19];
  xx[60] = xx[17];
  xx[61] = state[7];
  xx[62] = state[8];
  xx[63] = state[9];
  xx[64] = state[10];
  xx[65] = state[11];
  xx[66] = state[12];
  xx[67] = state[13];
  xx[68] = state[14];
  xx[69] = state[15];
  xx[70] = state[16];
  xx[71] = xx[20];
  xx[72] = xx[5];
  xx[73] = xx[13];
  xx[74] = xx[18];
  xx[75] = xx[16];
  xx[76] = xx[21];
  xx[77] = xx[27];
  xx[78] = state[24];
  xx[79] = state[25];
  xx[80] = state[26];
  xx[81] = state[27];
  xx[82] = state[28];
  xx[83] = state[29];
  xx[28] = xx[18] * xx[18];
  xx[29] = xx[16] * xx[21];
  xx[30] = xx[18] * xx[27];
  xx[31] = xx[16] * xx[27];
  xx[32] = xx[18] * xx[21];
  xx[33] = xx[1] * (xx[28] + xx[16] * xx[16]) - xx[2];
  xx[34] = xx[1] * (xx[29] + xx[30]);
  xx[35] = xx[1] * (xx[31] - xx[32]);
  xx[36] = - xx[22];
  xx[37] = - xx[14];
  xx[38] = - xx[19];
  xx[39] = - xx[17];
  pm_math_Quaternion_compose_ra(xx + 36, xx + 23, xx + 40);
  xx[23] = xx[41] * xx[42];
  xx[24] = xx[40] * xx[43];
  xx[25] = xx[40] * xx[40];
  xx[26] = xx[42] * xx[43];
  xx[45] = xx[40] * xx[41];
  xx[49] = xx[1] * (xx[23] - xx[24]);
  xx[50] = xx[1] * (xx[25] + xx[42] * xx[42]) - xx[2];
  xx[51] = xx[1] * (xx[26] + xx[45]);
  xx[46] = xx[21] * xx[27];
  xx[47] = xx[18] * xx[16];
  xx[84] = xx[1] * (xx[29] - xx[30]);
  xx[85] = xx[1] * (xx[28] + xx[21] * xx[21]) - xx[2];
  xx[86] = xx[1] * (xx[46] + xx[47]);
  xx[29] = xx[41] * xx[43];
  xx[30] = xx[40] * xx[42];
  xx[87] = xx[1] * (xx[29] + xx[30]);
  xx[88] = xx[1] * (xx[26] - xx[45]);
  xx[89] = xx[1] * (xx[25] + xx[43] * xx[43]) - xx[2];
  xx[90] = xx[1] * (xx[31] + xx[32]);
  xx[91] = xx[1] * (xx[46] - xx[47]);
  xx[92] = xx[1] * (xx[28] + xx[27] * xx[27]) - xx[2];
  xx[45] = xx[1] * (xx[25] + xx[41] * xx[41]) - xx[2];
  xx[46] = xx[1] * (xx[23] + xx[24]);
  xx[47] = xx[1] * (xx[29] - xx[30]);
  xx[23] = xx[52] * xx[21];
  xx[24] = xx[52] * xx[16];
  xx[25] = 1.045;
  xx[28] = xx[4];
  xx[29] = xx[48];
  xx[30] = xx[53] - xx[25];
  pm_math_Quaternion_xform_ra(xx + 36, xx + 28, xx + 93);
  xx[4] = xx[10] * xx[42];
  xx[26] = xx[10] * xx[41];
  xx[96] = pm_math_Vector3_dot_ra(xx + 33, xx + 49);
  xx[97] = pm_math_Vector3_dot_ra(xx + 84, xx + 87);
  xx[98] = pm_math_Vector3_dot_ra(xx + 90, xx + 45);
  xx[99] = xx[1] * (xx[23] * xx[18] + xx[24] * xx[27]) + xx[20] - (xx[93] + xx[7]
    + xx[1] * (xx[22] * xx[19] + xx[14] * xx[17]) - xx[1] * (xx[4] * xx[40] +
    xx[26] * xx[43]));
  xx[100] = xx[1] * (xx[23] * xx[27] - xx[24] * xx[18]) + xx[5] - (xx[1] * (xx
    [26] * xx[40] - xx[4] * xx[43]) + xx[94] + xx[11] - xx[1] * (xx[22] * xx[14]
    - xx[19] * xx[17]));
  xx[101] = xx[13] - xx[1] * (xx[24] * xx[16] + xx[23] * xx[21]) - (xx[95] + xx
    [12] - xx[1] * (xx[14] * xx[14] + xx[19] * xx[19]) + xx[1] * (xx[26] * xx[41]
    + xx[4] * xx[42])) - xx[15];
  zeroMajor(1, 6, ii + 0, xx + 96);
  xx[11] = fabs(xx[96]);
  xx[12] = fabs(xx[97]);
  xx[13] = fabs(xx[98]);
  xx[14] = fabs(xx[99]);
  xx[15] = fabs(xx[100]);
  xx[16] = fabs(xx[101]);
  ii[1] = 11;

  {
    int ll;
    for (ll = 12; ll < 17; ++ll)
      if (xx[ll] > xx[ii[1]])
        ii[1] = ll;
  }

  ii[1] -= 11;
  xx[4] = xx[11 + (ii[1])];
  xx[5] = 1.0e-9;
  if (xx[4] > xx[5]) {
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

  xx[4] = xx[3] * xx[67];
  xx[7] = cos(xx[4]);
  xx[11] = xx[3] * xx[68];
  xx[3] = sin(xx[11]);
  xx[12] = xx[7] * xx[3];
  xx[13] = xx[12] * xx[12];
  xx[14] = sin(xx[4]);
  xx[4] = xx[14] * xx[3];
  xx[3] = xx[4] * xx[4];
  xx[15] = xx[2] - xx[1] * (xx[13] + xx[3]);
  xx[16] = - xx[58];
  xx[17] = - xx[59];
  xx[18] = - xx[60];
  xx[19] = - xx[57];
  xx[20] = xx[16];
  xx[21] = xx[17];
  xx[22] = xx[18];
  xx[23] = cos(xx[11]);
  xx[11] = xx[7] * xx[23];
  xx[7] = xx[23] * xx[14];
  xx[14] = - xx[7];
  xx[23] = - xx[12];
  xx[24] = - xx[4];
  xx[26] = - xx[11];
  xx[27] = xx[14];
  xx[28] = xx[23];
  xx[29] = xx[24];
  pm_math_Quaternion_compose_ra(xx + 19, xx + 26, xx + 30);
  xx[34] = xx[15] * xx[32];
  xx[35] = xx[11] * xx[12];
  xx[36] = xx[7] * xx[4];
  xx[37] = xx[35] + xx[36];
  xx[38] = xx[1] * xx[37];
  xx[39] = xx[38] * xx[33];
  xx[40] = xx[15] * xx[31];
  xx[41] = xx[39] + xx[40];
  xx[42] = xx[38] * xx[32];
  xx[45] = xx[34];
  xx[46] = - xx[41];
  xx[47] = xx[42];
  pm_math_Vector3_cross_ra(xx + 31, xx + 45, xx + 48);
  xx[45] = xx[1] * (xx[34] * xx[30] + xx[48]) - xx[38];
  xx[46] = xx[1] * (xx[49] - xx[41] * xx[30]);
  xx[47] = xx[15] + xx[1] * (xx[42] * xx[30] + xx[50]);
  xx[34] = xx[74] * xx[74];
  xx[41] = xx[75] * xx[75];
  xx[42] = xx[75] * xx[76];
  xx[43] = xx[74] * xx[77];
  xx[48] = xx[75] * xx[77];
  xx[49] = xx[74] * xx[76];
  xx[50] = xx[1] * (xx[48] - xx[49]);
  xx[84] = xx[1] * (xx[34] + xx[41]) - xx[2];
  xx[85] = xx[1] * (xx[42] + xx[43]);
  xx[86] = xx[50];
  xx[51] = xx[11] * xx[4];
  xx[53] = xx[12] * xx[7];
  xx[87] = xx[1] * (xx[51] + xx[53]);
  xx[88] = xx[87] * xx[32];
  xx[89] = xx[12] * xx[4];
  xx[90] = xx[11] * xx[7];
  xx[91] = xx[1] * (xx[89] - xx[90]);
  xx[92] = xx[91] * xx[33];
  xx[93] = xx[87] * xx[31];
  xx[94] = xx[92] + xx[93];
  xx[95] = xx[91] * xx[32];
  xx[96] = xx[88];
  xx[97] = - xx[94];
  xx[98] = xx[95];
  pm_math_Vector3_cross_ra(xx + 31, xx + 96, xx + 99);
  xx[96] = xx[1] * (xx[88] * xx[30] + xx[99]) - xx[91];
  xx[97] = xx[1] * (xx[100] - xx[94] * xx[30]);
  xx[98] = xx[87] + xx[1] * (xx[95] * xx[30] + xx[101]);
  xx[88] = xx[1] * (xx[36] - xx[35]);
  xx[35] = xx[88] * xx[32];
  xx[36] = xx[7] * xx[7];
  xx[94] = xx[2] - xx[1] * (xx[36] + xx[13]);
  xx[13] = xx[94] * xx[33];
  xx[95] = xx[88] * xx[31];
  xx[99] = xx[13] + xx[95];
  xx[100] = xx[94] * xx[32];
  xx[101] = xx[35];
  xx[102] = - xx[99];
  xx[103] = xx[100];
  pm_math_Vector3_cross_ra(xx + 31, xx + 101, xx + 104);
  xx[101] = xx[1] * (xx[35] * xx[30] + xx[104]) - xx[94];
  xx[102] = xx[1] * (xx[105] - xx[99] * xx[30]);
  xx[103] = xx[88] + xx[1] * (xx[100] * xx[30] + xx[106]);
  xx[35] = xx[1] * (xx[49] + xx[48]);
  xx[99] = xx[74] * xx[75];
  xx[100] = xx[76] * xx[77];
  xx[104] = xx[76] * xx[76];
  xx[105] = xx[1] * (xx[41] + xx[104]);
  xx[106] = - xx[35];
  xx[107] = xx[1] * (xx[99] - xx[100]);
  xx[108] = xx[105] - xx[2];
  xx[109] = xx[31] * xx[32];
  xx[110] = xx[30] * xx[33];
  xx[111] = xx[30] * xx[30];
  xx[112] = xx[32] * xx[33];
  xx[113] = xx[30] * xx[31];
  xx[114] = xx[1] * (xx[109] - xx[110]);
  xx[115] = xx[1] * (xx[111] + xx[32] * xx[32]) - xx[2];
  xx[116] = xx[1] * (xx[112] + xx[113]);
  xx[117] = xx[1] * (xx[42] - xx[43]);
  xx[118] = xx[77] * xx[77];
  xx[119] = xx[1] * (xx[118] + xx[41]);
  xx[41] = xx[1] * (xx[99] + xx[100]);
  xx[120] = xx[117];
  xx[121] = xx[2] - xx[119];
  xx[122] = xx[41];
  xx[123] = xx[53] - xx[51];
  xx[51] = xx[1] * xx[123];
  xx[53] = xx[15] * xx[33];
  xx[124] = xx[51] * xx[33];
  xx[125] = xx[51] * xx[32];
  xx[126] = xx[40] + xx[125];
  xx[127] = xx[53];
  xx[128] = xx[124];
  xx[129] = - xx[126];
  pm_math_Vector3_cross_ra(xx + 31, xx + 127, xx + 130);
  xx[127] = xx[51] + xx[1] * (xx[53] * xx[30] + xx[130]);
  xx[128] = xx[1] * (xx[124] * xx[30] + xx[131]) - xx[15];
  xx[129] = xx[1] * (xx[132] - xx[126] * xx[30]);
  xx[130] = xx[117];
  xx[131] = xx[1] * (xx[34] + xx[104]) - xx[2];
  xx[132] = xx[1] * (xx[100] + xx[99]);
  xx[40] = xx[2] - xx[1] * (xx[3] + xx[36]);
  xx[3] = xx[87] * xx[33];
  xx[36] = xx[40] * xx[33];
  xx[53] = xx[40] * xx[32];
  xx[117] = xx[93] + xx[53];
  xx[133] = xx[3];
  xx[134] = xx[36];
  xx[135] = - xx[117];
  pm_math_Vector3_cross_ra(xx + 31, xx + 133, xx + 136);
  xx[133] = xx[40] + xx[1] * (xx[3] * xx[30] + xx[136]);
  xx[134] = xx[1] * (xx[36] * xx[30] + xx[137]) - xx[87];
  xx[135] = xx[1] * (xx[138] - xx[117] * xx[30]);
  xx[3] = xx[1] * (xx[90] + xx[89]);
  xx[36] = xx[88] * xx[33];
  xx[89] = xx[3] * xx[33];
  xx[90] = xx[3] * xx[32];
  xx[93] = xx[95] + xx[90];
  xx[136] = xx[36];
  xx[137] = xx[89];
  xx[138] = - xx[93];
  pm_math_Vector3_cross_ra(xx + 31, xx + 136, xx + 139);
  xx[136] = xx[3] + xx[1] * (xx[36] * xx[30] + xx[139]);
  xx[137] = xx[1] * (xx[89] * xx[30] + xx[140]) - xx[88];
  xx[138] = xx[1] * (xx[141] - xx[93] * xx[30]);
  xx[36] = xx[1] * (xx[100] - xx[99]);
  xx[139] = xx[35];
  xx[140] = xx[36];
  xx[141] = xx[2] - xx[105];
  xx[35] = xx[31] * xx[33];
  xx[89] = xx[30] * xx[32];
  xx[142] = xx[1] * (xx[35] + xx[89]);
  xx[143] = xx[1] * (xx[112] - xx[113]);
  xx[144] = xx[1] * (xx[111] + xx[33] * xx[33]) - xx[2];
  xx[93] = xx[1] * (xx[104] + xx[118]);
  xx[95] = xx[1] * (xx[43] + xx[42]);
  xx[145] = xx[93] - xx[2];
  xx[146] = - xx[95];
  xx[147] = xx[1] * (xx[49] - xx[48]);
  xx[99] = xx[125] + xx[39];
  xx[39] = xx[51] * xx[31];
  xx[100] = xx[38] * xx[31];
  xx[124] = - xx[99];
  xx[125] = xx[39];
  xx[126] = xx[100];
  pm_math_Vector3_cross_ra(xx + 31, xx + 124, xx + 148);
  xx[124] = xx[1] * (xx[148] - xx[99] * xx[30]);
  xx[125] = xx[38] + xx[1] * (xx[39] * xx[30] + xx[149]);
  xx[126] = xx[1] * (xx[100] * xx[30] + xx[150]) - xx[51];
  xx[148] = xx[1] * (xx[48] + xx[49]);
  xx[149] = xx[36];
  xx[150] = xx[1] * (xx[34] + xx[118]) - xx[2];
  xx[34] = xx[53] + xx[92];
  xx[36] = xx[40] * xx[31];
  xx[38] = xx[91] * xx[31];
  xx[151] = - xx[34];
  xx[152] = xx[36];
  xx[153] = xx[38];
  pm_math_Vector3_cross_ra(xx + 31, xx + 151, xx + 154);
  xx[151] = xx[1] * (xx[154] - xx[34] * xx[30]);
  xx[152] = xx[91] + xx[1] * (xx[36] * xx[30] + xx[155]);
  xx[153] = xx[1] * (xx[38] * xx[30] + xx[156]) - xx[40];
  xx[34] = xx[90] + xx[13];
  xx[13] = xx[3] * xx[31];
  xx[36] = xx[94] * xx[31];
  xx[90] = - xx[34];
  xx[91] = xx[13];
  xx[92] = xx[36];
  pm_math_Vector3_cross_ra(xx + 31, xx + 90, xx + 154);
  xx[90] = xx[1] * (xx[154] - xx[34] * xx[30]);
  xx[91] = xx[94] + xx[1] * (xx[13] * xx[30] + xx[155]);
  xx[92] = xx[1] * (xx[36] * xx[30] + xx[156]) - xx[3];
  xx[154] = xx[1] * (xx[43] - xx[42]);
  xx[155] = xx[119] - xx[2];
  xx[156] = - xx[41];
  xx[41] = xx[1] * (xx[111] + xx[31] * xx[31]) - xx[2];
  xx[42] = xx[1] * (xx[109] + xx[110]);
  xx[43] = xx[1] * (xx[35] - xx[89]);
  xx[34] = xx[2] - xx[93];
  xx[35] = xx[95];
  xx[36] = xx[50];
  xx[13] = xx[10] * xx[15];
  xx[38] = xx[13] * xx[33];
  xx[39] = xx[10] * xx[51];
  xx[48] = xx[39] * xx[33];
  xx[49] = xx[13] * xx[31] + xx[39] * xx[32];
  xx[93] = - xx[38];
  xx[94] = - xx[48];
  xx[95] = xx[49];
  pm_math_Vector3_cross_ra(xx + 31, xx + 93, xx + 109);
  xx[50] = xx[44] * xx[7];
  xx[51] = xx[44] * xx[12];
  xx[53] = xx[1] * (xx[50] * xx[11] - xx[51] * xx[4]);
  xx[89] = xx[53] * xx[59];
  xx[93] = xx[1] * (xx[50] * xx[7] + xx[51] * xx[12]);
  xx[94] = xx[93] - xx[44] - xx[2];
  xx[95] = xx[94] * xx[60];
  xx[99] = xx[89] + xx[95];
  xx[100] = xx[53] * xx[58];
  xx[104] = xx[94] * xx[58];
  xx[117] = - xx[99];
  xx[118] = xx[100];
  xx[119] = xx[104];
  pm_math_Vector3_cross_ra(xx + 16, xx + 117, xx + 157);
  xx[105] = xx[57] * xx[60];
  xx[112] = xx[58] * xx[59];
  xx[113] = xx[10] * xx[87];
  xx[87] = xx[113] * xx[33];
  xx[117] = xx[10] * xx[40];
  xx[40] = xx[117] * xx[33];
  xx[118] = xx[113] * xx[31] + xx[117] * xx[32];
  xx[160] = - xx[87];
  xx[161] = - xx[40];
  xx[162] = xx[118];
  pm_math_Vector3_cross_ra(xx + 31, xx + 160, xx + 163);
  xx[119] = xx[1] * (xx[51] * xx[11] + xx[50] * xx[4]);
  xx[50] = xx[119] * xx[59];
  xx[51] = xx[119] * xx[58];
  xx[160] = xx[95] - xx[51];
  xx[95] = xx[94] * xx[59];
  xx[166] = - xx[50];
  xx[167] = - xx[160];
  xx[168] = xx[95];
  pm_math_Vector3_cross_ra(xx + 16, xx + 166, xx + 169);
  xx[161] = xx[60] * xx[60];
  xx[162] = xx[10] * xx[88];
  xx[88] = xx[162] * xx[33];
  xx[166] = xx[10] * xx[3];
  xx[3] = xx[166] * xx[33];
  xx[167] = xx[162] * xx[31] + xx[166] * xx[32];
  xx[172] = - xx[88];
  xx[173] = - xx[3];
  xx[174] = xx[167];
  pm_math_Vector3_cross_ra(xx + 31, xx + 172, xx + 175);
  xx[168] = xx[119] * xx[60];
  xx[172] = xx[53] * xx[60];
  xx[173] = xx[89] - xx[51];
  xx[178] = - xx[168];
  xx[179] = xx[172];
  xx[180] = - xx[173];
  pm_math_Vector3_cross_ra(xx + 16, xx + 178, xx + 181);
  xx[51] = xx[52] * xx[77];
  xx[89] = xx[51] * xx[74];
  xx[174] = xx[52] * xx[75];
  xx[178] = xx[52] * xx[76];
  xx[179] = xx[51] * xx[77];
  xx[184] = xx[0];
  xx[185] = xx[0];
  xx[186] = xx[0];
  xx[187] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 45, xx + 84);
  xx[188] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 96, xx + 84);
  xx[189] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 101, xx + 84);
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
  xx[217] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 154, xx + 41);
  xx[218] = bb[0] ? xx[0] : pm_math_Vector3_dot_ra(xx + 34, xx + 41);
  xx[219] = xx[0];
  xx[220] = xx[9];
  xx[221] = xx[0];
  xx[222] = xx[0];
  xx[223] = bb[0] ? xx[0] : - (xx[1] * (xx[109] - xx[38] * xx[30]) - xx[39] +
    xx[1] * (xx[99] * xx[57] + xx[157]) + xx[1] * (xx[105] - xx[112]));
  xx[224] = bb[0] ? xx[0] : - (xx[1] * (xx[163] - xx[87] * xx[30]) - xx[117] +
    xx[94] + xx[1] * (xx[169] + xx[50] * xx[57]) - xx[1] * (xx[59] * xx[59] +
    xx[161]) + xx[2]);
  xx[225] = bb[0] ? xx[0] : - (xx[1] * (xx[175] - xx[88] * xx[30]) - xx[166] +
    xx[1] * (xx[181] + xx[168] * xx[57]) - xx[53]);
  xx[226] = xx[6];
  xx[227] = xx[0];
  xx[228] = xx[0];
  xx[229] = bb[0] ? xx[0] : xx[1] * (xx[89] - xx[174] * xx[76]);
  xx[230] = bb[0] ? xx[0] : xx[52] - xx[1] * (xx[178] * xx[76] + xx[179]);
  xx[231] = xx[0];
  xx[232] = xx[0];
  xx[233] = xx[9];
  xx[234] = xx[0];
  xx[235] = bb[0] ? xx[0] : - (xx[1] * (xx[110] - xx[48] * xx[30]) + xx[1] *
    (xx[158] - xx[100] * xx[57]) + xx[1] * (xx[161] + xx[58] * xx[58]) - xx[94]
    + xx[13] - xx[2]);
  xx[236] = bb[0] ? xx[0] : - (xx[1] * (xx[164] - xx[40] * xx[30]) + xx[113] +
    xx[1] * (xx[160] * xx[57] + xx[170]) + xx[1] * (xx[105] + xx[112]));
  xx[237] = bb[0] ? xx[0] : - (xx[1] * (xx[176] - xx[3] * xx[30]) + xx[162] +
    xx[1] * (xx[182] - xx[172] * xx[57]) - xx[119]);
  xx[238] = xx[0];
  xx[239] = xx[6];
  xx[240] = xx[0];
  xx[241] = bb[0] ? xx[0] : xx[1] * (xx[179] + xx[174] * xx[75]) - xx[52];
  xx[242] = bb[0] ? xx[0] : xx[1] * (xx[89] + xx[178] * xx[75]);
  xx[243] = xx[0];
  xx[244] = xx[0];
  xx[245] = xx[0];
  xx[246] = xx[9];
  xx[247] = bb[0] ? xx[0] : - (xx[1] * (xx[111] + xx[49] * xx[30]) + xx[53] +
    xx[1] * (xx[159] - xx[104] * xx[57]) - xx[1] * (xx[57] * xx[58] + xx[59] *
    xx[60]));
  xx[248] = bb[0] ? xx[0] : - (xx[1] * (xx[165] + xx[118] * xx[30]) + xx[1] *
    (xx[171] - xx[95] * xx[57]) + xx[119] + xx[1] * (xx[58] * xx[60] - xx[57] *
    xx[59]));
  xx[249] = bb[0] ? xx[0] : - (xx[1] * (xx[177] + xx[167] * xx[30]) + xx[1] *
    (xx[173] * xx[57] + xx[183]));
  xx[250] = xx[0];
  xx[251] = xx[0];
  xx[252] = xx[6];
  xx[253] = bb[0] ? xx[0] : - (xx[1] * (xx[174] * xx[74] + xx[51] * xx[76]));
  xx[254] = bb[0] ? xx[0] : xx[1] * (xx[51] * xx[75] - xx[178] * xx[74]);
  xx[255] = xx[0];
  xx[34] = - xx[75];
  xx[35] = - xx[76];
  xx[36] = - xx[77];
  xx[0] = xx[76] * xx[82];
  xx[2] = xx[77] * xx[83];
  xx[3] = xx[0] + xx[2];
  xx[6] = xx[75] * xx[82];
  xx[9] = xx[75] * xx[83];
  xx[38] = xx[3];
  xx[39] = - xx[6];
  xx[40] = - xx[9];
  pm_math_Vector3_cross_ra(xx + 34, xx + 38, xx + 45);
  xx[38] = xx[1] * (xx[45] - xx[3] * xx[74]);
  xx[39] = xx[83] + xx[1] * (xx[6] * xx[74] + xx[46]);
  xx[40] = xx[1] * (xx[9] * xx[74] + xx[47]) - xx[82];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 64, xx + 45);
  xx[3] = xx[15] * xx[69];
  xx[6] = xx[45] + xx[3];
  xx[9] = xx[6] * xx[32];
  xx[13] = xx[1] * xx[37] * xx[69];
  xx[15] = xx[47] + xx[13];
  xx[37] = xx[15] * xx[33];
  xx[48] = xx[6] * xx[31];
  xx[49] = xx[37] + xx[48];
  xx[50] = xx[15] * xx[32];
  xx[87] = xx[9];
  xx[88] = - xx[49];
  xx[89] = xx[50];
  pm_math_Vector3_cross_ra(xx + 31, xx + 87, xx + 90);
  xx[87] = xx[1] * (xx[9] * xx[30] + xx[90]) - xx[15];
  xx[88] = xx[1] * (xx[91] - xx[49] * xx[30]);
  xx[89] = xx[6] + xx[1] * (xx[50] * xx[30] + xx[92]);
  xx[9] = xx[76] * xx[81];
  xx[49] = xx[75] * xx[81];
  xx[50] = xx[2] + xx[49];
  xx[2] = xx[76] * xx[83];
  xx[90] = - xx[9];
  xx[91] = xx[50];
  xx[92] = - xx[2];
  pm_math_Vector3_cross_ra(xx + 34, xx + 90, xx + 95);
  xx[90] = xx[1] * (xx[9] * xx[74] + xx[95]) - xx[83];
  xx[91] = xx[1] * (xx[96] - xx[50] * xx[74]);
  xx[92] = xx[81] + xx[1] * (xx[2] * xx[74] + xx[97]);
  xx[2] = xx[1] * xx[123] * xx[69] + xx[70];
  xx[9] = xx[46] + xx[2];
  xx[45] = xx[6] * xx[33];
  xx[46] = xx[9] * xx[33];
  xx[47] = xx[9] * xx[32];
  xx[50] = xx[48] + xx[47];
  xx[95] = xx[45];
  xx[96] = xx[46];
  xx[97] = - xx[50];
  pm_math_Vector3_cross_ra(xx + 31, xx + 95, xx + 98);
  xx[95] = xx[9] + xx[1] * (xx[45] * xx[30] + xx[98]);
  xx[96] = xx[1] * (xx[46] * xx[30] + xx[99]) - xx[6];
  xx[97] = xx[1] * (xx[100] - xx[50] * xx[30]);
  xx[45] = xx[77] * xx[81];
  xx[46] = xx[77] * xx[82];
  xx[48] = xx[49] + xx[0];
  xx[49] = - xx[45];
  xx[50] = - xx[46];
  xx[51] = xx[48];
  pm_math_Vector3_cross_ra(xx + 34, xx + 49, xx + 98);
  xx[49] = xx[82] + xx[1] * (xx[45] * xx[74] + xx[98]);
  xx[50] = xx[1] * (xx[46] * xx[74] + xx[99]) - xx[81];
  xx[51] = xx[1] * (xx[100] - xx[48] * xx[74]);
  xx[0] = xx[47] + xx[37];
  xx[37] = xx[9] * xx[31];
  xx[45] = xx[15] * xx[31];
  xx[46] = - xx[0];
  xx[47] = xx[37];
  xx[48] = xx[45];
  pm_math_Vector3_cross_ra(xx + 31, xx + 46, xx + 98);
  xx[46] = xx[1] * (xx[98] - xx[0] * xx[30]);
  xx[47] = xx[15] + xx[1] * (xx[37] * xx[30] + xx[99]);
  xx[48] = xx[1] * (xx[45] * xx[30] + xx[100]) - xx[9];
  xx[0] = xx[52] * xx[82];
  xx[15] = xx[52] * xx[81];
  xx[37] = xx[15] * xx[77];
  xx[45] = xx[0] * xx[77];
  xx[98] = xx[15] * xx[75] + xx[0] * xx[76];
  xx[99] = - xx[37];
  xx[100] = - xx[45];
  xx[101] = xx[98];
  pm_math_Vector3_cross_ra(xx + 34, xx + 99, xx + 102);
  xx[99] = xx[10] * xx[6];
  xx[6] = xx[99] * xx[33];
  xx[100] = xx[10] * xx[9];
  xx[9] = xx[100] * xx[33];
  xx[101] = xx[99] * xx[31] + xx[100] * xx[32];
  xx[105] = - xx[6];
  xx[106] = - xx[9];
  xx[107] = xx[101];
  pm_math_Vector3_cross_ra(xx + 31, xx + 105, xx + 108);
  xx[105] = xx[60] * xx[64];
  xx[106] = xx[60] * xx[65];
  xx[107] = xx[58] * xx[64] + xx[59] * xx[65];
  xx[111] = - xx[105];
  xx[112] = - xx[106];
  xx[113] = xx[107];
  pm_math_Vector3_cross_ra(xx + 16, xx + 111, xx + 120);
  xx[111] = xx[14];
  xx[112] = xx[23];
  xx[113] = xx[24];
  xx[14] = xx[44] * xx[3];
  xx[23] = xx[14] * xx[4];
  xx[24] = xx[44] * xx[2];
  xx[44] = xx[24] * xx[4];
  xx[4] = xx[14] * xx[7] + xx[24] * xx[12];
  xx[123] = xx[23];
  xx[124] = xx[44];
  xx[125] = - xx[4];
  pm_math_Vector3_cross_ra(xx + 111, xx + 123, xx + 126);
  xx[7] = xx[1] * (xx[126] - xx[23] * xx[11]) - xx[24];
  xx[12] = - xx[119];
  xx[111] = xx[12];
  xx[112] = xx[53];
  xx[113] = xx[94];
  pm_math_Vector3_cross_ra(xx + 64, xx + 111, xx + 117);
  xx[23] = xx[1] * (xx[127] - xx[44] * xx[11]) + xx[14];
  xx[14] = xx[1] * (xx[128] + xx[11] * xx[4]);
  xx[111] = xx[7] + xx[117];
  xx[112] = xx[23] + xx[118];
  xx[113] = xx[14] + xx[119];
  pm_math_Quaternion_xform_ra(xx + 19, xx + 111, xx + 117);
  xx[123] = pm_math_Vector3_dot_ra(xx + 38, xx + 114) + pm_math_Vector3_dot_ra
    (xx + 87, xx + 84);
  xx[124] = pm_math_Vector3_dot_ra(xx + 90, xx + 142) + pm_math_Vector3_dot_ra
    (xx + 95, xx + 130);
  xx[125] = pm_math_Vector3_dot_ra(xx + 49, xx + 41) + pm_math_Vector3_dot_ra(xx
    + 46, xx + 148);
  xx[126] = xx[0] + xx[1] * (xx[37] * xx[74] + xx[102]) + xx[78] - (xx[1] * (xx
    [108] - xx[6] * xx[30]) - xx[100] + xx[65] + xx[1] * (xx[105] * xx[57] + xx
    [120]) + xx[61] + xx[117]);
  xx[127] = xx[1] * (xx[45] * xx[74] + xx[103]) - xx[15] + xx[79] - (xx[1] *
    (xx[109] - xx[9] * xx[30]) + xx[99] + xx[1] * (xx[106] * xx[57] + xx[121]) -
    xx[64] + xx[62] + xx[118]);
  xx[128] = xx[1] * (xx[104] - xx[98] * xx[74]) + xx[80] - (xx[1] * (xx[110] +
    xx[101] * xx[30]) + xx[1] * (xx[122] - xx[107] * xx[57]) + xx[63] + xx[119]);
  zeroMajor(1, 6, ii + 0, xx + 123);
  xx[44] = - xx[123];
  xx[45] = - xx[124];
  xx[46] = - xx[125];
  xx[47] = - xx[126];
  xx[48] = - xx[127];
  xx[49] = - xx[128];
  memcpy(xx + 256, xx + 184, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 256, xx + 87, xx + 106, ii + 1, xx + 44, xx[8],
                     xx + 94);
  xx[0] = xx[61] + xx[94];
  xx[4] = xx[62] + xx[95];
  xx[6] = xx[63] + xx[96];
  xx[8] = xx[64] + xx[97];
  xx[9] = xx[65] + xx[98];
  xx[11] = xx[66] + xx[99];
  xx[15] = xx[78] + xx[100];
  xx[24] = xx[79] + xx[101];
  xx[37] = xx[80] + xx[102];
  xx[38] = xx[81] + xx[103];
  xx[39] = xx[82] + xx[104];
  xx[40] = xx[83] + xx[105];
  xx[151] = xx[54];
  xx[152] = xx[55];
  xx[153] = xx[56];
  xx[154] = xx[57];
  xx[155] = xx[58];
  xx[156] = xx[59];
  xx[157] = xx[60];
  xx[158] = xx[0];
  xx[159] = xx[4];
  xx[160] = xx[6];
  xx[161] = xx[8];
  xx[162] = xx[9];
  xx[163] = xx[11];
  xx[164] = xx[67];
  xx[165] = xx[68];
  xx[166] = xx[69];
  xx[167] = xx[70];
  xx[168] = xx[71];
  xx[169] = xx[72];
  xx[170] = xx[73];
  xx[171] = xx[74];
  xx[172] = xx[75];
  xx[173] = xx[76];
  xx[174] = xx[77];
  xx[175] = xx[15];
  xx[176] = xx[24];
  xx[177] = xx[37];
  xx[178] = xx[38];
  xx[179] = xx[39];
  xx[180] = xx[40];
  xx[44] = xx[39] * xx[76];
  xx[45] = xx[40] * xx[77];
  xx[46] = xx[44] + xx[45];
  xx[47] = xx[39] * xx[75];
  xx[48] = xx[40] * xx[75];
  xx[49] = xx[46];
  xx[50] = - xx[47];
  xx[51] = - xx[48];
  pm_math_Vector3_cross_ra(xx + 34, xx + 49, xx + 54);
  xx[49] = xx[1] * (xx[54] - xx[46] * xx[74]);
  xx[50] = xx[40] + xx[1] * (xx[47] * xx[74] + xx[55]);
  xx[51] = xx[1] * (xx[48] * xx[74] + xx[56]) - xx[39];
  xx[46] = xx[8];
  xx[47] = xx[9];
  xx[48] = xx[11];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 46, xx + 54);
  xx[11] = xx[54] + xx[3];
  xx[3] = xx[11] * xx[32];
  xx[26] = xx[56] + xx[13];
  xx[13] = xx[26] * xx[33];
  xx[27] = xx[11] * xx[31];
  xx[28] = xx[13] + xx[27];
  xx[29] = xx[26] * xx[32];
  xx[61] = xx[3];
  xx[62] = - xx[28];
  xx[63] = xx[29];
  pm_math_Vector3_cross_ra(xx + 31, xx + 61, xx + 64);
  xx[61] = xx[1] * (xx[3] * xx[30] + xx[64]) - xx[26];
  xx[62] = xx[1] * (xx[65] - xx[28] * xx[30]);
  xx[63] = xx[11] + xx[1] * (xx[29] * xx[30] + xx[66]);
  xx[3] = xx[38] * xx[76];
  xx[28] = xx[38] * xx[75];
  xx[29] = xx[45] + xx[28];
  xx[45] = xx[40] * xx[76];
  xx[64] = - xx[3];
  xx[65] = xx[29];
  xx[66] = - xx[45];
  pm_math_Vector3_cross_ra(xx + 34, xx + 64, xx + 67);
  xx[64] = xx[1] * (xx[3] * xx[74] + xx[67]) - xx[40];
  xx[65] = xx[1] * (xx[68] - xx[29] * xx[74]);
  xx[66] = xx[38] + xx[1] * (xx[45] * xx[74] + xx[69]);
  xx[3] = xx[55] + xx[2];
  xx[2] = xx[11] * xx[33];
  xx[29] = xx[3] * xx[33];
  xx[40] = xx[3] * xx[32];
  xx[45] = xx[27] + xx[40];
  xx[54] = xx[2];
  xx[55] = xx[29];
  xx[56] = - xx[45];
  pm_math_Vector3_cross_ra(xx + 31, xx + 54, xx + 67);
  xx[54] = xx[3] + xx[1] * (xx[2] * xx[30] + xx[67]);
  xx[55] = xx[1] * (xx[29] * xx[30] + xx[68]) - xx[11];
  xx[56] = xx[1] * (xx[69] - xx[45] * xx[30]);
  xx[2] = xx[38] * xx[77];
  xx[27] = xx[39] * xx[77];
  xx[29] = xx[28] + xx[44];
  xx[67] = - xx[2];
  xx[68] = - xx[27];
  xx[69] = xx[29];
  pm_math_Vector3_cross_ra(xx + 34, xx + 67, xx + 70);
  xx[67] = xx[39] + xx[1] * (xx[2] * xx[74] + xx[70]);
  xx[68] = xx[1] * (xx[27] * xx[74] + xx[71]) - xx[38];
  xx[69] = xx[1] * (xx[72] - xx[29] * xx[74]);
  xx[2] = xx[40] + xx[13];
  xx[13] = xx[3] * xx[31];
  xx[27] = xx[26] * xx[31];
  xx[70] = - xx[2];
  xx[71] = xx[13];
  xx[72] = xx[27];
  pm_math_Vector3_cross_ra(xx + 31, xx + 70, xx + 78);
  xx[70] = xx[1] * (xx[78] - xx[2] * xx[30]);
  xx[71] = xx[26] + xx[1] * (xx[13] * xx[30] + xx[79]);
  xx[72] = xx[1] * (xx[27] * xx[30] + xx[80]) - xx[3];
  xx[2] = xx[52] * xx[39];
  xx[13] = xx[52] * xx[38];
  xx[26] = xx[13] * xx[77];
  xx[27] = xx[2] * xx[77];
  xx[28] = xx[13] * xx[75] + xx[2] * xx[76];
  xx[38] = - xx[26];
  xx[39] = - xx[27];
  xx[40] = xx[28];
  pm_math_Vector3_cross_ra(xx + 34, xx + 38, xx + 75);
  xx[29] = xx[10] * xx[11];
  xx[11] = xx[29] * xx[33];
  xx[34] = xx[10] * xx[3];
  xx[3] = xx[34] * xx[33];
  xx[10] = xx[29] * xx[31] + xx[34] * xx[32];
  xx[38] = - xx[11];
  xx[39] = - xx[3];
  xx[40] = xx[10];
  pm_math_Vector3_cross_ra(xx + 31, xx + 38, xx + 78);
  xx[31] = xx[8] * xx[60];
  xx[32] = xx[9] * xx[60];
  xx[33] = xx[8] * xx[58] + xx[9] * xx[59];
  xx[38] = - xx[31];
  xx[39] = - xx[32];
  xx[40] = xx[33];
  pm_math_Vector3_cross_ra(xx + 16, xx + 38, xx + 58);
  xx[16] = xx[12];
  xx[17] = xx[53];
  xx[18] = xx[93] - xx[25];
  pm_math_Vector3_cross_ra(xx + 46, xx + 16, xx + 38);
  xx[16] = xx[7] + xx[38];
  xx[17] = xx[23] + xx[39];
  xx[18] = xx[14] + xx[40];
  pm_math_Quaternion_xform_ra(xx + 19, xx + 16, xx + 38);
  xx[16] = pm_math_Vector3_dot_ra(xx + 49, xx + 114) + pm_math_Vector3_dot_ra(xx
    + 61, xx + 84);
  xx[17] = pm_math_Vector3_dot_ra(xx + 64, xx + 142) + pm_math_Vector3_dot_ra(xx
    + 54, xx + 130);
  xx[18] = pm_math_Vector3_dot_ra(xx + 67, xx + 41) + pm_math_Vector3_dot_ra(xx
    + 70, xx + 148);
  xx[19] = xx[2] + xx[1] * (xx[26] * xx[74] + xx[75]) + xx[15] - (xx[1] * (xx[78]
    - xx[11] * xx[30]) - xx[34] + xx[9] + xx[1] * (xx[31] * xx[57] + xx[58]) +
    xx[0] + xx[38]);
  xx[20] = xx[1] * (xx[27] * xx[74] + xx[76]) - xx[13] + xx[24] - (xx[1] * (xx
    [79] - xx[3] * xx[30]) + xx[29] + xx[1] * (xx[32] * xx[57] + xx[59]) - xx[8]
    + xx[4] + xx[39]);
  xx[21] = xx[1] * (xx[77] - xx[28] * xx[74]) + xx[37] - (xx[1] * (xx[80] + xx
    [10] * xx[30]) + xx[1] * (xx[60] - xx[33] * xx[57]) + xx[6] + xx[40]);
  zeroMajor(1, 6, ii + 0, xx + 16);
  xx[6] = fabs(xx[16]);
  xx[7] = fabs(xx[17]);
  xx[8] = fabs(xx[18]);
  xx[9] = fabs(xx[19]);
  xx[10] = fabs(xx[20]);
  xx[11] = fabs(xx[21]);
  ii[0] = 6;

  {
    int ll;
    for (ll = 7; ll < 12; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 6;
  xx[0] = xx[6 + (ii[0])];
  if (xx[0] > xx[5]) {
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
  xx[0] = 2.0;
  xx[1] = state[20] * state[20];
  xx[2] = 1.0;
  xx[3] = state[21] * state[22];
  xx[4] = state[20] * state[23];
  xx[5] = state[21] * state[23];
  xx[6] = state[20] * state[22];
  xx[7] = xx[0] * (xx[1] + state[21] * state[21]) - xx[2];
  xx[8] = xx[0] * (xx[3] + xx[4]);
  xx[9] = xx[0] * (xx[5] - xx[6]);
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
  xx[27] = xx[0] * (xx[14] - xx[19]);
  xx[28] = xx[0] * (xx[20] + xx[25] * xx[25]) - xx[2];
  xx[29] = xx[0] * (xx[21] + xx[22]);
  xx[30] = state[22] * state[23];
  xx[31] = state[20] * state[21];
  xx[32] = xx[0] * (xx[3] - xx[4]);
  xx[33] = xx[0] * (xx[1] + state[22] * state[22]) - xx[2];
  xx[34] = xx[0] * (xx[30] + xx[31]);
  xx[3] = xx[24] * xx[26];
  xx[4] = xx[23] * xx[25];
  xx[35] = xx[0] * (xx[3] + xx[4]);
  xx[36] = xx[0] * (xx[21] - xx[22]);
  xx[37] = xx[0] * (xx[20] + xx[26] * xx[26]) - xx[2];
  xx[38] = xx[0] * (xx[5] + xx[6]);
  xx[39] = xx[0] * (xx[30] - xx[31]);
  xx[40] = xx[0] * (xx[1] + state[23] * state[23]) - xx[2];
  xx[41] = xx[0] * (xx[20] + xx[24] * xx[24]) - xx[2];
  xx[42] = xx[0] * (xx[14] + xx[19]);
  xx[43] = xx[0] * (xx[3] - xx[4]);
  xx[1] = 0.0225;
  xx[2] = xx[1] * state[22];
  xx[3] = xx[1] * state[21];
  xx[1] = 0.04499999999999999;
  xx[4] = xx[1] * xx[17];
  xx[5] = xx[1] * xx[15];
  xx[19] = - (xx[0] * (xx[4] * xx[18] + xx[5] * xx[16]));
  xx[20] = xx[0] * (xx[5] * xx[18] - xx[4] * xx[16]);
  xx[21] = xx[0] * (xx[5] * xx[15] + xx[4] * xx[17]) - 1.045;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 19, xx + 4);
  xx[1] = 0.04500000000000001;
  xx[10] = xx[1] * xx[25];
  xx[11] = xx[1] * xx[24];
  xx[12] = pm_math_Vector3_dot_ra(xx + 7, xx + 27);
  xx[13] = pm_math_Vector3_dot_ra(xx + 32, xx + 35);
  xx[14] = pm_math_Vector3_dot_ra(xx + 38, xx + 41);
  xx[15] = xx[0] * (xx[2] * state[20] + xx[3] * state[23]) + state[17] - (xx[4]
    + state[0] + xx[0] * (state[3] * state[5] + state[4] * state[6]) - xx[0] *
    (xx[10] * xx[23] + xx[11] * xx[26]));
  xx[16] = xx[0] * (xx[2] * state[23] - xx[3] * state[20]) + state[18] - (xx[0] *
    (xx[11] * xx[23] - xx[10] * xx[26]) + xx[5] + state[1] - xx[0] * (state[3] *
    state[4] - state[5] * state[6]));
  xx[17] = state[19] - xx[0] * (xx[3] * state[21] + xx[2] * state[22]) - (xx[6]
    + state[2] - xx[0] * (state[4] * state[4] + state[5] * state[5]) + xx[0] *
    (xx[11] * xx[24] + xx[10] * xx[25])) - 0.9325;
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
  xx[1] = 2.0;
  xx[2] = 1.0;
  xx[3] = 0.5;
  xx[4] = xx[3] * state[13];
  xx[5] = cos(xx[4]);
  xx[6] = xx[3] * state[14];
  xx[7] = sin(xx[6]);
  xx[8] = xx[5] * xx[7];
  xx[9] = xx[8] * xx[8];
  xx[10] = sin(xx[4]);
  xx[4] = xx[10] * xx[7];
  xx[11] = xx[4] * xx[4];
  xx[12] = xx[2] - xx[1] * (xx[9] + xx[11]);
  xx[13] = sqrt(state[3] * state[3] + state[4] * state[4] + state[5] * state[5]
                + state[6] * state[6]);
  xx[14] = state[3] / xx[13];
  xx[15] = state[4] / xx[13];
  xx[16] = - xx[15];
  xx[17] = state[5] / xx[13];
  xx[18] = - xx[17];
  xx[19] = state[6] / xx[13];
  xx[13] = - xx[19];
  xx[20] = - xx[14];
  xx[21] = xx[16];
  xx[22] = xx[18];
  xx[23] = xx[13];
  xx[24] = cos(xx[6]);
  xx[6] = xx[5] * xx[24];
  xx[5] = xx[24] * xx[10];
  xx[25] = - xx[6];
  xx[26] = - xx[5];
  xx[27] = - xx[8];
  xx[28] = - xx[4];
  pm_math_Quaternion_compose_ra(xx + 20, xx + 25, xx + 29);
  xx[10] = xx[12] * xx[31];
  xx[33] = xx[6] * xx[8];
  xx[34] = xx[5] * xx[4];
  xx[35] = xx[1] * (xx[33] + xx[34]);
  xx[36] = xx[35] * xx[32];
  xx[37] = xx[12] * xx[30];
  xx[38] = xx[36] + xx[37];
  xx[39] = xx[35] * xx[31];
  xx[40] = xx[10];
  xx[41] = - xx[38];
  xx[42] = xx[39];
  pm_math_Vector3_cross_ra(xx + 30, xx + 40, xx + 43);
  xx[40] = xx[1] * (xx[10] * xx[29] + xx[43]) - xx[35];
  xx[41] = xx[1] * (xx[44] - xx[38] * xx[29]);
  xx[42] = xx[12] + xx[1] * (xx[39] * xx[29] + xx[45]);
  xx[10] = sqrt(state[20] * state[20] + state[21] * state[21] + state[22] *
                state[22] + state[23] * state[23]);
  xx[38] = state[20] / xx[10];
  xx[39] = xx[38] * xx[38];
  xx[43] = state[21] / xx[10];
  xx[44] = xx[43] * xx[43];
  xx[45] = state[22] / xx[10];
  xx[46] = xx[43] * xx[45];
  xx[47] = state[23] / xx[10];
  xx[10] = xx[38] * xx[47];
  xx[48] = xx[43] * xx[47];
  xx[49] = xx[38] * xx[45];
  xx[50] = xx[1] * (xx[48] - xx[49]);
  xx[51] = xx[1] * (xx[39] + xx[44]) - xx[2];
  xx[52] = xx[1] * (xx[46] + xx[10]);
  xx[53] = xx[50];
  xx[54] = xx[6] * xx[4];
  xx[55] = xx[8] * xx[5];
  xx[56] = xx[1] * (xx[54] + xx[55]);
  xx[57] = xx[56] * xx[31];
  xx[58] = xx[8] * xx[4];
  xx[59] = xx[6] * xx[5];
  xx[60] = xx[1] * (xx[58] - xx[59]);
  xx[61] = xx[60] * xx[32];
  xx[62] = xx[56] * xx[30];
  xx[63] = xx[61] + xx[62];
  xx[64] = xx[60] * xx[31];
  xx[65] = xx[57];
  xx[66] = - xx[63];
  xx[67] = xx[64];
  pm_math_Vector3_cross_ra(xx + 30, xx + 65, xx + 68);
  xx[65] = xx[1] * (xx[57] * xx[29] + xx[68]) - xx[60];
  xx[66] = xx[1] * (xx[69] - xx[63] * xx[29]);
  xx[67] = xx[56] + xx[1] * (xx[64] * xx[29] + xx[70]);
  xx[57] = xx[1] * (xx[34] - xx[33]);
  xx[63] = xx[57] * xx[31];
  xx[64] = xx[5] * xx[5];
  xx[68] = xx[2] - xx[1] * (xx[64] + xx[9]);
  xx[69] = xx[68] * xx[32];
  xx[70] = xx[57] * xx[30];
  xx[71] = xx[69] + xx[70];
  xx[72] = xx[68] * xx[31];
  xx[73] = xx[63];
  xx[74] = - xx[71];
  xx[75] = xx[72];
  pm_math_Vector3_cross_ra(xx + 30, xx + 73, xx + 76);
  xx[73] = xx[1] * (xx[63] * xx[29] + xx[76]) - xx[68];
  xx[74] = xx[1] * (xx[77] - xx[71] * xx[29]);
  xx[75] = xx[57] + xx[1] * (xx[72] * xx[29] + xx[78]);
  xx[63] = xx[1] * (xx[49] + xx[48]);
  xx[71] = xx[38] * xx[43];
  xx[72] = xx[45] * xx[47];
  xx[76] = xx[45] * xx[45];
  xx[77] = xx[1] * (xx[44] + xx[76]);
  xx[78] = - xx[63];
  xx[79] = xx[1] * (xx[71] - xx[72]);
  xx[80] = xx[77] - xx[2];
  xx[81] = xx[30] * xx[31];
  xx[82] = xx[29] * xx[32];
  xx[83] = xx[29] * xx[29];
  xx[84] = xx[31] * xx[32];
  xx[85] = xx[29] * xx[30];
  xx[86] = xx[1] * (xx[81] - xx[82]);
  xx[87] = xx[1] * (xx[83] + xx[31] * xx[31]) - xx[2];
  xx[88] = xx[1] * (xx[84] + xx[85]);
  xx[89] = xx[1] * (xx[46] - xx[10]);
  xx[90] = xx[47] * xx[47];
  xx[91] = xx[1] * (xx[90] + xx[44]);
  xx[44] = xx[1] * (xx[71] + xx[72]);
  xx[92] = xx[89];
  xx[93] = xx[2] - xx[91];
  xx[94] = xx[44];
  xx[95] = xx[1] * (xx[55] - xx[54]);
  xx[96] = xx[12] * xx[32];
  xx[97] = xx[95] * xx[32];
  xx[98] = xx[95] * xx[31];
  xx[99] = xx[37] + xx[98];
  xx[100] = xx[96];
  xx[101] = xx[97];
  xx[102] = - xx[99];
  pm_math_Vector3_cross_ra(xx + 30, xx + 100, xx + 103);
  xx[100] = xx[95] + xx[1] * (xx[96] * xx[29] + xx[103]);
  xx[101] = xx[1] * (xx[97] * xx[29] + xx[104]) - xx[12];
  xx[102] = xx[1] * (xx[105] - xx[99] * xx[29]);
  xx[103] = xx[89];
  xx[104] = xx[1] * (xx[39] + xx[76]) - xx[2];
  xx[105] = xx[1] * (xx[72] + xx[71]);
  xx[37] = xx[2] - xx[1] * (xx[11] + xx[64]);
  xx[89] = xx[56] * xx[32];
  xx[96] = xx[37] * xx[32];
  xx[97] = xx[37] * xx[31];
  xx[99] = xx[62] + xx[97];
  xx[106] = xx[89];
  xx[107] = xx[96];
  xx[108] = - xx[99];
  pm_math_Vector3_cross_ra(xx + 30, xx + 106, xx + 109);
  xx[106] = xx[37] + xx[1] * (xx[89] * xx[29] + xx[109]);
  xx[107] = xx[1] * (xx[96] * xx[29] + xx[110]) - xx[56];
  xx[108] = xx[1] * (xx[111] - xx[99] * xx[29]);
  xx[62] = xx[1] * (xx[59] + xx[58]);
  xx[89] = xx[57] * xx[32];
  xx[96] = xx[62] * xx[32];
  xx[99] = xx[62] * xx[31];
  xx[109] = xx[70] + xx[99];
  xx[110] = xx[89];
  xx[111] = xx[96];
  xx[112] = - xx[109];
  pm_math_Vector3_cross_ra(xx + 30, xx + 110, xx + 113);
  xx[110] = xx[62] + xx[1] * (xx[89] * xx[29] + xx[113]);
  xx[111] = xx[1] * (xx[96] * xx[29] + xx[114]) - xx[57];
  xx[112] = xx[1] * (xx[115] - xx[109] * xx[29]);
  xx[70] = xx[1] * (xx[72] - xx[71]);
  xx[113] = xx[63];
  xx[114] = xx[70];
  xx[115] = xx[2] - xx[77];
  xx[63] = xx[30] * xx[32];
  xx[71] = xx[29] * xx[31];
  xx[116] = xx[1] * (xx[63] + xx[71]);
  xx[117] = xx[1] * (xx[84] - xx[85]);
  xx[118] = xx[1] * (xx[83] + xx[32] * xx[32]) - xx[2];
  xx[72] = xx[1] * (xx[76] + xx[90]);
  xx[76] = xx[1] * (xx[10] + xx[46]);
  xx[119] = xx[72] - xx[2];
  xx[120] = - xx[76];
  xx[121] = xx[1] * (xx[49] - xx[48]);
  xx[77] = xx[98] + xx[36];
  xx[36] = xx[95] * xx[30];
  xx[84] = xx[35] * xx[30];
  xx[122] = - xx[77];
  xx[123] = xx[36];
  xx[124] = xx[84];
  pm_math_Vector3_cross_ra(xx + 30, xx + 122, xx + 125);
  xx[122] = xx[1] * (xx[125] - xx[77] * xx[29]);
  xx[123] = xx[35] + xx[1] * (xx[36] * xx[29] + xx[126]);
  xx[124] = xx[1] * (xx[84] * xx[29] + xx[127]) - xx[95];
  xx[125] = xx[1] * (xx[48] + xx[49]);
  xx[126] = xx[70];
  xx[127] = xx[1] * (xx[39] + xx[90]) - xx[2];
  xx[36] = xx[97] + xx[61];
  xx[39] = xx[37] * xx[30];
  xx[48] = xx[60] * xx[30];
  xx[96] = - xx[36];
  xx[97] = xx[39];
  xx[98] = xx[48];
  pm_math_Vector3_cross_ra(xx + 30, xx + 96, xx + 128);
  xx[96] = xx[1] * (xx[128] - xx[36] * xx[29]);
  xx[97] = xx[60] + xx[1] * (xx[39] * xx[29] + xx[129]);
  xx[98] = xx[1] * (xx[48] * xx[29] + xx[130]) - xx[37];
  xx[36] = xx[99] + xx[69];
  xx[39] = xx[62] * xx[30];
  xx[48] = xx[68] * xx[30];
  xx[128] = - xx[36];
  xx[129] = xx[39];
  xx[130] = xx[48];
  pm_math_Vector3_cross_ra(xx + 30, xx + 128, xx + 131);
  xx[128] = xx[1] * (xx[131] - xx[36] * xx[29]);
  xx[129] = xx[68] + xx[1] * (xx[39] * xx[29] + xx[132]);
  xx[130] = xx[1] * (xx[48] * xx[29] + xx[133]) - xx[62];
  xx[131] = xx[1] * (xx[10] - xx[46]);
  xx[132] = xx[91] - xx[2];
  xx[133] = - xx[44];
  xx[89] = xx[1] * (xx[83] + xx[30] * xx[30]) - xx[2];
  xx[90] = xx[1] * (xx[81] + xx[82]);
  xx[91] = xx[1] * (xx[63] - xx[71]);
  xx[69] = xx[2] - xx[72];
  xx[70] = xx[76];
  xx[71] = xx[50];
  xx[10] = bb[0] ? xx[0] : - xx[2];
  xx[36] = 0.04500000000000001;
  xx[39] = xx[36] * xx[12];
  xx[44] = xx[39] * xx[32];
  xx[46] = xx[36] * xx[95];
  xx[48] = xx[46] * xx[32];
  xx[49] = xx[39] * xx[30] + xx[46] * xx[31];
  xx[81] = - xx[44];
  xx[82] = - xx[48];
  xx[83] = xx[49];
  pm_math_Vector3_cross_ra(xx + 30, xx + 81, xx + 134);
  xx[50] = 0.04499999999999999;
  xx[61] = xx[50] * xx[5];
  xx[63] = xx[50] * xx[8];
  xx[72] = xx[1] * (xx[61] * xx[6] - xx[63] * xx[4]);
  xx[76] = xx[72] * xx[17];
  xx[77] = xx[1] * (xx[61] * xx[5] + xx[63] * xx[8]) - xx[50] - xx[2];
  xx[81] = xx[77] * xx[19];
  xx[82] = xx[76] + xx[81];
  xx[83] = xx[16];
  xx[84] = xx[18];
  xx[85] = xx[13];
  xx[13] = xx[72] * xx[15];
  xx[16] = xx[77] * xx[15];
  xx[137] = - xx[82];
  xx[138] = xx[13];
  xx[139] = xx[16];
  pm_math_Vector3_cross_ra(xx + 83, xx + 137, xx + 140);
  xx[18] = xx[14] * xx[19];
  xx[99] = xx[15] * xx[17];
  xx[109] = xx[36] * xx[56];
  xx[137] = xx[109] * xx[32];
  xx[138] = xx[36] * xx[37];
  xx[139] = xx[138] * xx[32];
  xx[143] = xx[109] * xx[30] + xx[138] * xx[31];
  xx[144] = - xx[137];
  xx[145] = - xx[139];
  xx[146] = xx[143];
  pm_math_Vector3_cross_ra(xx + 30, xx + 144, xx + 147);
  xx[144] = xx[1] * (xx[63] * xx[6] + xx[61] * xx[4]);
  xx[61] = xx[144] * xx[17];
  xx[63] = xx[144] * xx[15];
  xx[145] = xx[81] - xx[63];
  xx[81] = xx[77] * xx[17];
  xx[150] = - xx[61];
  xx[151] = - xx[145];
  xx[152] = xx[81];
  pm_math_Vector3_cross_ra(xx + 83, xx + 150, xx + 153);
  xx[146] = xx[17] * xx[17];
  xx[150] = xx[19] * xx[19];
  xx[151] = xx[36] * xx[57];
  xx[152] = xx[151] * xx[32];
  xx[156] = xx[36] * xx[62];
  xx[157] = xx[156] * xx[32];
  xx[158] = xx[151] * xx[30] + xx[156] * xx[31];
  xx[159] = - xx[152];
  xx[160] = - xx[157];
  xx[161] = xx[158];
  pm_math_Vector3_cross_ra(xx + 30, xx + 159, xx + 162);
  xx[159] = xx[144] * xx[19];
  xx[160] = xx[72] * xx[19];
  xx[161] = xx[76] - xx[63];
  xx[165] = - xx[159];
  xx[166] = xx[160];
  xx[167] = - xx[161];
  pm_math_Vector3_cross_ra(xx + 83, xx + 165, xx + 168);
  xx[63] = bb[0] ? xx[0] : xx[2];
  xx[76] = 0.0225;
  xx[83] = xx[76] * xx[47];
  xx[84] = xx[83] * xx[38];
  xx[85] = xx[76] * xx[43];
  xx[165] = xx[76] * xx[45];
  xx[166] = xx[165] * xx[45];
  xx[167] = xx[83] * xx[47];
  xx[171] = xx[15] * xx[15];
  xx[172] = xx[85] * xx[43];
  xx[173] = xx[14] * xx[15];
  xx[174] = xx[17] * xx[19];
  xx[175] = xx[15] * xx[19];
  xx[176] = xx[14] * xx[17];
  xx[177] = xx[85] * xx[38];
  xx[178] = xx[165] * xx[38];
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
  xx[215] = xx[10];
  xx[216] = xx[0];
  xx[217] = xx[0];
  xx[218] = bb[0] ? xx[0] : - (xx[1] * (xx[134] - xx[44] * xx[29]) - xx[46] +
    xx[1] * (xx[82] * xx[14] + xx[140]) + xx[1] * (xx[18] - xx[99]));
  xx[219] = bb[0] ? xx[0] : - (xx[1] * (xx[147] - xx[137] * xx[29]) - xx[138] +
    xx[77] + xx[1] * (xx[153] + xx[61] * xx[14]) - xx[1] * (xx[146] + xx[150]) +
    xx[2]);
  xx[220] = bb[0] ? xx[0] : - (xx[1] * (xx[162] - xx[152] * xx[29]) - xx[156] +
    xx[1] * (xx[168] + xx[159] * xx[14]) - xx[72]);
  xx[221] = xx[63];
  xx[222] = xx[0];
  xx[223] = xx[0];
  xx[224] = bb[0] ? xx[0] : xx[1] * (xx[84] - xx[85] * xx[45]);
  xx[225] = bb[0] ? xx[0] : xx[76] - xx[1] * (xx[166] + xx[167]);
  xx[226] = xx[0];
  xx[227] = xx[0];
  xx[228] = xx[10];
  xx[229] = xx[0];
  xx[230] = bb[0] ? xx[0] : - (xx[1] * (xx[135] - xx[48] * xx[29]) + xx[1] *
    (xx[141] - xx[13] * xx[14]) + xx[1] * (xx[150] + xx[171]) - xx[77] + xx[39]
    - xx[2]);
  xx[231] = bb[0] ? xx[0] : - (xx[1] * (xx[148] - xx[139] * xx[29]) + xx[109] +
    xx[1] * (xx[145] * xx[14] + xx[154]) + xx[1] * (xx[18] + xx[99]));
  xx[232] = bb[0] ? xx[0] : - (xx[1] * (xx[163] - xx[157] * xx[29]) + xx[151] +
    xx[1] * (xx[169] - xx[160] * xx[14]) - xx[144]);
  xx[233] = xx[0];
  xx[234] = xx[63];
  xx[235] = xx[0];
  xx[236] = bb[0] ? xx[0] : xx[1] * (xx[167] + xx[172]) - xx[76];
  xx[237] = bb[0] ? xx[0] : xx[1] * (xx[84] + xx[165] * xx[43]);
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[0];
  xx[241] = xx[10];
  xx[242] = bb[0] ? xx[0] : - (xx[1] * (xx[136] + xx[49] * xx[29]) + xx[72] +
    xx[1] * (xx[142] - xx[16] * xx[14]) - xx[1] * (xx[173] + xx[174]));
  xx[243] = bb[0] ? xx[0] : - (xx[1] * (xx[149] + xx[143] * xx[29]) + xx[1] *
    (xx[155] - xx[81] * xx[14]) + xx[144] + xx[1] * (xx[175] - xx[176]));
  xx[244] = bb[0] ? xx[0] : - (xx[1] * (xx[164] + xx[158] * xx[29]) + xx[1] *
    (xx[161] * xx[14] + xx[170]));
  xx[245] = xx[0];
  xx[246] = xx[0];
  xx[247] = xx[63];
  xx[248] = bb[0] ? xx[0] : - (xx[1] * (xx[177] + xx[83] * xx[45]));
  xx[249] = bb[0] ? xx[0] : xx[1] * (xx[83] * xx[43] - xx[178]);
  xx[250] = xx[0];
  ii[0] = bb[0] ? 0 : 1;
  xx[40] = - xx[144];
  xx[41] = xx[72];
  xx[42] = xx[77];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 40, xx + 65);
  xx[13] = xx[36] * xx[31];
  xx[16] = xx[36] * xx[30];
  xx[78] = pm_math_Vector3_dot_ra(xx + 51, xx + 86);
  xx[79] = pm_math_Vector3_dot_ra(xx + 103, xx + 116);
  xx[80] = pm_math_Vector3_dot_ra(xx + 125, xx + 89);
  xx[81] = xx[1] * (xx[178] + xx[85] * xx[47]) + state[17] - (xx[65] + state[0]
    + xx[1] * (xx[176] + xx[175]) - xx[1] * (xx[13] * xx[29] + xx[16] * xx[32]));
  xx[82] = xx[1] * (xx[165] * xx[47] - xx[177]) + state[18] - (xx[1] * (xx[16] *
    xx[29] - xx[13] * xx[32]) + xx[66] + state[1] - xx[1] * (xx[173] - xx[174]));
  xx[83] = state[19] - xx[1] * (xx[172] + xx[166]) - (xx[67] + state[2] - xx[1] *
    (xx[171] + xx[146]) + xx[1] * (xx[16] * xx[30] + xx[13] * xx[31])) - 0.9325;
  zeroMajor(1, 6, ii + 0, xx + 78);
  xx[84] = - xx[78];
  xx[85] = - xx[79];
  xx[86] = - xx[80];
  xx[87] = - xx[81];
  xx[88] = - xx[82];
  xx[89] = - xx[83];
  xx[13] = 1.0e-8;
  memcpy(xx + 251, xx + 179, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 251, xx + 78, xx + 110, ii + 1, xx + 84, xx[13],
                     xx + 96);
  xx[16] = state[0] + xx[96];
  xx[20] = xx[14];
  xx[21] = xx[15];
  xx[22] = xx[17];
  xx[23] = xx[19];
  pm_math_Quaternion_compDeriv_ra(xx + 20, xx + 99, xx + 29);
  xx[18] = xx[14] + xx[29];
  xx[14] = xx[15] + xx[30];
  xx[15] = xx[17] + xx[31];
  xx[17] = xx[19] + xx[32];
  xx[19] = 1.0e-64;
  xx[20] = sqrt(xx[18] * xx[18] + xx[14] * xx[14] + xx[15] * xx[15] + xx[17] *
                xx[17]);
  if (xx[19] > xx[20])
    xx[20] = xx[19];
  xx[21] = xx[18] / xx[20];
  xx[18] = xx[14] / xx[20];
  xx[14] = - xx[18];
  xx[22] = xx[15] / xx[20];
  xx[15] = - xx[22];
  xx[23] = xx[17] / xx[20];
  xx[17] = - xx[23];
  xx[29] = - xx[21];
  xx[30] = xx[14];
  xx[31] = xx[15];
  xx[32] = xx[17];
  pm_math_Quaternion_compose_ra(xx + 29, xx + 25, xx + 78);
  xx[20] = xx[12] * xx[80];
  xx[44] = xx[35] * xx[81];
  xx[48] = xx[12] * xx[79];
  xx[49] = xx[44] + xx[48];
  xx[51] = xx[35] * xx[80];
  xx[65] = xx[20];
  xx[66] = - xx[49];
  xx[67] = xx[51];
  pm_math_Vector3_cross_ra(xx + 79, xx + 65, xx + 69);
  xx[65] = xx[1] * (xx[20] * xx[78] + xx[69]) - xx[35];
  xx[66] = xx[1] * (xx[70] - xx[49] * xx[78]);
  xx[67] = xx[12] + xx[1] * (xx[51] * xx[78] + xx[71]);
  xx[82] = xx[38];
  xx[83] = xx[43];
  xx[84] = xx[45];
  xx[85] = xx[47];
  pm_math_Quaternion_compDeriv_ra(xx + 82, xx + 105, xx + 86);
  xx[20] = xx[38] + xx[86];
  xx[38] = xx[43] + xx[87];
  xx[43] = xx[45] + xx[88];
  xx[45] = xx[47] + xx[89];
  xx[47] = sqrt(xx[20] * xx[20] + xx[38] * xx[38] + xx[43] * xx[43] + xx[45] *
                xx[45]);
  if (xx[19] > xx[47])
    xx[47] = xx[19];
  xx[49] = xx[20] / xx[47];
  xx[20] = xx[49] * xx[49];
  xx[51] = xx[38] / xx[47];
  xx[38] = xx[51] * xx[51];
  xx[52] = xx[43] / xx[47];
  xx[43] = xx[51] * xx[52];
  xx[53] = xx[45] / xx[47];
  xx[45] = xx[49] * xx[53];
  xx[47] = xx[51] * xx[53];
  xx[61] = xx[49] * xx[52];
  xx[69] = xx[1] * (xx[47] - xx[61]);
  xx[73] = xx[1] * (xx[20] + xx[38]) - xx[2];
  xx[74] = xx[1] * (xx[43] + xx[45]);
  xx[75] = xx[69];
  xx[70] = xx[56] * xx[80];
  xx[71] = xx[60] * xx[81];
  xx[82] = xx[56] * xx[79];
  xx[83] = xx[71] + xx[82];
  xx[84] = xx[60] * xx[80];
  xx[85] = xx[70];
  xx[86] = - xx[83];
  xx[87] = xx[84];
  pm_math_Vector3_cross_ra(xx + 79, xx + 85, xx + 88);
  xx[85] = xx[1] * (xx[70] * xx[78] + xx[88]) - xx[60];
  xx[86] = xx[1] * (xx[89] - xx[83] * xx[78]);
  xx[87] = xx[56] + xx[1] * (xx[84] * xx[78] + xx[90]);
  xx[70] = xx[57] * xx[80];
  xx[83] = xx[68] * xx[81];
  xx[84] = xx[57] * xx[79];
  xx[88] = xx[83] + xx[84];
  xx[89] = xx[68] * xx[80];
  xx[90] = xx[70];
  xx[91] = - xx[88];
  xx[92] = xx[89];
  pm_math_Vector3_cross_ra(xx + 79, xx + 90, xx + 110);
  xx[90] = xx[1] * (xx[70] * xx[78] + xx[110]) - xx[68];
  xx[91] = xx[1] * (xx[111] - xx[88] * xx[78]);
  xx[92] = xx[57] + xx[1] * (xx[89] * xx[78] + xx[112]);
  xx[70] = xx[1] * (xx[61] + xx[47]);
  xx[88] = xx[49] * xx[51];
  xx[89] = xx[52] * xx[53];
  xx[93] = xx[52] * xx[52];
  xx[94] = xx[1] * (xx[38] + xx[93]);
  xx[110] = - xx[70];
  xx[111] = xx[1] * (xx[88] - xx[89]);
  xx[112] = xx[94] - xx[2];
  xx[108] = xx[79] * xx[80];
  xx[113] = xx[78] * xx[81];
  xx[114] = xx[78] * xx[78];
  xx[115] = xx[80] * xx[81];
  xx[116] = xx[78] * xx[79];
  xx[117] = xx[1] * (xx[108] - xx[113]);
  xx[118] = xx[1] * (xx[114] + xx[80] * xx[80]) - xx[2];
  xx[119] = xx[1] * (xx[115] + xx[116]);
  xx[120] = xx[1] * (xx[43] - xx[45]);
  xx[121] = xx[53] * xx[53];
  xx[122] = xx[1] * (xx[121] + xx[38]);
  xx[38] = xx[1] * (xx[88] + xx[89]);
  xx[123] = xx[120];
  xx[124] = xx[2] - xx[122];
  xx[125] = xx[38];
  xx[126] = xx[12] * xx[81];
  xx[127] = xx[95] * xx[81];
  xx[128] = xx[95] * xx[80];
  xx[129] = xx[48] + xx[128];
  xx[130] = xx[126];
  xx[131] = xx[127];
  xx[132] = - xx[129];
  pm_math_Vector3_cross_ra(xx + 79, xx + 130, xx + 133);
  xx[130] = xx[95] + xx[1] * (xx[126] * xx[78] + xx[133]);
  xx[131] = xx[1] * (xx[127] * xx[78] + xx[134]) - xx[12];
  xx[132] = xx[1] * (xx[135] - xx[129] * xx[78]);
  xx[133] = xx[120];
  xx[134] = xx[1] * (xx[20] + xx[93]) - xx[2];
  xx[135] = xx[1] * (xx[89] + xx[88]);
  xx[12] = xx[56] * xx[81];
  xx[48] = xx[37] * xx[81];
  xx[120] = xx[37] * xx[80];
  xx[126] = xx[82] + xx[120];
  xx[139] = xx[12];
  xx[140] = xx[48];
  xx[141] = - xx[126];
  pm_math_Vector3_cross_ra(xx + 79, xx + 139, xx + 145);
  xx[139] = xx[37] + xx[1] * (xx[12] * xx[78] + xx[145]);
  xx[140] = xx[1] * (xx[48] * xx[78] + xx[146]) - xx[56];
  xx[141] = xx[1] * (xx[147] - xx[126] * xx[78]);
  xx[12] = xx[57] * xx[81];
  xx[48] = xx[62] * xx[81];
  xx[56] = xx[62] * xx[80];
  xx[82] = xx[84] + xx[56];
  xx[145] = xx[12];
  xx[146] = xx[48];
  xx[147] = - xx[82];
  pm_math_Vector3_cross_ra(xx + 79, xx + 145, xx + 148);
  xx[145] = xx[62] + xx[1] * (xx[12] * xx[78] + xx[148]);
  xx[146] = xx[1] * (xx[48] * xx[78] + xx[149]) - xx[57];
  xx[147] = xx[1] * (xx[150] - xx[82] * xx[78]);
  xx[12] = xx[1] * (xx[89] - xx[88]);
  xx[148] = xx[70];
  xx[149] = xx[12];
  xx[150] = xx[2] - xx[94];
  xx[48] = xx[79] * xx[81];
  xx[70] = xx[78] * xx[80];
  xx[152] = xx[1] * (xx[48] + xx[70]);
  xx[153] = xx[1] * (xx[115] - xx[116]);
  xx[154] = xx[1] * (xx[114] + xx[81] * xx[81]) - xx[2];
  xx[82] = xx[1] * (xx[93] + xx[121]);
  xx[84] = xx[1] * (xx[45] + xx[43]);
  xx[157] = xx[82] - xx[2];
  xx[158] = - xx[84];
  xx[159] = xx[1] * (xx[61] - xx[47]);
  xx[88] = xx[128] + xx[44];
  xx[44] = xx[95] * xx[79];
  xx[89] = xx[35] * xx[79];
  xx[126] = - xx[88];
  xx[127] = xx[44];
  xx[128] = xx[89];
  pm_math_Vector3_cross_ra(xx + 79, xx + 126, xx + 160);
  xx[126] = xx[1] * (xx[160] - xx[88] * xx[78]);
  xx[127] = xx[35] + xx[1] * (xx[44] * xx[78] + xx[161]);
  xx[128] = xx[1] * (xx[89] * xx[78] + xx[162]) - xx[95];
  xx[160] = xx[1] * (xx[47] + xx[61]);
  xx[161] = xx[12];
  xx[162] = xx[1] * (xx[20] + xx[121]) - xx[2];
  xx[12] = xx[120] + xx[71];
  xx[20] = xx[37] * xx[79];
  xx[35] = xx[60] * xx[79];
  xx[163] = - xx[12];
  xx[164] = xx[20];
  xx[165] = xx[35];
  pm_math_Vector3_cross_ra(xx + 79, xx + 163, xx + 166);
  xx[163] = xx[1] * (xx[166] - xx[12] * xx[78]);
  xx[164] = xx[60] + xx[1] * (xx[20] * xx[78] + xx[167]);
  xx[165] = xx[1] * (xx[35] * xx[78] + xx[168]) - xx[37];
  xx[12] = xx[56] + xx[83];
  xx[20] = xx[62] * xx[79];
  xx[35] = xx[68] * xx[79];
  xx[166] = - xx[12];
  xx[167] = xx[20];
  xx[168] = xx[35];
  pm_math_Vector3_cross_ra(xx + 79, xx + 166, xx + 169);
  xx[166] = xx[1] * (xx[169] - xx[12] * xx[78]);
  xx[167] = xx[68] + xx[1] * (xx[20] * xx[78] + xx[170]);
  xx[168] = xx[1] * (xx[35] * xx[78] + xx[171]) - xx[62];
  xx[169] = xx[1] * (xx[45] - xx[43]);
  xx[170] = xx[122] - xx[2];
  xx[171] = - xx[38];
  xx[43] = xx[1] * (xx[114] + xx[79] * xx[79]) - xx[2];
  xx[44] = xx[1] * (xx[108] + xx[113]);
  xx[45] = xx[1] * (xx[48] - xx[70]);
  xx[113] = xx[2] - xx[82];
  xx[114] = xx[84];
  xx[115] = xx[69];
  xx[12] = xx[39] * xx[81];
  xx[20] = xx[46] * xx[81];
  xx[35] = xx[39] * xx[79] + xx[46] * xx[80];
  xx[68] = - xx[12];
  xx[69] = - xx[20];
  xx[70] = xx[35];
  pm_math_Vector3_cross_ra(xx + 79, xx + 68, xx + 82);
  xx[37] = xx[72] * xx[22];
  xx[38] = xx[77] * xx[23];
  xx[47] = xx[37] + xx[38];
  xx[68] = xx[14];
  xx[69] = xx[15];
  xx[70] = xx[17];
  xx[14] = xx[72] * xx[18];
  xx[15] = xx[77] * xx[18];
  xx[120] = - xx[47];
  xx[121] = xx[14];
  xx[122] = xx[15];
  pm_math_Vector3_cross_ra(xx + 68, xx + 120, xx + 172);
  xx[17] = xx[21] * xx[23];
  xx[48] = xx[18] * xx[22];
  xx[56] = xx[109] * xx[81];
  xx[61] = xx[138] * xx[81];
  xx[62] = xx[109] * xx[79] + xx[138] * xx[80];
  xx[120] = - xx[56];
  xx[121] = - xx[61];
  xx[122] = xx[62];
  pm_math_Vector3_cross_ra(xx + 79, xx + 120, xx + 175);
  xx[71] = xx[144] * xx[22];
  xx[88] = xx[144] * xx[18];
  xx[89] = xx[38] - xx[88];
  xx[38] = xx[77] * xx[22];
  xx[120] = - xx[71];
  xx[121] = - xx[89];
  xx[122] = xx[38];
  pm_math_Vector3_cross_ra(xx + 68, xx + 120, xx + 178);
  xx[93] = xx[22] * xx[22];
  xx[94] = xx[23] * xx[23];
  xx[108] = xx[151] * xx[81];
  xx[116] = xx[156] * xx[81];
  xx[120] = xx[151] * xx[79] + xx[156] * xx[80];
  xx[181] = - xx[108];
  xx[182] = - xx[116];
  xx[183] = xx[120];
  pm_math_Vector3_cross_ra(xx + 79, xx + 181, xx + 184);
  xx[121] = xx[144] * xx[23];
  xx[122] = xx[72] * xx[23];
  xx[129] = xx[37] - xx[88];
  xx[181] = - xx[121];
  xx[182] = xx[122];
  xx[183] = - xx[129];
  pm_math_Vector3_cross_ra(xx + 68, xx + 181, xx + 187);
  xx[37] = xx[76] * xx[53];
  xx[68] = xx[37] * xx[49];
  xx[69] = xx[76] * xx[51];
  xx[70] = xx[76] * xx[52];
  xx[88] = xx[70] * xx[52];
  xx[136] = xx[37] * xx[53];
  xx[137] = xx[18] * xx[18];
  xx[142] = xx[69] * xx[51];
  xx[143] = xx[21] * xx[18];
  xx[155] = xx[22] * xx[23];
  xx[181] = xx[18] * xx[23];
  xx[182] = xx[21] * xx[22];
  xx[183] = xx[69] * xx[49];
  xx[190] = xx[70] * xx[49];
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
  xx[227] = xx[10];
  xx[228] = xx[0];
  xx[229] = xx[0];
  xx[230] = bb[0] ? xx[0] : - (xx[1] * (xx[82] - xx[12] * xx[78]) - xx[46] + xx
    [1] * (xx[47] * xx[21] + xx[172]) + xx[1] * (xx[17] - xx[48]));
  xx[231] = bb[0] ? xx[0] : - (xx[1] * (xx[175] - xx[56] * xx[78]) - xx[138] +
    xx[77] + xx[1] * (xx[178] + xx[71] * xx[21]) - xx[1] * (xx[93] + xx[94]) +
    xx[2]);
  xx[232] = bb[0] ? xx[0] : - (xx[1] * (xx[184] - xx[108] * xx[78]) - xx[156] +
    xx[1] * (xx[187] + xx[121] * xx[21]) - xx[72]);
  xx[233] = xx[63];
  xx[234] = xx[0];
  xx[235] = xx[0];
  xx[236] = bb[0] ? xx[0] : xx[1] * (xx[68] - xx[69] * xx[52]);
  xx[237] = bb[0] ? xx[0] : xx[76] - xx[1] * (xx[88] + xx[136]);
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[10];
  xx[241] = xx[0];
  xx[242] = bb[0] ? xx[0] : - (xx[1] * (xx[83] - xx[20] * xx[78]) + xx[1] * (xx
    [173] - xx[14] * xx[21]) + xx[1] * (xx[94] + xx[137]) - xx[77] + xx[39] -
    xx[2]);
  xx[243] = bb[0] ? xx[0] : - (xx[1] * (xx[176] - xx[61] * xx[78]) + xx[109] +
    xx[1] * (xx[89] * xx[21] + xx[179]) + xx[1] * (xx[17] + xx[48]));
  xx[244] = bb[0] ? xx[0] : - (xx[1] * (xx[185] - xx[116] * xx[78]) + xx[151] +
    xx[1] * (xx[188] - xx[122] * xx[21]) - xx[144]);
  xx[245] = xx[0];
  xx[246] = xx[63];
  xx[247] = xx[0];
  xx[248] = bb[0] ? xx[0] : xx[1] * (xx[136] + xx[142]) - xx[76];
  xx[249] = bb[0] ? xx[0] : xx[1] * (xx[68] + xx[70] * xx[51]);
  xx[250] = xx[0];
  xx[251] = xx[0];
  xx[252] = xx[0];
  xx[253] = xx[10];
  xx[254] = bb[0] ? xx[0] : - (xx[1] * (xx[84] + xx[35] * xx[78]) + xx[72] + xx
    [1] * (xx[174] - xx[15] * xx[21]) - xx[1] * (xx[143] + xx[155]));
  xx[255] = bb[0] ? xx[0] : - (xx[1] * (xx[177] + xx[62] * xx[78]) + xx[1] *
    (xx[180] - xx[38] * xx[21]) + xx[144] + xx[1] * (xx[181] - xx[182]));
  xx[256] = bb[0] ? xx[0] : - (xx[1] * (xx[186] + xx[120] * xx[78]) + xx[1] *
    (xx[129] * xx[21] + xx[189]));
  xx[257] = xx[0];
  xx[258] = xx[0];
  xx[259] = xx[63];
  xx[260] = bb[0] ? xx[0] : - (xx[1] * (xx[183] + xx[37] * xx[52]));
  xx[261] = bb[0] ? xx[0] : xx[1] * (xx[37] * xx[51] - xx[190]);
  xx[262] = xx[0];
  xx[12] = state[17] + xx[102];
  pm_math_Quaternion_xform_ra(xx + 29, xx + 40, xx + 37);
  xx[14] = xx[36] * xx[80];
  xx[15] = xx[36] * xx[79];
  xx[17] = state[18] + xx[103];
  xx[20] = state[1] + xx[97];
  xx[29] = state[19] + xx[104];
  xx[30] = state[2] + xx[98];
  xx[82] = pm_math_Vector3_dot_ra(xx + 73, xx + 117);
  xx[83] = pm_math_Vector3_dot_ra(xx + 133, xx + 152);
  xx[84] = pm_math_Vector3_dot_ra(xx + 160, xx + 43);
  xx[85] = xx[1] * (xx[190] + xx[69] * xx[53]) + xx[12] - (xx[37] + xx[16] + xx
    [1] * (xx[182] + xx[181]) - xx[1] * (xx[14] * xx[78] + xx[15] * xx[81]));
  xx[86] = xx[1] * (xx[70] * xx[53] - xx[183]) + xx[17] - (xx[1] * (xx[15] * xx
    [78] - xx[14] * xx[81]) + xx[38] + xx[20] - xx[1] * (xx[143] - xx[155]));
  xx[87] = xx[76] - xx[1] * (xx[142] + xx[88]) + xx[29] - (xx[39] + xx[30] -
    (xx[1] * (xx[137] + xx[93]) - xx[2]) + xx[1] * (xx[15] * xx[79] + xx[14] *
    xx[80])) + xx[36];
  zeroMajor(1, 6, ii + 0, xx + 82);
  xx[43] = - xx[82];
  xx[44] = - xx[83];
  xx[45] = - xx[84];
  xx[46] = - xx[85];
  xx[47] = - xx[86];
  xx[48] = - xx[87];
  memcpy(xx + 96, xx + 191, 72 * sizeof(double));
  factorAndSolveWide(6, 12, xx + 96, xx + 65, xx + 89, ii + 1, xx + 43, xx[13],
                     xx + 77);
  xx[13] = xx[16] + xx[77];
  xx[14] = xx[20] + xx[78];
  xx[15] = xx[30] + xx[79];
  xx[43] = xx[21];
  xx[44] = xx[18];
  xx[45] = xx[22];
  xx[46] = xx[23];
  pm_math_Quaternion_compDeriv_ra(xx + 43, xx + 80, xx + 65);
  xx[16] = xx[21] + xx[65];
  xx[20] = xx[18] + xx[66];
  xx[18] = xx[22] + xx[67];
  xx[21] = xx[23] + xx[68];
  xx[22] = sqrt(xx[16] * xx[16] + xx[20] * xx[20] + xx[18] * xx[18] + xx[21] *
                xx[21]);
  if (xx[19] > xx[22])
    xx[22] = xx[19];
  xx[23] = xx[16] / xx[22];
  xx[16] = xx[20] / xx[22];
  xx[20] = xx[18] / xx[22];
  xx[18] = xx[21] / xx[22];
  xx[21] = xx[12] + xx[83];
  xx[12] = xx[17] + xx[84];
  xx[17] = xx[29] + xx[85];
  xx[29] = xx[49];
  xx[30] = xx[51];
  xx[31] = xx[52];
  xx[32] = xx[53];
  pm_math_Quaternion_compDeriv_ra(xx + 29, xx + 86, xx + 43);
  xx[22] = xx[49] + xx[43];
  xx[29] = xx[51] + xx[44];
  xx[30] = xx[52] + xx[45];
  xx[31] = xx[53] + xx[46];
  xx[32] = sqrt(xx[22] * xx[22] + xx[29] * xx[29] + xx[30] * xx[30] + xx[31] *
                xx[31]);
  if (xx[19] > xx[32])
    xx[32] = xx[19];
  xx[19] = xx[22] / xx[32];
  xx[22] = xx[29] / xx[32];
  xx[29] = xx[30] / xx[32];
  xx[30] = xx[31] / xx[32];
  xx[96] = xx[13];
  xx[97] = xx[14];
  xx[98] = xx[15];
  xx[99] = xx[23];
  xx[100] = xx[16];
  xx[101] = xx[20];
  xx[102] = xx[18];
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
  xx[113] = xx[21];
  xx[114] = xx[12];
  xx[115] = xx[17];
  xx[116] = xx[19];
  xx[117] = xx[22];
  xx[118] = xx[29];
  xx[119] = xx[30];
  xx[120] = state[24];
  xx[121] = state[25];
  xx[122] = state[26];
  xx[123] = state[27];
  xx[124] = state[28];
  xx[125] = state[29];
  xx[31] = xx[19] * xx[19];
  xx[32] = xx[22] * xx[29];
  xx[35] = xx[19] * xx[30];
  xx[37] = xx[22] * xx[30];
  xx[38] = xx[19] * xx[29];
  xx[43] = xx[1] * (xx[31] + xx[22] * xx[22]) - xx[2];
  xx[44] = xx[1] * (xx[32] + xx[35]);
  xx[45] = xx[1] * (xx[37] - xx[38]);
  xx[46] = - xx[23];
  xx[47] = - xx[16];
  xx[48] = - xx[20];
  xx[49] = - xx[18];
  pm_math_Quaternion_compose_ra(xx + 46, xx + 25, xx + 65);
  xx[39] = xx[66] * xx[67];
  xx[51] = xx[65] * xx[68];
  xx[52] = xx[65] * xx[65];
  xx[53] = xx[67] * xx[68];
  xx[56] = xx[65] * xx[66];
  xx[69] = xx[1] * (xx[39] - xx[51]);
  xx[70] = xx[1] * (xx[52] + xx[67] * xx[67]) - xx[2];
  xx[71] = xx[1] * (xx[53] + xx[56]);
  xx[61] = xx[29] * xx[30];
  xx[62] = xx[19] * xx[22];
  xx[72] = xx[1] * (xx[32] - xx[35]);
  xx[73] = xx[1] * (xx[31] + xx[29] * xx[29]) - xx[2];
  xx[74] = xx[1] * (xx[61] + xx[62]);
  xx[32] = xx[66] * xx[68];
  xx[35] = xx[65] * xx[67];
  xx[77] = xx[1] * (xx[32] + xx[35]);
  xx[78] = xx[1] * (xx[53] - xx[56]);
  xx[79] = xx[1] * (xx[52] + xx[68] * xx[68]) - xx[2];
  xx[80] = xx[1] * (xx[37] + xx[38]);
  xx[81] = xx[1] * (xx[61] - xx[62]);
  xx[82] = xx[1] * (xx[31] + xx[30] * xx[30]) - xx[2];
  xx[83] = xx[1] * (xx[52] + xx[66] * xx[66]) - xx[2];
  xx[84] = xx[1] * (xx[39] + xx[51]);
  xx[85] = xx[1] * (xx[32] - xx[35]);
  xx[31] = xx[76] * xx[29];
  xx[32] = xx[76] * xx[22];
  pm_math_Quaternion_xform_ra(xx + 46, xx + 40, xx + 37);
  xx[35] = xx[37] + xx[13] + xx[1] * (xx[23] * xx[20] + xx[16] * xx[18]);
  xx[13] = xx[36] * xx[67];
  xx[51] = xx[36] * xx[66];
  xx[52] = xx[38] + xx[14] - xx[1] * (xx[23] * xx[16] - xx[20] * xx[18]);
  xx[14] = xx[39] + xx[15] - xx[1] * (xx[16] * xx[16] + xx[20] * xx[20]) + xx[2];
  xx[86] = pm_math_Vector3_dot_ra(xx + 43, xx + 69);
  xx[87] = pm_math_Vector3_dot_ra(xx + 72, xx + 77);
  xx[88] = pm_math_Vector3_dot_ra(xx + 80, xx + 83);
  xx[89] = xx[1] * (xx[31] * xx[19] + xx[32] * xx[30]) + xx[21] - (xx[35] - xx[1]
    * (xx[13] * xx[65] + xx[51] * xx[68]));
  xx[90] = xx[1] * (xx[31] * xx[30] - xx[32] * xx[19]) + xx[12] - (xx[1] * (xx
    [51] * xx[65] - xx[13] * xx[68]) + xx[52]);
  xx[91] = xx[76] - xx[1] * (xx[32] * xx[22] + xx[31] * xx[29]) + xx[17] - (xx[1]
    * (xx[51] * xx[66] + xx[13] * xx[67]) - xx[36] + xx[14]);
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
  xx[13] = xx[77 + (ii[1])];
  xx[15] = 1.0e-9;
  if (!(!(xx[13] > xx[15]) && (bb[0] ? 1 : pm_math_Vector3_dot_ra(xx + 43, xx +
         83) > xx[3] && pm_math_Vector3_dot_ra(xx + 72, xx + 69) > xx[3]))) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:mechanism:mechanism:jointDisToNormPositionSatisfactionFailure",
      "The position components of the kinematic constraints of the mechanism cannot be satisfied when attempting to transition joints from disengaged to normal mode. This is more likely to occur for large rotation errors compared to large translation errors. Consider reducing the error in those joints before attempting the transition or consider increasing the joint mode transition nonlinear iterations parameter in the associated Mechanism Configuration block.",
      neDiagMgr);
  }

  xx[13] = xx[101] * xx[101];
  xx[16] = xx[102] * xx[102];
  xx[18] = xx[2] - xx[1] * (xx[13] + xx[16]);
  xx[20] = xx[100] * xx[101];
  xx[23] = xx[99] * xx[102];
  xx[31] = xx[1] * (xx[20] - xx[23]);
  xx[32] = xx[99] * xx[101];
  xx[37] = xx[100] * xx[102];
  xx[38] = xx[1] * (xx[32] + xx[37]);
  xx[43] = xx[18];
  xx[44] = xx[31];
  xx[45] = xx[38];
  xx[39] = 1.1;
  xx[51] = xx[6] * xx[6];
  xx[53] = xx[1] * (xx[51] + xx[64]) - xx[2];
  xx[56] = xx[1] * (xx[34] + xx[33]);
  xx[33] = xx[1] * (xx[55] + xx[54]);
  xx[34] = xx[1] * (xx[51] + xx[9]) - xx[2];
  xx[9] = xx[1] * (xx[58] + xx[59]);
  xx[54] = xx[1] * (xx[51] + xx[11]) - xx[2];
  xx[77] = xx[53];
  xx[78] = xx[95];
  xx[79] = xx[56];
  xx[80] = xx[33];
  xx[81] = xx[34];
  xx[82] = xx[60];
  xx[83] = xx[57];
  xx[84] = xx[9];
  xx[85] = xx[54];
  xx[11] = 0.1;
  xx[86] = xx[11] * xx[53];
  xx[87] = xx[11] * xx[33];
  xx[88] = xx[11] * xx[57];
  xx[89] = xx[11] * xx[95];
  xx[90] = xx[11] * xx[34];
  xx[91] = xx[11] * xx[9];
  xx[92] = xx[11] * xx[56];
  xx[93] = xx[11] * xx[60];
  xx[94] = xx[11] * xx[54];
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
  xx[39] = xx[1] * (xx[23] + xx[20]);
  xx[20] = xx[100] * xx[100];
  xx[23] = xx[2] - xx[1] * (xx[16] + xx[20]);
  xx[16] = xx[101] * xx[102];
  xx[58] = xx[99] * xx[100];
  xx[59] = xx[1] * (xx[16] - xx[58]);
  xx[72] = xx[39];
  xx[73] = xx[23];
  xx[74] = xx[59];
  pm_math_Matrix3x3_xform_ra(xx + 86, xx + 72, xx + 135);
  xx[61] = pm_math_Vector3_dot_ra(xx + 43, xx + 135);
  xx[62] = xx[1] * (xx[37] - xx[32]);
  xx[32] = xx[1] * (xx[58] + xx[16]);
  xx[16] = xx[2] - xx[1] * (xx[20] + xx[13]);
  xx[138] = xx[62];
  xx[139] = xx[32];
  xx[140] = xx[16];
  pm_math_Matrix3x3_xform_ra(xx + 86, xx + 138, xx + 141);
  xx[13] = pm_math_Vector3_dot_ra(xx + 43, xx + 141);
  pm_math_Matrix3x3_postCross_ra(xx + 126, xx + 40, xx + 86);
  xx[144] = - (xx[86] + xx[127]);
  xx[145] = - (xx[89] + xx[55]);
  xx[146] = - (xx[92] + xx[133]);
  xx[20] = pm_math_Vector3_dot_ra(xx + 43, xx + 144);
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
  xx[153] = xx[127] * xx[9];
  xx[154] = xx[128] * xx[56];
  xx[155] = xx[128] * xx[60];
  xx[156] = xx[128] * xx[54];
  pm_math_Matrix3x3_compose_ra(xx + 77, xx + 148, xx + 157);
  pm_math_Matrix3x3_preCross_ra(xx + 86, xx + 40, xx + 77);
  xx[9] = xx[158] - xx[78] - xx[86] - xx[37];
  xx[33] = xx[159] - xx[79] + xx[91];
  xx[34] = xx[162] - xx[82] - xx[88];
  xx[166] = pm_math_Vector3_dot_ra(xx + 43, xx + 69);
  xx[167] = xx[61];
  xx[168] = xx[13];
  xx[169] = xx[20];
  xx[170] = xx[58];
  xx[171] = xx[64];
  xx[172] = xx[61];
  xx[173] = pm_math_Vector3_dot_ra(xx + 72, xx + 135);
  xx[174] = xx[75];
  xx[175] = xx[132];
  xx[176] = xx[133];
  xx[177] = xx[134];
  xx[178] = xx[13];
  xx[179] = xx[75];
  xx[180] = pm_math_Vector3_dot_ra(xx + 138, xx + 141);
  xx[181] = xx[147];
  xx[182] = xx[144];
  xx[183] = xx[126];
  xx[184] = xx[20];
  xx[185] = xx[132];
  xx[186] = xx[147];
  xx[187] = xx[157] - xx[77] + xx[89] + xx[89] + xx[55] + xx[2];
  xx[188] = xx[9];
  xx[189] = xx[33];
  xx[190] = xx[58];
  xx[191] = xx[133];
  xx[192] = xx[144];
  xx[193] = xx[9];
  xx[194] = xx[161] - xx[81] - xx[87] + xx[51] - xx[87] + xx[2];
  xx[195] = xx[34];
  xx[196] = xx[64];
  xx[197] = xx[134];
  xx[198] = xx[126];
  xx[199] = xx[33];
  xx[200] = xx[34];
  xx[201] = xx[2] + xx[165] - xx[85];
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
  xx[9] = xx[118] * xx[118];
  xx[13] = xx[119] * xx[119];
  xx[20] = xx[2] - xx[1] * (xx[9] + xx[13]);
  xx[33] = xx[117] * xx[118];
  xx[34] = xx[116] * xx[119];
  xx[37] = xx[1] * (xx[33] - xx[34]);
  xx[51] = xx[116] * xx[118];
  xx[53] = xx[117] * xx[119];
  xx[54] = xx[1] * (xx[51] + xx[53]);
  xx[55] = xx[20];
  xx[56] = xx[37];
  xx[57] = xx[54];
  xx[69] = xx[11] * xx[20];
  xx[70] = xx[11] * xx[37];
  xx[71] = xx[11] * xx[54];
  xx[58] = xx[1] * (xx[34] + xx[33]);
  xx[60] = xx[117] * xx[117];
  xx[61] = xx[2] - xx[1] * (xx[13] + xx[60]);
  xx[64] = xx[118] * xx[119];
  xx[75] = xx[116] * xx[117];
  xx[77] = xx[1] * (xx[64] - xx[75]);
  xx[78] = xx[11] * xx[58];
  xx[79] = xx[11] * xx[61];
  xx[80] = xx[11] * xx[77];
  xx[81] = pm_math_Vector3_dot_ra(xx + 55, xx + 78);
  xx[82] = xx[1] * (xx[53] - xx[51]);
  xx[83] = xx[1] * (xx[75] + xx[64]);
  xx[84] = xx[2] - xx[1] * (xx[60] + xx[9]);
  xx[85] = xx[11] * xx[82];
  xx[86] = xx[11] * xx[83];
  xx[87] = xx[11] * xx[84];
  xx[11] = pm_math_Vector3_dot_ra(xx + 55, xx + 85);
  xx[88] = xx[58];
  xx[89] = xx[61];
  xx[90] = xx[77];
  xx[91] = pm_math_Vector3_dot_ra(xx + 88, xx + 85);
  xx[92] = 6.750000000000001e-5;
  xx[202] = pm_math_Vector3_dot_ra(xx + 55, xx + 69);
  xx[203] = xx[81];
  xx[204] = xx[11];
  xx[205] = xx[0];
  xx[206] = xx[0];
  xx[207] = xx[0];
  xx[208] = xx[81];
  xx[209] = pm_math_Vector3_dot_ra(xx + 88, xx + 78);
  xx[210] = xx[91];
  xx[211] = xx[0];
  xx[212] = xx[0];
  xx[213] = xx[0];
  xx[214] = xx[11];
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

  xx[11] = bb[1] ? xx[63] : xx[0];
  xx[126] = xx[0];
  xx[127] = xx[0];
  xx[128] = xx[0];
  xx[129] = xx[11];
  xx[130] = xx[0];
  xx[131] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 126, 6, 1, xx + 132, xx + 141);
  xx[78] = - xx[19];
  xx[79] = - xx[22];
  xx[80] = - xx[29];
  xx[81] = - xx[30];
  pm_math_Quaternion_inverseCompose_ra(xx + 78, xx + 65, xx + 91);
  xx[65] = - xx[91];
  xx[66] = - xx[92];
  xx[67] = - xx[93];
  xx[68] = - xx[94];
  xx[19] = bb[1] ? xx[10] : xx[0];
  xx[22] = xx[8] * xx[19];
  xx[29] = xx[22] * xx[8];
  xx[30] = xx[4] * xx[19];
  xx[69] = xx[30] * xx[4];
  xx[70] = xx[1] * (xx[29] + xx[69]);
  xx[71] = xx[30] * xx[6];
  xx[85] = xx[1] * (xx[71] + xx[22] * xx[5]);
  xx[86] = xx[22] * xx[6];
  xx[87] = xx[1] * (xx[86] - xx[30] * xx[5]);
  xx[126] = xx[0];
  xx[127] = xx[0];
  xx[128] = xx[0];
  xx[129] = xx[19] - xx[70];
  xx[130] = xx[85];
  xx[131] = - xx[87];
  solveSymmetricPosDef(xx + 166, xx + 126, 6, 1, xx + 141, xx + 147);
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 144, xx + 91);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 91, xx + 126);
  xx[91] = xx[135] - xx[126];
  xx[92] = xx[136] - xx[127];
  xx[93] = bb[0] ? xx[0] : xx[76];
  xx[94] = xx[76] * xx[63];
  xx[95] = xx[93] - xx[94];
  xx[129] = xx[35] - xx[21];
  xx[130] = xx[52] - xx[12];
  xx[131] = xx[14] - xx[17];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 129, xx + 147);
  xx[129] = xx[147];
  xx[130] = xx[148];
  xx[131] = xx[149] - xx[76];
  pm_math_Vector3_cross_ra(xx + 129, xx + 126, xx + 147);
  pm_math_Vector3_cross_ra(xx + 144, xx + 40, xx + 150);
  xx[153] = xx[18] * xx[141] + xx[39] * xx[142] + xx[62] * xx[143] + xx[145] +
    xx[150];
  xx[154] = xx[31] * xx[141] + xx[23] * xx[142] + xx[32] * xx[143] - xx[144] +
    xx[151];
  xx[155] = xx[38] * xx[141] + xx[59] * xx[142] + xx[16] * xx[143] + xx[152];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 153, xx + 141);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 141, xx + 144);
  xx[12] = bb[0] ? xx[0] : - xx[76];
  xx[14] = xx[12] + xx[94];
  xx[150] = xx[0];
  xx[151] = xx[0];
  xx[152] = xx[0];
  xx[153] = xx[0];
  xx[154] = xx[11];
  xx[155] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 150, 6, 1, xx + 156, xx + 238);
  xx[17] = xx[5] * xx[19];
  xx[21] = xx[1] * (xx[71] - xx[17] * xx[8]);
  xx[35] = xx[17] * xx[5];
  xx[52] = xx[1] * (xx[69] + xx[35]);
  xx[69] = xx[17] * xx[6];
  xx[71] = xx[1] * (xx[69] + xx[30] * xx[8]);
  xx[150] = xx[0];
  xx[151] = xx[0];
  xx[152] = xx[0];
  xx[153] = - xx[21];
  xx[154] = xx[19] - xx[52];
  xx[155] = xx[71];
  solveSymmetricPosDef(xx + 166, xx + 150, 6, 1, xx + 238, xx + 244);
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 241, xx + 141);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 141, xx + 150);
  xx[30] = xx[159] - xx[150];
  xx[94] = xx[160] - xx[151];
  pm_math_Vector3_cross_ra(xx + 129, xx + 150, xx + 141);
  pm_math_Vector3_cross_ra(xx + 241, xx + 40, xx + 153);
  xx[162] = xx[18] * xx[238] + xx[39] * xx[239] + xx[62] * xx[240] + xx[242] +
    xx[153];
  xx[163] = xx[31] * xx[238] + xx[23] * xx[239] + xx[32] * xx[240] - xx[241] +
    xx[154];
  xx[164] = xx[38] * xx[238] + xx[59] * xx[239] + xx[16] * xx[240] + xx[155];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 162, xx + 153);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 153, xx + 162);
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[0];
  xx[241] = xx[0];
  xx[242] = xx[0];
  xx[243] = xx[11];
  solveSymmetricPosDef(xx + 202, xx + 238, 6, 1, xx + 244, xx + 250);
  xx[126] = xx[1] * (xx[86] + xx[17] * xx[4]);
  xx[17] = xx[1] * (xx[69] - xx[22] * xx[4]);
  xx[22] = xx[1] * (xx[35] + xx[29]);
  xx[238] = xx[0];
  xx[239] = xx[0];
  xx[240] = xx[0];
  xx[241] = xx[126];
  xx[242] = - xx[17];
  xx[243] = xx[19] - xx[22];
  solveSymmetricPosDef(xx + 166, xx + 238, 6, 1, xx + 250, xx + 256);
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 253, xx + 153);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 153, xx + 238);
  xx[29] = xx[247] - xx[238];
  xx[35] = xx[248] - xx[239];
  pm_math_Vector3_cross_ra(xx + 129, xx + 238, xx + 153);
  pm_math_Vector3_cross_ra(xx + 253, xx + 40, xx + 241);
  xx[255] = xx[18] * xx[250] + xx[39] * xx[251] + xx[62] * xx[252] + xx[254] +
    xx[241];
  xx[256] = xx[31] * xx[250] + xx[23] * xx[251] + xx[32] * xx[252] - xx[253] +
    xx[242];
  xx[257] = xx[38] * xx[250] + xx[59] * xx[251] + xx[16] * xx[252] + xx[243];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 255, xx + 241);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 241, xx + 250);
  xx[253] = xx[20] * xx[11];
  xx[254] = xx[58] * xx[11];
  xx[255] = xx[82] * xx[11];
  xx[256] = xx[0];
  xx[257] = bb[1] ? xx[93] : xx[0];
  xx[258] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 253, 6, 1, xx + 259, xx + 265);
  xx[69] = xx[70] - xx[19];
  xx[241] = xx[69];
  xx[242] = - xx[85];
  xx[243] = xx[87];
  xx[70] = bb[0] ? xx[0] : xx[36];
  xx[86] = bb[1] ? xx[70] : xx[0];
  xx[87] = xx[4] * xx[86];
  xx[127] = xx[5] * xx[86];
  pm_math_Vector3_cross_ra(xx + 40, xx + 241, xx + 253);
  xx[265] = - pm_math_Vector3_dot_ra(xx + 43, xx + 241);
  xx[266] = - pm_math_Vector3_dot_ra(xx + 72, xx + 241);
  xx[267] = - pm_math_Vector3_dot_ra(xx + 138, xx + 241);
  xx[268] = - (xx[1] * (xx[87] * xx[6] - xx[127] * xx[8]) + xx[253] + xx[85]);
  xx[269] = - (xx[1] * (xx[87] * xx[4] + xx[127] * xx[5]) - xx[86] + xx[254] +
               xx[69]);
  xx[270] = xx[1] * (xx[127] * xx[6] + xx[87] * xx[8]) - xx[255];
  solveSymmetricPosDef(xx + 166, xx + 265, 6, 1, xx + 253, xx + 271);
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 256, xx + 85);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 85, xx + 241);
  xx[69] = xx[262] - xx[241];
  xx[85] = xx[263] - xx[242];
  pm_math_Vector3_cross_ra(xx + 129, xx + 241, xx + 265);
  pm_math_Vector3_cross_ra(xx + 256, xx + 40, xx + 268);
  xx[271] = xx[18] * xx[253] + xx[39] * xx[254] + xx[62] * xx[255] + xx[257] +
    xx[268];
  xx[272] = xx[31] * xx[253] + xx[23] * xx[254] + xx[32] * xx[255] - xx[256] +
    xx[269];
  xx[273] = xx[38] * xx[253] + xx[59] * xx[254] + xx[16] * xx[255] + xx[270];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 271, xx + 253);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 253, xx + 256);
  xx[268] = xx[37] * xx[11];
  xx[269] = xx[61] * xx[11];
  xx[270] = xx[83] * xx[11];
  xx[271] = bb[1] ? xx[12] : xx[0];
  xx[272] = xx[0];
  xx[273] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 268, 6, 1, xx + 274, xx + 280);
  xx[86] = xx[52] - xx[19];
  xx[253] = xx[21];
  xx[254] = xx[86];
  xx[255] = - xx[71];
  xx[52] = bb[0] ? xx[0] : - xx[36];
  xx[71] = bb[1] ? xx[52] : xx[0];
  xx[87] = xx[8] * xx[71];
  xx[127] = xx[4] * xx[71];
  pm_math_Vector3_cross_ra(xx + 40, xx + 253, xx + 268);
  xx[280] = - pm_math_Vector3_dot_ra(xx + 43, xx + 253);
  xx[281] = - pm_math_Vector3_dot_ra(xx + 72, xx + 253);
  xx[282] = - pm_math_Vector3_dot_ra(xx + 138, xx + 253);
  xx[283] = xx[86] - (xx[1] * (xx[87] * xx[8] + xx[127] * xx[4]) - xx[71] + xx
                      [268]);
  xx[284] = - (xx[269] - xx[1] * (xx[127] * xx[6] + xx[87] * xx[5]) + xx[21]);
  xx[285] = - (xx[1] * (xx[87] * xx[6] - xx[127] * xx[5]) + xx[270]);
  solveSymmetricPosDef(xx + 166, xx + 280, 6, 1, xx + 268, xx + 286);
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 271, xx + 4);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 4, xx + 253);
  xx[4] = xx[277] - xx[253];
  xx[5] = xx[278] - xx[254];
  pm_math_Vector3_cross_ra(xx + 129, xx + 253, xx + 280);
  pm_math_Vector3_cross_ra(xx + 271, xx + 40, xx + 283);
  xx[286] = xx[18] * xx[268] + xx[39] * xx[269] + xx[62] * xx[270] + xx[272] +
    xx[283];
  xx[287] = xx[31] * xx[268] + xx[23] * xx[269] + xx[32] * xx[270] - xx[271] +
    xx[284];
  xx[288] = xx[38] * xx[268] + xx[59] * xx[269] + xx[16] * xx[270] + xx[285];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 286, xx + 268);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 268, xx + 271);
  xx[283] = xx[54] * xx[11];
  xx[284] = xx[77] * xx[11];
  xx[285] = xx[84] * xx[11];
  xx[286] = xx[0];
  xx[287] = xx[0];
  xx[288] = xx[0];
  solveSymmetricPosDef(xx + 202, xx + 283, 6, 1, xx + 289, xx + 295);
  xx[268] = - xx[126];
  xx[269] = xx[17];
  xx[270] = xx[22] - xx[19];
  pm_math_Vector3_cross_ra(xx + 40, xx + 268, xx + 283);
  xx[295] = - pm_math_Vector3_dot_ra(xx + 43, xx + 268);
  xx[296] = - pm_math_Vector3_dot_ra(xx + 72, xx + 268);
  xx[297] = - pm_math_Vector3_dot_ra(xx + 138, xx + 268);
  xx[298] = xx[17] - xx[283];
  xx[299] = xx[126] - xx[284];
  xx[300] = - xx[285];
  solveSymmetricPosDef(xx + 166, xx + 295, 6, 1, xx + 283, xx + 301);
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 286, xx + 268);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 268, xx + 295);
  xx[6] = xx[292] - xx[295];
  xx[8] = xx[293] - xx[296];
  pm_math_Vector3_cross_ra(xx + 129, xx + 295, xx + 268);
  pm_math_Vector3_cross_ra(xx + 286, xx + 40, xx + 298);
  xx[301] = xx[18] * xx[283] + xx[39] * xx[284] + xx[62] * xx[285] + xx[287] +
    xx[298];
  xx[302] = xx[31] * xx[283] + xx[23] * xx[284] + xx[32] * xx[285] - xx[286] +
    xx[299];
  xx[303] = xx[38] * xx[283] + xx[59] * xx[284] + xx[16] * xx[285] + xx[300];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 301, xx + 16);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 16, xx + 21);
  xx[298] = xx[91] * xx[63];
  xx[299] = xx[92] * xx[63];
  xx[300] = (xx[137] - xx[128]) * xx[63];
  xx[301] = xx[92] * xx[95] + (xx[20] * xx[132] + xx[58] * xx[133] + xx[82] *
    xx[134] + xx[76] * xx[136] - (xx[147] + xx[144])) * xx[63];
  xx[302] = xx[91] * xx[14] + (xx[37] * xx[132] + xx[61] * xx[133] + xx[83] *
    xx[134] - xx[76] * xx[135] - (xx[148] + xx[145])) * xx[63];
  xx[303] = (xx[54] * xx[132] + xx[77] * xx[133] + xx[84] * xx[134] - (xx[149] +
              xx[146])) * xx[63];
  xx[304] = xx[30] * xx[63];
  xx[305] = xx[94] * xx[63];
  xx[306] = (xx[161] - xx[152]) * xx[63];
  xx[307] = xx[94] * xx[95] + (xx[20] * xx[156] + xx[58] * xx[157] + xx[82] *
    xx[158] + xx[76] * xx[160] - (xx[141] + xx[162])) * xx[63];
  xx[308] = xx[30] * xx[14] + (xx[37] * xx[156] + xx[61] * xx[157] + xx[83] *
    xx[158] - xx[76] * xx[159] - (xx[142] + xx[163])) * xx[63];
  xx[309] = (xx[54] * xx[156] + xx[77] * xx[157] + xx[84] * xx[158] - (xx[143] +
              xx[164])) * xx[63];
  xx[310] = xx[29] * xx[63];
  xx[311] = xx[35] * xx[63];
  xx[312] = (xx[249] - xx[240]) * xx[63];
  xx[313] = xx[35] * xx[95] + (xx[20] * xx[244] + xx[58] * xx[245] + xx[82] *
    xx[246] + xx[76] * xx[248] - (xx[153] + xx[250])) * xx[63];
  xx[314] = xx[29] * xx[14] + (xx[37] * xx[244] + xx[61] * xx[245] + xx[83] *
    xx[246] - xx[76] * xx[247] - (xx[154] + xx[251])) * xx[63];
  xx[315] = (xx[54] * xx[244] + xx[77] * xx[245] + xx[84] * xx[246] - (xx[155] +
              xx[252])) * xx[63];
  xx[316] = xx[69] * xx[63];
  xx[317] = xx[85] * xx[63];
  xx[318] = (xx[264] - xx[243]) * xx[63];
  xx[319] = xx[85] * xx[95] + (xx[20] * xx[259] + xx[58] * xx[260] + xx[82] *
    xx[261] + xx[76] * xx[263] - (xx[265] + xx[256])) * xx[63];
  xx[320] = xx[69] * xx[14] + (xx[37] * xx[259] + xx[61] * xx[260] + xx[83] *
    xx[261] - xx[76] * xx[262] - (xx[266] + xx[257])) * xx[63];
  xx[321] = (xx[54] * xx[259] + xx[77] * xx[260] + xx[84] * xx[261] - (xx[267] +
              xx[258])) * xx[63];
  xx[322] = xx[4] * xx[63];
  xx[323] = xx[5] * xx[63];
  xx[324] = (xx[279] - xx[255]) * xx[63];
  xx[325] = xx[5] * xx[95] + (xx[20] * xx[274] + xx[58] * xx[275] + xx[82] * xx
    [276] + xx[76] * xx[278] - (xx[280] + xx[271])) * xx[63];
  xx[326] = xx[4] * xx[14] + (xx[37] * xx[274] + xx[61] * xx[275] + xx[83] * xx
    [276] - xx[76] * xx[277] - (xx[281] + xx[272])) * xx[63];
  xx[327] = (xx[54] * xx[274] + xx[77] * xx[275] + xx[84] * xx[276] - (xx[282] +
              xx[273])) * xx[63];
  xx[328] = xx[6] * xx[63];
  xx[329] = xx[8] * xx[63];
  xx[330] = (xx[294] - xx[297]) * xx[63];
  xx[331] = xx[8] * xx[95] + (xx[20] * xx[289] + xx[58] * xx[290] + xx[82] * xx
    [291] + xx[76] * xx[293] - (xx[268] + xx[21])) * xx[63];
  xx[332] = xx[6] * xx[14] + (xx[37] * xx[289] + xx[61] * xx[290] + xx[83] * xx
    [291] - xx[76] * xx[292] - (xx[269] + xx[22])) * xx[63];
  xx[333] = (xx[54] * xx[289] + xx[77] * xx[290] + xx[84] * xx[291] - (xx[270] +
              xx[23])) * xx[63];
  reduceMatrix(6, 6, ii[2], ii + 9, xx + 298);
  transposeColMajor(6, ii[2], xx + 298, xx + 238);
  reduceMatrix(ii[2], 6, ii[2], ii + 9, xx + 238);
  transposeColMajor(ii[2], ii[2], xx + 238, xx + 274);
  svd(ii[2], ii[2], xx + 274, TRUE, xx + 16, xx + 238, xx + 310, xx + 346);
  bb[0] = ii[2] > 0 ? xx[16] > 9.999999999999999e-21 : 1;
  xx[4] = state[10];
  xx[5] = state[11];
  xx[6] = state[12];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 4, xx + 29);
  xx[8] = (xx[2] - xx[1] * xx[7] * xx[7]) * state[15];
  xx[85] = xx[29] + xx[8];
  xx[86] = xx[30] + state[16];
  xx[87] = xx[31] + xx[1] * xx[24] * xx[7] * state[15];
  pm_math_Quaternion_xform_ra(xx + 65, xx + 85, xx + 22);
  xx[7] = state[27] - xx[22];
  xx[11] = state[28] - xx[23];
  xx[29] = state[24];
  xx[30] = state[25];
  xx[31] = state[26];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 29, xx + 85);
  pm_math_Vector3_cross_ra(xx + 129, xx + 22, xx + 29);
  pm_math_Vector3_cross_ra(xx + 4, xx + 40, xx + 78);
  xx[4] = state[7];
  xx[5] = state[8];
  xx[6] = state[9];
  pm_math_Quaternion_inverseXform_ra(xx + 46, xx + 4, xx + 126);
  xx[4] = xx[78] + xx[126] + state[11];
  xx[5] = xx[79] + xx[127] - state[10];
  xx[6] = xx[80] + xx[128];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 4, xx + 46);
  xx[4] = xx[46] - xx[50] * state[16];
  xx[5] = xx[47] + xx[50] * xx[8];
  xx[6] = xx[48];
  pm_math_Quaternion_xform_ra(xx + 65, xx + 4, xx + 46);
  xx[126] = - (xx[7] * xx[63]);
  xx[127] = - (xx[11] * xx[63]);
  xx[128] = - ((state[29] - xx[24]) * xx[63]);
  xx[129] = - (xx[11] * xx[95] + (xx[85] + xx[76] * state[28] - (xx[29] + xx[46]))
               * xx[63]);
  xx[130] = - (xx[7] * xx[14] + (xx[86] - xx[76] * state[27] - (xx[30] + xx[47]))
               * xx[63]);
  xx[131] = - ((xx[87] - (xx[31] + xx[48])) * xx[63]);
  reduceMatrix(1, 6, ii[2], ii + 9, xx + 126);
  ii[1] = svdSolveFromFactorization(ii[2], ii[2], xx + 16, xx + 238, xx + 310,
    xx + 126, 1.0e-12, xx + 132, xx + 141);
  xx[16] = bb[0] ? xx[132] : xx[0];
  xx[17] = bb[0] ? xx[133] : xx[0];
  xx[18] = bb[0] ? xx[134] : xx[0];
  xx[19] = bb[0] ? xx[135] : xx[0];
  xx[20] = bb[0] ? xx[136] : xx[0];
  xx[21] = bb[0] ? xx[137] : xx[0];
  expandVector(6, ii[2], ii + 9, xx + 16);
  xx[4] = - (xx[19] * xx[10]);
  xx[5] = - (xx[20] * xx[10]);
  xx[6] = - (xx[21] * xx[10]);
  pm_math_Quaternion_xform_ra(xx + 25, xx + 4, xx + 22);
  xx[4] = - (xx[16] * xx[10] + xx[20] * xx[52]);
  xx[5] = - (xx[17] * xx[10] + xx[19] * xx[70]);
  xx[6] = - (xx[18] * xx[10]);
  pm_math_Quaternion_xform_ra(xx + 25, xx + 4, xx + 29);
  pm_math_Vector3_cross_ra(xx + 40, xx + 22, xx + 4);
  xx[65] = - pm_math_Vector3_dot_ra(xx + 43, xx + 22);
  xx[66] = - pm_math_Vector3_dot_ra(xx + 72, xx + 22);
  xx[67] = - pm_math_Vector3_dot_ra(xx + 138, xx + 22);
  xx[68] = xx[23] - (xx[29] + xx[4]);
  xx[69] = - (xx[30] + xx[5] + xx[22]);
  xx[70] = - (xx[31] + xx[6]);
  solveSymmetricPosDef(xx + 166, xx + 65, 6, 1, xx + 22, xx + 38);
  xx[0] = xx[103] + xx[22];
  xx[4] = xx[104] + xx[23];
  xx[5] = xx[105] + xx[24];
  xx[6] = xx[106] + xx[25];
  xx[7] = xx[107] + xx[26];
  xx[8] = xx[108] + xx[27];
  xx[22] = - (xx[19] * xx[63]);
  xx[23] = - (xx[20] * xx[63]);
  xx[24] = - (xx[21] * xx[63]);
  xx[25] = - pm_math_Vector3_dot_ra(xx + 55, xx + 22);
  xx[26] = - pm_math_Vector3_dot_ra(xx + 88, xx + 22);
  xx[27] = - pm_math_Vector3_dot_ra(xx + 82, xx + 22);
  xx[28] = xx[16] * xx[63] + xx[20] * xx[12];
  xx[29] = xx[17] * xx[63] + xx[19] * xx[93];
  xx[30] = xx[18] * xx[63];
  solveSymmetricPosDef(xx + 202, xx + 25, 6, 1, xx + 16, xx + 38);
  xx[10] = xx[120] + xx[16];
  xx[11] = xx[121] + xx[17];
  xx[12] = xx[122] + xx[18];
  xx[14] = xx[123] + xx[19];
  xx[22] = xx[124] + xx[20];
  xx[16] = xx[125] + xx[21];
  xx[120] = xx[96];
  xx[121] = xx[97];
  xx[122] = xx[98];
  xx[123] = xx[99];
  xx[124] = xx[100];
  xx[125] = xx[101];
  xx[126] = xx[102];
  xx[127] = xx[0];
  xx[128] = xx[4];
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
  xx[144] = xx[10];
  xx[145] = xx[11];
  xx[146] = xx[12];
  xx[147] = xx[14];
  xx[148] = xx[22];
  xx[149] = xx[16];
  xx[17] = - xx[117];
  xx[18] = - xx[118];
  xx[19] = - xx[119];
  xx[20] = xx[22] * xx[118];
  xx[21] = xx[16] * xx[119];
  xx[23] = xx[20] + xx[21];
  xx[24] = xx[22] * xx[117];
  xx[25] = xx[16] * xx[117];
  xx[26] = xx[23];
  xx[27] = - xx[24];
  xx[28] = - xx[25];
  pm_math_Vector3_cross_ra(xx + 17, xx + 26, xx + 29);
  xx[26] = xx[1] * (xx[29] - xx[23] * xx[116]);
  xx[27] = xx[16] + xx[1] * (xx[24] * xx[116] + xx[30]);
  xx[28] = xx[1] * (xx[25] * xx[116] + xx[31]) - xx[22];
  xx[23] = - xx[100];
  xx[24] = - xx[101];
  xx[25] = - xx[102];
  xx[29] = - xx[99];
  xx[30] = xx[23];
  xx[31] = xx[24];
  xx[32] = xx[25];
  xx[35] = xx[3] * xx[109];
  xx[38] = cos(xx[35]);
  xx[39] = xx[3] * xx[110];
  xx[3] = cos(xx[39]);
  xx[40] = xx[38] * xx[3];
  xx[41] = sin(xx[35]);
  xx[35] = xx[3] * xx[41];
  xx[3] = - xx[35];
  xx[42] = sin(xx[39]);
  xx[39] = xx[38] * xx[42];
  xx[38] = - xx[39];
  xx[43] = xx[41] * xx[42];
  xx[41] = - xx[43];
  xx[44] = - xx[40];
  xx[45] = xx[3];
  xx[46] = xx[38];
  xx[47] = xx[41];
  pm_math_Quaternion_compose_ra(xx + 29, xx + 44, xx + 54);
  xx[42] = xx[55] * xx[56];
  xx[48] = xx[54] * xx[57];
  xx[49] = xx[54] * xx[54];
  xx[52] = xx[56] * xx[57];
  xx[58] = xx[54] * xx[55];
  xx[61] = xx[1] * (xx[42] - xx[48]);
  xx[62] = xx[1] * (xx[49] + xx[56] * xx[56]) - xx[2];
  xx[63] = xx[1] * (xx[52] + xx[58]);
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 6, xx + 65);
  xx[44] = (xx[2] - xx[1] * (xx[39] * xx[39] + xx[43] * xx[43])) * xx[111];
  xx[45] = xx[65] + xx[44];
  xx[46] = xx[45] * xx[56];
  xx[47] = xx[67] + xx[1] * (xx[40] * xx[39] + xx[35] * xx[43]) * xx[111];
  xx[59] = xx[47] * xx[57];
  xx[68] = xx[45] * xx[55];
  xx[69] = xx[59] + xx[68];
  xx[70] = xx[47] * xx[56];
  xx[71] = xx[46];
  xx[72] = - xx[69];
  xx[73] = xx[70];
  pm_math_Vector3_cross_ra(xx + 55, xx + 71, xx + 78);
  xx[71] = xx[1] * (xx[46] * xx[54] + xx[78]) - xx[47];
  xx[72] = xx[1] * (xx[79] - xx[69] * xx[54]);
  xx[73] = xx[45] + xx[1] * (xx[70] * xx[54] + xx[80]);
  xx[46] = xx[116] * xx[116];
  xx[78] = xx[1] * (xx[46] + xx[60]) - xx[2];
  xx[79] = xx[1] * (xx[33] + xx[34]);
  xx[80] = xx[82];
  xx[33] = xx[14] * xx[118];
  xx[34] = xx[14] * xx[117];
  xx[60] = xx[21] + xx[34];
  xx[21] = xx[16] * xx[118];
  xx[81] = - xx[33];
  xx[82] = xx[60];
  xx[83] = - xx[21];
  pm_math_Vector3_cross_ra(xx + 17, xx + 81, xx + 84);
  xx[81] = xx[1] * (xx[33] * xx[116] + xx[84]) - xx[16];
  xx[82] = xx[1] * (xx[85] - xx[60] * xx[116]);
  xx[83] = xx[14] + xx[1] * (xx[21] * xx[116] + xx[86]);
  xx[16] = xx[55] * xx[57];
  xx[21] = xx[54] * xx[56];
  xx[84] = xx[1] * (xx[16] + xx[21]);
  xx[85] = xx[1] * (xx[52] - xx[58]);
  xx[86] = xx[1] * (xx[49] + xx[57] * xx[57]) - xx[2];
  xx[33] = xx[1] * (xx[39] * xx[35] - xx[40] * xx[43]) * xx[111] + xx[112];
  xx[52] = xx[66] + xx[33];
  xx[58] = xx[45] * xx[57];
  xx[60] = xx[52] * xx[57];
  xx[65] = xx[52] * xx[56];
  xx[66] = xx[68] + xx[65];
  xx[67] = xx[58];
  xx[68] = xx[60];
  xx[69] = - xx[66];
  pm_math_Vector3_cross_ra(xx + 55, xx + 67, xx + 87);
  xx[67] = xx[52] + xx[1] * (xx[58] * xx[54] + xx[87]);
  xx[68] = xx[1] * (xx[60] * xx[54] + xx[88]) - xx[45];
  xx[69] = xx[1] * (xx[89] - xx[66] * xx[54]);
  xx[87] = xx[37];
  xx[88] = xx[1] * (xx[46] + xx[9]) - xx[2];
  xx[89] = xx[1] * (xx[64] + xx[75]);
  xx[9] = xx[14] * xx[119];
  xx[37] = xx[22] * xx[119];
  xx[58] = xx[34] + xx[20];
  xx[90] = - xx[9];
  xx[91] = - xx[37];
  xx[92] = xx[58];
  pm_math_Vector3_cross_ra(xx + 17, xx + 90, xx + 93);
  xx[90] = xx[22] + xx[1] * (xx[9] * xx[116] + xx[93]);
  xx[91] = xx[1] * (xx[37] * xx[116] + xx[94]) - xx[14];
  xx[92] = xx[1] * (xx[95] - xx[58] * xx[116]);
  xx[93] = xx[1] * (xx[49] + xx[55] * xx[55]) - xx[2];
  xx[94] = xx[1] * (xx[42] + xx[48]);
  xx[95] = xx[1] * (xx[16] - xx[21]);
  xx[9] = xx[65] + xx[59];
  xx[16] = xx[52] * xx[55];
  xx[20] = xx[47] * xx[55];
  xx[58] = - xx[9];
  xx[59] = xx[16];
  xx[60] = xx[20];
  pm_math_Vector3_cross_ra(xx + 55, xx + 58, xx + 64);
  xx[58] = xx[1] * (xx[64] - xx[9] * xx[54]);
  xx[59] = xx[47] + xx[1] * (xx[16] * xx[54] + xx[65]);
  xx[60] = xx[1] * (xx[20] * xx[54] + xx[66]) - xx[52];
  xx[47] = xx[1] * (xx[53] + xx[51]);
  xx[48] = xx[77];
  xx[49] = xx[1] * (xx[46] + xx[13]) - xx[2];
  xx[2] = xx[76] * xx[22];
  xx[9] = xx[76] * xx[14];
  xx[13] = xx[9] * xx[119];
  xx[14] = xx[2] * xx[119];
  xx[16] = xx[9] * xx[117] + xx[2] * xx[118];
  xx[20] = - xx[13];
  xx[21] = - xx[14];
  xx[22] = xx[16];
  pm_math_Vector3_cross_ra(xx + 17, xx + 20, xx + 64);
  xx[17] = xx[36] * xx[45];
  xx[18] = xx[17] * xx[57];
  xx[19] = xx[36] * xx[52];
  xx[20] = xx[19] * xx[57];
  xx[21] = xx[17] * xx[55] + xx[19] * xx[56];
  xx[51] = - xx[18];
  xx[52] = - xx[20];
  xx[53] = xx[21];
  pm_math_Vector3_cross_ra(xx + 55, xx + 51, xx + 74);
  xx[22] = xx[6] * xx[102];
  xx[34] = xx[7] * xx[102];
  xx[36] = xx[6] * xx[100] + xx[7] * xx[101];
  xx[51] = - xx[22];
  xx[52] = - xx[34];
  xx[53] = xx[36];
  pm_math_Vector3_cross_ra(xx + 23, xx + 51, xx + 55);
  xx[23] = xx[3];
  xx[24] = xx[38];
  xx[25] = xx[41];
  xx[3] = xx[50] * xx[44];
  xx[37] = xx[3] * xx[43];
  xx[38] = xx[50] * xx[33];
  xx[33] = xx[38] * xx[43];
  xx[41] = xx[3] * xx[35] + xx[38] * xx[39];
  xx[44] = xx[37];
  xx[45] = xx[33];
  xx[46] = - xx[41];
  pm_math_Vector3_cross_ra(xx + 23, xx + 44, xx + 51);
  xx[23] = xx[50] * xx[39];
  xx[24] = xx[50] * xx[35];
  xx[44] = - (xx[1] * (xx[23] * xx[40] + xx[24] * xx[43]));
  xx[45] = xx[1] * (xx[24] * xx[40] - xx[23] * xx[43]);
  xx[46] = xx[1] * (xx[24] * xx[35] + xx[23] * xx[39]) - 1.045;
  pm_math_Vector3_cross_ra(xx + 6, xx + 44, xx + 23);
  xx[42] = xx[1] * (xx[51] - xx[37] * xx[40]) - xx[38] + xx[23];
  xx[43] = xx[1] * (xx[52] - xx[33] * xx[40]) + xx[3] + xx[24];
  xx[44] = xx[1] * (xx[53] + xx[40] * xx[41]) + xx[25];
  pm_math_Quaternion_xform_ra(xx + 29, xx + 42, xx + 23);
  xx[37] = pm_math_Vector3_dot_ra(xx + 26, xx + 61) + pm_math_Vector3_dot_ra(xx
    + 71, xx + 78);
  xx[38] = pm_math_Vector3_dot_ra(xx + 81, xx + 84) + pm_math_Vector3_dot_ra(xx
    + 67, xx + 87);
  xx[39] = pm_math_Vector3_dot_ra(xx + 90, xx + 93) + pm_math_Vector3_dot_ra(xx
    + 58, xx + 47);
  xx[40] = xx[2] + xx[1] * (xx[13] * xx[116] + xx[64]) + xx[10] - (xx[1] * (xx
    [74] - xx[18] * xx[54]) - xx[19] + xx[7] + xx[1] * (xx[22] * xx[99] + xx[55])
    + xx[0] + xx[23]);
  xx[41] = xx[1] * (xx[14] * xx[116] + xx[65]) - xx[9] + xx[11] - (xx[1] * (xx
    [75] - xx[20] * xx[54]) + xx[17] + xx[1] * (xx[34] * xx[99] + xx[56]) - xx[6]
    + xx[4] + xx[24]);
  xx[42] = xx[1] * (xx[66] - xx[16] * xx[116]) + xx[12] - (xx[1] * (xx[76] + xx
    [21] * xx[54]) + xx[1] * (xx[57] - xx[36] * xx[99]) + xx[5] + xx[25]);
  zeroMajor(1, 6, ii + 0, xx + 37);
  xx[0] = fabs(xx[37]);
  xx[1] = fabs(xx[38]);
  xx[2] = fabs(xx[39]);
  xx[3] = fabs(xx[40]);
  xx[4] = fabs(xx[41]);
  xx[5] = fabs(xx[42]);
  ii[0] = 0;

  {
    int ll;
    for (ll = 1; ll < 6; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 0;
  xx[6] = xx[0 + (ii[0])];
  if (xx[6] > xx[15]) {
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
