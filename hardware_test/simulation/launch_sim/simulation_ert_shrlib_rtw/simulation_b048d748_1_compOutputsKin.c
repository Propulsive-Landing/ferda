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
#include "simulation_b048d748_1_geometries.h"

PmfMessageId simulation_b048d748_1_compOutputsKin(const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *input, const double *inputDot, const double *inputDdot, const
  double *discreteState, double *output, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  boolean_T bb[1];
  double xx[19];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 9.87654321;
  xx[1] = 2.0;
  xx[2] = (state[3] * state[5] + state[4] * state[6]) * xx[1];
  xx[3] = xx[1] * (state[3] * state[4] - state[5] * state[6]);
  xx[4] = (state[4] * state[4] + state[5] * state[5]) * xx[1];
  xx[5] = - state[3];
  xx[6] = - state[4];
  xx[7] = - state[5];
  xx[8] = - state[6];
  xx[9] = state[0] + xx[2] - xx[2];
  xx[10] = xx[3] + state[1] - xx[3];
  xx[11] = xx[4] + state[2] - xx[4];
  xx[1] = 1.0;
  xx[2] = 0.0;
  xx[12] = xx[1];
  xx[13] = xx[2];
  xx[14] = xx[2];
  xx[15] = xx[2];
  xx[16] = xx[2];
  xx[17] = xx[2];
  xx[18] = - xx[1];
  bb[0] = sm_core_compiler_computeSignedDistanceCxpolyBrick(
    simulation_b048d748_1_geometry_0(NULL), simulation_b048d748_1_geometry_3
    (NULL), (pm_math_Transform3 *)(xx + 5), (pm_math_Transform3 *)(xx + 12), xx
    + 3);
  output[0] = state[0];
  output[1] = state[7];
  output[3] = state[1];
  output[4] = state[8];
  output[6] = state[2];
  output[7] = state[9];
  output[9] = state[3];
  output[10] = state[4];
  output[11] = state[5];
  output[12] = state[6];
  output[13] = xx[3] < xx[2] ? xx[1] : xx[2];
  output[14] = state[3];
  output[15] = state[4];
  output[16] = state[5];
  output[17] = state[6];
  output[18] = state[10];
  output[19] = state[11];
  output[20] = state[12];
  return NULL;
}
