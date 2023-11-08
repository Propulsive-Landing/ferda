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

PmfMessageId simulation_b048d748_1_checkDynamics(const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const double *input,
  const double *inputDot, const double *inputDdot, const double *discreteState,
  const int *modeVector, double *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) modeVector;
  (void) neDiagMgr;
  errorResult[0] = 0.0;
  return NULL;
}
