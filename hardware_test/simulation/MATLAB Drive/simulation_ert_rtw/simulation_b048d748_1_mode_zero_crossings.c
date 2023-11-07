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

const NeZCData simulation_b048d748_1_ZCData[1] = {
  { "simulation/Plant/ TVC Physics/Staging Joint", 0, 0,
    "Plant.TVC_Physics.Staging_Joint", "", 2 }
};

PmfMessageId simulation_b048d748_1_computeAsmModeVector(const double *input,
  const double *inputDot, const double *inputDdot, int *modeVector, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  boolean_T bb[1];
  int ii[1];
  double xx[1];
  (void) inputDot;
  (void) inputDdot;
  (void) neDiagMgr;
  bb[0] = input[2] == - 1.0;
  ii[0] = bb[0] ? -1 : 0;
  xx[0] = 0.0;
  if (!(input[2] == xx[0] || bb[0])) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:constraint:cutJoint:invalidModeInput",
      "The mode signal input to 'simulation/Plant/ TVC Physics/Staging Joint' is invalid. It must be either 0 or -1.",
      neDiagMgr);
  }

  modeVector[0] = ii[0];
  errorResult[0] = xx[0];
  return NULL;
}

PmfMessageId simulation_b048d748_1_computeSimModeVector(const double *input,
  const double *inputDot, const double *inputDdot, int *modeVector, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  boolean_T bb[1];
  int ii[1];
  double xx[1];
  (void) inputDot;
  (void) inputDdot;
  (void) neDiagMgr;
  bb[0] = input[2] == - 1.0;
  ii[0] = bb[0] ? -1 : 0;
  xx[0] = 0.0;
  if (!(input[2] == xx[0] || bb[0])) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:constraint:cutJoint:invalidModeInput",
      "The mode signal input to 'simulation/Plant/ TVC Physics/Staging Joint' is invalid. It must be either 0 or -1.",
      neDiagMgr);
  }

  modeVector[0] = ii[0];
  errorResult[0] = xx[0];
  return NULL;
}

PmfMessageId simulation_b048d748_1_computeZeroCrossings(const
  RuntimeDerivedValuesBundle *rtdv, const double *solverStateVector, const
  double *input, const double *inputDot, const double *inputDdot, const double
  *discreteStateVector, double *zeroCrossingsVector, double *errorResult,
  NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  boolean_T bb[1];
  double xx[3];
  (void) rtdvd;
  (void) rtdvi;
  (void) solverStateVector;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteStateVector;
  (void) neDiagMgr;
  xx[0] = 1.0;
  xx[1] = - xx[0];
  bb[0] = input[2] == xx[1];
  xx[2] = bb[0] ? xx[1] : xx[0];
  xx[0] = 0.0;
  if (!(input[2] == xx[0] || bb[0])) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:core:compiler:constraint:cutJoint:invalidModeInput",
      "The mode signal input to 'simulation/Plant/ TVC Physics/Staging Joint' is invalid. It must be either 0 or -1.",
      neDiagMgr);
  }

  zeroCrossingsVector[0] = xx[2];
  errorResult[0] = xx[0];
  return NULL;
}
