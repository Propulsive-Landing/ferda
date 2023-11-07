//
// File: rtGetNaN.cpp
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

#include "rtwtypes.h"

extern "C"
{

#include "rtGetNaN.h"

}

#include <stddef.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#define NumBitsPerChar                 8U

extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetNaN(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T nan{ 0.0 };

    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetNaNF(void)
  {
    IEEESingle nanF{ { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
