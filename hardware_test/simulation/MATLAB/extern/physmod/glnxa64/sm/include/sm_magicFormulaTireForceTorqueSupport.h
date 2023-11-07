#ifndef __sm_magicFormulaTireForceTorqueSupport_h__
#define __sm_magicFormulaTireForceTorqueSupport_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct sm_core_compiler_MFTP_ScalingCoefficients_Tag{real_T lfzo;real_T lcx;
real_T lmux;real_T lex;real_T lkx;real_T lhx;real_T lvx;real_T lcy;real_T lmuy
;real_T ley;real_T lky;real_T lkyc;real_T lkzc;real_T lhy;real_T lvy;real_T ltr
;real_T lres;real_T lxal;real_T lyka;real_T lvyka;real_T ls;real_T lmx;real_T
lvmx;real_T lmy;real_T lmp;};typedef struct
sm_core_compiler_MFTP_ScalingCoefficients_Tag
sm_core_compiler_MFTP_ScalingCoefficients;struct
sm_core_compiler_MFTP_Model_Tag{real_T tyreside;real_T longvl;real_T vxlow;};
typedef struct sm_core_compiler_MFTP_Model_Tag sm_core_compiler_MFTP_Model;
struct sm_core_compiler_MFTP_Dimension_Tag{real_T unloaded_radius;real_T width
;real_T aspect_ratio;};typedef struct sm_core_compiler_MFTP_Dimension_Tag
sm_core_compiler_MFTP_Dimension;struct
sm_core_compiler_MFTP_OperatingConditions_Tag{real_T inflpres;real_T nompres;}
;typedef struct sm_core_compiler_MFTP_OperatingConditions_Tag
sm_core_compiler_MFTP_OperatingConditions;struct
sm_core_compiler_MFTP_Vertical_Tag{real_T fnomin;real_T vertical_stiffness;
real_T breff;real_T dreff;real_T freff;real_T q_re0;real_T q_v1;real_T q_v2;
real_T q_fcy;real_T q_fcy2;real_T q_fz2;real_T q_cam1;real_T q_cam2;real_T
q_cam3;real_T q_fys1;real_T q_fys2;real_T q_fys3;real_T pfz1;};typedef struct
sm_core_compiler_MFTP_Vertical_Tag sm_core_compiler_MFTP_Vertical;struct
sm_core_compiler_MFTP_InflationPressureRange_Tag{real_T presmin;real_T presmax
;};typedef struct sm_core_compiler_MFTP_InflationPressureRange_Tag
sm_core_compiler_MFTP_InflationPressureRange;struct
sm_core_compiler_MFTP_VerticalForceRange_Tag{real_T fzmin;real_T fzmax;};
typedef struct sm_core_compiler_MFTP_VerticalForceRange_Tag
sm_core_compiler_MFTP_VerticalForceRange;struct
sm_core_compiler_MFTP_LongSlipRange_Tag{real_T kpumin;real_T kpumax;};typedef
struct sm_core_compiler_MFTP_LongSlipRange_Tag
sm_core_compiler_MFTP_LongSlipRange;struct
sm_core_compiler_MFTP_SlipAngleRange_Tag{real_T alpmin;real_T alpmax;};typedef
struct sm_core_compiler_MFTP_SlipAngleRange_Tag
sm_core_compiler_MFTP_SlipAngleRange;struct
sm_core_compiler_MFTP_InclinationAngleRange_Tag{real_T cammin;real_T cammax;};
typedef struct sm_core_compiler_MFTP_InclinationAngleRange_Tag
sm_core_compiler_MFTP_InclinationAngleRange;struct
sm_core_compiler_MFTP_LongitudinalCoefficients_Tag{real_T pcx1;real_T pdx1;
real_T pdx2;real_T pdx3;real_T pex1;real_T pex2;real_T pex3;real_T pex4;real_T
pkx1;real_T pkx2;real_T pkx3;real_T phx1;real_T phx2;real_T pvx1;real_T pvx2;
real_T rbx1;real_T rbx2;real_T rbx3;real_T rcx1;real_T rex1;real_T rex2;real_T
rhx1;real_T ppx1;real_T ppx2;real_T ppx3;real_T ppx4;};typedef struct
sm_core_compiler_MFTP_LongitudinalCoefficients_Tag
sm_core_compiler_MFTP_LongitudinalCoefficients;struct
sm_core_compiler_MFTP_OverturningCoefficients_Tag{real_T qsx1;real_T qsx2;
real_T qsx3;real_T qsx4;real_T qsx5;real_T qsx6;real_T qsx7;real_T qsx8;real_T
qsx9;real_T qsx10;real_T qsx11;real_T qsx12;real_T qsx13;real_T qsx14;real_T
ppmx1;};typedef struct sm_core_compiler_MFTP_OverturningCoefficients_Tag
sm_core_compiler_MFTP_OverturningCoefficients;struct
sm_core_compiler_MFTP_LateralCoefficients_Tag{real_T pcy1;real_T pdy1;real_T
pdy2;real_T pdy3;real_T pey1;real_T pey2;real_T pey3;real_T pey4;real_T pey5;
real_T pky1;real_T pky2;real_T pky3;real_T pky4;real_T pky5;real_T pky6;real_T
pky7;real_T phy1;real_T phy2;real_T pvy1;real_T pvy2;real_T pvy3;real_T pvy4;
real_T rby1;real_T rby2;real_T rby3;real_T rby4;real_T rcy1;real_T rey1;real_T
rey2;real_T rhy1;real_T rhy2;real_T rvy1;real_T rvy2;real_T rvy3;real_T rvy4;
real_T rvy5;real_T rvy6;real_T ppy1;real_T ppy2;real_T ppy3;real_T ppy4;real_T
ppy5;};typedef struct sm_core_compiler_MFTP_LateralCoefficients_Tag
sm_core_compiler_MFTP_LateralCoefficients;struct
sm_core_compiler_MFTP_RollingCoefficients_Tag{real_T qsy1;real_T qsy2;real_T
qsy3;real_T qsy4;real_T qsy5;real_T qsy6;real_T qsy7;real_T qsy8;};typedef
struct sm_core_compiler_MFTP_RollingCoefficients_Tag
sm_core_compiler_MFTP_RollingCoefficients;struct
sm_core_compiler_MFTP_AligningCoefficients_Tag{real_T qbz1;real_T qbz2;real_T
qbz3;real_T qbz4;real_T qbz5;real_T qbz9;real_T qbz10;real_T qcz1;real_T qdz1;
real_T qdz2;real_T qdz3;real_T qdz4;real_T qdz6;real_T qdz7;real_T qdz8;real_T
qdz9;real_T qdz10;real_T qdz11;real_T qez1;real_T qez2;real_T qez3;real_T qez4
;real_T qez5;real_T qhz1;real_T qhz2;real_T qhz3;real_T qhz4;real_T ssz1;
real_T ssz2;real_T ssz3;real_T ssz4;real_T ppz1;real_T ppz2;};typedef struct
sm_core_compiler_MFTP_AligningCoefficients_Tag
sm_core_compiler_MFTP_AligningCoefficients;struct
sm_core_compiler_MFTP_TurnSlipCoefficients_Tag{real_T pdxp1;real_T pdxp2;
real_T pdxp3;real_T pkyp1;real_T pdyp1;real_T pdyp2;real_T pdyp3;real_T pdyp4;
real_T phyp1;real_T phyp2;real_T phyp3;real_T phyp4;real_T pecp1;real_T pecp2;
real_T qdtp1;real_T qcrp1;real_T qcrp2;real_T qbrp1;real_T qdrp1;};typedef
struct sm_core_compiler_MFTP_TurnSlipCoefficients_Tag
sm_core_compiler_MFTP_TurnSlipCoefficients;struct
sm_core_compiler_MagicFormulaTireParametersTag{sm_core_compiler_MFTP_Model
model;sm_core_compiler_MFTP_Dimension dimension;
sm_core_compiler_MFTP_OperatingConditions operatingConditions;
sm_core_compiler_MFTP_Vertical vertical;
sm_core_compiler_MFTP_InflationPressureRange inflationPressureRange;
sm_core_compiler_MFTP_VerticalForceRange verticalForceRange;
sm_core_compiler_MFTP_LongSlipRange longSlipRange;
sm_core_compiler_MFTP_SlipAngleRange slipAngleRange;
sm_core_compiler_MFTP_InclinationAngleRange inclinationAngleRange;
sm_core_compiler_MFTP_LongitudinalCoefficients longitudinalCoefficients;
sm_core_compiler_MFTP_OverturningCoefficients overturningCoefficients;
sm_core_compiler_MFTP_LateralCoefficients lateralCoefficients;
sm_core_compiler_MFTP_RollingCoefficients rollingCoefficients;
sm_core_compiler_MFTP_AligningCoefficients aligningCoefficients;
sm_core_compiler_MFTP_TurnSlipCoefficients turnSlipCoefficients;};typedef
struct sm_core_compiler_MagicFormulaTireParametersTag
sm_core_compiler_MagicFormulaTireParameters;struct
sm_core_compiler_MagicFormulaNumericParametersTag{real_T epsx;real_T epsy;
real_T epsk;real_T trwSgnVx;real_T trwSgnOmega;real_T trwSgnKappaX;real_T
trwSgnAlphaY;real_T trwSgnAlphaR;real_T trwSgnAlphaT;real_T trwAbsGamma;real_T
trwAbsOmega;real_T trwSatAbsVx;real_T trwSatGamma;real_T trwSatKappa;real_T
trwSatAlpha;real_T trwSatDFz;real_T trwSatPi;real_T epsr;real_T trwAbsPhit;
real_T trwAbsPhi;real_T trwAbsDrp;};typedef struct
sm_core_compiler_MagicFormulaNumericParametersTag
sm_core_compiler_MagicFormulaNumericParameters;void
sm_core_compiler_computeMagicFormulaTireWrench(boolean_T isValid,const
pm_math_Transform3*Tct,const pm_math_Transform3*Tgt,const pm_math_SpatialVector
*Vgeom,const pm_math_SpatialVector*Vtire,int32_T tireSide,int32_T slipMode,
const sm_core_compiler_MFTP_ScalingCoefficients*scalingCoefficients,const
sm_core_compiler_MagicFormulaTireParameters*tireParameters,const
sm_core_compiler_MagicFormulaNumericParameters*numericParameters,
pm_math_Vector3*tireForce,pm_math_Vector3*tireTorque,real_T*intermediateVars);
real_T sm_core_compiler_computeMagicFormulaTireFreeRadius(real_T omega,const
sm_core_compiler_MagicFormulaTireParameters*tireParameters);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_magicFormulaTireForceTorqueSupport_h__ */
