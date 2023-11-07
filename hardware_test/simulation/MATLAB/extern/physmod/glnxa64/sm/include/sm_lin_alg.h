#ifndef __sm_lin_alg_h__
#define __sm_lin_alg_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void vectorAdd(uint32_T n,const double*u,const double*v,double*sum);void
vectorAddDynamicSize(uint32_T maxN,uint32_T n,const double*u,const double*v,
double*sum);void vectorSubtract(uint32_T n,const double*u,const double*v,
double*diff);void vectorSubtractDynamicSize(uint32_T maxN,uint32_T n,const
double*u,const double*v,double*diff);void vectorScale(uint32_T n,double s,
const double*v,double*sv);void vectorNegate(uint32_T n,const double*v,double*
negV);void submatrixAssign(uint32_T m,uint32_T n,uint32_T SOffset,uint32_T
SStride,uint32_T DOffset,uint32_T DStride,const double*S,double*D);void
submatrixAssignDynamicSize(uint32_T maxSrcStrideN,uint32_T maxDstStrideN,
uint32_T m,uint32_T n,uint32_T SOffset,uint32_T SStride,uint32_T DOffset,
uint32_T DStride,const double*S,double*D);void transposeColMajor(uint32_T m,
uint32_T n,const double*A,double*At);void transposeColMajorDynamicSize(
uint32_T maxM,uint32_T maxN,uint32_T m,uint32_T n,const double*A,double*At);
void matrixTransposeAdd(uint32_T n,const double*A,double*S);void
matrixTransposeSubtract(uint32_T n,const double*A,double*D);void shiftColumns(
uint32_T m,uint32_T n,uint32_T numCols,const double*A,double*S);void
shiftColumnsDynamicSize(uint32_T maxM,uint32_T maxN,uint32_T m,uint32_T n,
uint32_T numCols,const double*A,double*S);void scaleColumns(uint32_T m,
uint32_T n,const double*A,const double*s,double*sA);void
scaleColumnsDynamicSize(uint32_T maxM,uint32_T maxN,uint32_T m,uint32_T n,
const double*A,const double*s,double*sA);void inverseScaleColumns(uint32_T m,
uint32_T n,const double*A,const double*s,double*sA);void
inverseScaleColumnsDynamicSize(uint32_T maxM,uint32_T maxN,uint32_T m,uint32_T
n,const double*A,const double*s,double*sA);void scaleRows(uint32_T m,uint32_T n
,const double*A,const double*s,double*sA);void scaleRowsDynamicSize(uint32_T
maxM,uint32_T maxN,uint32_T m,uint32_T n,const double*A,const double*s,double*
sA);void inverseScaleRows(uint32_T m,uint32_T n,const double*A,const double*s,
double*sA);void inverseScaleRowsDynamicSize(uint32_T maxM,uint32_T maxN,
uint32_T m,uint32_T n,const double*A,const double*s,double*sA);void zeroMajor(
uint32_T majorDim,uint32_T minorDim,const int32_T*mask,double*A);void
reduceMatrix(uint32_T m,uint32_T n,uint32_T numRetainedCols,const int32_T*
retainedCols,double*A);void reduceMatrixDynamicSize(uint32_T maxM,uint32_T maxN
,uint32_T m,uint32_T n,uint32_T numRetainedCols,const int32_T*retainedCols,
double*A);void expandVector(uint32_T n,uint32_T nReduced,const int32_T*
reducedToFullMap,double*v);boolean_T hasUniqueRows(uint32_T m,uint32_T n,const
double*A,double*work);void matrixMultiply(uint32_T m,uint32_T k,uint32_T n,
const double*A,const double*B,double*C);void matrixMultiplyDynamicSize(
uint32_T maxM,uint32_T maxK,uint32_T maxN,uint32_T m,uint32_T k,uint32_T n,
const double*A,const double*B,double*C);void matrixMultiplyStrided(uint32_T m,
uint32_T k,uint32_T n,uint32_T AStride,uint32_T BStride,uint32_T CStride,const
double*A,const double*B,double*C);void matrixMultiplyStridedDynamicSize(
uint32_T maxK,uint32_T maxN,uint32_T maxAStride,uint32_T maxBStride,uint32_T
maxCStride,uint32_T m,uint32_T k,uint32_T n,uint32_T AStride,uint32_T BStride,
uint32_T CStride,const double*A,const double*B,double*C);void
matrixTransposeMultiply(uint32_T k,uint32_T m,uint32_T n,const double*A,const
double*B,double*C);void matrixTransposeMultiplyDynamicSize(uint32_T maxK,
uint32_T maxM,uint32_T maxN,uint32_T k,uint32_T m,uint32_T n,const double*A,
const double*B,double*C);void matrixMultiplyTranspose(uint32_T m,uint32_T k,
uint32_T n,const double*A,const double*B,double*C);void
matrixMultiplyTransposeDynamicSize(uint32_T maxM,uint32_T maxK,uint32_T maxN,
uint32_T m,uint32_T k,uint32_T n,const double*A,const double*B,double*C);void
matrixVectorMultiply(uint32_T m,uint32_T n,const double*A,const double*x,
double*Ax);void matrixVectorMultiplyDynamicSize(uint32_T maxM,uint32_T maxN,
uint32_T m,uint32_T n,const double*A,const double*x,double*Ax);void
matrixTransposeVectorMultiply(uint32_T m,uint32_T n,const double*A,const double
*x,double*Atx);void matrixTransposeVectorMultiplyDynamicSize(uint32_T maxM,
uint32_T maxN,uint32_T m,uint32_T n,const double*A,const double*x,double*Atx);
void addMatrixVectorProduct(uint32_T m,uint32_T n,const double*A,const double*
x,double*y);void addMatrixVectorProductDynamicSize(uint32_T maxM,uint32_T maxN
,uint32_T m,uint32_T n,const double*A,const double*x,double*y);void
addMatrixTransposeVectorProduct(uint32_T m,uint32_T n,const double*A,const
double*x,double*y);void addMatrixTransposeVectorProductDynamicSize(uint32_T
maxM,uint32_T maxN,uint32_T m,uint32_T n,const double*A,const double*x,double*
y);void subtractMatrixVectorProduct(uint32_T m,uint32_T n,const double*A,const
double*x,double*y);void subtractMatrixVectorProductDynamicSize(uint32_T maxM,
uint32_T maxN,uint32_T m,uint32_T n,const double*A,const double*x,double*y);
void subtractMatrixTransposeVectorProduct(uint32_T m,uint32_T n,const double*A
,const double*x,double*y);void subtractMatrixTransposeVectorProductDynamicSize
(uint32_T maxM,uint32_T maxN,uint32_T m,uint32_T n,const double*A,const double
*x,double*y);double dotProduct(uint32_T n,const double*u,const double*v);
double norm(uint32_T n,const double*v);double infNorm(uint32_T n,const double*
v);double computeQuadraticTerm(uint32_T n,const double*M,const double*u,const
double*v);void geSolve2x2(const real_T A[4],const real_T b[2],real_T x[2]);
int32_T factorSymmetricPosDef(real_T*A,uint32_T n,real_T*work);void
solveSymmetricPosDef(const real_T*A,const real_T*b,uint32_T n,uint32_T k,
real_T*x,real_T*work);void factorAndSolveSymmetric(real_T*A,uint32_T n,double*
qrAux,int32_T*qrPivots,const real_T*b,real_T*x,real_T*work);void
factorAndSolveWide(uint32_T m,uint32_T n,real_T*A,real_T*qraux,real_T*work,
int32_T*jpvt,const real_T*b,real_T relTol,real_T*x);void eig(uint32_T n,const
real_T*A,const real_T*B,real_T*d_re,real_T*d_im,real_T*V_re,real_T*V_im,real_T
*realWork,int32_T*int32Work);void eigDynamicSize(uint32_T maxN,uint32_T n,
const real_T*A,const real_T*B,real_T*d_re,real_T*d_im,real_T*V_re,real_T*V_im,
real_T*realWork,int32_T*int32Work);int32_T
orthonormalizeRealEigenvectorsWrtSpdB(uint32_T n,const real_T*B,real_T*V,
real_T*work);int32_T orthonormalizeRealEigenvectorsWrtSpdBDynamicSize(uint32_T
maxN,uint32_T n,const real_T*B,real_T*V,real_T*work);void svd(uint32_T m,
uint32_T n,const real_T*A,boolean_T doFull,real_T*Sdiag,real_T*U,real_T*V,
real_T*work);void svdDynamicSize(uint32_T maxM,uint32_T maxN,uint32_T m,
uint32_T n,const real_T*A,boolean_T doFull,real_T*Sdiag,real_T*U,real_T*V,
real_T*work);uint32_T svdComputeRank(uint32_T m,uint32_T n,const real_T*Sdiag,
real_T rankTol);uint32_T svdComputeRankDynamicSize(uint32_T maxM,uint32_T maxN
,uint32_T m,uint32_T n,const real_T*Sdiag,real_T rankTol);void svdSolve(
uint32_T m,uint32_T n,const real_T*A,const real_T*b,real_T rankTol,real_T*x,
real_T*work);void svdSolveDynamicSize(uint32_T maxM,uint32_T maxN,uint32_T m,
uint32_T n,const real_T*A,const real_T*b,real_T rankTol,real_T*x,real_T*work);
uint32_T svdSolveFromFactorization(uint32_T m,uint32_T n,const real_T*Sdiag,
const real_T*U,const real_T*V,const real_T*b,real_T rankTol,real_T*x,real_T*
work);uint32_T svdSolveFromFactorizationDynamicSize(uint32_T maxM,uint32_T maxN
,uint32_T m,uint32_T n,const real_T*Sdiag,const real_T*U,const real_T*V,const
real_T*b,real_T rankTol,real_T*x,real_T*work);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_lin_alg_h__ */
