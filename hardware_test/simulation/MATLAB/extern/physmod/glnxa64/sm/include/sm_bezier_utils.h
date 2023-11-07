#ifndef __sm_bezier_utils_h__
#define __sm_bezier_utils_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void sm_core_math_validNormal(double*Nc,const double*T,const int dim);void
sm_core_math_castelEval(const size_t n,const size_t dim,const double*y,const
double u,double*res);int sm_core_math_indexOfLowerBound(const size_t n,const
double*array,const double u);double sm_core_math_findClosestPointOnBezier(
const size_t order,const size_t dim,const double*bcps,const double*S1k,const
double*S2k,const double*p,double*cp);double
sm_core_math_findClosestPointOnSpline(const size_t order,const size_t dim,
const size_t nBzs,const unsigned char isClosed,int sbzi,const double*knots,
const double*bcps,const double*tcps,const double*S1k,const double*S2k,const
double*pt,double*cpt);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_bezier_utils_h__ */
