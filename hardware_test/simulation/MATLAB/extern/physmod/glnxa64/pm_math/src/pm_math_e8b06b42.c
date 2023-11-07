#include "pm_std.h"
#include "math.h"
real_T pm_math_lin_alg_sgn(real_T x);real_T pm_math_lin_alg_biasedSgn(real_T x
);real_T pm_math_lin_alg_guardedDivide(real_T pm_math_FhiZqzaqadOsVP1Uvke9jy,
real_T pm_math_FVXcwERBgq_pgLcAw7YO4S);real_T pm_math_lin_alg_guardedSqrt(
real_T x);real_T pm_math_lin_alg_guardedAsin(real_T x);real_T
pm_math_lin_alg_guardedAcos(real_T x);real_T pm_math_lin_alg_smoothSgn(real_T x
,real_T pm_math_kUImP92nK540_i3uFj5UsY);real_T pm_math_lin_alg_smoothAbs(
real_T x,real_T pm_math_kUImP92nK540_i3uFj5UsY);real_T
pm_math_lin_alg_smoothSat(real_T x,real_T pm_math__lO81KuDBk41W9Wd2wAkb0,
real_T pm_math_kEbBObcYFIxUZ5_77V3CO_,real_T pm_math_kUImP92nK540_i3uFj5UsY);
real_T pm_math_lin_alg_maxReal(real_T a,real_T b);real_T pm_math_lin_alg_sgn(
real_T x){return x==0.0?0.0:(x<0.0?-1.0:+1.0);}real_T pm_math_lin_alg_biasedSgn
(real_T x){return x<0.0?-1.0:1.0;}real_T pm_math_lin_alg_guardedDivide(real_T
pm_math_FhiZqzaqadOsVP1Uvke9jy,real_T pm_math_FVXcwERBgq_pgLcAw7YO4S){return
pm_math_FVXcwERBgq_pgLcAw7YO4S!=0.0?pm_math_FhiZqzaqadOsVP1Uvke9jy/
pm_math_FVXcwERBgq_pgLcAw7YO4S:0.0;}real_T pm_math_lin_alg_guardedSqrt(real_T x
){return x<0.0?0.0:sqrt(x);}real_T pm_math_lin_alg_guardedAsin(real_T x){
return fabs(x)>1.0?atan2(x,0.0):asin(x);}real_T pm_math_lin_alg_guardedAcos(
real_T x){return fabs(x)>1.0?atan2(0.0,x):acos(x);}real_T
pm_math_lin_alg_smoothSgn(real_T x,real_T pm_math_kUImP92nK540_i3uFj5UsY){
real_T pm_math_FzyLWRgau0pMYq2XSI3ETL;const real_T
pm_math_kObNwJpzZMthiTC6_YfYhi=0.5*pm_math_kUImP92nK540_i3uFj5UsY;if(x<= -
pm_math_kObNwJpzZMthiTC6_YfYhi)pm_math_FzyLWRgau0pMYq2XSI3ETL= -1.0;else if(x<
pm_math_kObNwJpzZMthiTC6_YfYhi){const real_T pm_math_VATnYten07t9e5a13sKpTN=
pm_math_kUImP92nK540_i3uFj5UsY*pm_math_kUImP92nK540_i3uFj5UsY;
pm_math_FzyLWRgau0pMYq2XSI3ETL=x*(3.0*pm_math_VATnYten07t9e5a13sKpTN-4.0*x*x)/
(pm_math_kUImP92nK540_i3uFj5UsY*pm_math_VATnYten07t9e5a13sKpTN);}else
pm_math_FzyLWRgau0pMYq2XSI3ETL= +1.0;return pm_math_FzyLWRgau0pMYq2XSI3ETL;}
real_T pm_math_lin_alg_smoothAbs(real_T x,real_T pm_math_kUImP92nK540_i3uFj5UsY
){return x*pm_math_lin_alg_smoothSgn(x,pm_math_kUImP92nK540_i3uFj5UsY);}real_T
pm_math_lin_alg_smoothSat(real_T x,real_T pm_math__lO81KuDBk41W9Wd2wAkb0,
real_T pm_math_kEbBObcYFIxUZ5_77V3CO_,real_T pm_math_kUImP92nK540_i3uFj5UsY){
real_T pm_math_FzyLWRgau0pMYq2XSI3ETL;const real_T
pm_math_kObNwJpzZMthiTC6_YfYhi=0.5*pm_math_kUImP92nK540_i3uFj5UsY;if(x<=
pm_math__lO81KuDBk41W9Wd2wAkb0-pm_math_kObNwJpzZMthiTC6_YfYhi)
pm_math_FzyLWRgau0pMYq2XSI3ETL=pm_math__lO81KuDBk41W9Wd2wAkb0;else if(x<
pm_math__lO81KuDBk41W9Wd2wAkb0+pm_math_kObNwJpzZMthiTC6_YfYhi){const real_T
pm_math_Fm0rCgILnQdlZ5BjuXYWCk=2.0*pm_math__lO81KuDBk41W9Wd2wAkb0;
pm_math_FzyLWRgau0pMYq2XSI3ETL=(x*(pm_math_kUImP92nK540_i3uFj5UsY-
pm_math_Fm0rCgILnQdlZ5BjuXYWCk)+x*x)/(2.0*pm_math_kUImP92nK540_i3uFj5UsY)+pow(
pm_math_kUImP92nK540_i3uFj5UsY+pm_math_Fm0rCgILnQdlZ5BjuXYWCk,2.0)/(8.0*
pm_math_kUImP92nK540_i3uFj5UsY);}else if(x<=pm_math_kEbBObcYFIxUZ5_77V3CO_-
pm_math_kObNwJpzZMthiTC6_YfYhi)pm_math_FzyLWRgau0pMYq2XSI3ETL=x;else if(x<
pm_math_kEbBObcYFIxUZ5_77V3CO_+pm_math_kObNwJpzZMthiTC6_YfYhi){const real_T
pm_math_kp1X_PEfYyhUWDHZ86FyLb=2.0*pm_math_kEbBObcYFIxUZ5_77V3CO_;
pm_math_FzyLWRgau0pMYq2XSI3ETL=(x*(pm_math_kUImP92nK540_i3uFj5UsY+
pm_math_kp1X_PEfYyhUWDHZ86FyLb)-x*x)/(2.0*pm_math_kUImP92nK540_i3uFj5UsY)-pow(
pm_math_kUImP92nK540_i3uFj5UsY-pm_math_kp1X_PEfYyhUWDHZ86FyLb,2.0)/(8.0*
pm_math_kUImP92nK540_i3uFj5UsY);}else pm_math_FzyLWRgau0pMYq2XSI3ETL=
pm_math_kEbBObcYFIxUZ5_77V3CO_;return pm_math_FzyLWRgau0pMYq2XSI3ETL;}real_T
pm_math_lin_alg_maxReal(real_T a,real_T b){return a>=b?a:b;}
