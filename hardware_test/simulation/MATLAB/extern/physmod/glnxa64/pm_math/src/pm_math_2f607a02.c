#include "pm_std.h"
#include "math.h"
#include "pm_std.h"
double pm_math_canonicalAngle(double pm_math_Flz54uVsi__UbP_LLGOB06);double
pm_math_synchronizeAngle(double pm_math_k2YW1d8jJI4zZH1tKJFKPV,double
pm_math_ktvnnQE_5n0DXXEqVLWCD3);double pm_math_canonicalAngle(double
pm_math__AExROh1PVWNeP7hmMWuJv){const double pm_math_FAgKZ8DJk6pig9GuHdUp8H=
pmf_get_pi();const double pm_math_kn2XwHZfIMKAd92Buf9DSH=2.0*
pm_math_FAgKZ8DJk6pig9GuHdUp8H;if(pm_math__AExROh1PVWNeP7hmMWuJv>=0.0)return
fmod(pm_math_FAgKZ8DJk6pig9GuHdUp8H+pm_math__AExROh1PVWNeP7hmMWuJv,
pm_math_kn2XwHZfIMKAd92Buf9DSH)-pm_math_FAgKZ8DJk6pig9GuHdUp8H;else{const
double pm_math_FmI56rPhzzKAX5Ej10Sy5Z=pm_math_FAgKZ8DJk6pig9GuHdUp8H-fmod(
pm_math_FAgKZ8DJk6pig9GuHdUp8H-pm_math__AExROh1PVWNeP7hmMWuJv,
pm_math_kn2XwHZfIMKAd92Buf9DSH);return(pm_math_FmI56rPhzzKAX5Ej10Sy5Z<
pm_math_FAgKZ8DJk6pig9GuHdUp8H)?pm_math_FmI56rPhzzKAX5Ej10Sy5Z:-
pm_math_FAgKZ8DJk6pig9GuHdUp8H;}}double pm_math_synchronizeAngle(double
pm_math_k2YW1d8jJI4zZH1tKJFKPV,double pm_math_ktvnnQE_5n0DXXEqVLWCD3){const
double pm_math_kEw8RhbAZIGdfeViKpSqbH=pm_math_canonicalAngle(
pm_math_ktvnnQE_5n0DXXEqVLWCD3-pm_math_k2YW1d8jJI4zZH1tKJFKPV);return
pm_math_k2YW1d8jJI4zZH1tKJFKPV+pm_math_kEw8RhbAZIGdfeViKpSqbH;}
