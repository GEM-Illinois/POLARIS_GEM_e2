#ifndef APPROX_REAL_H
#define APPROX_REAL_H

#if APPROX == CBMC
    #include "approx_real.cbmc.h"
#elif APPROX == ESBMC
    #error ESBMC is going to be supported soon.
#elif APPROX == double 
    // Use double precision floating point and math function from C standard libraries
    #include <math.h>
    typedef double ApproxReal;
    static const ApproxReal ONE = 1.0;
    static const ApproxReal APPROX_PI = M_PI;
    static const ApproxReal APPROX_PI_2 = M_PI_2;
    static inline ApproxReal approx_abs(ApproxReal x){return fabs(x);}
    static inline ApproxReal approx_mul(ApproxReal x, ApproxReal y){return x*y;}
    static inline ApproxReal approx_div(ApproxReal x, ApproxReal y){return x/y;}
    static inline ApproxReal approx_sin(ApproxReal x){return sin(x);}
    static inline ApproxReal approx_cos(ApproxReal x){return cos(x);}
    static inline ApproxReal approx_atan(ApproxReal x){return atan(x);}
    static inline ApproxReal approx_atan2(ApproxReal y, ApproxReal x){return atan2(y, x);}
#elif APPROX == libfixmath
    #include <fix16.h>
    typedef fix16_t ApproxReal;
    static const ApproxReal ONE = fix16_one;
    static const ApproxReal APPROX_PI = fix16_pi;
    static const ApproxReal APPROX_PI_2 = fix16_pi/2;
    static inline ApproxReal approx_abs(ApproxReal x){return fix16_abs(x);}
    static inline ApproxReal approx_mul(ApproxReal x, ApproxReal y){return fix16_mul(x, y);}
    static inline ApproxReal approx_div(ApproxReal x, ApproxReal y){return fix16_div(x, y);}
    static inline ApproxReal approx_sin(ApproxReal x){return fix16_sin(x);}
    static inline ApproxReal approx_cos(ApproxReal x){return fix16_cos(x);}
    static inline ApproxReal approx_atan(ApproxReal x){return fix16_atan(x);}
    static inline ApproxReal approx_atan2(ApproxReal y, ApproxReal x){return fix16_atan2(y, x);}
#else
    #warning APPROX macro is not set or set to unknown value. Assume using CBMC
    #include "approx_real.cbmc.h"
#endif

extern ApproxReal nondet_approx_real();

#endif // APPROX_REAL_H
