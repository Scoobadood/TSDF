/*
 *  Roots3And4.c
 *
 *  Utility functions to find cubic and quartic roots,
 *  coefficients are passed like this:
 *
 *      c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3 + c[4]*x^4 = 0
 *
 *  The functions return the number of non-complex roots and
 *  put the values into the s array.
 *
 *  Author:         Jochen Schwarze (schwarze@isa.de)
 *
 *  Jan 26, 1990    Version for Graphics Gems
 *  Oct 11, 1990    Fixed sign problem for negative q's in SolveQuartic
 *  	    	    (reported by Mark Podlipec),
 *  	    	    Old-style function definitions,
 *  	    	    IsZero() as a macro
 *  Nov 23, 1990    Some systems do not declare acos() and cbrt() in
 *                  <math.h>, though the functions exist in the library.
 *                  If large coefficients are used, EQN_EPS should be
 *                  reduced considerably (e.g. to 1E-30), results will be
 *                  correct but multiple roots might be reported more
 *                  than once.
 */

#include    <math.h>
#ifndef M_PI
#define M_PI          3.14159265358979323846
#endif
extern double   sqrt(), cbrt(), cos(), acos();

/* epsilon surrounding for near zero values */

#define     EQN_EPS     1e-30
#define	    IsZero(x)	((x) > -EQN_EPS && (x) < EQN_EPS)

#ifdef NOCBRT
#define     cbrt(x)     ((x) > 0.0 ? pow((double)(x), 1.0/3.0) : \
((x) < 0.0 ? -pow((double)-(x), 1.0/3.0) : 0.0))
#endif


/**
 * Solve for ax^3 + bx^2 + cx + d = 0
 * Solutions in s
 * Number of roots returned
 */
int solve_cubic( double a, double b, double c, double d, double s[3] ) {
    /* normal form: x^3 + Ax^2 + Bx + C = 0 */
    
    double A = b / a;
    double B = c / a;
    double C = d / a;
    
    /*  substitute x = y - A/3 to eliminate quadric term:
     x^3 +px + q = 0 */
    
    double sq_A = A * A;
    double p = 1.0/3 * (- 1.0/3 * sq_A + B);
    double q = 1.0/2 * (2.0/27 * A * sq_A - 1.0/3 * A * B + C);
    
    /* use Cardano's formula */
    
    double cb_p = p * p * p;
    double D = q * q + cb_p;
    
    int num;
    
    if (IsZero(D)) {
        if (IsZero(q)) {/* one triple solution */
            s[ 0 ] = 0;
            num = 1;
        } else { /* one single and one double solution */
            double u = cbrt(-q);
            s[ 0 ] = 2 * u;
            s[ 1 ] = - u;
            num = 2;
        }
    } else if (D < 0) { /* Casus irreducibilis: three real solutions */
        double phi = 1.0/3 * acos(-q / sqrt(-cb_p));
        double t = 2 * sqrt(-p);
        
        s[ 0 ] =   t * cos(phi);
        s[ 1 ] = - t * cos(phi + M_PI / 3);
        s[ 2 ] = - t * cos(phi - M_PI / 3);
        num = 3;
    } else { /* one real solution */
        double sqrt_D = sqrt(D);
        double u = cbrt(sqrt_D - q);
        double v = - cbrt(sqrt_D + q);
        
        s[ 0 ] = u + v;
        num = 1;
    }
    
    /* resubstitute */
    
    double sub = 1.0/3 * A;
    
    for ( int i = 0; i < num; ++i) {
        s[ i ] -= sub;
    }
    
    return num;
}