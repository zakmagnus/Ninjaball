#ifndef NB_MATH_UTILS_H
#define NB_MATH_UTILS_H

#include <math.h>

#define min(a,b) ((a)<(b)? (a) : (b))
#define max(a,b) ((a)>=(b)? (a) : (b))
#define SINGLE_DIM_OVERLAP(v1min,v1max,v2min,v2max) \
	(max((v1max), (v2max)) - min((v1min), (v2min))\
	 - fabs((v1max) - (v2max))\
	 - fabs((v1min) - (v2min)))

#define between_ord(v1,v2,v3) ((v1) <= (v2) && (v2) <= (v3))
#define between(v1,v2,v3) (between_ord((v1),(v2),(v3)) ||\
		between_ord((v3),(v2),(v1)))

#define LINE_Y_COORD(x,m,x0,y0) ((((x) - (x0)) * (m)) + (y0))

#define REAL_EPSILON (0.001)
#define SIMILAR_REALS(r1,r2) (fabs((r1) - (r2)) <= REAL_EPSILON)

#endif
