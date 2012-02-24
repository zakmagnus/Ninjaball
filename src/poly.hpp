#ifndef NB_POLY_H
#define NB_POLY_H

#include <assert.h>
#include "solid.hpp"

class poly : public solid {
	public:
		poly(real_t *points, unsigned num_pts, real_t x=0.0, real_t y=0.0, real_t e=0.0);
		//TODO maybe have a vararg version of the above
		poly(real_t x, real_t y, real_t e, int num_points, ...);
};

#endif
