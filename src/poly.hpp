#ifndef NB_POLY_H
#define NB_POLY_H

#include <assert.h>
#include "solid.hpp"

class poly : public solid {
	public:
		poly(real_t *points, unsigned num_pts, real_t e=0.0);
		//TODO maybe have a vararg version of the above
};

#endif
