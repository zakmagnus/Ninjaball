#ifndef NB_RECT_H
#define NB_RECT_H

#include <assert.h>
#include "solid.hpp"

class rect : public solid {
	public:
		rect(real_t x=0.0, real_t y=0.0, real_t h=0.0, real_t w=0.0,
				real_t e=0.0);
};

#endif
