#ifndef NB_SEG_H
#define NB_SEG_H

#include <assert.h>
#include "solid.hpp"

class seg : public solid {
	public:
		seg(real_t x=0.0, real_t y=0.0, real_t dx=1.0, real_t dy=1.0,
				bool directed = false, real_t e=0.0);
};

bool point_on_segment(real_t x, real_t y, seg& segment);

#endif
