#ifndef NB_ROPE_H
#define NB_ROPE_H

#include "vector.hpp"

/* margin of error within which rope is taut, not over/underextended */
#define NB_ROPE_SLACK (1.0)

#define NB_ROPE_COLOR (0xEEee00ff)
#define NB_ROPE_LOOSE_COLOR (0xb00f0Fff)

struct ropedata {
	real_t x1, y1, x2, y2;
	real_t length;
};

#endif
