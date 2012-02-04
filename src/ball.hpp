#ifndef NB_BALL_H
#define NB_BALL_H

#include <assert.h>
#include "solid.hpp"

class ball : public solid {
	public:
		ball(real_t x=0.0, real_t y=0.0, real_t r=0.0, real_t e=0.0);
		real_t left_edge(void);
		real_t top_edge(void);
};

#endif
