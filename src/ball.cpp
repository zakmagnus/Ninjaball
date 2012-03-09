#include <assert.h>
#include "ball.hpp"

/* requires r >= 0 */
ball::ball(real_t x, real_t y, real_t r, SDL_Surface *img, real_t e)
	: solid(x, y, e) {
	this->solid_type = NB_SLD_BALL;
	assert(r >= 0.0);
	this->solid_data->ball_data.r = r;

	this->visible_type = NB_SLD_BALL;
	this->visible_data = img;
}

real_t ball::left_edge(void) {
	return this->x - this->solid_data->ball_data.r;
}

real_t ball::top_edge(void) {
	return this->y - this->solid_data->ball_data.r;
}
