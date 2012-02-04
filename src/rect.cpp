#include <assert.h>
#include "rect.hpp"

/* requires h, w >= 0 */
rect::rect(real_t x, real_t y, real_t h, real_t w, real_t e)
	: solid(x, y, e) {
	this->solid_type = NB_SLD_RECT;
	assert(h >= 0.0);
	assert(w >= 0.0);
	this->solid_data->rect_data.h = h;
	this->solid_data->rect_data.w = w;
}
