#include <assert.h>
#include "seg.hpp"

/* requires dx, dy > 0 */
seg::seg(real_t x, real_t y, real_t dx, real_t dy, bool directed, real_t e)
	: solid(x, y, e) {
	this->solid_type = NB_SLD_SEG;
	assert(dx > 0.0);
	assert(dy > 0.0);
	vector2d_t *dir = new vector2d_t(dx, dy);
	this->solid_data->seg_data.dir = dir;
	this->solid_data->seg_data.directed = directed;
}
