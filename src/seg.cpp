#include <assert.h>
#include "seg.hpp"
#include "test_utils.hpp"

/* requires dx, dy > 0 */
seg::seg(real_t x, real_t y, real_t dx, real_t dy, bool directed, real_t e)
	: solid(x, y, e) {
	this->solid_type = NB_SLD_SEG;
	assert(dx != 0 || dy != 0);
	vector2d_t *dir = new vector2d_t(dx, dy);
	this->solid_data->seg_data.dir = dir;
	this->solid_data->seg_data.directed = directed;
}

bool point_on_segment(real_t x, real_t y, seg& segment) {
	seg_data_t& sd = segment.solid_data->seg_data;
	real_t x1 = segment.x, y1 = segment.y;
	real_t x2 = x1 + sd.dir->x, y2 = y1 + sd.dir->y;
	real_t m = (y1 - y2) / (x1 - x2);

	if (x1 == x2) { /* vertical */
		//XXX should it be SIMILAR_REALS(x, x1)?
		return ((x == x1) && between(y1, y, y2));
	}

	real_t y_coord = LINE_Y_COORD(x, m, x1, y1);
	return between(y1, y_coord, y2) && SIMILAR_REALS(y_coord, y);
}
