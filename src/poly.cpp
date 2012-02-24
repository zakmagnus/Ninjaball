#include <assert.h>
#include <stdarg.h>
#include "poly.hpp"
#include "seg.hpp"

/* requires points != null */
/* requires num_pts >= 3 */
/* The vector FROM point i INTO point i+1 must be such that a negative rotation
 * of it points INTO the polygon. */
poly::poly(real_t *points, unsigned num_pts, real_t x, real_t y, real_t e) : solid() {
	assert(points);
	assert(num_pts >= 3);
	this->x = x;
	this->y = y;
	this->elasticity = e;
	this->solid_type = NB_SLD_POLY;

#define X_COORD(i) (points[(2 * (i))])
#define Y_COORD(i) (points[(2 * (i)) + 1])
	seg **segs = new seg *[num_pts];
	for (int i = 0; i < num_pts; i++) {
		int next_i = (i + 1) % num_pts;
		seg *segment = new seg(X_COORD(i), Y_COORD(i),
				X_COORD(next_i) - X_COORD(i),
				Y_COORD(next_i) - Y_COORD(i), true);
		segs[i] = segment;
	}

	this->solid_data->poly_data.segs = (void **) segs;
	this->solid_data->poly_data.num_segs = num_pts;
}

poly::poly(real_t y, real_t x, real_t e, int num_points, ...) {
	assert(num_points >= 3);
	this->x = x;
	this->y = y;
	this->elasticity = e;
	this->solid_type = NB_SLD_POLY;

	seg **segs = new seg *[num_points];
	va_list args;
	va_start(args, num_points);
	/* varargs create doubles :^/ */
	real_t prev_x = va_arg(args, double);
	real_t prev_y = va_arg(args, double);
	for (int i = 0; i < num_points - 1; i++) {
		real_t x = va_arg(args, double);
		real_t y = va_arg(args, double);
		seg *s = new seg(prev_x, prev_y, x - prev_x, y - prev_y, true);
		segs[i] = s;

		prev_x = x;
		prev_y = y;
	}
	segs[num_points - 1] = new seg(prev_x, prev_y, segs[0]->x - prev_x,
			segs[0]->y - prev_y);
	this->solid_data->poly_data.segs = (void **) segs;
	this->solid_data->poly_data.num_segs = num_points;
}
