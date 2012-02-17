#include <assert.h>
#include "poly.hpp"
#include "seg.hpp"

/* requires points != null */
/* requires num_pts >= 3 */
/* The vector FROM point i INTO point i+1 must be such that a negative rotation
 * of it points INTO the polygon. */
poly::poly(real_t *points, unsigned num_pts, real_t e) : solid() {
	assert(points);
	assert(num_pts >= 3);
	this->elasticity = e;
	this->solid_type = NB_SLD_POLY;

#define X_COORD(i) (points[(2 * (i))])
#define Y_COORD(i) (points[(2 * (i)) + 1])
	seg *segs[num_pts];
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
