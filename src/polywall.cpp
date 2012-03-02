#include "polywall.hpp"
#include "SDL/SDL_gfxPrimitives.h"
#include "seg.hpp"

polywall::polywall(real_t x, real_t y, real_t h, real_t w) : poly(x, y, 0, 4, (real_t)0, (real_t)0, (real_t)w, (real_t)0, (real_t)w, (real_t)-h, (real_t)0, (real_t)-h) {
	physics = new physics_t((mass_t)1, true);
}

void polywall::show(SDL_Surface *screen, SDL_Rect *camera) {
	real_t x1, x2, y1, y2;
	seg **segs = (seg **) this->solid_data->poly_data.segs;
	for (int i = 0; i < this->solid_data->
			poly_data.num_segs; i++) {
		x1 = this->x + segs[i]->x;
		y1 = this->y + segs[i]->y;
		if (camera) {
			x1 = x1 - camera->x;
			y1 -= camera->y;
		}
		x2 = x1 + segs[i]->solid_data->seg_data.dir->x;
		y2 = y1 + segs[i]->solid_data->seg_data.dir->y;
		//TODO magic number
		lineColor(screen, x1, y1, x2, y2, 0xFFffFFff);
	}
}

