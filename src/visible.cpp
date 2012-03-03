#include "SDL/SDL.h"
#include "SDL/SDL_gfxPrimitives.h"
#include "visible.hpp"
#include "solid.hpp"
#include "test_utils.hpp"
#include "vector.hpp"
#include "seg.hpp"

visible::visible(void *data) {
	this->visible_data = data;
}

void show_visible(solid& s, visible *v, SDL_Surface *screen,
		SDL_Rect *camera) {
	int type = s.get_solid_type();
	//TODO is there any way to not have to declare all this crap out here
	real_t x1, x2, y1, y2;
	seg **segs;
	SDL_Surface *img;
	struct ball_data_t *bd;
	switch (type) {
		case NB_SLD_BALL:
			img = (SDL_Surface *) v->visible_data;
			bd = &s.solid_data->ball_data;
			apply_surface(img, screen, s.x - bd->r, s.y - bd->r, camera);
			break;
		case NB_SLD_POLY:
			segs = (seg **) s.solid_data->poly_data.segs;
			for (int i = 0; i < s.solid_data->poly_data.num_segs; i++) {
				x1 = s.x + segs[i]->x;
				y1 = s.y + segs[i]->y;
				if (camera) {
					x1 -= camera->x;
					y1 -= camera->y;
				}
				x2 = x1 + segs[i]->solid_data->seg_data.dir->x;
				y2 = y1 + segs[i]->solid_data->seg_data.dir->y;
				//TODO magic number
				lineColor(screen, x1, y1, x2, y2, 0xFFffFFff);
			}
			break;
		case NB_SLD_SEG:
			x1 = s.x;
			y1 = s.y;
			if (camera) {
				x1 -= camera->x;
				y1 -= camera->y;
			}
			x2 = x1 + s.solid_data->seg_data.dir->x;
			y2 = y1 + s.solid_data->seg_data.dir->y;
			//TODO magic number
			lineColor(screen, x1, y1, x2, y2, 0xFFffFFff);
			break;
		default:
			printf("WTF! trying to show solid of type %d\n", type);
			break;
	}
}
