#include "SDL/SDL.h"
#include "SDL/SDL_gfxPrimitives.h"
#include "visible.hpp"
#include "solid.hpp"
#include "level_utils.hpp"
#include "vector.hpp"

visible::visible(int type, void *data) {
	this->visible_type = type;
	this->visible_data = data;
}

void show_visible(visible *v, SDL_Surface *screen, SDL_Rect *camera) {
	int type = v->visible_type;
	//TODO is there any way to not have to declare all this crap out here
	real_t x1, x2, y1, y2;
	segment **segs;
	SDL_Surface *img;
	struct ball_data_t *bd;
	solid *s = NULL;
	if (type >= 0 && type < NB_NUM_SLD_TYPES)
		s = (solid *) v;
	//printf("visible type %d\n", type);
	switch (type) {
		case NB_SLD_BALL:
			img = (SDL_Surface *) v->visible_data;
			bd = &s->solid_data->ball_data;
			apply_surface(img, screen, s->x - bd->r, s->y - bd->r, camera);
			//printf("drew a sprite\n");
			break;
		case NB_SLD_POLY:
			segs = s->solid_data->poly_data.segs;
			for (int i = 0; i < s->solid_data->poly_data.num_segs; i++) {
				x1 = s->x + segs[i]->x;
				y1 = s->y + segs[i]->y;
				if (camera) {
					x1 -= camera->x;
					y1 -= camera->y;
				}
				x2 = x1 + segs[i]->dir.x;
				y2 = y1 + segs[i]->dir.y;
				//TODO cast better?
				lineColor(screen, x1, y1, x2, y2,
						(unsigned long) v->visible_data);
				//printf("drew seg %d colored %x\n", i, v->visible_data);
			}
			break;
		case NB_NUM_VIS_TYPES:
			printf("abstract visible attempting to get shown!\n");
			break;
		default:
			printf("showing visible of type %d not supported\n", type);
			break;
	}
}
