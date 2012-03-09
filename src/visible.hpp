#ifndef NB_VISIBLE_H
#define NB_VISIBLE_H

#include "SDL/SDL.h"
#include "solid_types.hpp"

/* visible types: they begin with the solid types, and the rest are below */
enum {
	NB_VIS_UI = NB_NUM_SLD_TYPES,
	NB_NUM_VIS_TYPES /* a count, not a type */
};

class visible {
	public:
		int visible_type;
		void *visible_data;
		visible(int type, void *data=NULL);
};

#include "solid.hpp"

void show_visible(visible *v, SDL_Surface *screen, SDL_Rect *camera);

#endif
