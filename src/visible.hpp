#include "SDL/SDL.h"
#include "solid.hpp"

#ifndef NB_VISIBLE_H
#define NB_VISIBLE_H

//TODO all solids should be visibles! but some visible might not be solids
class visible {
	public:
		void *visible_data;
		visible(void *data=NULL);
};

void show_visible(solid& s, visible *v, SDL_Surface *screen,
		SDL_Rect *camera);

#endif
