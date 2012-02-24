#include "polywall.hpp"
#include "SDL/SDL_gfxPrimitives.h"
#include "seg.hpp"

//TODO this ordering is wrong, considering the SDL coordinate system
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
		/*
		printf("x1: %g = %g + %g\n",x1,this->x,segs[i]->x);
		printf("y1: %g = %g + %g\n",y1,this->y,segs[i]->y);
		*/
		if (camera) {
			/*
			printf("%g - %g = ",x1,camera->x);
			*/
			x1 = x1 - camera->x;
			/*
			printf("%g\n",x1);
			printf("%g -= %g = ",y1,camera->y);
			*/
			y1 -= camera->y;
			/*
			printf("%g\n",y1);
			*/
		}
		/*
		printf("x1: %g = %g + %g - %g\n",x1,this->x,segs[i]->x,camera->x);
		printf("y1: %g = %g + %g - %g\n",y1,this->y,segs[i]->x,camera->y);
		*/
		x2 = x1 + segs[i]->solid_data->seg_data.dir->x;
		y2 = y1 + segs[i]->solid_data->seg_data.dir->y;
		//TODO magic number
		/*
		printf("seg: %g,%g (%g,%g), poly %g,%g, camera %g,%g\n",
		segs[i]->x,segs[i]->y,
		segs[i]->solid_data->seg_data.dir->x,
		segs[i]->solid_data->seg_data.dir->y,
		this->x, this->y,
		camera->x, camera->y);
		printf("drawing (%g,%g) to (%g,%g)\n",x1,y1,x2,y2);
		*/
		lineColor(screen, x1, y1, x2, y2, 0xFFffFFff);
	}
}

