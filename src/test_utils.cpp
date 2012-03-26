#include <math.h>
#include "SDL/SDL_gfxPrimitives.h"
#include "test_utils.hpp"

SDL_Surface *load_img (char *filename) {
	SDL_Surface *tmp = IMG_Load(filename);
	if (!tmp)
		return NULL;
	SDL_Surface *opt = SDL_DisplayFormatAlpha(tmp);
	SDL_FreeSurface(tmp);
	return opt;
}

void apply_surface (SDL_Surface *src, SDL_Surface *dst, int x, int y,
		SDL_Rect *clip) {
	SDL_Rect off;
	off.x = x;
	off.y = y;
	if (clip) {
		off.x = x - clip->x;
		off.y = y - clip->y;
	}

	SDL_BlitSurface(src, NULL, dst, &off);
}

//TODO use a visible here
void render_rope(SDL_Surface *srf, ropedata& rope, SDL_Rect *camera) {
	//TODO render loose rope hanging loosely
	real_t x1 = rope.x1, x2 = rope.x2, y1 = rope.y1, y2 = rope.y2;
	if (camera) {
		x1 -= camera->x;
		x2 -= camera->x;
		y1 -= camera->y;
		y2 -= camera->y;
	}
	real_t d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	unsigned color = NB_ROPE_COLOR;
	if (d < rope.length) {
		real_t looseness = 1 - (d / rope.length);
		unsigned tight_comp_r = ((unsigned)
			((NB_ROPE_COLOR&0xff000000) * (1 - looseness)))
			& 0xFF000000;
		unsigned loose_comp_r = ((unsigned)
			((NB_ROPE_LOOSE_COLOR&0xff000000) * looseness))
			& 0xFF000000;
		unsigned tight_comp_g = ((unsigned)
			((NB_ROPE_COLOR&0x00ff0000) * (1 - looseness)))
			& 0x00ff0000;
		unsigned loose_comp_g = ((unsigned)
			((NB_ROPE_LOOSE_COLOR&0x00ff0000) * looseness))
			& 0x00ff0000;
		unsigned tight_comp_b = ((unsigned)
			((NB_ROPE_COLOR&0x0000FF00) * (1 - looseness)))
			& 0x0000FF00;
		unsigned loose_comp_b = ((unsigned)
			((NB_ROPE_LOOSE_COLOR&0x0000FF00) * looseness))
			& 0x0000FF00;
		color = loose_comp_r + tight_comp_r + loose_comp_g +
			tight_comp_g + loose_comp_b + tight_comp_b;
		color |= 0xff;
	}
	lineColor(srf, x1, y1, x2, y2, color);
}

//TODO use a visible here
void render_pointer(SDL_Surface *srf, solid *b, vector2d_t& dir, SDL_Rect *camera) {
	vector2d_t endpt;
	endpt.x = b->x;
	endpt.y = b->y;
	endpt += dir * (b->solid_data->ball_data.r + 2); //TODO magic!
	real_t x1 = b->x, x2 = endpt.x, y1 = b->y, y2 = endpt.y;
	if (camera) {
		x1 -= camera->x;
		x2 -= camera->x;
		y1 -= camera->y;
		y2 -= camera->y;
	}
	lineColor(srf, x1, y1, x2, y2, 0xFFffFFff);
}

//TODO move this come on
void hook::init(Moveable *m, vector2d_t& dir) {
	this->x = m->x;
	this->y = m->y;
	*this->solid_data->seg_data.dir = dir;
}

bool hook::advance(Moveable *m, real_t dt) {
	real_t d = vector2d_t(m->x - (this->x + this->solid_data->seg_data.dir->x),
			m->y - (this->y + this->solid_data->seg_data.dir->y)).norm();
	if (d >= NB_ROPE_MAX_LEN)
		return false;

	this->x += this->solid_data->seg_data.dir->x;
	this->y += this->solid_data->seg_data.dir->y;
	this->solid_data->seg_data.dir->normalize();
	/*
	this->x = m->x + ((*this->solid_data->seg_data.dir) * d).x;
	this->y = m->y + ((*this->solid_data->seg_data.dir) * d).y;
	*/
	(*this->solid_data->seg_data.dir) *= dt * NB_HOOK_SPD;
	return true;
}
