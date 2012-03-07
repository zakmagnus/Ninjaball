#ifndef NB_TEST_UTILS_H
#define NB_TEST_UTILS_H

#include <math.h>
#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include "physics.hpp"
#include "vector.hpp"
#include "Moveable.hpp"
#include "rope.hpp"
#include "ball.hpp"
#include "seg.hpp"

struct moveable_data {
	Moveable *m;
	real_t old_x, old_y;
	bool collided;
};

#define NB_HOOK_SPD (1800.0)
#define NB_ROPE_MAX_LEN (600)

class hook : public seg {
	public:
		hook() : seg(0, 0, 1, 1, false) {};
		void init(Moveable *m, vector2d_t& dir);
		bool advance(Moveable *m, real_t dt);
};

SDL_Surface *load_img (char *filename);
void apply_surface (SDL_Surface *src, SDL_Surface *dst, int x, int y,
		SDL_Rect *clip);
void render_rope(SDL_Surface *srf, ropedata& rope, SDL_Rect *camera);
void render_pointer(SDL_Surface *srf, ball *b, vector2d_t& dir, SDL_Rect *camera);

/* mathy stuff */
#define min(a,b) ((a)<(b)? (a) : (b))
#define max(a,b) ((a)>=(b)? (a) : (b))
#define SINGLE_DIM_OVERLAP(v1min,v1max,v2min,v2max) \
	(max((v1max), (v2max)) - min((v1min), (v2min))\
	 - fabs((v1max) - (v2max))\
	 - fabs((v1min) - (v2min)))

#define between_ord(v1,v2,v3) ((v1) <= (v2) && (v2) <= (v3))
#define between(v1,v2,v3) (between_ord((v1),(v2),(v3)) ||\
		between_ord((v3),(v2),(v1)))

#define LINE_Y_COORD(x,m,x0,y0) ((((x) - (x0)) * (m)) + (y0))

#define REAL_EPSILON (0.001)
#define SIMILAR_REALS(r1,r2) (fabs((r1) - (r2)) <= REAL_EPSILON)

#endif
