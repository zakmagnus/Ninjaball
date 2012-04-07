#ifndef NB_LEVEL_UTILS_HPP
#define NB_LEVEL_UTILS_HPP

#include <stdio.h>
#include <math.h>
#include <vector>
#include "player.hpp"
#include "Moveable.hpp"
#include "solid.hpp"
using namespace std;

#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 700
#define SCREEN_BPP    32
#define CAM_DEADZONE_H 100
#define CAM_DEADZONE_W 150
#define CAM_INIT_X (0)
#define CAM_INIT_Y (0)
extern int WORLD_TOP_LIMIT;
extern int WORLD_BOT_LIMIT;
extern int WORLD_LEFT_LIMIT;
extern int WORLD_RIGHT_LIMIT;
#define CAM_MAX_Y (CAM_INIT_Y + WORLD_BOT_LIMIT)
#define CAM_MIN_Y (CAM_INIT_Y - WORLD_TOP_LIMIT)
#define CAM_MAX_X (CAM_INIT_X + WORLD_RIGHT_LIMIT)
#define CAM_MIN_X (CAM_INIT_X - WORLD_LEFT_LIMIT)

#define GUY_INIT_X (170)
#define GUY_INIT_Y (200)

#define FRAMES_PER_SEC (60)
#define MIN_RENDER_TIME (1000 / FRAMES_PER_SEC) /* ms */

#define AIR_FRICTION (0.7) /* approx. amount of velocity remaining second */

extern real_t frame_air_drag;

extern vector2d_t gravity;

struct moveable_data {
	Moveable *m;
	real_t old_x, old_y;
	bool collided;
};

extern SDL_Surface *screen, *img1, *img2, *img3;
extern SDL_Rect camera;
extern vector<solid *> *walls;
extern vector<moveable_data> *moves;
extern player *guy;

extern int quit;

#define MOVES_PUSH(p) do {\
	moves->resize(moves->size() + 1);\
	moves->back().m = (p);\
	moves->back().collided = false;\
} while (0)

void apply_surface (SDL_Surface *src, SDL_Surface *dst, int x, int y,
		SDL_Rect *clip);
SDL_Surface *load_img (char *filename);
int init_stuff (void);
void teardown_stuff (void);
void update_camera(void);
void init_guy(void);
void dead(void);

#endif
