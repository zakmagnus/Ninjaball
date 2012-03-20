#include <stdio.h>
#include <math.h>
#include <vector>
#include "test_utils.hpp"
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
#define WORLD_TOP_LIMIT 1000
#define WORLD_BOT_LIMIT 500
#define WORLD_LEFT_LIMIT 500
#define WORLD_RIGHT_LIMIT 500
#define CAM_MAX_Y (CAM_INIT_Y + WORLD_BOT_LIMIT)
#define CAM_MIN_Y (CAM_INIT_Y - WORLD_TOP_LIMIT)
#define CAM_MAX_X (CAM_INIT_X + WORLD_RIGHT_LIMIT)
#define CAM_MIN_X (CAM_INIT_X - WORLD_LEFT_LIMIT)

#define GUY_INIT_X (170)
#define GUY_INIT_Y (200)

#define FRAMES_PER_SEC (60)
#define MIN_RENDER_TIME (1000 / FRAMES_PER_SEC) /* ms */

#define AIR_FRICTION (0.6) /* approx. % velocity lost per second */

static real_t frame_air_drag = pow(AIR_FRICTION, 1.0 / FRAMES_PER_SEC);

static vector2d_t gravity = downvec * 150;

SDL_Surface *load_img (char *);
int init_stuff (void);
void teardown_stuff (void);
void update_camera(void);
void init_guy(void);
void dead(void);

static SDL_Surface *screen, *img1, *img2, *img3;
static SDL_Rect camera;
static vector<solid *> *walls;
static vector<moveable_data> *moves;
static player *guy = NULL;

static bool quit = false;

#define MOVES_PUSH(p) do {\
	moves->resize(moves->size() + 1);\
	moves->back().m = (p);\
	moves->back().collided = false;\
} while (0)


int main (int argc, char **argv) {
	if (init_stuff())
		exit(1);

	Uint32 render_start = 0;
	Uint32 render_time = 0;
	Uint32 second_start;
	unsigned frames = 0;

	init_guy();
	MOVES_PUSH(guy);

	real_t wall2_pts[] = {0, 0, 500, 0, 500, -100, 0, -100};
	walls->push_back(new_poly(NULL, wall2_pts, 4, 200, 600));
	real_t sloped_pts[] = {0, 0, 0, 50, 300, 50, 200, 0};
	walls->push_back(new_poly(NULL, sloped_pts, 4, 100, 100));

	real_t wall1_pts[] = {0, 0, 50, 0, 50, -400, 0, -400};
	walls->push_back(new_poly(NULL, wall1_pts, 4, 500, 300));
	walls->push_back(new_poly(NULL, wall1_pts, 4, 650, 300));
	
	real_t sq_points[] = {0, 0, 20, 0, 20, -20, 0, -20};
	//MOVES_PUSH(new Moveable(new_poly(NULL, sq_points, 4, 270, 290, 1)));
	//MOVES_PUSH(new Moveable(new_poly(NULL, sq_points, 4, 290, 490, 1)));
	real_t tri_points[] = {0, 0, 30, -10, 15, -30};
	//MOVES_PUSH(new Moveable(new_poly(NULL, tri_points, 3, 200, 290, 1)));

	real_t hex_points[] = {0, 0, 40, 0, 50, -20,
		45, -40, 30, -35, -10, -10};
	MOVES_PUSH((Moveable *)new_poly(new Moveable(), hex_points, 6, 170, 50, 1));
	
	SDL_Event event;

	int i, j;
	second_start = SDL_GetTicks();
	while (!quit) {
		render_start = SDL_GetTicks();

		for (i = 0; i < moves->size(); i++) {
			moves->at(i).m->add_tmp_force(gravity);
		}

		//TODO how about ACTUAL render time?
		real_t dt = MIN_RENDER_TIME / 1000.0;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				quit = true;
				break;
			}

			guy->handle_input(event, dt);
		}

		guy->choose_action(walls, dt); //TODO everyone should do this

		//TODO move this somewhere better?
#define CHECK_COLLISION(s1,s2,p1,p2,mi1,mi2) do {\
	vector2d_t dir;\
	if (solids_collide(s1, s2, &dir)) {\
		resolve_collision(s1, s2, p1, p2, dir);\
		if (mi1 >= 0) \
			moves->at(mi1).collided = true;\
		if (mi2 >= 0) \
			moves->at(mi2).collided = true;\
	}\
} while (0)
		for (i = 0; i < moves->size(); i++) {
			moves->at(i).m->slow_down(frame_air_drag);
			moves->at(i).old_x = moves->at(i).m->x;
			moves->at(i).old_y = moves->at(i).m->y;
			moves->at(i).m->move(dt);
			/* moveable-wall collisions */
			for (j = 0; j < walls->size(); j++) {
				CHECK_COLLISION(*moves->at(i).m,
						*walls->at(j),
						moves->at(i).m,
						&immobile_physics,
						//walls->at(j)->physics,
						i, -1);
			}
		}
		/* moveable-moveable collisions */
		for (i = 0; i < moves->size(); i++) {
			for (j = i + 1; j < moves->size(); j++) {
				CHECK_COLLISION(*moves->at(i).m,
						*moves->at(j).m,
						moves->at(i).m,
						moves->at(j).m,
						i, j);
			}

			if (moves->at(i).collided) {
				/*
				printf("collision! reverting %g,%g to %g,%g\n",
				moves->at(i).m->s->x,moves->at(i).m->s->y,
				moves->at(i).old_x,moves->at(i).old_y);
				*/
				moves->at(i).m->x = moves->at(i).old_x;
				moves->at(i).m->y = moves->at(i).old_y;
				moves->at(i).collided = false;
			}
		}

		for (i = 0; i < moves->size(); i++) {
			moves->at(i).m->verify_onbases();
		}

		if (guy->x > SCREEN_WIDTH + WORLD_RIGHT_LIMIT)
			dead();
		else if (guy->x < -WORLD_LEFT_LIMIT)
			dead();
		else if (guy->y > SCREEN_HEIGHT + WORLD_BOT_LIMIT)
			dead();
		else if (guy->y < -WORLD_LEFT_LIMIT)
			dead();
		update_camera();

		SDL_FillRect(screen, NULL, 0);
		for (j = 0; j < walls->size(); j++)
			show_visible(walls->at(j), screen, &camera);
		for (i = 0; i < moves->size(); i++)
			moves->at(i).m->show(screen, &camera);

		SDL_Flip(screen);

		render_time = SDL_GetTicks() - render_start;
		if (render_time < MIN_RENDER_TIME)
			SDL_Delay(MIN_RENDER_TIME - render_time);

		frames++;
		if (SDL_GetTicks() - second_start >= 1000) {
			second_start = SDL_GetTicks();
			frames = 0;
			//printf("****TICK****\n");
		}
	}

	teardown_stuff();

	return 1;
}

void update_camera(void) {
	real_t center_x = camera.x + (SCREEN_WIDTH / 2);
	real_t center_y = camera.y + (SCREEN_HEIGHT / 2);

	real_t dz_top = center_y - (CAM_DEADZONE_H / 2);
	real_t dz_bot = center_y + (CAM_DEADZONE_H / 2);
	real_t dz_left = center_x - (CAM_DEADZONE_W / 2);
	real_t dz_right = center_x + (CAM_DEADZONE_W / 2);

	if (guy->x > dz_right) {
		camera.x = guy->x - (SCREEN_WIDTH / 2)
			- (CAM_DEADZONE_W / 2);
	}
	else if (guy->x < dz_left) {
		camera.x = guy->x - (SCREEN_WIDTH / 2)
			+ (CAM_DEADZONE_W / 2);
	}

	if (guy->y > dz_bot) {
		camera.y = guy->y - (SCREEN_HEIGHT / 2)
			- (CAM_DEADZONE_H / 2);
	}
	else if (guy->y < dz_top) {
		camera.y = guy->y - (SCREEN_HEIGHT / 2)
			+ (CAM_DEADZONE_H / 2);
	}
	if (camera.y > CAM_MAX_Y)
		camera.y = CAM_MAX_Y;
	if (camera.y < CAM_MIN_Y)
		camera.y = CAM_MIN_Y;
	if (camera.x > CAM_MAX_X)
		camera.x = CAM_MAX_X;
	if (camera.x < CAM_MIN_X)
		camera.x = CAM_MIN_X;
}

void init_guy(void) {
	if (guy)
		delete guy;

	guy = (player *)new_ball(new player(), GUY_INIT_X, GUY_INIT_Y,
			img3->h / 2.0, img3, 1.0);
}

void dead(void) {
	//TODO
	init_guy();
}

int init_stuff (void) {
	SDL_Init(SDL_INIT_EVERYTHING);

	screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP,
			SDL_SWSURFACE);

	camera.x = CAM_INIT_X;
	camera.y = CAM_INIT_Y;
	camera.h = SCREEN_HEIGHT;
	camera.w = SCREEN_WIDTH;

	SDL_WM_SetCaption("eventest!", NULL);

	img1 = load_img("../img/nometroid.jpg");
	if (!img1) {
		printf("img failed...\n");
		return -1;
	}
	img2 = load_img("../img/leefs.png");
	if (!img2) {
		printf("img2 failed...\n");
		return -1;
	}
	img3 = load_img("../img/ball.png");
	if (!img3) {
		printf("img3 failed...\n");
		return -1;
	}

	walls = new vector<solid *>();
	if (!walls) {
		printf("could not allocate walls vector\n");
		return -1;
	}
	moves = new vector<moveable_data>();
	if (!moves) {
		printf("could not allocate moveables vector\n");
		return -1;
	}

	init_coll_funcs();

	return 0;
}

void teardown_stuff (void) {
	SDL_FreeSurface(img1);
	SDL_FreeSurface(img2);
	SDL_FreeSurface(img3);
	SDL_Quit();
}
