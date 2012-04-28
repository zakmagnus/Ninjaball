#include "level_utils.hpp"
#include "level.hpp"
#include "SDL/SDL_image.h"
#include "SDL/SDL_ttf.h"
#include "ui.hpp"

real_t frame_air_drag = pow(AIR_FRICTION, 1.0 / FRAMES_PER_SEC);

vector2d_t gravity = downvec * 150;

SDL_Surface *screen, *img1, *img2, *img3;
TTF_Font *font;
SDL_Rect camera;
vector<solid *> *walls;
vector<moveable_data> *moves;
player *guy = NULL;

int quit = 0;

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

SDL_Surface *load_img (char *filename) {
	SDL_Surface *tmp = IMG_Load(filename);
	if (!tmp)
		return NULL;
	SDL_Surface *opt = SDL_DisplayFormatAlpha(tmp);
	SDL_FreeSurface(tmp);
	return opt;
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

//TODO this stuff is just for debug convenience
extern bool guy_pos_override;
extern int guy_override_x, guy_override_y;
void init_guy(void) {
	if (guy)
		delete guy;

	if (!guy_pos_override) {
		guy = (player *)new_ball(new player(), false, GUY_INIT_X, GUY_INIT_Y,
				img3->h / 2.0, img3, 1.0);
	}
	else {
		guy = (player *)new_ball(new player(), false, guy_override_x,
				guy_override_y, img3->h / 2.0, img3, 1.0);
	}
}

void dead(void) {
	quit = -NB_LEVEL_DEAD;
}

void success(void) {
	quit = NB_LEVEL_SUCCESS;
}

int init_stuff (void) {
	camera.x = CAM_INIT_X;
	camera.y = CAM_INIT_Y;
	camera.h = SCREEN_HEIGHT;
	camera.w = SCREEN_WIDTH;

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
}

