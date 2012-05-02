#include "level.hpp"
#include "level_utils.hpp"

bool guy_pos_override = false;
int guy_override_x, guy_override_y;

int WORLD_TOP_LIMIT;
int WORLD_BOT_LIMIT;
int WORLD_LEFT_LIMIT;
int WORLD_RIGHT_LIMIT;
level_func level_funcs[NB_NUM_LEVELS];

//TODO where should globals go? it's confusing, sort em out
int run_level (level_func level_init) {
	if (init_stuff())
		return -NB_LEVEL_ERROR;

	level_init();

	Uint32 render_start = 0;
	Uint32 render_time = 0;
	Uint32 second_start;
	unsigned frames = 0;

	init_guy();
	MOVES_PUSH(guy);

	SDL_Event event;

	int i, j;
	second_start = SDL_GetTicks();
	quit = 0;
	while (!quit) {
		render_start = SDL_GetTicks();

		for (i = 0; i < moves->size(); i++) {
			moves->at(i).m->add_tmp_force(gravity);
		}

		//TODO how about ACTUAL render time?
		real_t dt = MIN_RENDER_TIME / 1000.0;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				quit = -NB_LEVEL_QUIT;
				break;
			}

			guy->handle_input(event, dt);
		}
		int mx, my;
		char button = SDL_GetMouseState(&mx, &my);
		mx += camera.x;
		my += camera.y;

		guy->mouse_at(mx, my, button);

		guy->choose_action(walls, dt); //TODO everyone should do this

		//TODO move this somewhere better?
#define CHECK_COLLISION(s1,s2,mi1,mi2) do {\
	struct collision_data data;\
	if (solids_collide(s1, s2, &data)) {\
		resolve_collision(s1, s2, data);\
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
						i, -1);
			}
		}
		/* moveable-moveable collisions */
		for (i = 0; i < moves->size(); i++) {
			for (j = i + 1; j < moves->size(); j++) {
				CHECK_COLLISION(*moves->at(i).m,
						*moves->at(j).m,
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

		if (!guy->alive)
			dead();
		if (guy->x > SCREEN_WIDTH + WORLD_RIGHT_LIMIT) {
			dead();
		} else if (guy->x < -WORLD_LEFT_LIMIT) {
			dead();
		} else if (guy->y > SCREEN_HEIGHT + WORLD_BOT_LIMIT) {
			dead();
		} else if (guy->y < -WORLD_TOP_LIMIT) {
			dead();
		} else if (guy->won) {
			success();
		}
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

	return quit;
}

void test_level (void); //XXX debug
void init_level_funcs (void) {
	level_funcs[0] = load_tutorial;
	level_funcs[1] = load_level_one;

	//level_funcs[0] = test_level;
}

real_t wall1_pts[] = {0, 0, 50, 0, 50, -400, 0, -400};
real_t wall2_pts[] = {0, 0, 500, 0, 500, -100, 0, -100};
real_t wall2thin_pts[] = {0, 0, 1500, 0, 1500, -70, 0, -70};
real_t wall2thinner_pts[] = {0, 0, 1500, 0, 1500, -20, 0, -20};
real_t sloped_pts[] = {0, 0, 0, 50, 300, 50, 200, 0};
real_t wallthin_pts[] = {0, 0, 20, 0, 20, -380, 0, -380};
real_t wallthin2_pts[] = {0, 0, 20, 0, 20, -320, 0, -320};
real_t sq_points[] = {0, 0, 20, 0, 20, -20, 0, -20};
real_t tri_points[] = {0, 0, 30, -10, 15, -30};
real_t hex_points[] = {0, 0, 40, 0, 50, -20,
	45, -40, 30, -35, -10, -10};
real_t anchor[] = {0, 0, 15, -40, -15, -40};
real_t spike_pts[] = {0, 0, 30, -20, 0, -40};
real_t unstick_pts[] = {0, 0, 50, 0, -100, -100, -150, -100};
real_t upright_tri[] = {0, 0, 100, 0, 0, -100};
real_t rev_upright_tri[] = {0, 0, 100, 0, 100, -100};
real_t small_rect[] = {0, 0, 250, 0, 250, -40, 0, -40};
real_t med_wall[] = {0, 0, 240, 0, 240, -40, 0, -40};
real_t med_wall_thin[] = {0, 0, 200, 0, 200, -20, 0, -20};
real_t med_wall_thin2[] = {0, 0, 240, 0, 240, -20, 0, -20};
real_t med_wall_up[] = {0, 0, 40, 0, 40, -220, 0, -220};
real_t med_wall_up_thin[] = {0, 0, 20, 0, 20, -170, 0, -170};

void test_level (void) {
	WORLD_TOP_LIMIT = 500;
	WORLD_BOT_LIMIT = 500;
	WORLD_LEFT_LIMIT = 500;
	WORLD_RIGHT_LIMIT = 500;

	walls->push_back(new_poly(NULL, true, wall1_pts, 4, GUY_INIT_X, GUY_INIT_Y+590));
}

void load_tutorial (void) {
	WORLD_TOP_LIMIT = 500;
	WORLD_BOT_LIMIT = 100;
	WORLD_LEFT_LIMIT = 200;
	WORLD_RIGHT_LIMIT = 2000;

	walls->push_back(new_poly(NULL, true, wall2_pts, 4, GUY_INIT_X - 20,
				GUY_INIT_Y - 150));
	walls->push_back(new_poly(NULL, true, wall2_pts, 4, GUY_INIT_X - 20,
				GUY_INIT_Y + 150));
	solid *sbuf = new_poly(NULL, true, wall1_pts, 4, GUY_INIT_X - 20,
			GUY_INIT_Y + 570, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, wall2_pts, 4, GUY_INIT_X + 500,
			GUY_INIT_Y + 200, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);
	sbuf = new_poly(NULL, true, wall2_pts, 4, GUY_INIT_X + 1000,
			GUY_INIT_Y + 200, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	walls->push_back(new_poly(NULL, true, wall2_pts, 4, GUY_INIT_X + 500,
				GUY_INIT_Y - 200));
	walls->push_back(new_poly(NULL, true, wall2_pts, 4, GUY_INIT_X + 1001,
				GUY_INIT_Y - 200));
	walls->push_back(new_poly(NULL, true, small_rect, 4, GUY_INIT_X + 1550,
				GUY_INIT_Y - 90));
	walls->push_back(new_poly(NULL, true, small_rect, 4, GUY_INIT_X + 1550,
				GUY_INIT_Y + 30));

	sbuf = new_poly(NULL, true, upright_tri, 3, GUY_INIT_X + 1520,
			GUY_INIT_Y - 150, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, sq_points, 4, GUY_INIT_X + 1520,
			GUY_INIT_Y + 70, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, sq_points, 4, GUY_INIT_X + 1750,
			GUY_INIT_Y - 40, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, anchor, 3, GUY_INIT_X + 1700,
			GUY_INIT_Y - 30, 1, 0xFFff00ff);
	put_solid_prop(NB_VICTORY, sbuf->props);
	walls->push_back(sbuf);

	SDL_WM_SetCaption("Ninjaball intro level", NULL);

	guy_pos_override = false;
}

void load_level_one (void) {
	WORLD_TOP_LIMIT = 5000;
	WORLD_BOT_LIMIT = 0;
	WORLD_LEFT_LIMIT = 800;
	WORLD_RIGHT_LIMIT = 1500;

	walls->push_back(new_poly(NULL, true, sloped_pts, 4, GUY_INIT_X - 30,
				GUY_INIT_Y + 30));

	walls->push_back(new_poly(NULL, true, wall1_pts, 4, GUY_INIT_X + 100,
				GUY_INIT_Y - 230));
	walls->push_back(new_poly(NULL, true, wall1_pts, 4, GUY_INIT_X + 250,
				GUY_INIT_Y - 230));

	solid *sbuf = new_poly(NULL, true, wallthin_pts, 4, GUY_INIT_X + 65,
			GUY_INIT_Y - 232, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);
	sbuf = new_poly(NULL, true, wallthin2_pts, 4, GUY_INIT_X + 310,
			GUY_INIT_Y - 240, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);
	sbuf = new_poly(NULL, true, small_rect, 4, GUY_INIT_X + 310,
			GUY_INIT_Y - 585, 1, 0x00ff00ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 680,
				GUY_INIT_Y - 900));
	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 980,
				GUY_INIT_Y - 900));
	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 1280,
				GUY_INIT_Y - 900));
	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 1580,
				GUY_INIT_Y - 900));

	sbuf = new_poly(NULL, true, unstick_pts, 4, GUY_INIT_X + 1800,
			GUY_INIT_Y - 880, 1, 0x00ff00ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);
	sbuf = new_poly(NULL, true, spike_pts, 3, GUY_INIT_X + 1850,
			GUY_INIT_Y - 860, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 1730,
				GUY_INIT_Y - 1040));

	walls->push_back(new_poly(NULL, true, wall2thin_pts, 4, GUY_INIT_X + 145,
				GUY_INIT_Y - 1000));
	sbuf = new_poly(NULL, true, wall2thinner_pts, 4, GUY_INIT_X + 130,
			GUY_INIT_Y - 970, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, med_wall_up, 4, GUY_INIT_X + 80,
			GUY_INIT_Y - 770, 1, 0x00ff00ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 80,
				GUY_INIT_Y - 1300));
	walls->push_back(new_poly(NULL, true, anchor, 3, GUY_INIT_X + 20,
				GUY_INIT_Y - 1600));

	sbuf = new_poly(NULL, true, med_wall, 4, GUY_INIT_X - 240,
			GUY_INIT_Y - 1000, 1, 0x00ff00ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, med_wall_thin2, 4, GUY_INIT_X - 270,
			GUY_INIT_Y - 970, 1, 0xFF0000ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, med_wall_thin2, 4, GUY_INIT_X - 230,
			GUY_INIT_Y - 740, 1, 0xFF0000ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, med_wall_up, 4, GUY_INIT_X - 230,
			GUY_INIT_Y - 519, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, med_wall_thin, 4, GUY_INIT_X - 440,
			GUY_INIT_Y - 549, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, anchor, 3, GUY_INIT_X - 350,
			GUY_INIT_Y - 600, 1, 0xFFff00ff);
	put_solid_prop(NB_VICTORY, sbuf->props);
	walls->push_back(sbuf);

	walls->push_back(new_poly(NULL, true, rev_upright_tri, 3, GUY_INIT_X - 21,
				GUY_INIT_Y - 775));

	SDL_WM_SetCaption("Ninjaball level 1", NULL);

	/*
	guy_pos_override = true;
	guy_override_x = 500;
	guy_override_y = GUY_INIT_Y - 1200;
	*/
}
