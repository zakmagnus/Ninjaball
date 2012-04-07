#include "level.hpp"
#include "level_utils.hpp"

int run_level (void) {
	if (init_stuff())
		exit(1);

	Uint32 render_start = 0;
	Uint32 render_time = 0;
	Uint32 second_start;
	unsigned frames = 0;

	init_guy();
	MOVES_PUSH(guy);

	real_t wall2_pts[] = {0, 0, 500, 0, 500, -100, 0, -100};
	walls->push_back(new_poly(NULL, true, wall2_pts, 4, 200, 600));
	real_t sloped_pts[] = {0, 0, 0, 50, 300, 50, 200, 0};
	walls->push_back(new_poly(NULL, true, sloped_pts, 4, 100, 100));

	real_t wall1_pts[] = {0, 0, 50, 0, 50, -400, 0, -400};
	walls->push_back(new_poly(NULL, true, wall1_pts, 4, 500, 300));
	walls->push_back(new_poly(NULL, true, wall1_pts, 4, 650, 300));
	
	real_t sq_points[] = {0, 0, 20, 0, 20, -20, 0, -20};
	//MOVES_PUSH(new Moveable(new_poly(NULL, sq_points, 4, 270, 290, 1)));
	//MOVES_PUSH(new Moveable(new_poly(NULL, sq_points, 4, 290, 490, 1)));
	real_t tri_points[] = {0, 0, 30, -10, 15, -30};
	//MOVES_PUSH(new Moveable(new_poly(NULL, tri_points, 3, 200, 290, 1)));

	real_t hex_points[] = {0, 0, 40, 0, 50, -20,
		45, -40, 30, -35, -10, -10};
	MOVES_PUSH((Moveable *)new_poly(new Moveable(), false, hex_points, 6, 170, 50, 1));

	solid *sbuf = new_poly(NULL, true, tri_points, 3, 800, 450,
			1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, sq_points, 4, 0, 350, 1, 0x00ff00ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	sbuf = new_poly(NULL, true, sloped_pts, 4, -180, 200, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	walls->push_back(sbuf);

	real_t anchor[] = {0, 0, 15, -40, -15, -40};
	walls->push_back(new_poly(NULL, true, anchor, 3, 1000, 390));
	walls->push_back(new_poly(NULL, true, anchor, 3, 1300, 390));
	walls->push_back(new_poly(NULL, true, anchor, 3, 1600, 390));
	walls->push_back(new_poly(NULL, true, anchor, 3, 1900, 390));
	walls->push_back(new_poly(NULL, true, anchor, 3, 2200, 250));

	real_t unstick_pts[] = {0, 0, 50, 0, -100, -100, -150, -100};
	sbuf = new_poly(NULL, true, unstick_pts, 4, 2200, 390, 1, 0x00ff00ff);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);
	real_t spike_pts[] = {0, 0, 30, -20, 0, -40};
	sbuf = new_poly(NULL, true, spike_pts, 3, 2250, 410, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	real_t upright_tri[] = {0, 0, 100, -100, 100, 0};
	walls->push_back(new_poly(NULL, true, upright_tri, 3, 70, 800));
	sbuf = new_poly(NULL, true, wall2_pts, 4, -450, 700, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);

	real_t small_rect[] = {0, 0, 250, 0, 250, -40, 0, -40};
	sbuf = new_poly(NULL, true, small_rect, 4, -170, 860, 1, 0xFF0000ff);
	put_solid_prop(NB_DEADLY, sbuf->props);
	put_solid_prop(NB_UNSTICKABLE, sbuf->props);
	walls->push_back(sbuf);
	
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
				quit = true;
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
	vector2d_t dir;\
	if (solids_collide(s1, s2, &dir)) {\
		resolve_collision(s1, s2, dir);\
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

	return quit;
}
