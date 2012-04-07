#include "SDL/SDL_ttf.h"
#include "ui.hpp"
#include "level_utils.hpp"

static SDL_Color white = {0xff, 0xff, 0xff, 0xff};

int init_sdl (void) {
	SDL_Init(SDL_INIT_EVERYTHING);
	TTF_Init();
	font = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf",
			15);

	screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP,
			SDL_SWSURFACE);
}

void teardown_sdl (void) {
	TTF_Quit();
	SDL_Quit();
}

void show_screen (char **message) {
	SDL_FillRect(screen, NULL, 0);

	int line = 0;
	while (*message) {
		SDL_Surface *text = TTF_RenderText_Solid(font, *message, white);
		apply_surface(text, screen, 15, 15 + line * 15, NULL);
		line++;
		message++;
	}
	SDL_Flip(screen);

	SDL_Event event;
	bool end = false;
	while (!end) {
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT || event.type == SDL_KEYDOWN) {
				end = true;
				break;
			}
		}
	}
}
