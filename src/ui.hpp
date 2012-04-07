#ifndef NB_UI_HPP
#define NB_UI_HPP

#include "SDL/SDL_ttf.h"

extern TTF_Font *font;

int init_sdl (void);
void teardown_sdl (void);
void show_screen (char **message);

#endif
